"""连续听 + 关键词唤醒：把麦克风 PCM 推给流式 ASR。

状态机：
  idle --(能量超阈)--> capturing --(识别到唤醒词或无唤醒词配置)--> command
  command --(静音/超时/收到最终 ASR)--> idle

WSL 环境的两个坑（本节点已内置对策）：
  1. RDP 转发的麦克风底噪电平因机器而异 -> 启动头几秒自动标定能量门限
     （auto_calibrate_sec / calibrate_margin，floor 为 energy_threshold 参数）；
  2. WSLg PulseAudio 采集流会偶发停滞（尤其边播 TTS 边采麦）-> 看门狗发现
     capture_stall_sec 内无数据就自动重启录音进程。

listen_mode=ptt 时本节点不采麦，保留旧的 MicrophoneRecorderNode 按键说话路径。
"""

from __future__ import annotations

import base64
import json
import math
import os
import subprocess
import threading
import time
from typing import Sequence

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .aliyun_speech_nodes import select_microphone_backend


def pcm_rms(frame: bytes) -> float:
    """16-bit little-endian mono PCM 的 RMS 能量。"""
    if not frame or len(frame) < 2:
        return 0.0
    count = len(frame) // 2
    if count <= 0:
        return 0.0
    total = 0.0
    for i in range(0, count * 2, 2):
        sample = int.from_bytes(frame[i : i + 2], byteorder="little", signed=True)
        total += float(sample * sample)
    return math.sqrt(total / float(count))


def calibrated_energy_threshold(
    samples: Sequence[float],
    floor: float,
    margin: float = 2.5,
) -> float:
    """由启动时的环境底噪样本推算能量门限。

    WSL/RDP 转发的麦克风底噪电平因机器而异，固定门限要么误触发
    要么听不见。取底噪 RMS 中位数乘以 margin，且不低于 floor。
    """
    valid = sorted(float(s) for s in samples if s >= 0.0)
    if not valid:
        return floor
    median = valid[len(valid) // 2]
    return max(floor, median * margin)


def text_contains_wake_word(text: str, wake_words: Sequence[str]) -> bool:
    """大小写不敏感的子串唤醒匹配。"""
    normalized = text.strip().lower()
    if not normalized:
        return False
    for word in wake_words:
        token = str(word).strip().lower()
        if token and token in normalized:
            return True
    return False


def strip_wake_words(text: str, wake_words: Sequence[str]) -> str:
    """去掉识别文本里的唤醒词，留下真正指令。"""
    result = text.strip()
    for word in wake_words:
        token = str(word).strip()
        if not token:
            continue
        # 反复剥，避免「小度小度去卧室」残留。
        while True:
            lower = result.lower()
            idx = lower.find(token.lower())
            if idx < 0:
                break
            result = (result[:idx] + result[idx + len(token) :]).strip(" ，,。.!！?？")
    return result.strip()


def build_raw_capture_command(
    backend: str,
    executable: str,
    device: str,
    sample_rate: int,
    channels: int,
) -> list[str]:
    """拼装持续输出 raw PCM 到 stdout 的录音命令。"""
    if sample_rate <= 0 or channels <= 0:
        raise ValueError("sample_rate and channels must be positive")
    if backend == "arecord":
        command = [
            executable,
            "-q",
            "-t",
            "raw",
            "-f",
            "S16_LE",
            "-r",
            str(sample_rate),
            "-c",
            str(channels),
        ]
        if device:
            command.extend(["-D", device])
        return command
    if backend == "ffmpeg_pulse":
        return [
            executable,
            "-nostdin",
            "-loglevel",
            "error",
            "-f",
            "pulse",
            "-i",
            device or "default",
            "-f",
            "s16le",
            "-ac",
            str(channels),
            "-ar",
            str(sample_rate),
            "pipe:1",
        ]
    raise ValueError(f"unsupported recorder backend: {backend}")


class VoiceListenNode(Node):
    """能量门限 + 关键词唤醒的连续听节点。"""

    def __init__(self) -> None:
        super().__init__("mecamind_voice_listen")
        self.declare_parameter("listen_mode", "continuous")  # continuous | ptt
        self.declare_parameter("pcm_topic", "/mecamind/audio_pcm")
        self.declare_parameter("pcm_end_topic", "/mecamind/audio_pcm_end")
        self.declare_parameter("status_topic", "/mecamind/listen_status")
        self.declare_parameter("asr_partial_topic", "/mecamind/asr_partial")
        self.declare_parameter("asr_result_topic", "/mecamind/asr_result")
        self.declare_parameter("voice_command_topic", "/mecamind/voice_command")
        self.declare_parameter("robot_reply_topic", "/mecamind/robot_reply")
        self.declare_parameter("tts_status_topic", "/mecamind/tts_status")
        self.declare_parameter("recorder_backend", "auto")
        self.declare_parameter("device", "default")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("channels", 1)
        self.declare_parameter("frame_ms", 100)
        self.declare_parameter("energy_threshold", 450.0)
        self.declare_parameter("auto_calibrate_sec", 3.0)
        self.declare_parameter("calibrate_margin", 2.5)
        self.declare_parameter("capture_stall_sec", 5.0)
        self.declare_parameter("silence_sec", 1.2)
        self.declare_parameter("command_window_sec", 8.0)
        self.declare_parameter("max_capture_sec", 12.0)
        # 唤醒词「小度小度」：ASR 常转写成「小度，小度」（带标点），
        # 所以列表用单个「小度」做子串兜底；「小杜/小渡/小肚」是同音误转写兜底。
        self.declare_parameter("wake_words", "小度小度,小度,小杜,小渡,小肚")
        self.declare_parameter("pause_while_tts", True)

        self._mode = str(self.get_parameter("listen_mode").value).strip().lower()
        self._wake_words = [
            item.strip()
            for item in str(self.get_parameter("wake_words").value).split(",")
            if item.strip()
        ]
        self._state = "idle"
        self._woken = False
        self._expect_result = False
        self._session_woken = False
        self._capture_started_at = 0.0
        self._last_voice_at = 0.0
        self._tts_playing = False
        self._stop_event = threading.Event()
        self._proc: subprocess.Popen | None = None
        self._reader: threading.Thread | None = None
        self._lock = threading.Lock()
        self._latest_partial = ""

        self.pcm_pub = self.create_publisher(String, str(self.get_parameter("pcm_topic").value), 20)
        self.pcm_end_pub = self.create_publisher(
            String, str(self.get_parameter("pcm_end_topic").value), 10
        )
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.voice_pub = self.create_publisher(
            String, str(self.get_parameter("voice_command_topic").value), 10
        )
        self.reply_pub = self.create_publisher(
            String, str(self.get_parameter("robot_reply_topic").value), 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter("asr_partial_topic").value),
            self._partial_cb,
            20,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("asr_result_topic").value),
            self._result_cb,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("tts_status_topic").value),
            self._tts_status_cb,
            10,
        )

        self._last_chunk_at = time.time()
        if self._mode == "continuous":
            self._start_capture_process()
            # WSLg/PulseAudio 的采集流会偶发停滞（尤其边播 TTS 边采麦时），
            # 看门狗发现长时间无数据就自动重启录音进程。
            self.create_timer(2.0, self._capture_watchdog)
            self.get_logger().info(
                f"Voice listen continuous ready; wake_words={self._wake_words or ['(any speech)']}"
            )
        else:
            self._publish_status("disabled", "listen_mode=ptt")
            self.get_logger().info("Voice listen disabled (ptt mode); use record_voice service")

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._proc and self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        if self._reader and self._reader.is_alive():
            self._reader.join(timeout=2.0)
        return super().destroy_node()

    def _publish_status(self, state: str, detail: str = "") -> None:
        msg = String()
        msg.data = json.dumps({"state": state, "detail": detail}, ensure_ascii=False)
        self.status_pub.publish(msg)

    def _publish_reply(self, text: str) -> None:
        if not text:
            return
        msg = String()
        msg.data = text
        self.reply_pub.publish(msg)

    def _tts_status_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data) if msg.data.strip().startswith("{") else {"state": msg.data}
        except json.JSONDecodeError:
            data = {"state": msg.data.strip()}
        state = str(data.get("state", "")).strip().lower()
        self._tts_playing = state == "playing"

    def _start_capture_process(self) -> None:
        backend, executable = select_microphone_backend(
            str(self.get_parameter("recorder_backend").value)
        )
        command = build_raw_capture_command(
            backend=backend,
            executable=executable,
            device=str(self.get_parameter("device").value),
            sample_rate=int(self.get_parameter("sample_rate").value),
            channels=int(self.get_parameter("channels").value),
        )
        self.get_logger().info(f"Starting continuous mic capture backend={backend}")
        self._proc = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=os.environ.copy(),
        )
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self._publish_status("idle", f"backend={backend}")

    def _capture_watchdog(self) -> None:
        """采集流停滞自愈：超过 capture_stall_sec 没读到数据就重启录音进程。"""
        if self._stop_event.is_set():
            return
        stall_sec = float(self.get_parameter("capture_stall_sec").value)
        if stall_sec <= 0 or time.time() - self._last_chunk_at < stall_sec:
            return
        self.get_logger().warning(
            f"Mic capture stalled >{stall_sec:.0f}s; restarting capture process"
        )
        with self._lock:
            if self._state != "idle":
                self._end_capture_locked("capture_stalled")
        if self._proc and self._proc.poll() is None:
            self._proc.kill()
        if self._reader and self._reader.is_alive():
            self._reader.join(timeout=2.0)
        self._last_chunk_at = time.time()
        self._start_capture_process()

    def _read_loop(self) -> None:
        assert self._proc is not None and self._proc.stdout is not None
        proc = self._proc
        sample_rate = int(self.get_parameter("sample_rate").value)
        channels = int(self.get_parameter("channels").value)
        frame_ms = int(self.get_parameter("frame_ms").value)
        bytes_per_frame = int(sample_rate * (frame_ms / 1000.0) * channels * 2)
        energy_threshold = float(self.get_parameter("energy_threshold").value)
        silence_sec = float(self.get_parameter("silence_sec").value)
        command_window_sec = float(self.get_parameter("command_window_sec").value)
        max_capture_sec = float(self.get_parameter("max_capture_sec").value)
        pause_while_tts = bool(self.get_parameter("pause_while_tts").value)
        calibrate_sec = float(self.get_parameter("auto_calibrate_sec").value)
        calibrate_margin = float(self.get_parameter("calibrate_margin").value)
        calib_until = time.time() + calibrate_sec if calibrate_sec > 0 else 0.0
        calib_samples: list[float] = []
        seq = 0

        while not self._stop_event.is_set():
            chunk = proc.stdout.read(bytes_per_frame)
            if not chunk:
                time.sleep(0.05)
                if proc.poll() is not None:
                    self.get_logger().warning("Microphone capture process exited")
                    self._publish_status("error", "capture_process_exited")
                    break
                continue

            self._last_chunk_at = time.time()
            rms = pcm_rms(chunk)
            now = time.time()

            # 启动头几秒只采底噪样本，用于自动标定能量门限。
            if calib_until:
                if now < calib_until:
                    calib_samples.append(rms)
                    continue
                energy_threshold = calibrated_energy_threshold(
                    calib_samples, energy_threshold, calibrate_margin
                )
                self.get_logger().info(
                    f"Energy threshold calibrated to {energy_threshold:.0f} "
                    f"(ambient median x {calibrate_margin})"
                )
                calib_until = 0.0
            with self._lock:
                if pause_while_tts and self._tts_playing:
                    if self._state != "idle":
                        self._end_capture_locked("paused_for_tts")
                    continue

                if self._state == "idle":
                    if rms >= energy_threshold:
                        self._state = "capturing"
                        self._woken = not bool(self._wake_words)
                        self._capture_started_at = now
                        self._last_voice_at = now
                        self._latest_partial = ""
                        self.get_logger().info(
                            f"Capture start (rms={rms:.0f} > thr={energy_threshold:.0f})"
                        )
                        self._publish_status(
                            "capturing" if self._woken else "wake",
                            f"rms={rms:.0f}",
                        )
                        if self._woken:
                            self._publish_reply("我在，请说指令。")
                    else:
                        continue
                else:
                    if rms >= energy_threshold:
                        self._last_voice_at = now
                    elapsed = now - self._capture_started_at
                    silent_for = now - self._last_voice_at
                    should_end = False
                    reason = ""
                    if self._woken and silent_for >= silence_sec and elapsed > 0.4:
                        should_end = True
                        reason = "silence"
                    elif elapsed >= max_capture_sec:
                        should_end = True
                        reason = "max_capture"
                    elif self._woken and elapsed >= command_window_sec and silent_for >= 0.4:
                        should_end = True
                        reason = "command_window"

                    seq += 1
                    payload = String()
                    payload.data = json.dumps(
                        {
                            "pcm_b64": base64.b64encode(chunk).decode("ascii"),
                            "seq": seq,
                            "sample_rate": sample_rate,
                            "channels": channels,
                            "rms": rms,
                            "phase": "command" if self._woken else "wake",
                        },
                        ensure_ascii=False,
                    )
                    self.pcm_pub.publish(payload)

                    if should_end:
                        self._end_capture_locked(reason)

    def _end_capture_locked(self, reason: str) -> None:
        self.get_logger().info(
            f"Capture end ({reason}); woken={self._woken} partial={self._latest_partial!r}"
        )
        end = String()
        end.data = json.dumps(
            {
                "reason": reason,
                "woken": self._woken,
                "partial": self._latest_partial,
            },
            ensure_ascii=False,
        )
        self.pcm_end_pub.publish(end)
        self._expect_result = True
        self._session_woken = self._woken
        self._state = "idle"
        self._woken = False
        self._publish_status("idle", reason)

    def _partial_cb(self, msg: String) -> None:
        if self._mode != "continuous":
            return
        text = self._extract_text(msg.data)
        if not text:
            return
        with self._lock:
            self._latest_partial = text
            if self._state == "idle":
                return
            if not self._woken and (
                not self._wake_words or text_contains_wake_word(text, self._wake_words)
            ):
                self._woken = True
                self._capture_started_at = time.time()
                self._last_voice_at = time.time()
                self._publish_status("listening", text)
                # 注意：这里不播报「我在」——用户通常一口气说完
                # 「小度小度停止」，此刻插播 TTS 会暂停采麦、掐断后半句。
                # 只有最终结果里确实只有唤醒词时，_result_cb 才会提示。
                self.get_logger().info(f"Wake word heard: {text!r}")

    def _result_cb(self, msg: String) -> None:
        """流式会话结束时的最终结果：已唤醒则发出 voice_command。"""
        if self._mode != "continuous":
            return
        text = self._extract_text(msg.data)
        self.get_logger().info(f"ASR final: {text!r}")
        with self._lock:
            if not self._expect_result and self._state == "idle":
                return
            woken = self._session_woken or self._woken
            self._expect_result = False
            self._session_woken = False
            self._state = "idle"
            self._woken = False
            self._publish_status("idle", "command_ready" if text else "empty_result")

        if not text:
            # 只有确实被唤醒过的会话才值得口头提示；
            # 环境噪音误触发的会话静默丢弃，否则 TTS 会反复插话「没听清」。
            if woken:
                self._publish_reply("没听清，请再说一遍。")
            return
        if not woken and self._wake_words and not text_contains_wake_word(text, self._wake_words):
            return
        command = strip_wake_words(text, self._wake_words) if self._wake_words else text.strip()
        if not command:
            self._publish_reply("我在，请说具体指令。" if woken or text_contains_wake_word(text, self._wake_words) else "没听清指令，请再说一遍。")
            return
        self.get_logger().info(f"Voice command: {command!r}")
        out = String()
        out.data = command
        self.voice_pub.publish(out)

    @staticmethod
    def _extract_text(payload: str) -> str:
        clean = payload.strip()
        if not clean:
            return ""
        if clean.startswith("{"):
            try:
                data = json.loads(clean)
            except json.JSONDecodeError:
                return clean
            return str(data.get("text", "")).strip()
        return clean


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceListenNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
