"""语音链路相关的 ROS 2 节点：麦克风录音、阿里云 ASR、阿里云 TTS。

本文件把 aliyun_clients.py 里的"纯云服务客户端"接入 ROS 话题系统，
组成完整的语音交互管线（"按键说话"模式）：

    [用户调用 /mecamind/record_voice 服务]
        -> MicrophoneRecorderNode 录一段 WAV，路径发布到 /mecamind/audio_file
        -> AliyunAsrFileNode 订阅音频路径，调阿里云 ASR，识别文字发布到
           /mecamind/voice_command（给任务调度器）和 /mecamind/asr_result（调试用）
        -> task_scheduler.py 解析出任务，把回复语发布到 /mecamind/robot_reply
        -> AliyunTtsNode 订阅回复语，合成语音文件，路径发布到 /mecamind/tts_audio_file
        -> （由播放节点或用户播放该音频）

三个节点各司其职、只通过 topic 通信，体现了 ROS "小节点 + 消息解耦" 的设计
哲学：任何一环都可以单独替换（例如把假麦克风换成真麦克风、把阿里云换成本地
模型）而不影响其他环节。

文件前半部分是不依赖 ROS 的纯函数（payload 解析、录音后端选择、录音命令
拼装、执行录音），方便单元测试；后半部分是三个 Node 类和它们的 main 入口。

初学者重点阅读：
1. select_microphone_backend —— 如何在 arecord / ffmpeg 之间自动选择录音工具；
2. MicrophoneRecorderNode._record_cb —— 一个典型的 ROS service 回调写法；
3. AliyunAsrFileNode._audio_cb —— "订阅 -> 调外部服务 -> 发布" 的桥接节点范式。
"""

from __future__ import annotations

import base64
import json
import os
import shutil
import subprocess
import time
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .aliyun_clients import AliyunSpeechClient, AliyunStreamingRecognition


def parse_audio_file_payload(payload: str) -> str:
    """从 topic 消息中解析出音频文件路径，兼容两种消息格式。

    上游可能直接发裸路径字符串（如 "/tmp/a.wav"），也可能发 JSON
    （如 '{"path": "/tmp/a.wav"}'）。这里统一成一个路径字符串返回，
    解析失败时抛 ValueError 由调用方记日志——宽容输入、严格输出。
    """
    clean = payload.strip()
    if not clean:
        raise ValueError("empty audio path payload")
    if clean.startswith("{"):
        # JSON 格式：优先取 "path" 键，其次兼容 "audio_path" 键。
        data: Any = json.loads(clean)
        if not isinstance(data, dict):
            raise ValueError("audio path JSON payload must be an object")
        clean = str(data.get("path", data.get("audio_path", ""))).strip()
    if not clean:
        raise ValueError("audio path payload has no path")
    return clean


def select_microphone_backend(
    requested: str,
    environment: Mapping[str, str] | None = None,
    which: Callable[[str], str | None] = shutil.which,
) -> tuple[str, str]:
    """选择录音后端，返回 (后端名, 可执行文件路径)。

    支持两种录音工具：
    - arecord（ALSA 自带）：直接读声卡，普通 Linux 桌面/开发板最常用；
    - ffmpeg_pulse（ffmpeg 读 PulseAudio）：WSL2 等环境声卡通过 PulseAudio
      转发（PULSE_SERVER 环境变量），只能走这条路。

    requested 为 "auto" 时的自动决策顺序：
    1. 有 PULSE_SERVER 且装了 ffmpeg -> 用 ffmpeg_pulse（典型 WSL2 场景）；
    2. 装了 arecord -> 用 arecord；
    3. 只装了 ffmpeg -> 退而求其次用 ffmpeg_pulse；
    4. 都没有 -> 报错并提示安装。

    参数 environment 和 which 允许在测试中注入假环境变量表和假的
    "查找可执行文件"函数，从而不依赖真实系统状态就能测全部分支。
    """
    normalized = requested.strip().lower()
    if normalized not in {"auto", "arecord", "ffmpeg_pulse"}:
        raise ValueError("recorder_backend must be auto, arecord, or ffmpeg_pulse")

    # 用户显式指定了后端：找不到对应工具就直接报错，不做静默替换。
    if normalized == "arecord":
        executable = which("arecord")
        if not executable:
            raise RuntimeError("arecord not found; install alsa-utils")
        return normalized, executable
    if normalized == "ffmpeg_pulse":
        executable = which("ffmpeg")
        if not executable:
            raise RuntimeError("ffmpeg not found")
        return normalized, executable

    # auto 模式：按上文注释的优先级自动探测。
    env = environment if environment is not None else os.environ
    ffmpeg = which("ffmpeg")
    if env.get("PULSE_SERVER") and ffmpeg:
        return "ffmpeg_pulse", ffmpeg
    arecord = which("arecord")
    if arecord:
        return "arecord", arecord
    if ffmpeg:
        return "ffmpeg_pulse", ffmpeg
    raise RuntimeError("no microphone recorder found; install alsa-utils or ffmpeg")


def build_microphone_record_command(
    backend: str,
    executable: str,
    output_path: str | Path,
    device: str,
    duration_sec: float,
    sample_rate: int,
    channels: int,
) -> list[str]:
    """根据后端类型拼装录音命令行（返回参数列表，供 subprocess 执行）。

    两个后端的目标一致：录制指定时长、指定采样率的 16 位小端 PCM WAV
    （S16_LE / pcm_s16le），这是阿里云 ASR 要求的格式。
    先做参数合法性检查，把"时长为负"这类错误挡在拼命令之前，
    避免生成一条看起来能跑、实际行为诡异的命令。
    """
    if duration_sec <= 0.0:
        raise ValueError("duration_sec must be positive")
    if sample_rate <= 0:
        raise ValueError("sample_rate must be positive")
    if channels <= 0:
        raise ValueError("channels must be positive")

    path = str(Path(output_path).expanduser())
    if backend == "arecord":
        command = [
            executable,
            "-q",            # 安静模式，不往终端刷进度
            "-t",
            "wav",           # 输出容器格式
            "-f",
            "S16_LE",        # 采样格式：16 位有符号小端
            "-r",
            str(sample_rate),
            "-c",
            str(channels),
            "-d",
            # arecord 的 -d 只接受整数秒，四舍五入且至少录 1 秒。
            str(max(1, int(round(duration_sec)))),
        ]
        if device:
            command.extend(["-D", device])
        command.append(path)
        return command
    if backend == "ffmpeg_pulse":
        return [
            executable,
            "-nostdin",      # 不读标准输入，防止 ffmpeg 在后台等待按键卡死
            "-loglevel",
            "error",         # 只输出错误日志，保持终端干净
            "-y",            # 输出文件已存在时直接覆盖
            "-f",
            "pulse",         # 输入设备类型：PulseAudio
            "-i",
            device or "default",
            "-t",
            # ffmpeg 支持小数秒，保留 3 位（毫秒精度）。
            f"{duration_sec:.3f}",
            "-ac",
            str(channels),
            "-ar",
            str(sample_rate),
            "-c:a",
            "pcm_s16le",     # 与 arecord 的 S16_LE 等价的编码
            path,
        ]
    raise ValueError(f"unsupported recorder backend: {backend}")


def record_microphone_clip(
    command: Sequence[str],
    output_path: str | Path,
    timeout_sec: float,
    runner: Callable[..., subprocess.CompletedProcess] = subprocess.run,
) -> Path:
    """执行录音命令并校验产物，返回录好的 WAV 文件路径。

    runner 参数同样是依赖注入：测试时传假的 subprocess.run 即可不真正录音。
    check=True 让子进程返回非零码时直接抛 CalledProcessError；
    timeout 防止录音工具挂死拖住整个服务回调。

    录完后检查文件大小 > 44 字节：44 字节恰好是标准 WAV 文件头的长度，
    只有文件头没有音频数据说明麦克风没采到声音（设备错误但进程正常退出
    的情况并不少见），必须显式报错。
    """
    path = Path(output_path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    runner(
        list(command),
        check=True,
        timeout=timeout_sec,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )
    if not path.is_file() or path.stat().st_size <= 44:
        raise RuntimeError(f"microphone recorder produced no usable WAV data: {path}")
    return path


class MicrophoneRecorderNode(Node):
    """Record a push-to-talk WAV clip and feed the existing file ASR topic.

    "按键说话"录音节点：对外提供一个 Trigger 服务 /mecamind/record_voice，
    每调用一次就录一段固定时长的音频，把文件路径发布到 /mecamind/audio_file
    供 ASR 节点消费，同时在 /mecamind/microphone_status 上发布 JSON 状态
    （recording / recorded / error），方便上层 UI 或调试工具展示进度。

    选择"服务触发 + 固定时长"而不是持续监听，是教学上的简化：
    不需要 VAD（语音活动检测），流程完全确定，学生容易理解和复现。
    """

    def __init__(self) -> None:
        super().__init__("mecamind_microphone_recorder")
        # 所有 topic 名、录音参数都做成 ROS parameter，
        # 这样上课演示时不改代码、只改 launch/命令行参数就能调整行为。
        self.declare_parameter("audio_file_topic", "/mecamind/audio_file")
        self.declare_parameter("status_topic", "/mecamind/microphone_status")
        self.declare_parameter("record_service", "/mecamind/record_voice")
        self.declare_parameter("recorder_backend", "auto")
        self.declare_parameter("device", "default")
        self.declare_parameter("duration_sec", 4.0)
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("channels", 1)
        self.declare_parameter("output_dir", str(Path("~/.ros/mecamind_mic").expanduser()))

        self.audio_pub = self.create_publisher(
            String, str(self.get_parameter("audio_file_topic").value), 10
        )
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.create_service(
            Trigger,
            str(self.get_parameter("record_service").value),
            self._record_cb,
        )
        # 简单的"忙碌标志"，防止上一段还没录完又触发一次录音。
        self._recording = False
        self.get_logger().info(
            "MecaMind microphone ready; call the record_voice service and speak after recording starts"
        )

    def _publish_status(self, state: str, detail: str = "", path: str = "") -> None:
        """把当前录音状态打包成 JSON 发布，供 UI/调试工具订阅展示。"""
        message = String()
        message.data = json.dumps(
            {"state": state, "detail": detail, "path": path},
            ensure_ascii=False,
        )
        self.status_pub.publish(message)

    def _record_cb(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """record_voice 服务回调：录一段音频并把路径发到 audio_file topic。

        注意：录音是同步阻塞的（会占住回调线程几秒钟），教学场景可以接受；
        生产环境应改成异步/独立线程以免阻塞节点里其他回调。
        Trigger 服务的约定：success 表示是否成功，message 携带
        录音文件路径（成功时）或错误原因（失败时）。
        """
        # 步骤 0：拒绝并发录音。ROS 默认单线程执行器下其实不会并发进入
        # 这个回调，但显式加锁标志让意图更清楚，也兼容多线程执行器。
        if self._recording:
            response.success = False
            response.message = "microphone recording already in progress"
            return response

        self._recording = True
        # 步骤 1：用毫秒时间戳生成唯一输出文件名，避免覆盖历史录音。
        output_path = (
            Path(str(self.get_parameter("output_dir").value)).expanduser()
            / f"mecamind_mic_{int(time.time() * 1000)}.wav"
        )
        try:
            # 步骤 2：选择录音后端（arecord 或 ffmpeg_pulse）。
            backend, executable = select_microphone_backend(
                str(self.get_parameter("recorder_backend").value)
            )
            duration_sec = float(self.get_parameter("duration_sec").value)
            # 步骤 3：拼装录音命令行。
            command = build_microphone_record_command(
                backend=backend,
                executable=executable,
                output_path=output_path,
                device=str(self.get_parameter("device").value),
                duration_sec=duration_sec,
                sample_rate=int(self.get_parameter("sample_rate").value),
                channels=int(self.get_parameter("channels").value),
            )
            # 步骤 4：先广播 "recording" 状态再开录——用户听到/看到提示后开始说话。
            self._publish_status("recording", f"backend={backend}", str(output_path))
            # 超时取"录音时长 + 5 秒余量"，且不低于 5 秒，防止工具启动慢被误杀。
            recorded_path = record_microphone_clip(
                command,
                output_path,
                timeout_sec=max(5.0, duration_sec + 5.0),
            )
            # 步骤 5：把录好的文件路径发给下游 ASR 节点。
            audio = String()
            audio.data = str(recorded_path)
            self.audio_pub.publish(audio)
            self._publish_status("recorded", f"backend={backend}", str(recorded_path))
            response.success = True
            response.message = str(recorded_path)
        except Exception as exc:  # noqa: BLE001
            # 录音失败不能让节点崩溃：记日志、广播 error 状态、在服务响应里返回原因。
            detail = str(exc)
            self.get_logger().error(f"Microphone recording failed: {detail}")
            self._publish_status("error", detail, str(output_path))
            response.success = False
            response.message = detail
        finally:
            # 无论成败都要复位忙碌标志，否则一次失败会永久锁死录音功能。
            self._recording = False
        return response


def classify_voice_failure(exc: BaseException) -> tuple[str, str]:
    """把 ASR/TTS/网络异常映射成 (error_code, 用户可读中文提示)。"""
    detail = str(exc).lower()
    if "no recognized text" in detail or "empty" in detail:
        return "asr_empty", "没听清，请再说一遍。"
    if "api key" in detail or "401" in detail or "unauthorized" in detail or "invalid" in detail:
        return "auth", "语音服务鉴权失败，请检查本地密钥配置。"
    if "429" in detail or "quota" in detail or "throttl" in detail:
        return "quota", "语音服务暂时繁忙，请稍后再试。"
    if "timed out" in detail or "timeout" in detail or "unreachable" in detail or "network" in detail:
        return "network", "语音服务暂时不可用，请稍后再试。"
    return "asr_error", "语音识别失败，请再说一遍。"


class AliyunAsrFileNode(Node):
    """ASR 桥接节点：订阅音频文件路径，调阿里云识别，发布识别文字。

    典型的"胶水节点"：本身没有业务逻辑，只负责把 topic 消息翻译成
    对 AliyunSpeechClient 的调用，再把结果发回 topic。识别文字同时
    发到两个 topic：
    - voice_command_topic（裸文本）：给任务调度器直接消费；
    - asr_result_topic（JSON，带 source 字段）：给调试/记录工具，
      保留元信息以便日后区分识别来源。
    """

    def __init__(self) -> None:
        super().__init__("mecamind_aliyun_asr_file")
        self.declare_parameter("audio_file_topic", "/mecamind/audio_file")
        self.declare_parameter("voice_command_topic", "/mecamind/voice_command")
        self.declare_parameter("asr_result_topic", "/mecamind/asr_result")
        self.declare_parameter("voice_error_topic", "/mecamind/voice_error")
        self.declare_parameter("robot_reply_topic", "/mecamind/robot_reply")
        self.declare_parameter("api_key_env", "DASHSCOPE_API_KEY")
        self.declare_parameter("websocket_url", "")
        self.declare_parameter("asr_model", "paraformer-realtime-v2")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("max_retries", 3)
        self.declare_parameter("publish_voice_command", True)

        # 客户端在构造时创建一次、之后复用；API Key 的检查推迟到第一次
        # 真正识别时才发生（见 aliyun_clients），所以没配 Key 也能启动节点。
        self.client = AliyunSpeechClient(
            api_key_env=str(self.get_parameter("api_key_env").value),
            websocket_url=str(self.get_parameter("websocket_url").value),
            asr_model=str(self.get_parameter("asr_model").value),
            sample_rate=int(self.get_parameter("sample_rate").value),
        )
        self._fail_count = 0
        self.create_subscription(
            String,
            str(self.get_parameter("audio_file_topic").value),
            self._audio_cb,
            10,
        )
        self.voice_pub = self.create_publisher(
            String,
            str(self.get_parameter("voice_command_topic").value),
            10,
        )
        self.result_pub = self.create_publisher(
            String,
            str(self.get_parameter("asr_result_topic").value),
            10,
        )
        self.error_pub = self.create_publisher(
            String,
            str(self.get_parameter("voice_error_topic").value),
            10,
        )
        self.reply_pub = self.create_publisher(
            String,
            str(self.get_parameter("robot_reply_topic").value),
            10,
        )
        self.get_logger().info("MecaMind Aliyun ASR file bridge ready")

    def _publish_failure(self, code: str, reply: str, detail: str) -> None:
        self._fail_count += 1
        max_retries = int(self.get_parameter("max_retries").value)
        if self._fail_count >= max_retries:
            reply = "请靠近麦克风，或改用文字指令。"
            code = "asr_give_up"
            self._fail_count = 0
        err = String()
        err.data = json.dumps(
            {"code": code, "detail": detail, "reply": reply},
            ensure_ascii=False,
        )
        self.error_pub.publish(err)
        out = String()
        out.data = reply
        self.reply_pub.publish(out)

    def _audio_cb(self, msg: String) -> None:
        """收到音频路径 -> 调云端 ASR -> 发布识别文字。

        注意这里是同步网络调用，识别期间会阻塞本节点的回调队列。
        失败时发布 voice_error + robot_reply，引导用户重说，节点本身不退出。
        """
        try:
            audio_path = parse_audio_file_payload(msg.data)
            text = self.client.recognize_file(audio_path)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Aliyun ASR failed: {exc}")
            code, reply = classify_voice_failure(exc)
            self._publish_failure(code, reply, str(exc))
            return

        self._fail_count = 0
        # 先发带元信息的 JSON 结果（调试用），再发裸文本指令（业务用）。
        result = String()
        result.data = json.dumps({"text": text, "source": "aliyun_asr_file"}, ensure_ascii=False)
        self.result_pub.publish(result)

        if bool(self.get_parameter("publish_voice_command").value):
            command = String()
            command.data = text
            self.voice_pub.publish(command)


class AliyunAsrStreamNode(Node):
    """流式 ASR：订阅 PCM 帧，用 DashScope Recognition 双向流识别。"""

    def __init__(self) -> None:
        super().__init__("mecamind_aliyun_asr_stream")
        self.declare_parameter("pcm_topic", "/mecamind/audio_pcm")
        self.declare_parameter("pcm_end_topic", "/mecamind/audio_pcm_end")
        self.declare_parameter("voice_command_topic", "/mecamind/voice_command")
        self.declare_parameter("asr_partial_topic", "/mecamind/asr_partial")
        self.declare_parameter("asr_result_topic", "/mecamind/asr_result")
        self.declare_parameter("voice_error_topic", "/mecamind/voice_error")
        self.declare_parameter("robot_reply_topic", "/mecamind/robot_reply")
        self.declare_parameter("api_key_env", "DASHSCOPE_API_KEY")
        self.declare_parameter("websocket_url", "")
        self.declare_parameter("asr_model", "paraformer-realtime-v2")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("max_retries", 3)
        # continuous 模式下由 voice_listen 过滤唤醒后再发 voice_command
        self.declare_parameter("publish_voice_command", False)

        self.client = AliyunSpeechClient(
            api_key_env=str(self.get_parameter("api_key_env").value),
            websocket_url=str(self.get_parameter("websocket_url").value),
            asr_model=str(self.get_parameter("asr_model").value),
            sample_rate=int(self.get_parameter("sample_rate").value),
        )
        self._session: AliyunStreamingRecognition | None = None
        self._fail_count = 0
        self._last_text = ""

        self.partial_pub = self.create_publisher(
            String, str(self.get_parameter("asr_partial_topic").value), 20
        )
        self.result_pub = self.create_publisher(
            String, str(self.get_parameter("asr_result_topic").value), 10
        )
        self.voice_pub = self.create_publisher(
            String, str(self.get_parameter("voice_command_topic").value), 10
        )
        self.error_pub = self.create_publisher(
            String, str(self.get_parameter("voice_error_topic").value), 10
        )
        self.reply_pub = self.create_publisher(
            String, str(self.get_parameter("robot_reply_topic").value), 10
        )
        self.create_subscription(
            String, str(self.get_parameter("pcm_topic").value), self._pcm_cb, 50
        )
        self.create_subscription(
            String, str(self.get_parameter("pcm_end_topic").value), self._pcm_end_cb, 10
        )
        self.get_logger().info("MecaMind Aliyun ASR stream bridge ready")

    def _ensure_session(self) -> AliyunStreamingRecognition:
        if self._session is not None:
            return self._session

        def on_partial(text: str) -> None:
            self._last_text = text
            msg = String()
            msg.data = json.dumps({"text": text, "final": False}, ensure_ascii=False)
            self.partial_pub.publish(msg)

        def on_final(text: str) -> None:
            self._last_text = text

        def on_error(message: str) -> None:
            self.get_logger().error(f"Streaming ASR error: {message}")

        self._session = self.client.open_streaming_recognition(
            on_partial=on_partial,
            on_final=on_final,
            on_error=on_error,
        )
        self._session.start()
        return self._session

    def _pcm_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            pcm = base64.b64decode(str(data.get("pcm_b64", "")))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Invalid PCM payload: {exc}")
            return
        try:
            session = self._ensure_session()
            session.send_audio_frame(pcm)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Streaming ASR send failed: {exc}")
            code, reply = classify_voice_failure(exc)
            self._publish_failure(code, reply, str(exc))
            self._reset_session()

    def _pcm_end_cb(self, msg: String) -> None:
        # 听节点在 pcm_end 里带了 woken 标志：未唤醒的会话大多是环境
        # 噪音误触发，识别为空时静默丢弃，不播报失败提示（否则 TTS
        # 会不停插话「没听清」）。
        try:
            end_info = json.loads(msg.data) if msg.data.strip() else {}
        except json.JSONDecodeError:
            end_info = {}
        woken = bool(end_info.get("woken", True))
        session = self._session
        self._session = None
        text = ""
        error = ""
        if session is not None:
            try:
                text = session.stop() or session.final_text or self._last_text
                error = session.error
            except Exception as exc:  # noqa: BLE001
                error = str(exc)
        self._last_text = ""
        if not text:
            if not woken:
                self.get_logger().info("Unwoken session ended with no text; ignored")
                return
            if error:
                code, reply = classify_voice_failure(RuntimeError(error))
                self._publish_failure(code, reply, error)
            else:
                code, reply = classify_voice_failure(
                    RuntimeError("Aliyun ASR returned no recognized text")
                )
                self._publish_failure(code, reply, "empty_stream_result")
            return
        self.get_logger().info(f"ASR stream final: {text!r}")
        self._fail_count = 0
        result = String()
        result.data = json.dumps({"text": text, "source": "aliyun_asr_stream"}, ensure_ascii=False)
        self.result_pub.publish(result)
        if bool(self.get_parameter("publish_voice_command").value):
            command = String()
            command.data = text
            self.voice_pub.publish(command)

    def _reset_session(self) -> None:
        session = self._session
        self._session = None
        self._last_text = ""
        if session is not None:
            try:
                session.stop()
            except Exception:  # noqa: BLE001
                pass

    def _publish_failure(self, code: str, reply: str, detail: str) -> None:
        self._fail_count += 1
        max_retries = int(self.get_parameter("max_retries").value)
        if self._fail_count >= max_retries:
            reply = "请靠近麦克风，或改用文字指令。"
            code = "asr_give_up"
            self._fail_count = 0
        err = String()
        err.data = json.dumps(
            {"code": code, "detail": detail, "reply": reply},
            ensure_ascii=False,
        )
        self.error_pub.publish(err)
        out = String()
        out.data = reply
        self.reply_pub.publish(out)


class AliyunTtsNode(Node):
    """TTS 桥接节点：订阅机器人回复文本，调阿里云合成语音，发布音频文件路径。

    与 ASR 节点结构对称：订阅 /mecamind/robot_reply 上的文本，
    合成后把音频文件路径发布到 /mecamind/tts_audio_file，由播放节点
    （或用户手动）播放。发布"路径"而不是音频数据本身，避免在 topic 上
    传输大块二进制。
    """

    def __init__(self) -> None:
        super().__init__("mecamind_aliyun_tts")
        self.declare_parameter("input_topic", "/mecamind/robot_reply")
        self.declare_parameter("output_topic", "/mecamind/tts_audio_file")
        self.declare_parameter("voice_error_topic", "/mecamind/voice_error")
        self.declare_parameter("api_key_env", "DASHSCOPE_API_KEY")
        self.declare_parameter("websocket_url", "")
        self.declare_parameter("tts_model", "cosyvoice-v3-flash")
        self.declare_parameter("tts_voice", "longanyang")
        self.declare_parameter("output_dir", str(Path("~/.ros/mecamind_tts").expanduser()))
        # wav 在 WSL/RDP 音频链路下播放更顺滑（paplay 原生支持、无需解码）。
        self.declare_parameter("audio_format", "wav")

        self.client = AliyunSpeechClient(
            api_key_env=str(self.get_parameter("api_key_env").value),
            websocket_url=str(self.get_parameter("websocket_url").value),
            tts_model=str(self.get_parameter("tts_model").value),
            tts_voice=str(self.get_parameter("tts_voice").value),
        )
        self.create_subscription(
            String,
            str(self.get_parameter("input_topic").value),
            self._text_cb,
            10,
        )
        self.pub = self.create_publisher(String, str(self.get_parameter("output_topic").value), 10)
        self.error_pub = self.create_publisher(
            String, str(self.get_parameter("voice_error_topic").value), 10
        )
        self.get_logger().info("MecaMind Aliyun TTS bridge ready")

    def _text_cb(self, msg: String) -> None:
        """收到回复文本 -> 调云端 TTS -> 发布合成音频的文件路径。

        空文本直接忽略（没必要为空串浪费一次云调用）；合成失败只记日志
        不抛异常，保证节点常驻。
        """
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info(f"TTS synth: {text!r}")
        try:
            path = self.client.synthesize_to_file(
                text,
                str(self.get_parameter("output_dir").value),
                audio_format=str(self.get_parameter("audio_format").value),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Aliyun TTS failed: {exc}")
            err = String()
            err.data = json.dumps(
                {
                    "code": "tts_error",
                    "detail": str(exc),
                    "reply": "语音播报失败。",
                },
                ensure_ascii=False,
            )
            self.error_pub.publish(err)
            return
        output = String()
        output.data = str(path)
        self.pub.publish(output)


def select_playback_backend(
    requested: str = "auto",
    which: Callable[[str], str | None] = shutil.which,
) -> tuple[str, str]:
    """选择 TTS 播放后端：paplay / ffplay / aplay。

    auto 优先 paplay：直接走 PulseAudio、无需解码，WSLg 下播放 wav
    最顺滑；ffplay 兜底可解码 mp3；aplay 是纯 ALSA 环境的最后手段。
    """
    normalized = requested.strip().lower()
    candidates = [
        ("paplay", "paplay"),
        ("ffplay", "ffplay"),
        ("aplay", "aplay"),
    ]
    if normalized != "auto":
        for name, binary in candidates:
            if name == normalized:
                path = which(binary)
                if not path:
                    raise RuntimeError(f"{binary} not found")
                return name, path
        raise ValueError("playback_backend must be auto, ffplay, paplay, or aplay")
    for name, binary in candidates:
        path = which(binary)
        if path:
            return name, path
    raise RuntimeError("no audio player found; install ffmpeg, pulseaudio-utils, or alsa-utils")


def build_playback_command(backend: str, executable: str, audio_path: str) -> list[str]:
    """拼装阻塞式播放命令。"""
    path = str(Path(audio_path).expanduser())
    if backend == "ffplay":
        return [executable, "-nodisp", "-autoexit", "-loglevel", "error", path]
    if backend == "paplay":
        return [executable, path]
    if backend == "aplay":
        return [executable, path]
    raise ValueError(f"unsupported playback backend: {backend}")


class TtsPlaybackNode(Node):
    """订阅 TTS 音频路径并自动播放，发布 playing/idle/failed 状态。"""

    def __init__(self) -> None:
        super().__init__("mecamind_tts_playback")
        self.declare_parameter("input_topic", "/mecamind/tts_audio_file")
        self.declare_parameter("status_topic", "/mecamind/tts_status")
        self.declare_parameter("playback_backend", "auto")
        self._busy = False
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter("input_topic").value),
            self._play_cb,
            10,
        )
        self._publish_status("idle")
        self.get_logger().info("MecaMind TTS playback ready")

    def _publish_status(self, state: str, detail: str = "", path: str = "") -> None:
        msg = String()
        msg.data = json.dumps(
            {"state": state, "detail": detail, "path": path},
            ensure_ascii=False,
        )
        self.status_pub.publish(msg)

    def _play_cb(self, msg: String) -> None:
        path = parse_audio_file_payload(msg.data) if msg.data.strip() else ""
        if not path:
            return
        if self._busy:
            self.get_logger().warn("TTS playback busy; dropping new audio")
            return
        audio = Path(path).expanduser()
        if not audio.is_file():
            self._publish_status("failed", "file_missing", str(audio))
            return
        self._busy = True
        self._publish_status("playing", path=str(audio))
        self.get_logger().info(f"Playing TTS audio: {audio.name}")
        try:
            backend, executable = select_playback_backend(
                str(self.get_parameter("playback_backend").value)
            )
            command = build_playback_command(backend, executable, str(audio))
            subprocess.run(
                command,
                check=True,
                timeout=60.0,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
            )
            self._publish_status("idle", f"backend={backend}", str(audio))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"TTS playback failed: {exc}")
            self._publish_status("failed", str(exc), str(audio))
        finally:
            self._busy = False


def asr_main(args=None) -> None:
    """ASR 节点入口（对应 setup.py 中的 console_scripts）。

    标准的 rclpy 节点生命周期：init -> 建节点 -> spin 阻塞处理回调 ->
    finally 里销毁节点并 shutdown，保证 Ctrl+C 退出时资源被正确释放。
    """
    rclpy.init(args=args)
    node = AliyunAsrFileNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def asr_stream_main(args=None) -> None:
    """流式 ASR 节点入口。"""
    rclpy.init(args=args)
    node = AliyunAsrStreamNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def microphone_main(args=None) -> None:
    """麦克风录音节点入口。"""
    rclpy.init(args=args)
    node = MicrophoneRecorderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def tts_main(args=None) -> None:
    """TTS 节点入口。"""
    rclpy.init(args=args)
    node = AliyunTtsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def tts_playback_main(args=None) -> None:
    """TTS 播放节点入口。"""
    rclpy.init(args=args)
    node = TtsPlaybackNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    asr_main()
