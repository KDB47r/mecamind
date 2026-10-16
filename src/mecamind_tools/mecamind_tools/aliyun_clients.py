"""阿里云 DashScope 客户端封装（ASR 语音识别 / TTS 语音合成 / LLM 大模型）。

本模块不依赖 ROS，是纯 Python 的"云服务访问层"，被 aliyun_speech_nodes.py
和 task_scheduler.py 等 ROS 节点调用。把云服务访问逻辑与 ROS 节点分开的好处：
一是可以在没有 ROS 环境的机器上单独做单元测试；二是初学者可以先看懂
"怎么调云 API"，再看"怎么接入 ROS topic"，降低学习坡度。

模块内容概览：
- API Key 读取：优先读环境变量 DASHSCOPE_API_KEY，找不到时再去若干候选路径
  找本地 YAML 配置文件（见 _credential_config_candidates / load_api_key_from_config）。
- AliyunLlmClient：通过 HTTP（OpenAI 兼容接口）调用通义千问，把自然语言
  指令解析成结构化的 AliyunTaskPlan（意图 + 目标 + 是否需确认 + 回复语）。
- AliyunSpeechClient：通过 DashScope SDK 调用 ASR（录音文件转文字）和
  TTS（文字合成音频文件）。

初学者重点阅读顺序建议：
1. AliyunTaskPlan / parse_task_plan_payload —— 理解 LLM 输出如何变成结构化任务；
2. AliyunLlmClient.parse_task —— 一个最朴素的 HTTP POST 调用大模型的完整流程；
3. AliyunSpeechClient.recognize_file / synthesize_to_file —— ASR/TTS 的最小用法。
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import time
from typing import Any, Callable, Dict, Iterable
from urllib import error, request

import yaml


# DashScope 的 "OpenAI 兼容模式" HTTP 入口：请求/响应格式与 OpenAI ChatCompletion 一致，
# 这样代码里不需要引入 openai SDK，用标准库 urllib 就能调用。
DEFAULT_DASHSCOPE_BASE_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"
# 本地凭据文件名。放在 .local.yaml 里而不是代码里，避免把 API Key 提交进 git。
LOCAL_CREDENTIALS_FILENAME = "mecamind_aliyun.local.yaml"


@dataclass(frozen=True)
class AliyunTaskPlan:
    """LLM 解析用户指令后得到的结构化任务计划。

    frozen=True 让实例不可变：解析结果一旦生成就不应该被下游偷偷修改，
    出问题时更容易定位。字段含义：
    - intent: 任务意图，只允许固定集合（stop/cancel/patrol/follow/mapping/navigate/unknown）；
    - target: 导航目标（房间名、路点名），非导航任务时为空串；
    - requires_confirmation: 指令含糊或有风险时为 True，提示上层先向用户确认再执行；
    - reply: 机器人回复给用户的话，后续会送去 TTS 播报。
    """

    intent: str
    target: str = ""
    requires_confirmation: bool = False
    reply: str = ""


def _env_value(name: str) -> str:
    """读取环境变量并去掉首尾空白；不存在时返回空串（而不是 None），方便直接做真值判断。"""
    return os.environ.get(name, "").strip()


def _credential_config_candidates() -> list[Path]:
    """按优先级列出凭据 YAML 文件的所有候选路径。

    查找顺序（先找到先用）：
    1. 环境变量 MECAMIND_ALIYUN_CONFIG 显式指定的路径（用户手动指定优先级最高）；
    2. 本包源码目录旁的 config/ 目录（开发时最方便）；
    3. 当前工作目录下的 config/（在工作区根目录启动节点时能命中）；
    4. 用户主目录 ~/.config/mecamind/（跨项目共享的全局配置）。
    """
    candidates = []
    configured_path = _env_value("MECAMIND_ALIYUN_CONFIG")
    if configured_path:
        candidates.append(Path(configured_path).expanduser())

    candidates.extend(
        [
            Path(__file__).resolve().parents[1] / "config" / LOCAL_CREDENTIALS_FILENAME,
            Path.cwd() / "config" / LOCAL_CREDENTIALS_FILENAME,
            Path("~/.config/mecamind").expanduser() / LOCAL_CREDENTIALS_FILENAME,
        ]
    )
    # dict.fromkeys 在去重的同时保持原有顺序（set 会打乱顺序，不能用）。
    return list(dict.fromkeys(path.resolve() for path in candidates))


def load_api_key_from_config(
    env_name: str = "DASHSCOPE_API_KEY",
    config_file: str | Path | None = None,
) -> str:
    """从本地 YAML 配置文件中读取 API Key。

    支持三种写法（按优先级）：顶层键 DASHSCOPE_API_KEY、顶层键 dashscope_api_key、
    或嵌套的 aliyun.api_key。如果找到了配置文件但里面 Key 为空，会直接抛错——
    这比静默返回空串更友好：说明用户"想配但配错了"，应立刻提示而不是让
    后续请求莫名失败。所有候选文件都不存在时返回空串，交由调用方决定怎么处理。
    """
    candidates = (
        [Path(config_file).expanduser().resolve()]
        if config_file
        else _credential_config_candidates()
    )
    for path in candidates:
        if not path.is_file():
            continue
        with path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        if not isinstance(data, dict):
            raise RuntimeError(f"Aliyun credential config must be a mapping: {path}")
        aliyun = data.get("aliyun", {})
        if aliyun and not isinstance(aliyun, dict):
            raise RuntimeError(f"'aliyun' must be a mapping in credential config: {path}")
        # or 链：三种键名依次兜底，任何一个非空即采用。
        value = str(
            data.get(env_name)
            or data.get("dashscope_api_key")
            or aliyun.get("api_key", "")
        ).strip()
        if value:
            return value
        raise RuntimeError(f"Aliyun API key is empty in credential config: {path}")
    return ""


def _require_api_key(env_name: str = "DASHSCOPE_API_KEY") -> str:
    """获取 API Key，拿不到就抛出带解决办法的错误信息。

    先查环境变量；查不到再读本地配置文件，并把读到的值写回环境变量——
    这样同进程内后续调用（包括 DashScope SDK 内部读取环境变量）都不用重复读文件。
    """
    value = _env_value(env_name)
    if not value:
        value = load_api_key_from_config(env_name)
        if value:
            os.environ[env_name] = value
    if not value:
        raise RuntimeError(
            f"Missing {env_name}; set it in the shell or create config/{LOCAL_CREDENTIALS_FILENAME}."
        )
    return value


def _strip_json_fence(text: str) -> str:
    """从 LLM 返回的文本中剥离 Markdown 代码围栏，提取纯 JSON 字符串。

    大模型即使被要求"只输出 JSON"，也经常输出 ```json ... ``` 这样的围栏，
    或者在 JSON 前后夹带说明文字。处理策略分三步兜底：
    1. 优先匹配 ``` 围栏内的内容；
    2. 没有围栏时，截取第一个 '{' 到最后一个 '}' 之间的片段；
    3. 都失败就原样返回，让上层的 json.loads 抛错并给出诊断信息。
    """
    clean = text.strip()
    fenced = re.search(r"```(?:json)?\s*(.*?)\s*```", clean, flags=re.DOTALL | re.IGNORECASE)
    if fenced:
        return fenced.group(1).strip()
    start = clean.find("{")
    end = clean.rfind("}")
    if start >= 0 and end > start:
        return clean[start : end + 1]
    return clean


def parse_task_plan_payload(payload: str, fallback_text: str = "") -> AliyunTaskPlan:
    """把 LLM 输出的 JSON 文本解析成 AliyunTaskPlan，并做安全兜底。

    防御性设计（LLM 输出不可全信，必须逐字段校验）：
    - intent 不在白名单里就强制降级为 unknown，防止 LLM"发明"新意图导致下游崩溃；
    - requires_confirmation 缺省时按 intent 是否为 unknown 推断——识别不出意图
      的指令必须先向用户确认，绝不能直接执行；
    - reply 为空时用 fallback_text（通常是用户原话）兜底，保证 TTS 总有话可播。
    """
    data = json.loads(_strip_json_fence(payload))
    if not isinstance(data, dict):
        raise ValueError("LLM task payload must be a JSON object")
    intent = str(data.get("intent", "unknown")).strip().lower() or "unknown"
    allowed = {"stop", "cancel", "confirm", "patrol", "follow", "mapping", "navigate", "move", "unknown"}
    if intent not in allowed:
        intent = "unknown"
    return AliyunTaskPlan(
        intent=intent,
        target=str(data.get("target", "") or ""),
        requires_confirmation=bool(data.get("requires_confirmation", intent == "unknown")),
        reply=str(data.get("reply", "") or fallback_text),
    )


def build_task_parser_messages(text: str) -> list[dict[str, str]]:
    """构造发给 LLM 的对话消息列表（system 提示词 + 用户指令）。

    system 提示词把 LLM 约束成"任务解析器"：只允许输出固定 schema 的 JSON、
    只允许白名单里的 intent。提示词里直接给出 JSON 示例（few-shot 技巧），
    比只用文字描述 schema 更能让模型稳定输出正确格式。
    """
    system = (
        "You are the task parser for a ROS 2 indoor mobile robot. "
        "Convert the user's command into strict JSON only. "
        "Allowed intents are stop, cancel, confirm, patrol, follow, mapping, navigate, move, unknown. "
        "Use target for room or waypoint names. "
        "For short relative motions (go forward, back up, turn left, turn right) use intent move "
        "with target one of forward, backward, left, right. "
        "Set requires_confirmation true when the command is ambiguous or risky. "
        "The JSON schema is: "
        '{"intent":"navigate","target":"bedroom","requires_confirmation":false,"reply":"OK"}'
    )
    return [
        {"role": "system", "content": system},
        {"role": "user", "content": text},
    ]


class AliyunLlmClient:
    """通义千问 LLM 客户端：把自然语言指令解析为结构化任务。

    只用标准库 urllib 发 HTTP 请求，不额外依赖 openai/requests 等第三方库，
    方便初学者看清"调用大模型 = 一次普通的 HTTPS POST"这个本质。

    构造参数中的 opener 是"依赖注入"技巧：默认用真实的 request.urlopen，
    单元测试时可以传入一个假的 opener，从而在不联网的情况下测试解析逻辑。
    """

    def __init__(
        self,
        model: str = "qwen-plus",
        api_key_env: str = "DASHSCOPE_API_KEY",
        base_url: str = DEFAULT_DASHSCOPE_BASE_URL,
        timeout_sec: float = 15.0,
        opener: Callable[[request.Request, float], Any] | None = None,
    ) -> None:
        self.model = model
        self.api_key_env = api_key_env
        # 去掉末尾的 '/'，避免后面拼接 URL 时出现 "//chat/completions"。
        self.base_url = base_url.rstrip("/")
        self.timeout_sec = timeout_sec
        self._opener = opener or request.urlopen

    def parse_task(self, text: str) -> AliyunTaskPlan:
        """调用 LLM，把一句自然语言指令解析成 AliyunTaskPlan。

        完整流程：取 API Key -> 组装 JSON 请求体 -> POST -> 解析响应。
        temperature=0.0 让模型输出尽量确定（同样的输入尽量给同样的输出），
        任务解析场景需要稳定性而不是创造性；response_format 要求服务端
        强制返回 JSON 对象，进一步降低格式出错概率。
        """
        api_key = _require_api_key(self.api_key_env)
        payload = {
            "model": self.model,
            "messages": build_task_parser_messages(text),
            "temperature": 0.0,
            "response_format": {"type": "json_object"},
        }
        raw = json.dumps(payload).encode("utf-8")
        req = request.Request(
            f"{self.base_url}/chat/completions",
            data=raw,
            headers={
                # Bearer Token 是 HTTP API 最常见的鉴权方式。
                "Authorization": f"Bearer {api_key}",
                "Content-Type": "application/json",
            },
            method="POST",
        )
        try:
            with self._opener(req, timeout=self.timeout_sec) as resp:
                body = resp.read().decode("utf-8")
        except error.HTTPError as exc:
            # HTTPError：服务器有响应但状态码不是 2xx（如 401 鉴权失败、429 限流）。
            # 把响应体也读出来放进错误信息，方便定位具体原因。
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"Aliyun LLM request failed: HTTP {exc.code}: {detail}") from exc
        except error.URLError as exc:
            # URLError：请求根本没到达服务器（断网、DNS 解析失败、超时等）。
            raise RuntimeError(f"Aliyun LLM request failed: {exc.reason}") from exc

        # OpenAI 兼容格式：真正的模型回复藏在 choices[0].message.content 里。
        data = json.loads(body)
        content = data["choices"][0]["message"]["content"]
        return parse_task_plan_payload(content, fallback_text=text)


def _set_dashscope_runtime(api_key_env: str, websocket_url: str = "") -> Any:
    """惰性导入 DashScope SDK 并配置好 API Key，返回配置完成的模块对象。

    放在函数里 import（而不是文件顶部）是有意为之：dashscope 是可选依赖，
    只有真正用到 ASR/TTS 时才导入。这样没装 SDK 的同学仍然可以使用
    本模块的 LLM 客户端和纯解析函数，导入失败时给出明确的安装提示。
    """
    try:
        import dashscope  # type: ignore
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError("DashScope SDK is required for Aliyun ASR/TTS: pip install -U dashscope") from exc
    dashscope.api_key = _require_api_key(api_key_env)
    if websocket_url:
        # ASR/TTS 走 WebSocket 长连接；允许覆盖默认地址以便使用私有化部署或代理。
        dashscope.base_websocket_api_url = websocket_url
    return dashscope


def _audio_format_from_path(path: str | Path, fallback: str = "wav") -> str:
    """根据文件扩展名推断音频格式（如 .wav -> "wav"），没有扩展名时用 fallback。"""
    suffix = Path(path).suffix.lower().lstrip(".")
    return suffix or fallback


def extract_recognition_text(sentences: Any) -> str:
    """从 ASR 返回结果中提取纯文本，兼容多种返回形态。
     把结果打印出来
    DashScope 的 get_sentence() 返回结构不固定：可能是字符串、单个句子 dict
    （含 "text" 键）、嵌套 dict（含 "sentences" 键）、或句子 dict 列表。
    这里用递归逐层拆解，把所有句子文本拼成一个字符串，屏蔽 SDK 返回格式
    的差异，让上层调用方只需要处理"一个字符串"。
    """
    if isinstance(sentences, str):
        return sentences.strip()
    if isinstance(sentences, dict):
        text = sentences.get("text", "")
        if text:
            return str(text).strip()
        # dict 里没有直接的 text，可能嵌套了句子列表，递归处理。
        nested = sentences.get("sentences", [])
        return extract_recognition_text(nested)
    if isinstance(sentences, list):
        parts = [
            str(item.get("text", "")).strip()
            for item in sentences
            if isinstance(item, dict) and item.get("text")
        ]
        return "".join(parts)
    return str(sentences or "").strip()


class AliyunSpeechClient:
    """阿里云语音客户端：ASR（录音文件识别为文字）+ TTS（文字合成为音频文件）。

    默认模型说明：
    - asr_model paraformer-realtime-v2：中英文混合实时识别模型；
    - tts_model cosyvoice-v3-flash + tts_voice longanyang：低延迟中文音色；
    - sample_rate 16000：语音识别的标准采样率，需与录音节点的采样率一致，
      否则识别准确率会明显下降。
    """

    def __init__(
        self,
        api_key_env: str = "DASHSCOPE_API_KEY",
        websocket_url: str = "",
        asr_model: str = "paraformer-realtime-v2",
        tts_model: str = "cosyvoice-v3-flash",
        tts_voice: str = "longanyang",
        sample_rate: int = 16000,
    ) -> None:
        self.api_key_env = api_key_env
        self.websocket_url = websocket_url
        self.asr_model = asr_model
        self.tts_model = tts_model
        self.tts_voice = tts_voice
        self.sample_rate = sample_rate

    def recognize_file(
        self,
        audio_path: str | Path,
        language_hints: Iterable[str] | None = ("zh", "en"),
    ) -> str:
        """把本地音频文件送去阿里云 ASR，返回识别出的文字。

        language_hints 提示模型优先按中文/英文识别，可提高中英混说的准确率。
        识别结果为空时主动抛错——上层 ROS 节点会捕获并打日志，比返回空串
        然后让下游"静默地什么都不做"更容易排查问题。
        """
        _set_dashscope_runtime(self.api_key_env, self.websocket_url)
        # SDK 相关 import 放在方法内：只有确认 dashscope 可用后才导入其子模块。
        from http import HTTPStatus
        from dashscope.audio.asr import Recognition  # type: ignore

        audio_path = str(Path(audio_path).expanduser())
        recognition = Recognition(
            model=self.asr_model,
            format=_audio_format_from_path(audio_path),
            sample_rate=self.sample_rate,
            language_hints=list(language_hints or []),
            # callback=None 表示同步阻塞调用：call() 会等整个文件识别完才返回。
            # 教学场景下同步写法最直观；实时流式识别才需要 callback。
            callback=None,
        )
        result = recognition.call(audio_path)
        if result.status_code != HTTPStatus.OK:
            raise RuntimeError(f"Aliyun ASR failed: {getattr(result, 'message', result)}")
        text = extract_recognition_text(result.get_sentence())
        if not text:
            raise RuntimeError("Aliyun ASR returned no recognized text")
        return text

    def synthesize_to_file(
        self,
        text: str,
        output_dir: str | Path,
        audio_format: str = "mp3",
    ) -> Path:
        """调用阿里云 TTS 把文字合成为语音，写到 output_dir 下并返回文件路径。

        文件名带毫秒时间戳，保证连续多次合成不会互相覆盖；返回路径而不是
        音频字节流，是因为下游（播放器节点）通过 topic 传"文件路径字符串"
        比传大块二进制数据更简单可靠。
        """
        _set_dashscope_runtime(self.api_key_env, self.websocket_url)
        from dashscope.audio.tts_v2 import AudioFormat, SpeechSynthesizer  # type: ignore

        # 注意：audio_format 必须真正传给 SDK，否则 call() 按默认 mp3 输出，
        # 文件扩展名和实际内容不一致会让播放器解码失败。
        # WSL/RDP 音频转发下 wav + paplay 播放最顺滑（无需解码、缓冲友好）。
        format_map = {
            "wav": AudioFormat.WAV_22050HZ_MONO_16BIT,
            "mp3": AudioFormat.MP3_22050HZ_MONO_256KBPS,
        }
        fmt = audio_format.lower()
        if fmt not in format_map:
            raise ValueError("audio_format must be wav or mp3")
        root = Path(output_dir).expanduser()
        root.mkdir(parents=True, exist_ok=True)
        safe_stamp = int(time.time() * 1000)
        path = root / f"mecamind_tts_{safe_stamp}.{fmt}"
        synthesizer = SpeechSynthesizer(
            model=self.tts_model,
            voice=self.tts_voice,
            format=format_map[fmt],
        )
        # call() 同步返回完整音频字节；空结果说明合成失败（如文本不合法、配额用尽）。
        audio = synthesizer.call(text)
        if not audio:
            raise RuntimeError("Aliyun TTS returned empty audio")
        path.write_bytes(audio)
        return path

    def open_streaming_recognition(
        self,
        language_hints: Iterable[str] | None = ("zh", "en"),
        on_partial: Callable[[str], None] | None = None,
        on_final: Callable[[str], None] | None = None,
        on_error: Callable[[str], None] | None = None,
    ) -> "AliyunStreamingRecognition":
        """打开双向流式 ASR 会话（PCM 帧推送，回调返回部分/最终结果）。"""
        return AliyunStreamingRecognition(
            api_key_env=self.api_key_env,
            websocket_url=self.websocket_url,
            asr_model=self.asr_model,
            sample_rate=self.sample_rate,
            language_hints=list(language_hints or []),
            on_partial=on_partial,
            on_final=on_final,
            on_error=on_error,
        )


class AliyunStreamingRecognition:
    """DashScope Recognition 双向流式封装：start → send_audio_frame → stop。"""

    def __init__(
        self,
        api_key_env: str = "DASHSCOPE_API_KEY",
        websocket_url: str = "",
        asr_model: str = "paraformer-realtime-v2",
        sample_rate: int = 16000,
        language_hints: list[str] | None = None,
        on_partial: Callable[[str], None] | None = None,
        on_final: Callable[[str], None] | None = None,
        on_error: Callable[[str], None] | None = None,
    ) -> None:
        self.api_key_env = api_key_env
        self.websocket_url = websocket_url
        self.asr_model = asr_model
        self.sample_rate = sample_rate
        self.language_hints = language_hints or ["zh", "en"]
        self.on_partial = on_partial
        self.on_final = on_final
        self.on_error = on_error
        self._recognition = None
        self._final_text = ""
        self._error = ""
        self._started = False

    @property
    def final_text(self) -> str:
        return self._final_text

    @property
    def error(self) -> str:
        return self._error

    def start(self) -> None:
        """建立 WebSocket 流式识别会话。"""
        import threading

        _set_dashscope_runtime(self.api_key_env, self.websocket_url)
        from dashscope.audio.asr import Recognition, RecognitionCallback  # type: ignore

        owner = self

        class _Callback(RecognitionCallback):  # type: ignore[misc,valid-type]
            def on_open(self) -> None:
                return None

            def on_close(self) -> None:
                return None

            def on_complete(self) -> None:
                return None

            def on_error(self, result: Any) -> None:
                message = str(getattr(result, "message", result) or "streaming ASR error")
                owner._error = message
                if owner.on_error:
                    owner.on_error(message)

            def on_event(self, result: Any) -> None:
                try:
                    sentence = result.get_sentence()
                except Exception:  # noqa: BLE001
                    return
                text = extract_recognition_text(sentence)
                if not text:
                    return
                # request_status / end 标记因 SDK 版本而异，尽量兼容。
                is_end = False
                try:
                    is_end = bool(result.is_sentence_end(sentence))
                except Exception:  # noqa: BLE001
                    if isinstance(sentence, dict):
                        is_end = bool(sentence.get("end_time")) and sentence.get(
                            "sentence_end", True
                        )
                if is_end:
                    owner._final_text = text
                    if owner.on_final:
                        owner.on_final(text)
                else:
                    if owner.on_partial:
                        owner.on_partial(text)

        self._recognition = Recognition(
            model=self.asr_model,
            format="pcm",
            sample_rate=self.sample_rate,
            language_hints=self.language_hints,
            callback=_Callback(),
        )
        self._recognition.start()
        self._started = True
        # 给握手一点时间，避免首帧丢失。
        threading.Event().wait(0.05)

    def send_audio_frame(self, pcm: bytes) -> None:
        if not self._started or self._recognition is None:
            raise RuntimeError("streaming recognition not started")
        if not pcm:
            return
        self._recognition.send_audio_frame(pcm)

    def stop(self) -> str:
        """结束会话并返回最终文本（可能为空）。"""
        if self._recognition is not None and self._started:
            try:
                self._recognition.stop()
            except Exception as exc:  # noqa: BLE001
                self._error = self._error or str(exc)
                if self.on_error:
                    self.on_error(str(exc))
        self._started = False
        return self._final_text
