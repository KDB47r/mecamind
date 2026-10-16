#!/usr/bin/env bash
# 产品核心语音验收（文本路径 + 密钥探测；不强制真麦克风）。
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# ROS setup 脚本在 set -u 下会触发未绑定变量；先 source 再开 -u。
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [[ -f "$ROOT/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$ROOT/install/setup.bash"
fi
set -u

echo "[voice-core] checking API key discovery (value not printed)"
python3 - <<'PY'
from mecamind_tools.aliyun_clients import load_api_key_from_config
import os

key = os.environ.get("DASHSCOPE_API_KEY", "").strip() or load_api_key_from_config()
if not key:
    raise SystemExit("MECAMIND_VOICE_CORE_FAIL: missing DASHSCOPE_API_KEY / mecamind_aliyun.local.yaml")
print(f"key_ok len={len(key)} prefix={key[:4]}...suffix={key[-4:]}")
os.environ["DASHSCOPE_API_KEY"] = key
PY

echo "[voice-core] unit checks for confirm/cancel parsing"
python3 - <<'PY'
from mecamind_tools.task_scheduler import parse_task_command

assert parse_task_command("确认").intent == "confirm"
assert parse_task_command("取消").intent == "cancel"
unknown = parse_task_command("随便做点什么")
assert unknown.requires_confirmation and "确认" in unknown.reply
print("parse_ok")
PY

echo "[voice-core] launching voice stack (rule provider, text path)"
bash "$ROOT/scripts/cleanup_ros2.sh" >/dev/null 2>&1 || true

set +u
ros2 launch mecamind_tools mecamind_aliyun_voice.launch.py \
  listen_mode:=ptt \
  enable_microphone:=false \
  llm_provider:=rule \
  enable_tts:=false \
  enable_tts_playback:=false &
LAUNCH_PID=$!
set -u
trap 'kill "$LAUNCH_PID" >/dev/null 2>&1 || true; bash "$ROOT/scripts/cleanup_ros2.sh" >/dev/null 2>&1 || true' EXIT

sleep 3

python3 - <<'PY'
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("mecamind_voice_core_probe")
        self.tasks: list[str] = []
        self.replies: list[str] = []
        self.create_subscription(String, "/mecamind/task_command", self._on_task, 10)
        self.create_subscription(String, "/mecamind/robot_reply", self._on_reply, 10)
        self.pub = self.create_publisher(String, "/mecamind/voice_command", 10)

    def _on_task(self, msg: String) -> None:
        self.tasks.append(msg.data)

    def _on_reply(self, msg: String) -> None:
        self.replies.append(msg.data)

    def publish(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.pub.publish(msg)


def wait_until(predicate, timeout_sec: float = 8.0) -> bool:
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if predicate():
            return True
    return False


rclpy.init()
node = Probe()
# 等 discovery
for _ in range(10):
    rclpy.spin_once(node, timeout_sec=0.2)

node.publish("随便做点什么")
ok = wait_until(lambda: bool(node.tasks) and bool(node.replies))
if not ok:
    raise SystemExit("MECAMIND_VOICE_CORE_FAIL: timeout waiting for task/reply")

task = json.loads(node.tasks[-1])
if not task.get("requires_confirmation"):
    raise SystemExit(f"MECAMIND_VOICE_CORE_FAIL: expected confirmation task, got {task}")
if "确认" not in node.replies[-1]:
    raise SystemExit(f"MECAMIND_VOICE_CORE_FAIL: reply missing 确认: {node.replies[-1]}")

before = len(node.tasks)
node.publish("确认")
ok = wait_until(lambda: len(node.tasks) > before)
if not ok:
    raise SystemExit("MECAMIND_VOICE_CORE_FAIL: timeout waiting for confirm task")
confirm = json.loads(node.tasks[-1])
if confirm.get("intent") != "confirm":
    raise SystemExit(f"MECAMIND_VOICE_CORE_FAIL: expected confirm intent, got {confirm}")

print("dialog_ok")
node.destroy_node()
rclpy.shutdown()
PY

echo "MECAMIND_VOICE_CORE_OK"
