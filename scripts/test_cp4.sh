#!/usr/bin/env bash
# CP4 验收（第四课）：无界面启动目标检测链路 → 断言三层输出全部在流。
#
# 验收点（对应 PROJECT_PLAN 10.3.4）：
#   1. /mecamind/detections 持续发布结构化 Detection2DArray；
#   2. 每条检测含 label / confidence / 归一化 bbox 且数值合法；
#   3. JSON 适配器输出的 /mecamind/raw_detections 与旧契约兼容
#      （下游 perception_filter 能消费并产出 confirmed 事件）；
#   4. 带框调试图 /mecamind/detection_image 在发布。
#
# 默认零依赖路径：source=synthetic + backend=auto（无 ML 包时落到 hsv）。
# 想验证真实模型（YOLO/ONNX 认不出合成画面的红色矩形，要配真实视频源）：
#   MECAMIND_CP4_SOURCE=video MECAMIND_CP4_VIDEO=assets/yolo_demo.mp4 \
#     MECAMIND_CP4_BACKEND=ultralytics ./scripts/test_cp4.sh
#   MECAMIND_CP4_SOURCE=video MECAMIND_CP4_VIDEO=assets/yolo_demo.mp4 \
#     MECAMIND_CP4_BACKEND=onnx MECAMIND_CP4_MODEL=models/yolov8n.onnx ./scripts/test_cp4.sh
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SOURCE="${MECAMIND_CP4_SOURCE:-synthetic}"
BACKEND="${MECAMIND_CP4_BACKEND:-auto}"
MODEL="${MECAMIND_CP4_MODEL:-yolov8n.pt}"
VIDEO="${MECAMIND_CP4_VIDEO:-}"
LOG_FILE="${MECAMIND_CP4_LOG:-/tmp/mecamind_cp4.log}"
REPORT_PATH="maps/mecamind_cp4_report.json"
ROS_DOMAIN_ID="${MECAMIND_ROS_DOMAIN_ID:-84}"
export ROS_DOMAIN_ID
export ROS2CLI_NO_DAEMON=1

LOCK_DIR="/tmp/mecamind_cp4.lock.d"
if ! mkdir "$LOCK_DIR" 2>/dev/null; then
  lock_owner=""
  [[ -f "$LOCK_DIR/owner" ]] && lock_owner="$(<"$LOCK_DIR/owner")"
  if [[ -n "$lock_owner" ]] && kill -0 "$lock_owner" 2>/dev/null; then
    echo "已有一套 CP4 验收正在运行，请等待其结束。" >&2
    exit 1
  fi
  rm -f "$LOCK_DIR/owner"; rmdir "$LOCK_DIR" 2>/dev/null || true; mkdir "$LOCK_DIR"
fi
echo "$$" >"$LOCK_DIR/owner"

set +u
source /opt/ros/jazzy/setup.bash
source "$PROJECT_ROOT/install/setup.bash"
set -u

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    for _ in $(seq 1 40); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 0.25; done
    pkill -P "$LAUNCH_PID" 2>/dev/null || true
    kill -TERM "$LAUNCH_PID" 2>/dev/null || true
  fi
  rm -f "$LOCK_DIR/owner"; rmdir "$LOCK_DIR" 2>/dev/null || true
}
trap cleanup EXIT

cd "$PROJECT_ROOT"
mkdir -p maps
rm -f "$REPORT_PATH"

echo "[CP4] source=$SOURCE backend=$BACKEND model=$MODEL video=${VIDEO:-无}，日志：$LOG_FILE"
LAUNCH_ARGS=(source:="$SOURCE" backend:="$BACKEND" model_path:="$MODEL" with_ui:=false)
[[ -n "$VIDEO" ]] && LAUNCH_ARGS+=(video_path:="$VIDEO")
ros2 launch mecamind_bringup perception.launch.py "${LAUNCH_ARGS[@]}" \
  >"$LOG_FILE" 2>&1 &
LAUNCH_PID=$!

sleep 6
if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
  echo "[失败] 感知链路启动即退出，日志：$LOG_FILE" >&2
  exit 1
fi

# 用 rclpy 订阅三层输出并做结构断言，结果写 JSON 报告。
timeout 60 python3 - "$REPORT_PATH" <<'PY'
import json
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from mecamind_interfaces.msg import Detection2DArray


class Probe(Node):
    def __init__(self):
        super().__init__("cp4_probe")
        self.det_msgs = []
        self.raw_msgs = []
        self.event_msgs = []
        self.image_count = 0
        self.create_subscription(Detection2DArray, "/mecamind/detections", self.det_msgs.append, 10)
        self.create_subscription(String, "/mecamind/raw_detections", self.raw_msgs.append, 10)
        self.create_subscription(String, "/mecamind/perception_event", self.event_msgs.append, 10)
        self.create_subscription(Image, "/mecamind/detection_image", lambda m: setattr(self, "image_count", self.image_count + 1), 2)


rclpy.init()
probe = Probe()
deadline = time.monotonic() + 20.0
while time.monotonic() < deadline:
    rclpy.spin_once(probe, timeout_sec=0.2)
    if len(probe.det_msgs) >= 20 and probe.event_msgs and probe.image_count >= 5:
        break

checks = {}
checks["detections_flowing"] = len(probe.det_msgs) >= 10
checks["debug_image_flowing"] = probe.image_count >= 5
checks["json_bridge_flowing"] = len(probe.raw_msgs) >= 10

# 结构断言：字段齐全、数值在合法区间
det_ok = False
backend = ""
for msg in probe.det_msgs:
    if not msg.detections:
        continue
    d = msg.detections[0]
    backend = msg.backend
    det_ok = (
        bool(d.label)
        and 0.0 <= d.confidence <= 1.0
        and 0.0 <= d.cx <= 1.0 and 0.0 <= d.cy <= 1.0
        and 0.0 < d.width <= 1.0 and 0.0 < d.height <= 1.0
        and msg.inference_ms >= 0.0
    )
    if det_ok:
        break
checks["detection_fields_valid"] = det_ok

# 旧 JSON 契约兼容：bbox 嵌套结构存在
json_ok = False
for msg in probe.raw_msgs:
    try:
        data = json.loads(msg.data)
        item = data["detections"][0]
        json_ok = "label" in item and "bbox" in item and "cx" in item["bbox"]
        if json_ok:
            break
    except Exception:
        pass
checks["json_contract_compatible"] = json_ok

# 下游过滤器产出 confirmed 事件（证明整条链在动）
confirmed = any(json.loads(m.data).get("confirmed") for m in probe.event_msgs if m.data)
checks["filter_confirmed_event"] = confirmed

report = {
    "backend": backend,
    "detection_frames": len(probe.det_msgs),
    "debug_images": probe.image_count,
    "checks": checks,
    "overall_ok": all(checks.values()),
}
with open(sys.argv[1], "w", encoding="utf-8") as fh:
    json.dump(report, fh, ensure_ascii=False, indent=2)
for name, ok in checks.items():
    print(f"[CP4] [{'正常' if ok else '失败'}] {name}")
print(f"[CP4] 推理后端：{backend}，20 秒收到 {len(report and probe.det_msgs)} 帧检测")
probe.destroy_node()
rclpy.shutdown()
sys.exit(0 if report["overall_ok"] else 1)
PY

echo "MECAMIND_CP4_OK"
