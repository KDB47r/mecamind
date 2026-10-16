#!/usr/bin/env bash
# CP5 验收（第五课）：无界面启动"视觉跟随 + 速度仲裁"链路 → 多项断言。
#
# 验收点（对应 PROJECT_PLAN 10.3.5）：
#   1. 跟随方向正确：synthetic 目标左右移动时，/cmd_vel_follow 的
#      angular.z 与目标偏离方向相反（目标偏右 -> 右转 = 负角速度）；
#   2. 目标丢失自动停车：关闭跟随开关后 /cmd_vel 归零；
#   3. 速度仲裁：遥控指令能抢占跟随（active 源切换为 teleop），
#      遥控沉默后自动交还；
#   4. 急停最高优先级：estop=True 时输出恒为零速，解除后恢复。
#
# 默认 lite 后端 + synthetic 视觉源，全程无 GUI、无 ML 依赖。
# Gazebo：MECAMIND_CP5_BACKEND=gazebo bash scripts/test_cp5.sh
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BACKEND="${MECAMIND_CP5_BACKEND:-lite}"
LOG_FILE="${MECAMIND_CP5_LOG:-/tmp/mecamind_cp5_${BACKEND}.log}"
REPORT_PATH="maps/mecamind_cp5_report.json"
ROS_DOMAIN_ID="${MECAMIND_ROS_DOMAIN_ID:-85}"
export ROS_DOMAIN_ID
export ROS2CLI_NO_DAEMON=1

if [[ "$BACKEND" != "lite" && "$BACKEND" != "gazebo" ]]; then
  echo "不支持的后端：$BACKEND（仅 lite / gazebo）" >&2
  exit 1
fi

LOCK_DIR="/tmp/mecamind_cp5.lock.d"
if ! mkdir "$LOCK_DIR" 2>/dev/null; then
  lock_owner=""
  [[ -f "$LOCK_DIR/owner" ]] && lock_owner="$(<"$LOCK_DIR/owner")"
  if [[ -n "$lock_owner" ]] && kill -0 "$lock_owner" 2>/dev/null; then
    echo "已有一套 CP5 验收正在运行，请等待其结束。" >&2
    exit 1
  fi
  rm -f "$LOCK_DIR/owner"; rmdir "$LOCK_DIR" 2>/dev/null || true; mkdir "$LOCK_DIR"
fi
echo "$$" >"$LOCK_DIR/owner"

set +u
source /opt/ros/jazzy/setup.bash
source "$PROJECT_ROOT/install/setup.bash"
set -u

collect_descendants() {
  local parent_pid="$1"
  local child_pid
  for child_pid in $(pgrep -P "$parent_pid" 2>/dev/null || true); do
    collect_descendants "$child_pid"
    echo "$child_pid"
  done
}

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    local descendants
    descendants="$(collect_descendants "$LAUNCH_PID")"
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    for _ in $(seq 1 60); do
      kill -0 "$LAUNCH_PID" 2>/dev/null || break
      sleep 0.25
    done
    kill -TERM "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
    for child_pid in $descendants; do
      kill -TERM "$child_pid" 2>/dev/null || true
    done
    sleep 1
    for child_pid in $descendants; do
      kill -KILL "$child_pid" 2>/dev/null || true
    done
  fi
  rm -f "$LOCK_DIR/owner"; rmdir "$LOCK_DIR" 2>/dev/null || true
}
trap cleanup EXIT

cd "$PROJECT_ROOT"
mkdir -p maps
rm -f "$REPORT_PATH"

# Gazebo 冷启动更慢；lite 约 8s 即可。
BOOT_WAIT_SEC=8
PROBE_TIMEOUT_SEC=150
if [[ "$BACKEND" == "gazebo" ]]; then
  BOOT_WAIT_SEC="${MECAMIND_CP5_BOOT_WAIT_SEC:-35}"
  PROBE_TIMEOUT_SEC="${MECAMIND_CP5_PROBE_TIMEOUT_SEC:-240}"
fi

echo "[CP5] 后端：$BACKEND，启动等待 ${BOOT_WAIT_SEC}s，日志：$LOG_FILE"
ros2 launch mecamind_bringup follow.launch.py \
  backend:="$BACKEND" use_gui:=false \
  source:=synthetic infer_backend:=hsv \
  >"$LOG_FILE" 2>&1 &
LAUNCH_PID=$!

sleep "$BOOT_WAIT_SEC"
if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
  echo "[失败] 跟随链路启动即退出，日志：$LOG_FILE" >&2
  tail -n 80 "$LOG_FILE" >&2 || true
  exit 1
fi

# 等感知事件出现，避免 Gazebo 尚未就绪时探针空转失败。
READY_DEADLINE=$((SECONDS + 60))
while (( SECONDS < READY_DEADLINE )); do
  if ros2 topic echo /mecamind/perception_metrics --once >/dev/null 2>&1; then
    break
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "[失败] 跟随链路中途退出，日志：$LOG_FILE" >&2
    tail -n 80 "$LOG_FILE" >&2 || true
    exit 1
  fi
  sleep 1
done

timeout "$PROBE_TIMEOUT_SEC" python3 - "$REPORT_PATH" "$BACKEND" <<'PY'
import json
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


class Probe(Node):
    """CP5 探针：订阅链路各环 + 主动注入开关/遥控/急停指令。"""

    def __init__(self):
        super().__init__("cp5_probe")
        self.follow_cmds = []      # (时刻, cx 参考, Twist)
        self.final_cmds = []       # /cmd_vel（安全门之后）
        self.arbiter_states = []
        self.create_subscription(Twist, "/cmd_vel_follow", self._follow_cb, 20)
        self.create_subscription(Twist, "/cmd_vel", self._final_cb, 20)
        self.create_subscription(String, "/mecamind/perception_event", self._event_cb, 20)
        self.create_subscription(String, "/mecamind/arbiter_state", self._state_cb, 10)
        self.enable_pub = self.create_publisher(Bool, "/mecamind/follow_enable", 1)
        self.estop_pub = self.create_publisher(Bool, "/mecamind/estop", 1)
        self.teleop_pub = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        self._last_cx = None

    def _event_cb(self, msg):
        try:
            self._last_cx = float(json.loads(msg.data).get("cx", 0.5))
        except Exception:
            pass

    def _follow_cb(self, msg):
        self.follow_cmds.append((time.monotonic(), self._last_cx, msg))

    def _final_cb(self, msg):
        self.final_cmds.append((time.monotonic(), msg))

    def _state_cb(self, msg):
        try:
            self.arbiter_states.append((time.monotonic(), json.loads(msg.data)))
        except Exception:
            pass

    def spin_for(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def set_follow(self, on):
        self.enable_pub.publish(Bool(data=on))

    def set_estop(self, on):
        self.estop_pub.publish(Bool(data=on))

    def send_teleop(self, sec, vx=0.1):
        """以约 10Hz 持续发遥控指令 sec 秒。"""
        end = time.monotonic() + sec
        t = Twist()
        t.linear.x = vx
        while time.monotonic() < end:
            self.teleop_pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.1)


rclpy.init()
p = Probe()
checks = {}
backend = sys.argv[2]

# ---- ① 打开跟随，采样 12 秒（synthetic 目标周期 8 秒，覆盖左右两侧）----
p.spin_for(3.0)
p.set_follow(True)
p.spin_for(12.0)
samples = [(cx, m.angular.z) for (_, cx, m) in p.follow_cmds if cx is not None]
# 只统计目标明显偏离中心的样本（死区外），验证角速度方向与偏差相反
directional = [(cx, wz) for cx, wz in samples if abs(cx - 0.5) > 0.10]
good = sum(1 for cx, wz in directional if (cx - 0.5) * wz < 0)
checks["follow_direction_correct"] = len(directional) >= 10 and good / max(1, len(directional)) > 0.9
checks["follow_publishing"] = len(p.follow_cmds) >= 20
active_during_follow = any(s.get("active") == "follow" for _, s in p.arbiter_states[-8:])
checks["arbiter_selects_follow"] = active_during_follow

# ---- ② 遥控抢占：发 3 秒遥控，仲裁器应切到 teleop；停发后交还 ----
mark = time.monotonic()
p.send_teleop(3.0)
teleop_active = any(s.get("active") == "teleop" for t, s in p.arbiter_states if t > mark)
checks["teleop_preempts_follow"] = teleop_active
p.spin_for(4.0)
back = [s.get("active") for t, s in p.arbiter_states if t > mark + 4.0]
checks["control_returns_after_teleop"] = "follow" in back

# ---- ③ 急停：置 True 后 /cmd_vel 应全为零速；解除后恢复 ----
p.set_estop(True)
p.spin_for(1.0)
mark = time.monotonic()
p.spin_for(3.0)
during = [m for t, m in p.final_cmds if t > mark]
all_zero = during and all(abs(m.linear.x) < 1e-6 and abs(m.angular.z) < 1e-6 for m in during)
estop_active = any(s.get("active") == "estop" for t, s in p.arbiter_states if t > mark)
checks["estop_forces_zero"] = bool(all_zero) and estop_active
p.set_estop(False)
p.spin_for(3.0)

# ---- ④ 目标丢失/关闭跟随 -> 停车：关掉开关，输出应归零并沉默 ----
p.set_follow(False)
p.spin_for(1.5)
mark = time.monotonic()
p.spin_for(3.0)
after_off = [m for t, m in p.final_cmds if t > mark]
stopped = all(abs(m.linear.x) < 1e-6 and abs(m.angular.z) < 1e-6 for m in after_off)
checks["disable_stops_robot"] = stopped

report = {
    "backend": backend,
    "follow_samples": len(samples),
    "directional_samples": len(directional),
    "direction_correct_ratio": round(good / max(1, len(directional)), 3),
    "checks": checks,
    "overall_ok": all(checks.values()),
}
with open(sys.argv[1], "w", encoding="utf-8") as fh:
    json.dump(report, fh, ensure_ascii=False, indent=2)
for name, ok in checks.items():
    print(f"[CP5] [{'正常' if ok else '失败'}] {name}")
print(f"[CP5] 方向正确率 {report['direction_correct_ratio']*100:.0f}%（{len(directional)} 个方向样本）")
p.destroy_node()
rclpy.shutdown()
sys.exit(0 if report["overall_ok"] else 1)
PY

echo "MECAMIND_CP5_OK"
