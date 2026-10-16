#!/usr/bin/env bash
# CP3 验收：无界面启动 Nav2 → 连续导航三个命名目标 → 运动中取消 → JSON 报告。
# 默认使用轻量 2D 后端（快、无 GUI 依赖）；MECAMIND_CP3_BACKEND=gazebo 可切换 Gazebo headless。
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BACKEND="${MECAMIND_CP3_BACKEND:-lite}"
LOG_FILE="${MECAMIND_CP3_LOG:-/tmp/mecamind_cp3_${BACKEND}.log}"
REPORT_PATH="maps/mecamind_cp3_${BACKEND}_nav_report.json"
NAV_TIMEOUT_SEC="${MECAMIND_CP3_TIMEOUT_SEC:-900}"
ROS_DOMAIN_ID="${MECAMIND_ROS_DOMAIN_ID:-83}"
export ROS_DOMAIN_ID
export ROS2CLI_NO_DAEMON=1

LOCK_DIR="/tmp/mecamind_cp3.lock.d"
if ! mkdir "$LOCK_DIR" 2>/dev/null; then
  lock_owner=""
  if [[ -f "$LOCK_DIR/owner" ]]; then
    lock_owner="$(<"$LOCK_DIR/owner")"
  fi
  if [[ -n "$lock_owner" ]] && kill -0 "$lock_owner" 2>/dev/null; then
    echo "已有一套 CP3 验收正在运行，请等待其结束。" >&2
    exit 1
  fi
  rm -f "$LOCK_DIR/owner"
  rmdir "$LOCK_DIR" 2>/dev/null || {
    echo "CP3 锁目录异常：$LOCK_DIR" >&2
    exit 1
  }
  mkdir "$LOCK_DIR"
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
  rm -f "$LOCK_DIR/owner"
  rmdir "$LOCK_DIR" 2>/dev/null || true
}
trap cleanup EXIT

cd "$PROJECT_ROOT"
mkdir -p maps
rm -f "$REPORT_PATH"

echo "[CP3] 后端：$BACKEND，日志：$LOG_FILE"
ros2 launch mecamind_bringup navigation.launch.py \
  backend:="$BACKEND" \
  use_gui:=false \
  with_rviz:=false \
  >"$LOG_FILE" 2>&1 &
LAUNCH_PID=$!

sleep 5
if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
  echo "[失败] 导航栈启动即退出，日志：$LOG_FILE" >&2
  exit 1
fi

# Gazebo 在 WSL2 上 RTF 常低于 1，单目标墙钟超时按后端区分。
GOAL_TIMEOUT_SEC=180
if [[ "$BACKEND" == "gazebo" ]]; then
  GOAL_TIMEOUT_SEC=420
fi

echo "[CP3] 运行导航验收（三个命名目标 + 取消测试，最长 ${NAV_TIMEOUT_SEC}s）..."
if ! timeout "$NAV_TIMEOUT_SEC" ros2 run mecamind_tools mecamind_nav_acceptance \
  --ros-args \
  -p use_sim_time:=true \
  -p goals:="[living_room, bedroom, hall_entry]" \
  -p goal_timeout_sec:="$GOAL_TIMEOUT_SEC".0 \
  -p report_path:="$REPORT_PATH" \
  >>"$LOG_FILE" 2>&1; then
  echo "[失败] 导航验收未通过，报告：$REPORT_PATH，日志：$LOG_FILE" >&2
  if [[ -f "$REPORT_PATH" ]]; then
    python3 -c "import json,sys; print(json.dumps(json.load(open(sys.argv[1])), ensure_ascii=False, indent=2))" "$REPORT_PATH" >&2 || true
  fi
  exit 1
fi

python3 - "$REPORT_PATH" <<'PY'
import json, sys
report = json.load(open(sys.argv[1], encoding="utf-8"))
for item in report["goals"]:
    mark = "正常" if item["reached"] else "失败"
    print(f"[CP3] [{mark}] 目标 {item['goal']}: {item['final_detail']} 用时 {item['elapsed_sec']}s")
cancel = report.get("cancel_test") or {}
print(f"[CP3] 取消测试：activated={cancel.get('activated')} stopped={cancel.get('stopped')}")
if not report.get("overall_ok"):
    print("[失败] CP3 报告 overall_ok=false", file=sys.stderr)
    sys.exit(1)
PY

echo "MECAMIND_CP3_OK"
