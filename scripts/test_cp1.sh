#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_FILE="${MECAMIND_CP1_LOG:-/tmp/mecamind_cp1_gazebo.log}"
ROS_DOMAIN_ID="${MECAMIND_ROS_DOMAIN_ID:-81}"
export ROS_DOMAIN_ID
export ROS2CLI_NO_DAEMON=1
LOCK_DIR="/tmp/mecamind_cp1.lock.d"
if ! mkdir "$LOCK_DIR" 2>/dev/null; then
  lock_owner=""
  if [[ -f "$LOCK_DIR/owner" ]]; then
    lock_owner="$(<"$LOCK_DIR/owner")"
  fi
  if [[ -n "$lock_owner" ]] && kill -0 "$lock_owner" 2>/dev/null; then
    echo "已有一套 CP1 验收正在运行，请等待其结束。" >&2
    exit 1
  fi
  rm -f "$LOCK_DIR/owner"
  rmdir "$LOCK_DIR" 2>/dev/null || {
    echo "CP1 锁目录异常：$LOCK_DIR" >&2
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
  if [[ -n "${LAUNCH_PID:-}" ]]; then
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

ros2 launch mecamind_bringup gazebo.launch.py use_gui:=false >"$LOG_FILE" 2>&1 &
LAUNCH_PID=$!

for topic in /clock /scan /imu/data /odom; do
  ready=0
  for _ in $(seq 1 15); do
    if timeout 10 ros2 topic echo --once "$topic" >/dev/null 2>&1; then
      ready=1
      break
    fi
    sleep 0.5
  done
  if [[ "$ready" -ne 1 ]]; then
    echo "未收到 $topic，Gazebo 日志：$LOG_FILE" >&2
    exit 1
  fi
  echo "[正常] $topic"
done

python3 "$PROJECT_ROOT/tests/cp1_motion_check.py"
echo "MECAMIND_CP1_OK"
