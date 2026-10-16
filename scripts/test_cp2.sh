#!/usr/bin/env bash
# CP2 验收：无界面半自动建图 → 保存 PGM/YAML 地图对 → 地图质量与资产报告。
# 默认使用轻量 2D 后端（快、无 GUI 依赖）；MECAMIND_CP2_BACKEND=gazebo 可切换 Gazebo headless。
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BACKEND="${MECAMIND_CP2_BACKEND:-lite}"
LOG_FILE="${MECAMIND_CP2_LOG:-/tmp/mecamind_cp2_${BACKEND}.log}"
MAP_PREFIX="maps/mecamind_cp2_${BACKEND}_map"
REPORT_PATH="maps/mecamind_cp2_${BACKEND}_route_report.json"
MANIFEST_PATH="maps/mecamind_cp2_${BACKEND}_manifest.json"
ROUTE_TIMEOUT_SEC="${MECAMIND_CP2_TIMEOUT_SEC:-600}"
ROS_DOMAIN_ID="${MECAMIND_ROS_DOMAIN_ID:-82}"
export ROS_DOMAIN_ID
export ROS2CLI_NO_DAEMON=1

LOCK_DIR="/tmp/mecamind_cp2.lock.d"
if ! mkdir "$LOCK_DIR" 2>/dev/null; then
  lock_owner=""
  if [[ -f "$LOCK_DIR/owner" ]]; then
    lock_owner="$(<"$LOCK_DIR/owner")"
  fi
  if [[ -n "$lock_owner" ]] && kill -0 "$lock_owner" 2>/dev/null; then
    echo "已有一套 CP2 验收正在运行，请等待其结束。" >&2
    exit 1
  fi
  rm -f "$LOCK_DIR/owner"
  rmdir "$LOCK_DIR" 2>/dev/null || {
    echo "CP2 锁目录异常：$LOCK_DIR" >&2
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
rm -f "$MAP_PREFIX.pgm" "$MAP_PREFIX.yaml" "$REPORT_PATH" "$MANIFEST_PATH"

echo "[CP2] 后端：$BACKEND，日志：$LOG_FILE"
ros2 launch mecamind_bringup mapping.launch.py \
  backend:="$BACKEND" \
  use_gui:=false \
  with_rviz:=false \
  auto_route:=true \
  shutdown_on_route_complete:=true \
  map_save_path:="$MAP_PREFIX" \
  route_report_path:="$REPORT_PATH" \
  >"$LOG_FILE" 2>&1 &
LAUNCH_PID=$!

echo "[CP2] 等待半自动建图路线完成（最长 ${ROUTE_TIMEOUT_SEC}s）..."
elapsed=0
while kill -0 "$LAUNCH_PID" 2>/dev/null; do
  if (( elapsed >= ROUTE_TIMEOUT_SEC )); then
    echo "[失败] 建图路线在 ${ROUTE_TIMEOUT_SEC}s 内未完成，日志：$LOG_FILE" >&2
    exit 1
  fi
  sleep 5
  elapsed=$((elapsed + 5))
done
wait "$LAUNCH_PID" 2>/dev/null || true
unset LAUNCH_PID

for f in "$MAP_PREFIX.pgm" "$MAP_PREFIX.yaml" "$REPORT_PATH"; do
  if [[ ! -f "$f" ]]; then
    echo "[失败] 缺少产物 $f，日志：$LOG_FILE" >&2
    exit 1
  fi
  echo "[正常] $f"
done

python3 - "$REPORT_PATH" <<'PY'
import json, sys
report = json.load(open(sys.argv[1], encoding="utf-8"))
reached = int(report.get("reached_waypoints", 0))
route_size = max(int(report.get("route_size", 0)), 1)
print(f"[CP2] 路线报告：phase={report['phase']} "
      f"reached={reached}/{route_size} "
      f"skipped={report['skipped_waypoints']}")
if not report.get("auto_map_saved"):
    print("[失败] 路线报告显示自动存图失败", file=sys.stderr)
    sys.exit(1)
# 半自动路线至少完成 2/3 路点，否则地图覆盖通常不足，后续质量分析也会失败。
if reached * 3 < route_size * 2:
    print(
        f"[失败] 路点到达过少（{reached}/{route_size}）。"
        "请先 bash scripts/cleanup_ros2.sh --all 清理残留仿真后重试。",
        file=sys.stderr,
    )
    sys.exit(1)
PY

WORLD_FILE="$(ros2 pkg prefix mecamind_tools)/share/mecamind_tools/worlds/three_room_house.world"

echo "[CP2] 地图质量分析..."
QUALITY_PATH="maps/mecamind_cp2_${BACKEND}_quality.json"
ros2 run mecamind_tools mecamind_map_quality_analyzer "$MAP_PREFIX.yaml" \
  --world "$WORLD_FILE" --json >"$QUALITY_PATH"
python3 - "$QUALITY_PATH" <<'PY'
import json, sys
result = json.load(open(sys.argv[1], encoding="utf-8"))
print(f"[CP2] coverage={result['coverage']*100:.1f}% "
      f"unknown={result['unknown_ratio']*100:.1f}% "
      f"free_connectivity={result['largest_free_component_ratio']*100:.1f}% "
      f"missing_walls={result['missing_walls'] or '无'}")
print(f"[CP2] verdict={result['verdict']}")
if result["verdict"] not in ("commercial_ready", "navigation_ready_best_effort"):
    print("[失败] 地图质量未达到可导航标准，需要重新建图", file=sys.stderr)
    sys.exit(1)
PY

echo "[CP2] 生成地图资产清单..."
ros2 run mecamind_tools mecamind_map_asset_manager maps --world "$WORLD_FILE" \
  --output "$MANIFEST_PATH" >/dev/null
echo "[正常] $MANIFEST_PATH"

echo "MECAMIND_CP2_OK"
