#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="project"
DRY_RUN=0

usage() {
  cat <<'EOF'
用法：
  bash scripts/cleanup_ros2.sh [--all] [--dry-run]

选项：
  --all      清理当前用户启动的全部 ROS 2、RViz 和 Gazebo 进程
  --dry-run  仅显示将要清理的进程，不发送信号
  -h, --help 显示帮助

默认只清理 MecaMind ROS2 项目相关进程和临时锁。
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --all)
      MODE="all"
      ;;
    --dry-run)
      DRY_RUN=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "未知参数：$1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if [[ "$DRY_RUN" -eq 0 ]]; then
  set +u
  source /opt/ros/jazzy/setup.bash 2>/dev/null || true
  set -u
  ros2 daemon stop >/dev/null 2>&1 || true
fi

mapfile -t TARGET_PIDS < <(
  python3 - "$MODE" "$PROJECT_ROOT" "$$" <<'PY'
import os
import re
import sys
from pathlib import Path

mode, project_root, shell_pid = sys.argv[1], sys.argv[2], int(sys.argv[3])
excluded = {os.getpid(), shell_pid}


def parent_pid(pid):
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().split()
        return int(fields[3])
    except (OSError, ValueError, IndexError):
        return 0


pid = shell_pid
while pid > 1:
    excluded.add(pid)
    pid = parent_pid(pid)

project_markers = (
    project_root,
    "mecamind_",
    "mecamind_ros2",
    "mecamind-mecanum",
    "/tmp/mecamind_",
)
all_patterns = (
    re.compile(r"(^|/)(ros2|_ros2_daemon|rviz2|gzserver|gzclient)(\s|$)"),
    re.compile(r"(^|/)(parameter_bridge|robot_state_publisher|teleop_twist_keyboard)(\s|$)"),
    re.compile(r"(^|/)gz(\s+sim|\s+-)"),
    re.compile(r"/opt/ros/[^ ]*/lib/"),
    re.compile(r"/install/[^ ]+/lib/"),
)

for entry in Path("/proc").iterdir():
    if not entry.name.isdigit():
        continue
    pid = int(entry.name)
    if pid in excluded:
        continue
    try:
        if entry.stat().st_uid != os.getuid():
            continue
        raw = (entry / "cmdline").read_bytes()
    except OSError:
        continue
    command = raw.replace(b"\0", b" ").decode(errors="replace").strip()
    if not command:
        continue
    matched = any(marker in command for marker in project_markers)
    if mode == "all":
        matched = matched or any(pattern.search(command) for pattern in all_patterns)
    if matched:
        print(f"{pid}\t{command}")
PY
)

if [[ "${#TARGET_PIDS[@]}" -eq 0 ]]; then
  echo "未发现需要清理的进程。"
else
  echo "发现 ${#TARGET_PIDS[@]} 个待清理进程："
  for item in "${TARGET_PIDS[@]}"; do
    printf '  %s\n' "$item"
  done

  if [[ "$DRY_RUN" -eq 0 ]]; then
    for item in "${TARGET_PIDS[@]}"; do
      pid="${item%%$'\t'*}"
      kill -TERM "$pid" 2>/dev/null || true
    done

    for _ in $(seq 1 20); do
      alive=0
      for item in "${TARGET_PIDS[@]}"; do
        pid="${item%%$'\t'*}"
        if kill -0 "$pid" 2>/dev/null; then
          alive=1
          break
        fi
      done
      [[ "$alive" -eq 0 ]] && break
      sleep 0.1
    done

    for item in "${TARGET_PIDS[@]}"; do
      pid="${item%%$'\t'*}"
      kill -KILL "$pid" 2>/dev/null || true
    done
  fi
fi

if [[ "$DRY_RUN" -eq 0 ]]; then
  rm -f /tmp/mecamind_cp1.lock.d/owner 2>/dev/null || true
  rmdir /tmp/mecamind_cp1.lock.d 2>/dev/null || true
  rm -f /tmp/mecamind_cp1_gazebo.log 2>/dev/null || true
  ros2 daemon stop >/dev/null 2>&1 || true
  echo "清理完成（模式：$MODE）。"
elif [[ "$MODE" == "all" ]]; then
  echo "预览完成：--all 模式。"
else
  echo "预览完成：项目模式。"
fi
