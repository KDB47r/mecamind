#!/usr/bin/env bash
set -euo pipefail

missing=0

if [[ -r /etc/os-release ]]; then
  . /etc/os-release
  echo "Ubuntu: ${PRETTY_NAME:-unknown}"
fi

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "[缺失] ROS 2 Jazzy: /opt/ros/jazzy/setup.bash"
  missing=1
else
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
  echo "[正常] ROS_DISTRO=$ROS_DISTRO"
fi

for command_name in colcon gz ros2; do
  if command -v "$command_name" >/dev/null 2>&1; then
    echo "[正常] $command_name=$(command -v "$command_name")"
  else
    echo "[缺失] $command_name"
    missing=1
  fi
done

for package_name in ros_gz_sim ros_gz_bridge robot_state_publisher xacro teleop_twist_keyboard tf2_tools rqt_image_view rviz2; do
  if ros2 pkg prefix "$package_name" >/dev/null 2>&1; then
    echo "[正常] ROS 包 $package_name"
  else
    echo "[缺失] ROS 包 $package_name"
    missing=1
  fi
done

echo "DISPLAY=${DISPLAY:-未设置}"
echo "WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-未设置}"
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "[提示] 当前终端没有 GUI 显示变量，只能使用 use_gui:=false。"
elif command -v xdpyinfo >/dev/null 2>&1 && ! xdpyinfo -display "${DISPLAY:-:0}" >/dev/null 2>&1; then
  if xdpyinfo -display :0 >/dev/null 2>&1; then
    echo "[提示] 当前 DISPLAY 不可连接，但 WSLg 的 :0 可用；项目启动文件会自动使用 :0。"
  else
    echo "[警告] 未找到可连接的 X11 显示，Gazebo UI 需要先修复 WSL GUI。"
  fi
fi

if [[ "$missing" -ne 0 ]]; then
  echo "环境检查失败，请运行 setup/install_dependencies.sh。" >&2
  exit 1
fi

echo "MECAMIND_ENVIRONMENT_OK"
