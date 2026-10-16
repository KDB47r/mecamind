#!/usr/bin/env bash
set -euo pipefail

sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-pytest \
  ros-jazzy-ros-gz \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-tf2-tools \
  ros-jazzy-rqt-image-view \
  ros-jazzy-rqt-graph \
  ros-jazzy-rviz2 \
  ros-jazzy-xacro

echo "MecaMind ROS2 第一次课依赖安装完成。"

# 第四课（10.3.4）可选 ML 依赖：./setup/install_dependencies.sh --with-ml
# 不装也能上课：detector 会自动降级到 HSV 兜底后端。
if [[ "${1:-}" == "--with-ml" ]]; then
  # setuptools>=80 移除了 setup.py develop，colcon --symlink-install 会挂，钉在 79.x
  pip install --user --break-system-packages "setuptools<80" ultralytics onnx onnxruntime
  echo "第四课 ML 依赖安装完成（ultralytics + onnx + onnxruntime）。"
fi
