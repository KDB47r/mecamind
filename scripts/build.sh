#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
set -u

cd "$PROJECT_ROOT"
colcon build --symlink-install

echo
echo "构建完成，请执行："
echo "source \"$PROJECT_ROOT/install/setup.bash\""
