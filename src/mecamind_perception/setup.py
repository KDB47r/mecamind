"""mecamind_perception 的安装脚本。

与 mecamind_tools 的 setup.py 结构相同：
- data_files 安装 launch / config 资源；
- entry_points.console_scripts 注册可执行节点。
"""

from glob import glob
import os

from setuptools import find_packages, setup


package_name = "mecamind_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROS2 AI Course",
    maintainer_email="course@example.com",
    description="MecaMind perception nodes: detector with fallbacks, JSON bridge, cmd_vel arbiter.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mecamind_detector = mecamind_perception.detector_node:main",
            "mecamind_detection_json_bridge = mecamind_perception.json_bridge_node:main",
            "mecamind_cmd_vel_arbiter = mecamind_perception.cmd_vel_arbiter:main",
        ],
    },
)
