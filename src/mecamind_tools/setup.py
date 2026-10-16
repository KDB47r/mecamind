"""mecamind_tools 包的安装脚本（ROS 2 Python 包用 setuptools 打包）。

【初学者阅读提示】
- data_files：把 launch/config/maps/rviz/worlds 等资源装进
  install/<pkg>/share/<pkg>/，运行时由 get_package_share_directory 找到；
- entry_points.console_scripts：把 "命令名 = 模块:函数" 注册成
  可执行入口，`ros2 run mecamind_tools <命令名>` 就是靠这张表；
- 改了本文件或新增资源后，必须重新 `colcon build` 才会生效。
"""

from glob import glob
import os

from setuptools import find_packages, setup


package_name = "mecamind_tools"
# 排除 *.local.yaml：本地私密配置（如阿里云密钥）不随包安装/提交
public_config_files = [
    path for path in glob("config/*.yaml") if not path.endswith(".local.yaml")
]

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), public_config_files),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.world")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROS2 AI Course",
    maintainer_email="course@example.com",
    description="MecaMind ROS2 engineering tools for project orchestration.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        # ros2 run mecamind_tools <左边的名字> 会调用右边 模块:函数
        "console_scripts": [
            "mission_brief_node = mecamind_tools.mission_brief_node:main",
            "mecamind_lifecycle_activator = mecamind_tools.lifecycle_activator:main",
            "mecamind_frontier_explorer = mecamind_tools.frontier_explorer:main",
            "mecamind_auto_explore = mecamind_tools.auto_explore:main",
            "mecamind_mapping_route_driver = mecamind_tools.mapping_route_driver:main",
            "mecamind_waypoint_patrol = mecamind_tools.waypoint_patrol:main",
            "mecamind_simulator_node = mecamind_tools.simulator_node:main",
            "mecamind_map_publisher_node = mecamind_tools.map_publisher_node:main",
            "mecamind_map_quality_analyzer = mecamind_tools.map_quality_analyzer:main",
            "mecamind_safety_gate = mecamind_tools.safety_layer:main",
            "mecamind_perception_filter = mecamind_tools.perception_filter:main",
            "mecamind_vision_follow_controller = mecamind_tools.vision_follow_controller:main",
            "mecamind_follow_target_mover = mecamind_tools.follow_target_mover:main",
            "mecamind_fake_detection_publisher = mecamind_tools.fake_detection_publisher:main",
            "mecamind_mission_executor = mecamind_tools.mission_executor:main",
            "mecamind_nav_acceptance = mecamind_tools.nav_acceptance:main",
            "mecamind_task_scheduler = mecamind_tools.task_scheduler:main",
            "mecamind_boundary_revisit_planner = mecamind_tools.boundary_revisit_planner:main",
            "mecamind_map_asset_manager = mecamind_tools.map_asset_manager:main",
            "mecamind_aliyun_asr_file = mecamind_tools.aliyun_speech_nodes:asr_main",
            "mecamind_aliyun_asr_stream = mecamind_tools.aliyun_speech_nodes:asr_stream_main",
            "mecamind_microphone_recorder = mecamind_tools.aliyun_speech_nodes:microphone_main",
            "mecamind_aliyun_tts = mecamind_tools.aliyun_speech_nodes:tts_main",
            "mecamind_tts_playback = mecamind_tools.aliyun_speech_nodes:tts_playback_main",
            "mecamind_voice_listen = mecamind_tools.voice_listen_node:main",
        ],
    },
)
