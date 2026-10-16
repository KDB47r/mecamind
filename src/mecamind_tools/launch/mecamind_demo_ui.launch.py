"""综合演示入口（轻量仿真 + 导航 + RViz，使用预存地图）。

历史工具入口：组合包含 mecamind_navigation.launch.py 并默认加载
包内自带的演示地图。课堂主路径请优先使用
mecamind_bringup 下的统一入口（mapping/navigation.launch.py）。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _default_map_file() -> str:
    package_share = get_package_share_directory("mecamind_tools")
    return f"{package_share}/maps/mecamind_three_room_map_revisit_v3.yaml"


def generate_launch_description():
    package_share = get_package_share_directory("mecamind_tools")
    navigation_launch = f"{package_share}/launch/mecamind_navigation.launch.py"
    interaction_launch = f"{package_share}/launch/mecamind_interaction.launch.py"
    aliyun_voice_launch = f"{package_share}/launch/mecamind_aliyun_voice.launch.py"

    map_file = LaunchConfiguration("map_file")
    with_rviz = LaunchConfiguration("with_rviz")
    task_provider = LaunchConfiguration("task_provider")
    enable_fake_detections = LaunchConfiguration("enable_fake_detections")
    enable_microphone = LaunchConfiguration("enable_microphone")
    microphone_backend = LaunchConfiguration("microphone_backend")
    microphone_device = LaunchConfiguration("microphone_device")
    microphone_duration_sec = LaunchConfiguration("microphone_duration_sec")
    enable_voice_tts = LaunchConfiguration("enable_voice_tts")
    api_key_env = LaunchConfiguration("api_key_env")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_file",
                default_value=_default_map_file(),
                description="Saved map YAML for classroom Nav2 demo (revisit_v3 recommended).",
            ),
            DeclareLaunchArgument("with_rviz", default_value="true"),
            DeclareLaunchArgument("task_provider", default_value="rule"),
            DeclareLaunchArgument("enable_fake_detections", default_value="true"),
            DeclareLaunchArgument("enable_microphone", default_value="false"),
            DeclareLaunchArgument("microphone_backend", default_value="auto"),
            DeclareLaunchArgument("microphone_device", default_value="default"),
            DeclareLaunchArgument("microphone_duration_sec", default_value="4.0"),
            DeclareLaunchArgument("enable_voice_tts", default_value="true"),
            DeclareLaunchArgument("api_key_env", default_value="DASHSCOPE_API_KEY"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(navigation_launch),
                launch_arguments={
                    "with_rviz": with_rviz,
                    "use_saved_map": "true",
                    "map_file": map_file,
                    "enable_auto_patrol": "false",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(interaction_launch),
                launch_arguments={
                    "task_provider": task_provider,
                    "enable_fake_detections": enable_fake_detections,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(aliyun_voice_launch),
                condition=IfCondition(enable_microphone),
                launch_arguments={
                    "api_key_env": api_key_env,
                    "enable_microphone": "true",
                    "enable_task_scheduler": "false",
                    "enable_tts": enable_voice_tts,
                    "microphone_backend": microphone_backend,
                    "microphone_device": microphone_device,
                    "microphone_duration_sec": microphone_duration_sec,
                }.items(),
            ),
        ]
    )
