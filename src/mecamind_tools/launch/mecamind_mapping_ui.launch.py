"""半自动建图的带界面版本：包含 mecamind_mapping 并强制打开 RViz。"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory("mecamind_tools")
    mapping_launch = f"{package_share}/launch/mecamind_mapping.launch.py"
    lidar_noise_std = LaunchConfiguration("lidar_noise_std")
    lidar_dropout_prob = LaunchConfiguration("lidar_dropout_prob")
    odom_xy_noise_std = LaunchConfiguration("odom_xy_noise_std")
    odom_yaw_noise_std = LaunchConfiguration("odom_yaw_noise_std")
    cmd_latency_sec = LaunchConfiguration("cmd_latency_sec")

    return LaunchDescription(
        [
            DeclareLaunchArgument("lidar_noise_std", default_value="0.01"),
            DeclareLaunchArgument("lidar_dropout_prob", default_value="0.0"),
            DeclareLaunchArgument("odom_xy_noise_std", default_value="0.0"),
            DeclareLaunchArgument("odom_yaw_noise_std", default_value="0.0"),
            DeclareLaunchArgument("cmd_latency_sec", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mapping_launch),
                launch_arguments={
                    "with_rviz": "true",
                    "lidar_noise_std": lidar_noise_std,
                    "lidar_dropout_prob": lidar_dropout_prob,
                    "odom_xy_noise_std": odom_xy_noise_std,
                    "odom_yaw_noise_std": odom_yaw_noise_std,
                    "cmd_latency_sec": cmd_latency_sec,
                }.items(),
            )
        ]
    )
