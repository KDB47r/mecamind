"""轻量仿真 + 任务编排的组合入口（第一课早期演示用）。

启动轻量仿真器和任务简报/调度等编排节点，用 mode/mission
参数选择运行模式。后续课程的建图/导航请使用
mecamind_bringup 下的统一入口。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("mecamind_tools")
    mode = LaunchConfiguration("mode")
    mission = LaunchConfiguration("mission")
    world = LaunchConfiguration("world")
    lidar_noise_std = LaunchConfiguration("lidar_noise_std")
    lidar_dropout_prob = LaunchConfiguration("lidar_dropout_prob")
    odom_xy_noise_std = LaunchConfiguration("odom_xy_noise_std")
    odom_yaw_noise_std = LaunchConfiguration("odom_yaw_noise_std")
    cmd_latency_sec = LaunchConfiguration("cmd_latency_sec")

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="sim"),
            DeclareLaunchArgument("mission", default_value="mapping"),
            DeclareLaunchArgument(
                "world",
                default_value=f"{package_share}/worlds/three_room_house.world",
            ),
            DeclareLaunchArgument("lidar_noise_std", default_value="0.0"),
            DeclareLaunchArgument("lidar_dropout_prob", default_value="0.0"),
            DeclareLaunchArgument("odom_xy_noise_std", default_value="0.0"),
            DeclareLaunchArgument("odom_yaw_noise_std", default_value="0.0"),
            DeclareLaunchArgument("cmd_latency_sec", default_value="0.0"),
            Node(
                package="mecamind_tools",
                executable="mecamind_simulator_node",
                name="mecamind_simulator",
                output="screen",
                parameters=[
                    {
                        "world_path": world,
                        "initial_x": -3.2,
                        "initial_y": -2.4,
                        "initial_yaw": 0.0,
                        "lidar_noise_std": ParameterValue(lidar_noise_std, value_type=float),
                        "lidar_dropout_prob": ParameterValue(lidar_dropout_prob, value_type=float),
                        "odom_xy_noise_std": ParameterValue(odom_xy_noise_std, value_type=float),
                        "odom_yaw_noise_std": ParameterValue(odom_yaw_noise_std, value_type=float),
                        "cmd_latency_sec": ParameterValue(cmd_latency_sec, value_type=float),
                    }
                ],
            ),
            Node(
                package="mecamind_tools",
                executable="mecamind_map_publisher_node",
                name="mecamind_map_publisher",
                output="screen",
                parameters=[
                    {
                        "world_path": world,
                    }
                ],
            ),
            Node(
                package="mecamind_tools",
                executable="mission_brief_node",
                name="mecamind_mission_brief",
                output="screen",
                parameters=[
                    {
                        "mode": mode,
                        "mission": mission,
                        "output_topic": "/mecamind/mission_brief",
                        "world_path": world,
                    }
                ],
            ),
        ]
    )
