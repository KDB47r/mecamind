"""手动遥控建图入口（仅轻量仿真后端，历史版本）。

拉起 轻量仿真 + SLAM + 安全门，由学员用键盘遥控开车建图。
功能与 mecamind_bringup/mapping.launch.py（默认手动模式）相同，
但不支持 Gazebo 后端，保留用于轻量后端独立调试。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("mecamind_tools")
    mode = LaunchConfiguration("mode")
    world = LaunchConfiguration("world")
    with_rviz = LaunchConfiguration("with_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    lidar_noise_std = LaunchConfiguration("lidar_noise_std")
    lidar_dropout_prob = LaunchConfiguration("lidar_dropout_prob")
    odom_xy_noise_std = LaunchConfiguration("odom_xy_noise_std")
    odom_yaw_noise_std = LaunchConfiguration("odom_yaw_noise_std")
    cmd_latency_sec = LaunchConfiguration("cmd_latency_sec")
    slam_params_path = f"{package_share}/config/mecamind_slam_toolbox_params.yaml"

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_params_path,
            {
                "use_sim_time": True,
                "autostart": True,
                "use_lifecycle_manager": False,
                "scan_topic": "/scan",
            },
        ],
    )

    slam_activator = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="mecamind_tools",
                executable="mecamind_lifecycle_activator",
                output="screen",
                parameters=[
                    {"node_names": ["slam_toolbox"]},
                    {"publish_initial_pose": False},
                ],
            )
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="mecamind_mapping_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(with_rviz),
    )

    safety_gate = Node(
        package="mecamind_tools",
        executable="mecamind_safety_gate",
        name="mecamind_mapping_safety_gate",
        output="screen",
        parameters=[
            {"scan_topic": "/scan"},
            {"input_cmd_topic": "/cmd_vel_nav"},
            {"output_cmd_topic": "/cmd_vel"},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="sim"),
            DeclareLaunchArgument(
                "world",
                default_value=f"{package_share}/worlds/three_room_house.world",
            ),
            DeclareLaunchArgument("with_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=f"{package_share}/rviz/mecamind_mapping.rviz",
            ),
            DeclareLaunchArgument("lidar_noise_std", default_value="0.01"),
            DeclareLaunchArgument("lidar_dropout_prob", default_value="0.0"),
            DeclareLaunchArgument("odom_xy_noise_std", default_value="0.0"),
            DeclareLaunchArgument("odom_yaw_noise_std", default_value="0.0"),
            DeclareLaunchArgument("cmd_latency_sec", default_value="0.0"),
            Node(
                package="mecamind_tools",
                executable="mecamind_simulator_node",
                name="mecamind_simulator",
                output="screen",
                remappings=[
                    ("/scan_raw", "/scan"),
                    ("/imu/data_raw", "/imu/data"),
                    ("/controller/cmd_vel", "/cmd_vel"),
                ],
                parameters=[
                    {"world_path": world},
                    {"initial_x": 0.0},
                    {"initial_y": 0.0},
                    {"initial_yaw": 0.0},
                    {"lidar_noise_std": ParameterValue(lidar_noise_std, value_type=float)},
                    {"lidar_dropout_prob": ParameterValue(lidar_dropout_prob, value_type=float)},
                    {"odom_xy_noise_std": ParameterValue(odom_xy_noise_std, value_type=float)},
                    {"odom_yaw_noise_std": ParameterValue(odom_yaw_noise_std, value_type=float)},
                    {"cmd_latency_sec": ParameterValue(cmd_latency_sec, value_type=float)},
                ],
            ),
            slam_toolbox,
            Node(
                package="mecamind_tools",
                executable="mission_brief_node",
                name="mecamind_mapping_brief",
                output="screen",
                parameters=[
                    {
                        "mode": mode,
                        "mission": "manual_mapping_assist",
                        "output_topic": "/mecamind/mission_brief",
                        "world_path": world,
                    }
                ],
            ),
            slam_activator,
            safety_gate,
            rviz_node,
        ]
    )
