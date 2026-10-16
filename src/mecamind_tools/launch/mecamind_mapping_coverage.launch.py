"""全自动探索建图入口（轻量仿真 + SLAM + 自动探索，进阶内容）。

与半自动的"按预设路线开车"不同，这里由 mecamind_auto_explore
根据激光与地图自主决定往哪儿开（frontier 探索），
无需人工路线即可覆盖整个环境。课堂主路径是
mecamind_bringup/mapping.launch.py，本文件供探索算法实验用。
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
    map_save_path = LaunchConfiguration("map_save_path")
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

    auto_explore = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="mecamind_tools",
                executable="mecamind_auto_explore",
                name="mecamind_auto_explore",
                output="screen",
                remappings=[("/scan_raw", "/scan")],
                parameters=[
                    {"auto_save_map": True},
                    {"map_save_path": map_save_path},
                    {"startup_wait_sec": 8.0},
                    {"progress_report_interval": 10.0},
                    {"linear_speed": 0.07},
                    {"linear_speed_slow": 0.04},
                    {"lateral_speed": 0.06},
                    {"angular_speed": 0.45},
                    {"angular_speed_slow": 0.25},
                    {"cmd_topic": "/cmd_vel_nav"},
                    {"cross_goal_timeout_sec": 75.0},
                    {"cross_goal_stall_sec": 30.0},
                    {"checkpoint_save_interval_sec": 90.0},
                ],
            )
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
                    {
                        "node_names": ["slam_toolbox"],
                        "publish_initial_pose": False,
                    }
                ],
            )
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="mecamind_mapping_coverage_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(with_rviz),
    )

    safety_gate = Node(
        package="mecamind_tools",
        executable="mecamind_safety_gate",
        name="mecamind_mapping_coverage_safety_gate",
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
            DeclareLaunchArgument(
                "map_save_path",
                default_value="maps/mecamind_three_room_map_coverage",
            ),
            DeclareLaunchArgument("with_rviz", default_value="false"),
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
                    {"initial_x": -3.2},
                    {"initial_y": -2.4},
                    {"initial_yaw": 0.0},
                    {"lidar_noise_std": ParameterValue(lidar_noise_std, value_type=float)},
                    {"lidar_dropout_prob": ParameterValue(lidar_dropout_prob, value_type=float)},
                    {"odom_xy_noise_std": ParameterValue(odom_xy_noise_std, value_type=float)},
                    {"odom_yaw_noise_std": ParameterValue(odom_yaw_noise_std, value_type=float)},
                    {"cmd_latency_sec": ParameterValue(cmd_latency_sec, value_type=float)},
                ],
            ),
            slam_toolbox,
            slam_activator,
            Node(
                package="mecamind_tools",
                executable="mission_brief_node",
                name="mecamind_mapping_coverage_brief",
                output="screen",
                parameters=[
                    {
                        "mode": mode,
                        "mission": "mapping",
                        "output_topic": "/mecamind/mission_brief",
                        "world_path": world,
                    }
                ],
            ),
            auto_explore,
            safety_gate,
            rviz_node,
        ]
    )
