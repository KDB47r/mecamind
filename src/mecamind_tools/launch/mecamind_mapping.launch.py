"""半自动建图入口（仅轻量仿真后端，历史版本）。

功能与 mecamind_bringup/mapping.launch.py（auto_route:=true）相同，
但不支持 Gazebo 后端。课堂请优先用 bringup 的统一入口；
本文件保留用于轻量后端的独立调试。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("mecamind_tools")
    mode = LaunchConfiguration("mode")
    world = LaunchConfiguration("world")
    map_save_path = LaunchConfiguration("map_save_path")
    manual_save_path = LaunchConfiguration("manual_save_path")
    route_report_path = LaunchConfiguration("route_report_path")
    route_file = LaunchConfiguration("route_file")
    checkpoint_save_interval_sec = LaunchConfiguration("checkpoint_save_interval_sec")
    checkpoint_keep_snapshots = LaunchConfiguration("checkpoint_keep_snapshots")
    shutdown_on_route_complete = LaunchConfiguration("shutdown_on_route_complete")
    waypoint_timeout_sec = LaunchConfiguration("waypoint_timeout_sec")
    waypoint_stall_sec = LaunchConfiguration("waypoint_stall_sec")
    waypoint_min_progress = LaunchConfiguration("waypoint_min_progress")
    with_rviz = LaunchConfiguration("with_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    lidar_noise_std = LaunchConfiguration("lidar_noise_std")
    lidar_dropout_prob = LaunchConfiguration("lidar_dropout_prob")
    odom_xy_noise_std = LaunchConfiguration("odom_xy_noise_std")
    odom_yaw_noise_std = LaunchConfiguration("odom_yaw_noise_std")
    cmd_latency_sec = LaunchConfiguration("cmd_latency_sec")
    cmd_timeout_sec = LaunchConfiguration("cmd_timeout_sec")
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

    route_driver_node = Node(
        package="mecamind_tools",
        executable="mecamind_mapping_route_driver",
        name="mecamind_mapping_route_driver",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"route_file": route_file},
            {"map_save_path": map_save_path},
            {"manual_save_path": manual_save_path},
            {"route_report_path": route_report_path},
            {"cmd_topic": "/cmd_vel_nav"},
            {"shutdown_after_route": ParameterValue(shutdown_on_route_complete, value_type=bool)},
            {
                "checkpoint_save_interval_sec": ParameterValue(
                    checkpoint_save_interval_sec,
                    value_type=float,
                )
            },
            {"checkpoint_keep_snapshots": ParameterValue(checkpoint_keep_snapshots, value_type=bool)},
            {"waypoint_timeout_sec": ParameterValue(waypoint_timeout_sec, value_type=float)},
            {"waypoint_stall_sec": ParameterValue(waypoint_stall_sec, value_type=float)},
            {"waypoint_min_progress": ParameterValue(waypoint_min_progress, value_type=float)},
        ],
    )

    route_driver = TimerAction(
        period=10.0,
        actions=[route_driver_node],
    )

    shutdown_when_route_done = RegisterEventHandler(
        OnProcessExit(
            target_action=route_driver_node,
            on_exit=[EmitEvent(event=Shutdown(reason="mapping route complete"))],
        ),
        condition=IfCondition(shutdown_on_route_complete),
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
                    },
                    {
                        "publish_initial_pose": False,
                    },
                ],
            )
        ],
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
            {"cmd_timeout_sec": ParameterValue(cmd_timeout_sec, value_type=float)},
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
                default_value="maps/mecamind_three_room_map",
            ),
            DeclareLaunchArgument("manual_save_path", default_value=""),
            DeclareLaunchArgument("route_report_path", default_value=""),
            DeclareLaunchArgument(
                "route_file",
                default_value=f"{package_share}/config/mecamind_mapping_route.yaml",
            ),
            DeclareLaunchArgument("checkpoint_save_interval_sec", default_value="90.0"),
            DeclareLaunchArgument("checkpoint_keep_snapshots", default_value="false"),
            DeclareLaunchArgument("shutdown_on_route_complete", default_value="true"),
            DeclareLaunchArgument("waypoint_timeout_sec", default_value="50.0"),
            DeclareLaunchArgument("waypoint_stall_sec", default_value="18.0"),
            DeclareLaunchArgument("waypoint_min_progress", default_value="0.08"),
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
            DeclareLaunchArgument("cmd_timeout_sec", default_value="0.6"),
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
                        "mission": "mapping",
                        "output_topic": "/mecamind/mission_brief",
                        "world_path": world,
                    }
                ],
            ),
            slam_activator,
            safety_gate,
            route_driver,
            shutdown_when_route_done,
            rviz_node,
        ]
    )
