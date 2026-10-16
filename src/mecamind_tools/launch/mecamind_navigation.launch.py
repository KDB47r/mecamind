"""Nav2 导航入口（仅轻量仿真后端，历史版本）。

功能与 mecamind_bringup/navigation.launch.py 类似但不支持 Gazebo。
课堂请优先用 bringup 的统一入口；本文件保留用于轻量后端独立调试。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("mecamind_tools")
    mode = LaunchConfiguration("mode")
    mission = LaunchConfiguration("mission")
    world = LaunchConfiguration("world")
    map_file = LaunchConfiguration("map_file")
    use_saved_map = LaunchConfiguration("use_saved_map")
    params_file = LaunchConfiguration("params_file")
    waypoints_file = LaunchConfiguration("waypoints_file")
    with_rviz = LaunchConfiguration("with_rviz")
    enable_auto_patrol = LaunchConfiguration("enable_auto_patrol")
    rviz_config = LaunchConfiguration("rviz_config")
    lidar_noise_std = LaunchConfiguration("lidar_noise_std")
    lidar_dropout_prob = LaunchConfiguration("lidar_dropout_prob")
    odom_xy_noise_std = LaunchConfiguration("odom_xy_noise_std")
    odom_yaw_noise_std = LaunchConfiguration("odom_yaw_noise_std")
    cmd_latency_sec = LaunchConfiguration("cmd_latency_sec")

    nav2_common = [params_file, {"use_sim_time": True}]

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=nav2_common,
    )
    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=nav2_common,
    )
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=nav2_common,
    )
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=nav2_common,
    )
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=nav2_common,
    )
    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=nav2_common,
    )
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=nav2_common,
        remappings=[("cmd_vel_smoothed", "/controller/cmd_vel_nav")],
    )
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=nav2_common,
    )

    map_publisher = Node(
        package="mecamind_tools",
        executable="mecamind_map_publisher_node",
        name="mecamind_map_publisher",
        output="screen",
        parameters=[{"world_path": world}],
        condition=UnlessCondition(use_saved_map),
    )

    saved_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_file},
            {"use_sim_time": True},
        ],
        condition=IfCondition(use_saved_map),
    )

    simulator = Node(
        package="mecamind_tools",
        executable="mecamind_simulator_node",
        name="mecamind_simulator",
        output="screen",
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
    )

    lifecycle_activator = TimerAction(
        period=8.0,
        condition=UnlessCondition(use_saved_map),
        actions=[
            Node(
                package="mecamind_tools",
                executable="mecamind_lifecycle_activator",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {
                        "node_names": [
                            "amcl",
                            "controller_server",
                            "smoother_server",
                            "planner_server",
                            "behavior_server",
                            "bt_navigator",
                            "waypoint_follower",
                            "velocity_smoother",
                        ]
                    },
                    {"initial_pose_x": -3.2},
                    {"initial_pose_y": -2.4},
                    {"initial_pose_yaw": 0.0},
                    {"publish_initial_pose": True},
                ],
            )
        ],
    )

    saved_map_lifecycle_activator = TimerAction(
        period=8.0,
        condition=IfCondition(use_saved_map),
        actions=[
            Node(
                package="mecamind_tools",
                executable="mecamind_lifecycle_activator",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {
                        "node_names": [
                            "map_server",
                            "amcl",
                            "controller_server",
                            "smoother_server",
                            "planner_server",
                            "behavior_server",
                            "bt_navigator",
                            "waypoint_follower",
                            "velocity_smoother",
                        ]
                    },
                    {"initial_pose_x": -3.2},
                    {"initial_pose_y": -2.4},
                    {"initial_pose_yaw": 0.0},
                    {"publish_initial_pose": True},
                ],
            )
        ],
    )

    patrol = TimerAction(
        period=16.0,
        actions=[
            Node(
                package="mecamind_tools",
                executable="mecamind_waypoint_patrol",
                name="mecamind_waypoint_patrol",
                output="screen",
                parameters=[{"waypoints_file": waypoints_file}],
            )
        ],
        condition=IfCondition(enable_auto_patrol),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="mecamind_navigation_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(with_rviz),
    )

    safety_gate = Node(
        package="mecamind_tools",
        executable="mecamind_safety_gate",
        name="mecamind_safety_gate",
        output="screen",
        parameters=[
            {"scan_topic": "/scan_raw"},
            {"input_cmd_topic": "/controller/cmd_vel_nav"},
            {"output_cmd_topic": "/controller/cmd_vel"},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="sim"),
            DeclareLaunchArgument("mission", default_value="navigation"),
            DeclareLaunchArgument(
                "world",
                default_value=f"{package_share}/worlds/three_room_house.world",
            ),
            DeclareLaunchArgument("use_saved_map", default_value="false"),
            DeclareLaunchArgument("map_file", default_value=""),
            DeclareLaunchArgument(
                "params_file",
                default_value=f"{package_share}/config/mecamind_nav2_params.yaml",
            ),
            DeclareLaunchArgument(
                "waypoints_file",
                default_value=f"{package_share}/config/mecamind_patrol_waypoints.yaml",
            ),
            DeclareLaunchArgument("with_rviz", default_value="false"),
            DeclareLaunchArgument(
                "enable_auto_patrol",
                default_value="true",
                description="Start waypoint_patrol after Nav2 is active. Disable for mission_executor demos.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=f"{package_share}/rviz/mecamind_navigation.rviz",
            ),
            DeclareLaunchArgument("lidar_noise_std", default_value="0.0"),
            DeclareLaunchArgument("lidar_dropout_prob", default_value="0.0"),
            DeclareLaunchArgument("odom_xy_noise_std", default_value="0.0"),
            DeclareLaunchArgument("odom_yaw_noise_std", default_value="0.0"),
            DeclareLaunchArgument("cmd_latency_sec", default_value="0.0"),
            simulator,
            map_publisher,
            saved_map_server,
            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            safety_gate,
            amcl,
            Node(
                package="mecamind_tools",
                executable="mission_brief_node",
                name="mecamind_navigation_brief",
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
            lifecycle_activator,
            saved_map_lifecycle_activator,
            patrol,
            rviz_node,
        ]
    )
