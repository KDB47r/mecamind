"""真机部署入口（预留）。

面向未来接真实底盘/雷达的场景：不启动仿真器，假设驱动层
已由硬件侧提供 /scan、/odom 并订阅 /cmd_vel，本文件只拉起
上层的任务/安全/调度节点。当前课程阶段仅供参考。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("mecamind_tools")
    mode = LaunchConfiguration("mode")
    mission = LaunchConfiguration("mission")
    world = LaunchConfiguration("world")
    params_file = LaunchConfiguration("params_file")
    waypoints_file = LaunchConfiguration("waypoints_file")

    nav2_common = [params_file, {"use_sim_time": False}]

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
    )

    lifecycle_activator = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="mecamind_tools",
                executable="mecamind_lifecycle_activator",
                output="screen",
                parameters=[
                    {"use_sim_time": False},
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

    patrol = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="mecamind_tools",
                executable="mecamind_waypoint_patrol",
                name="mecamind_waypoint_patrol",
                output="screen",
                parameters=[{"waypoints_file": waypoints_file}],
            )
        ],
    )

    safety_gate = Node(
        package="mecamind_tools",
        executable="mecamind_safety_gate",
        name="mecamind_safety_gate",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"scan_topic": "/scan_raw"},
            {"input_cmd_topic": "/controller/cmd_vel_nav"},
            {"output_cmd_topic": "/controller/cmd_vel"},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="real"),
            DeclareLaunchArgument("mission", default_value="navigation"),
            DeclareLaunchArgument(
                "world",
                default_value=f"{package_share}/worlds/three_room_house.world",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=f"{package_share}/config/mecamind_nav2_params.yaml",
            ),
            DeclareLaunchArgument(
                "waypoints_file",
                default_value=f"{package_share}/config/mecamind_patrol_waypoints.yaml",
            ),
            map_publisher,
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
                name="mecamind_real_brief",
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
            patrol,
        ]
    )
