"""轻量 2D 仿真后端的启动文件（backend:=lite 时被上层 launch 包含）。

【这个文件是干什么的】
只启动一个节点：mecamind_simulator_node（自研的纯 Python 2D 仿真器）。
它一个节点就顶替了 Gazebo 的全部职责，对外发布与 Gazebo 完全相同的
接口：/clock、/scan、/odom、/imu/data、TF odom->base_footprint，
并订阅 /cmd_vel 驱动机器人。

【为什么需要它】
- Gazebo 吃 CPU/GPU，课堂电脑或 CI 服务器跑不动时用它兜底；
- 接口与 Gazebo 一致，上层 SLAM/Nav2 代码一行都不用改。

【初学者阅读提示】
注意 remappings：仿真器节点内部用的是 /scan_raw 等"原始"名字，
这里通过话题重映射统一改成全项目约定的 /scan、/imu/data、/cmd_vel。
重映射是 ROS 2 的重要机制——不改代码就能改话题接线。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    tools_share = get_package_share_directory("mecamind_tools")
    world = LaunchConfiguration("world")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")

    return LaunchDescription(
        [
            # 世界文件描述墙体/障碍几何，与 Gazebo 的三室户世界一致，
            # 保证两个后端"看到"的环境相同。
            DeclareLaunchArgument(
                "world",
                default_value=f"{tools_share}/worlds/three_room_house.world",
            ),
            # 默认出生点是历史遗留值；建图/导航入口会显式传 (0,0,0)
            # 覆盖它，让 lite 与 Gazebo 的出生位姿保持一致。
            DeclareLaunchArgument("initial_x", default_value="-3.2"),
            DeclareLaunchArgument("initial_y", default_value="-2.4"),
            DeclareLaunchArgument("initial_yaw", default_value="0.0"),
            Node(
                package="mecamind_tools",
                executable="mecamind_simulator_node",
                name="mecamind_lite_simulator",
                output="screen",
                # 话题重映射：把节点内部话题名换成全项目统一的名字。
                remappings=[
                    ("/scan_raw", "/scan"),
                    ("/imu/data_raw", "/imu/data"),
                    ("/controller/cmd_vel", "/cmd_vel"),
                ],
                parameters=[
                    {"world_path": world},
                    # launch 参数本质是字符串，ParameterValue(value_type=float)
                    # 显式声明成浮点，否则节点端会收到 "0.0" 这样的字符串。
                    {"initial_x": ParameterValue(initial_x, value_type=float)},
                    {"initial_y": ParameterValue(initial_y, value_type=float)},
                    {"initial_yaw": ParameterValue(initial_yaw, value_type=float)},
                ],
            ),
        ]
    )
