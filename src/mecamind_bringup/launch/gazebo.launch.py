"""Gazebo 后端的顶层快捷入口（第一课使用）。

【这个文件是干什么的】
它只是个"转发器"：把真正的启动逻辑委托给
mecamind_gazebo/launch/gazebo_sim.launch.py（那边负责起 Gazebo
服务器、生成机器人、桥接话题、发布 TF）。

放在 mecamind_bringup 包里是为了让学员记一个统一入口：
    ros2 launch mecamind_bringup gazebo.launch.py
而不必关心底层包结构。use_gui:=false 可无头运行（不开 Gazebo 窗口）。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    gazebo_share = get_package_share_directory("mecamind_gazebo")
    use_gui = LaunchConfiguration("use_gui")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_gui",
                default_value="true",
                description="Start the Gazebo graphical client when true.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    f"{gazebo_share}/launch/gazebo_sim.launch.py"
                ),
                launch_arguments={"use_gui": use_gui}.items(),
            ),
        ]
    )
