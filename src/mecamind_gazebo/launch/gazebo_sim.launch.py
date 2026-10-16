"""Gazebo 后端的真正启动逻辑（被 bringup 各入口包含）。

【这个文件启动了什么】
1. Gazebo Harmonic 仿真器本体（gz sim），加载三室户世界，
   机器人模型直接写在世界文件里（不需要单独 spawn）；
2. robot_state_publisher：读 URDF(xacro)，把各连杆的固定 TF
   （base_footprint->base_link->lidar_link 等）发布出来；
3. ros_gz_bridge：Gazebo 与 ROS 2 是两套独立的通信系统，
   桥接器按 config/bridge.yaml 的映射表把 Gazebo 话题
   （时钟、激光、里程计、速度指令等）翻译成 ROS 2 话题。

【初学者阅读提示】
- use_gui:=false 时用 `-s` 参数以纯服务器（无窗口）模式跑 Gazebo，
  CI/无头验收就是这么运行的；
- DISPLAY 环境变量处理是为 WSLg 准备的：WSLg 的 X server 固定在 :0。
"""

from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_share = get_package_share_directory("mecamind_gazebo")
    description_share = get_package_share_directory("mecamind_description")
    ros_gz_share = get_package_share_directory("ros_gz_sim")

    world = LaunchConfiguration("world")
    use_gui = LaunchConfiguration("use_gui")
    display = LaunchConfiguration("display")
    world_default = f"{gazebo_share}/worlds/three_room_house.sdf"
    model_path = f"{gazebo_share}/models"
    # WSLg 环境下 X server 的套接字固定是 /tmp/.X11-unix/X0（即 :0）。
    # 有些 WSL 终端会把 DISPLAY 误设成 Windows 主机 IP，导致 GUI 连不上，
    # 所以这里优先探测 :0，避免学员被 DISPLAY 问题卡住。
    display_default = (
        ":0"
        if Path("/tmp/.X11-unix/X0").exists()
        else os.environ.get("DISPLAY", "")
    )
    # Command 替换：launch 启动时才执行 `xacro <文件>`，把 xacro 宏
    # 展开成完整 URDF 字符串，作为 robot_description 参数传给
    # robot_state_publisher。
    robot_description = Command(
        ["xacro ", f"{description_share}/urdf/mecamind_mecanum.urdf.xacro"]
    )

    # gz 参数含义：-r 立即开始仿真（不暂停），-v 2 日志级别，
    # -s 只跑服务器不开 GUI 窗口。GUI/无头两个版本按 use_gui 二选一。
    gz_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{ros_gz_share}/launch/gz_sim.launch.py"),
        launch_arguments={"gz_args": ["-r -v 2 ", world]}.items(),
        condition=IfCondition(use_gui),
    )
    gz_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{ros_gz_share}/launch/gz_sim.launch.py"),
        launch_arguments={"gz_args": ["-r -s -v 2 ", world]}.items(),
        condition=UnlessCondition(use_gui),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("world", default_value=world_default),
            DeclareLaunchArgument(
                "display",
                default_value=display_default,
                description="X11 display used by Gazebo GUI; WSLg normally uses :0.",
            ),
            SetEnvironmentVariable("DISPLAY", display),
            # 把本包的 models/ 目录加进 Gazebo 的模型搜索路径，
            # 世界文件里 <include><uri>model://mecamind_mecanum 才找得到。
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH",
                [model_path, ":", EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")],
            ),
            gz_with_gui,
            gz_headless,
            # 发布机器人"骨架"TF：根据 URDF 中各连杆的固定装配关系，
            # 广播 base_footprint->base_link->lidar_link 等静态变换。
            # SLAM/Nav2 需要靠它把激光数据从 lidar_link 换算到底盘坐标。
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {"robot_description": robot_description, "use_sim_time": True}
                ],
            ),
            # Gazebo<->ROS 2 话题桥：具体桥哪些话题、什么消息类型、
            # 哪个方向，全部写在 config/bridge.yaml 里（重点配置文件）。
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="mecamind_gz_bridge",
                output="screen",
                parameters=[
                    {
                        "config_file": f"{gazebo_share}/config/bridge.yaml",
                        "use_sim_time": True,
                    }
                ],
            ),
        ]
    )
