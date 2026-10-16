"""第二课（10.3.2）SLAM 建图的统一启动入口。

【这个文件是干什么的】
一条命令拉起"仿真 + SLAM + 安全门 + （可选）自动路线 + RViz"整套建图系统：

    ros2 launch mecamind_bringup mapping.launch.py backend:=gazebo|lite

【为什么两个后端能共用同一个入口】
两个仿真后端对外暴露完全一致的接口（/scan、/odom、/cmd_vel、
TF odom->base_footprint），SLAM Toolbox 和下面这些建图工具
根本不关心底下跑的是哪个仿真器：

- backend:=gazebo  Gazebo Harmonic 物理仿真（课堂主路径，支持 use_gui）
- backend:=lite    自研轻量 2D 仿真器（兜底 / CI 无头测试路径）

【两种建图模式】
- 手动建图（默认）：自己用键盘遥控开车，建完手动存图。
- 半自动建图（auto_route:=true）：路线驱动器按预设路点带机器人
  绕完三室户，自动存图并写路线报告，然后整套系统自动关闭。

【初学者阅读提示】
launch 文件本质是"用 Python 描述要启动哪些节点、传什么参数"。
重点看 generate_launch_description() 里每个 Node(...) 的
package/executable/parameters 三要素，以及 IfCondition 如何按
launch 参数决定某个节点启不启动。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # share 目录 = colcon 安装后各包的资源目录（install/<pkg>/share/<pkg>），
    # 配置文件、世界文件、RViz 配置都从这里找，而不是源码目录。
    bringup_share = get_package_share_directory("mecamind_bringup")
    gazebo_share = get_package_share_directory("mecamind_gazebo")
    tools_share = get_package_share_directory("mecamind_tools")

    # LaunchConfiguration 是"占位符"：真正的值要到 ros2 launch 解析
    # 命令行参数（如 backend:=lite）时才确定。
    backend = LaunchConfiguration("backend")
    use_gui = LaunchConfiguration("use_gui")
    with_rviz = LaunchConfiguration("with_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    auto_route = LaunchConfiguration("auto_route")
    map_save_path = LaunchConfiguration("map_save_path")
    route_report_path = LaunchConfiguration("route_report_path")
    route_file = LaunchConfiguration("route_file")
    shutdown_on_route_complete = LaunchConfiguration("shutdown_on_route_complete")

    # IfCondition + PythonExpression：把 launch 参数拼成一段 Python
    # 表达式，运行时求值决定条件是否成立（launch 里的 if/else 写法）。
    is_gazebo = IfCondition(PythonExpression(["'", backend, "' == 'gazebo'"]))
    is_lite = IfCondition(PythonExpression(["'", backend, "' == 'lite'"]))
    # 只有"开自动路线"且"路线跑完要自动关"同时为真时才注册退出事件。
    auto_route_and_shutdown = IfCondition(
        PythonExpression(
            [
                "'",
                auto_route,
                "'.lower() in ('true', '1') and '",
                shutdown_on_route_complete,
                "'.lower() in ('true', '1')",
            ]
        )
    )

    # IncludeLaunchDescription = 在本 launch 里"嵌套"另一个 launch 文件，
    # 类似函数调用；两个后端各自的启动细节都封装在被包含的文件里。
    gazebo_backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{gazebo_share}/launch/gazebo_sim.launch.py"),
        launch_arguments={"use_gui": use_gui}.items(),
        condition=is_gazebo,
    )
    # 轻量仿真的机器人出生在原点 (0,0)，与 Gazebo 里的出生位姿保持一致，
    # 这样两个后端可以共用同一份建图路线文件（路点坐标才对得上）。
    lite_backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{bringup_share}/launch/lite_sim.launch.py"),
        launch_arguments={
            "initial_x": "0.0",
            "initial_y": "0.0",
            "initial_yaw": "0.0",
        }.items(),
        condition=is_lite,
    )

    # SLAM 核心节点：订阅 /scan 与 TF odom->base_footprint，
    # 输出 /map（栅格地图）和 TF map->odom（漂移修正量）。
    # 参数先加载 YAML 文件，再用后面的字典覆盖个别项（后者优先）。
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            f"{tools_share}/config/mecamind_slam_toolbox_params.yaml",
            {
                # use_sim_time=True：节点时钟改用 /clock（仿真时间），
                # 否则激光时间戳和 TF 时间戳对不上，SLAM 会丢数据。
                "use_sim_time": True,
                "autostart": True,
                "use_lifecycle_manager": False,
                "scan_topic": "/scan",
            },
        ],
    )

    # slam_toolbox 是 lifecycle（生命周期）节点，启动后停在
    # unconfigured 状态，必须有人调 configure->activate 它才开始干活。
    # 这里延迟 4 秒（等仿真时钟先跑起来）再由激活器统一拉起。
    slam_activator = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="mecamind_tools",
                executable="mecamind_lifecycle_activator",
                output="screen",
                parameters=[
                    {"node_names": ["slam_toolbox"]},
                    # 建图阶段不存在 AMCL，没有初始位姿要发。
                    {"publish_initial_pose": False},
                ],
            )
        ],
    )

    # 安全门：遥控/路线驱动器发出的速度先进 /cmd_vel_nav，
    # 安全门用激光检查前方障碍后，把"净化"过的速度转发到 /cmd_vel
    # （两个后端真正执行的都是 /cmd_vel）。相当于速度链路上的保险丝。
    safety_gate = Node(
        package="mecamind_tools",
        executable="mecamind_safety_gate",
        name="mecamind_mapping_safety_gate",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"scan_topic": "/scan"},
            {"input_cmd_topic": "/cmd_vel_nav"},
            {"output_cmd_topic": "/cmd_vel"},
            # Gazebo RTF 偏低时激光墙钟间隔变长，避免误判 scan_stale 刹死。
            {"scan_timeout_sec": 2.0},
            {"cmd_timeout_sec": 1.5},
        ],
    )

    # 半自动建图的"司机"：按路线文件里的路点依次开车，跑完自动存图。
    route_driver_node = Node(
        package="mecamind_tools",
        executable="mecamind_mapping_route_driver",
        name="mecamind_mapping_route_driver",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"route_file": route_file},
            {"map_save_path": map_save_path},
            {"route_report_path": route_report_path},
            {"cmd_topic": "/cmd_vel_nav"},
            {"scan_topic": "/scan"},
            # 下面三个超时都按"仿真时间"计。Gazebo 在 WSL2 上 RTF
            # （实时因子）常低于 1，给宽一些避免误判卡死/超时。
            {"startup_delay_sec": 12.0},
            {"waypoint_timeout_sec": 80.0},
            {"waypoint_stall_sec": 30.0},
            {
                "shutdown_after_route": ParameterValue(
                    shutdown_on_route_complete, value_type=bool
                )
            },
        ],
    )

    # 延迟 10 秒再上路：等 SLAM 激活、TF/激光都稳定后再开车，
    # 否则一开始的运动没有地图参考，容易画歪。仅 auto_route 时启用。
    route_driver = TimerAction(
        period=10.0,
        actions=[route_driver_node],
        condition=IfCondition(auto_route),
    )

    # 事件处理器：监听路线驱动器进程退出，一退出就向整个 launch
    # 发 Shutdown 事件，把仿真、SLAM、RViz 全部一起收掉。
    # 这就是 CP2 无头验收能"跑完自动结束"的机关。
    shutdown_when_route_done = RegisterEventHandler(
        OnProcessExit(
            target_action=route_driver_node,
            on_exit=[EmitEvent(event=Shutdown(reason="mapping route complete"))],
        ),
        condition=auto_route_and_shutdown,
    )

    # RViz 可视化：-d 指定预置布局（地图 + 激光 + TF），
    # 无头验收（CP2）时传 with_rviz:=false 跳过它。
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="mecamind_mapping_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(with_rviz),
    )

    # LaunchDescription 里先声明所有可调参数（DeclareLaunchArgument，
    # 命令行用 名字:=值 覆盖默认值），再列出要启动的动作/节点。
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend",
                default_value="gazebo",
                description="Simulation backend: gazebo or lite.",
            ),
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("with_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=f"{tools_share}/rviz/mecamind_mapping.rviz",
            ),
            DeclareLaunchArgument(
                "auto_route",
                default_value="false",
                description="Run the semi-automatic mapping route driver.",
            ),
            DeclareLaunchArgument(
                "map_save_path",
                default_value="maps/mecamind_three_room_map",
                description="Map save prefix, relative to the launch directory.",
            ),
            DeclareLaunchArgument("route_report_path", default_value=""),
            DeclareLaunchArgument(
                "route_file",
                default_value=f"{tools_share}/config/mecamind_mapping_route.yaml",
            ),
            DeclareLaunchArgument("shutdown_on_route_complete", default_value="true"),
            gazebo_backend,
            lite_backend,
            slam_toolbox,
            slam_activator,
            safety_gate,
            route_driver,
            shutdown_when_route_done,
            rviz_node,
        ]
    )
