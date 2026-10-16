"""第三课（10.3.3）Nav2 导航的统一启动入口。

【这个文件是干什么的】
一条命令拉起"仿真 + 地图服务 + AMCL 定位 + Nav2 全家桶 +
安全门 + 任务执行器 + RViz"整套导航系统：

    ros2 launch mecamind_bringup navigation.launch.py backend:=gazebo|lite

【为什么两个后端能共用】
两个仿真后端对外接口完全一致（/scan、/odom、/cmd_vel、
TF odom->base_footprint），下面整个 Nav2 栈不关心底下是谁：

- backend:=gazebo  Gazebo Harmonic 物理仿真（课堂主路径，支持 use_gui）
- backend:=lite    自研轻量 2D 仿真器（兜底 / CI 无头测试路径）

【速度指令链路（重点理解）】
Nav2 算出的速度并不直接给机器人，而是层层把关：

  controller_server/behavior_server --(cmd_vel_raw)--> velocity_smoother
  velocity_smoother --(/cmd_vel_nav)--> safety_gate --(/cmd_vel)--> 机器人

velocity_smoother 负责加减速平滑（防急启急停），
safety_gate 用激光做最后一道避障截断。这条链全靠话题重映射拼接，
所以看懂本文件里每个 remappings 就看懂了整条链。

【地图与定位】
用第二课（CP2）建好的 PGM/YAML 地图，由 nav2 map_server 发布；
AMCL 在这张图上做粒子滤波定位，运动模型选了适配麦轮全向底盘的
omni 模型（参数在 mecamind_nav2_params.yaml）。

【初学者阅读提示】
Nav2 的所有核心节点都是 lifecycle（生命周期）节点，启动后不会
自动工作，要靠最下面的 lifecycle_activator 依次 configure/activate。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# 需要被激活器依次 configure->activate 的 Nav2 lifecycle 节点清单。
# 顺序有讲究：map_server 先有地图，amcl 才能定位，其余才能规划/控制。
NAV2_LIFECYCLE_NODES = [
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


def generate_launch_description():
    bringup_share = get_package_share_directory("mecamind_bringup")
    gazebo_share = get_package_share_directory("mecamind_gazebo")
    tools_share = get_package_share_directory("mecamind_tools")

    backend = LaunchConfiguration("backend")
    use_gui = LaunchConfiguration("use_gui")
    with_rviz = LaunchConfiguration("with_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    map_file = LaunchConfiguration("map_file")
    params_file = LaunchConfiguration("params_file")
    named_goals_file = LaunchConfiguration("named_goals_file")
    waypoints_file = LaunchConfiguration("waypoints_file")

    is_gazebo = IfCondition(PythonExpression(["'", backend, "' == 'gazebo'"]))
    is_lite = IfCondition(PythonExpression(["'", backend, "' == 'lite'"]))

    # 按 backend 参数二选一地包含对应后端的 launch 文件。
    gazebo_backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{gazebo_share}/launch/gazebo_sim.launch.py"),
        launch_arguments={"use_gui": use_gui}.items(),
        condition=is_gazebo,
    )
    # lite 机器人出生在原点：第二课的地图就是从原点开始建的，
    # 因此 map 坐标系与世界坐标系近似重合，初始位姿 (0,0,0) 才成立。
    lite_backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{bringup_share}/launch/lite_sim.launch.py"),
        launch_arguments={
            "initial_x": "0.0",
            "initial_y": "0.0",
            "initial_yaw": "0.0",
        }.items(),
        condition=is_lite,
    )

    # 所有 Nav2 节点共用的参数：先加载统一 YAML，再强制仿真时间。
    nav2_common = [params_file, {"use_sim_time": True}]

    # 地图服务：把第二课存的 PGM/YAML 地图以 /map 话题发布出来
    # （latched 语义，后来的订阅者也能立刻拿到地图）。
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_file}, {"use_sim_time": True}],
    )
    # AMCL 粒子滤波定位：拿激光 /scan 对着已知地图匹配，
    # 输出 TF map->odom（即"里程计漂移的修正量"）和 /amcl_pose。
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=nav2_common,
    )
    # controller/behavior 的原始输出统一走 /cmd_vel_raw，经速度平滑与安全门
    # 之后才落到机器人真正订阅的 /cmd_vel，避免绕过限速与避障。
    # 局部控制器（DWB）：沿全局路径逐周期计算即时速度，带局部代价地图避障。
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=nav2_common,
        remappings=[("cmd_vel", "/cmd_vel_raw")],
    )
    # 路径平滑器：把全局规划出的折线路径打磨得更平顺。
    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=nav2_common,
    )
    # 全局规划器：在全局代价地图上按 A*/NavFn 搜一条从当前位置到目标的路径。
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=nav2_common,
    )
    # 恢复行为服务：卡住时执行原地旋转、后退、等待等"自救"动作，
    # 它也会直接发速度，所以同样要重映射进 /cmd_vel_raw 这条受控链。
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=nav2_common,
        remappings=[("cmd_vel", "/cmd_vel_raw")],
    )
    # 行为树导航器：Nav2 的"总指挥"，对外提供 NavigateToPose action，
    # 内部用行为树协调 规划->控制->失败恢复 的完整流程。
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=nav2_common,
    )
    # 多路点跟随：按顺序把一串路点逐个交给 bt_navigator（巡航用）。
    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=nav2_common,
    )
    # 速度平滑器：限制加速度/加加速度，防止速度指令跳变损伤电机或打滑。
    # 输入接 /cmd_vel_raw，输出改名为 /cmd_vel_nav 交给安全门。
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=nav2_common,
        remappings=[
            ("cmd_vel", "/cmd_vel_raw"),
            ("cmd_vel_smoothed", "/cmd_vel_nav"),
        ],
    )

    # 安全门：速度链路最后一道关卡。用 /scan 检查前方障碍，
    # 必要时衰减或清零速度，再转发到机器人真正执行的 /cmd_vel。
    safety_gate = Node(
        package="mecamind_tools",
        executable="mecamind_safety_gate",
        name="mecamind_nav_safety_gate",
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

    # 任务执行器：订阅 /mecamind/task_command（JSON 字符串指令，如
    # {"intent":"navigate","target":"living_room"}），解析后通过
    # NavigateToPose action 驱动 Nav2，并向 /mecamind/mission_state
    # 汇报进度。它是"业务层"，Nav2 是"执行层"。
    mission_executor = Node(
        package="mecamind_tools",
        executable="mecamind_mission_executor",
        name="mecamind_mission_executor",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"named_goals_file": named_goals_file},
            {"waypoints_file": waypoints_file},
            {"cmd_vel_topic": "/cmd_vel_nav"},
            # 仿真时间下 Gazebo RTF 偏低，放宽单目标超时避免误取消。
            {"goal_timeout_sec": 240.0},
        ],
    )

    # 生命周期激活器：Nav2 节点启动后停在 unconfigured 状态，
    # 由它按 NAV2_LIFECYCLE_NODES 的顺序依次 configure->activate，
    # 最后向 AMCL 发布初始位姿 (0,0,0)（与出生点一致）。
    # Gazebo 冷启动到 /clock 有数据要好几秒，所以延迟 8 秒再激活，
    # 确保 map_server/amcl 的服务已经就绪。
    lifecycle_activator = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="mecamind_tools",
                executable="mecamind_lifecycle_activator",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"node_names": NAV2_LIFECYCLE_NODES},
                    {"service_timeout_sec": 60.0},
                    {"publish_initial_pose": True},
                    {"initial_pose_x": 0.0},
                    {"initial_pose_y": 0.0},
                    {"initial_pose_yaw": 0.0},
                ],
            )
        ],
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
                default_value=f"{tools_share}/rviz/mecamind_navigation.rviz",
            ),
            DeclareLaunchArgument(
                "map_file",
                default_value=f"{tools_share}/maps/mecamind_three_room_nav.yaml",
                description="PGM/YAML map pair produced in lesson 10.3.2.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=f"{tools_share}/config/mecamind_nav2_params.yaml",
            ),
            DeclareLaunchArgument(
                "named_goals_file",
                default_value=f"{tools_share}/config/mecamind_named_goals.yaml",
            ),
            DeclareLaunchArgument(
                "waypoints_file",
                default_value=f"{tools_share}/config/mecamind_patrol_waypoints.yaml",
            ),
            gazebo_backend,
            lite_backend,
            map_server,
            amcl,
            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            safety_gate,
            mission_executor,
            lifecycle_activator,
            rviz_node,
        ]
    )
