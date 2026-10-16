"""第五课（10.3.5）视觉跟随 + 速度仲裁的统一启动入口。

【这个文件是干什么的】
一条命令拉起"仿真后端 + 视觉链路 + 跟随控制 + 速度仲裁 + 安全门"：

    ros2 launch mecamind_bringup follow.launch.py backend:=lite
    ros2 launch mecamind_bringup follow.launch.py backend:=gazebo source:=camera
    # Gazebo 房间内红色圆柱往返 + 车载相机 + HSV（推荐课堂演示）：
    ros2 launch mecamind_bringup follow.launch.py \\
      backend:=gazebo source:=camera infer_backend:=hsv moving_target:=true

【速度链路（本课核心，与第三课的区别是多了仲裁器）】

    遥控 ───────────── /cmd_vel_teleop ──┐
    跟随控制器 ─────── /cmd_vel_follow ──┤（estop > teleop > follow > nav）
    Nav2（本课不启） ─ /cmd_vel_nav_in ──┴─► cmd_vel_arbiter
                                              │ /cmd_vel_nav
                                              ▼
                                          safety_gate ──► /cmd_vel ──► 后端

仲裁器决定"听谁的"，安全门决定"听了之后撞不撞墙"，两层各管一事。

【视觉链路】
detector(synthetic/camera/video) -> /mecamind/detections
  -> json_bridge -> /mecamind/raw_detections
  -> perception_filter -> /mecamind/perception_event
  -> vision_follow_controller -> /cmd_vel_follow

【上课操作提示】
- 约 10s 后自动打开跟随；也可手动：
  ros2 topic pub --once /mecamind/follow_enable std_msgs/msg/Bool '{data: true}'
- Gazebo 课堂演示：红柱在放大后的客厅南侧空地单向绕圈（远离车头），小车朝南跟随；
  跟随距离由 desired_distance 调节（画面宽度反算真实距离后闭环）；
  转向/距离均为 PID（Kp/Ki/Kd），离得越远越快（饱和于 max_linear），过近会主动后退。
  红柱默认慢(0.09)↔快(0.23)梯形变速，便于观察小车跟随加速。
- 绕圈回路中央有一个 0.7m 高的木箱（红柱绕它转、绕到背面会被遮挡），
  东北侧还有一个矮箱；跟随控制器融合了激光避障（转向偏置+麦轮横移），
  跟丢时朝目标消失方向弧线绕行找回。把 scan_avoidance_enable 设为 false
  可以现场对比"纯视觉 PID 卡箱子"和"融合避障绕箱子"的差别。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("mecamind_bringup")
    gazebo_share = get_package_share_directory("mecamind_gazebo")

    backend = LaunchConfiguration("backend")
    use_gui = LaunchConfiguration("use_gui")
    source = LaunchConfiguration("source")
    infer_backend = LaunchConfiguration("infer_backend")
    model_path = LaunchConfiguration("model_path")
    video_path = LaunchConfiguration("video_path")
    target_label = LaunchConfiguration("target_label")
    moving_target = LaunchConfiguration("moving_target")

    is_gazebo = IfCondition(PythonExpression(["'", backend, "' == 'gazebo'"]))
    is_lite = IfCondition(PythonExpression(["'", backend, "' == 'lite'"]))
    # 可动红柱只在 Gazebo 世界有意义；lite 无对应实体
    use_moving_target = IfCondition(
        PythonExpression(
            ["'", backend, "' == 'gazebo' and '", moving_target, "' == 'true'"]
        )
    )

    gazebo_backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{gazebo_share}/launch/gazebo_sim.launch.py"),
        launch_arguments={
            "use_gui": use_gui,
            # 跟随专用世界：小车直接出生在客厅起点后方朝南
            "world": f"{gazebo_share}/worlds/three_room_house_follow.sdf",
        }.items(),
        condition=is_gazebo,
    )
    lite_backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{bringup_share}/launch/lite_sim.launch.py"),
        condition=is_lite,
    )

    # ---- 视觉链路（复用第四课的三个节点）----
    detector = Node(
        package="mecamind_perception",
        executable="mecamind_detector",
        name="mecamind_detector",
        output="screen",
        parameters=[
            {"source": source},
            {"backend": infer_backend},
            {"model_path": model_path},
            {"video_path": video_path},
            {"target_label": target_label},
        ],
    )
    json_bridge = Node(
        package="mecamind_perception",
        executable="mecamind_detection_json_bridge",
        name="mecamind_detection_json_bridge",
        output="screen",
    )
    perception_filter = Node(
        package="mecamind_tools",
        executable="mecamind_perception_filter",
        name="mecamind_perception_filter",
        output="screen",
        parameters=[{"target_label": target_label}],
    )

    # ---- Gazebo：客厅南侧空地绕圈（不朝车头对撞）----
    follow_target_mover = Node(
        package="mecamind_tools",
        executable="mecamind_follow_target_mover",
        name="mecamind_follow_target_mover",
        output="screen",
        condition=use_moving_target,
        parameters=[
            {"use_sim_time": True},
            {"world_name": "three_room_house"},
            {"model_name": "follow_target"},
            # 大环形回路（周长约 11.2m），圈内包住 crate_center 和 crate_north
            # 两个高箱，与 FOLLOW_DEMO_WAYPOINTS / SDF 障碍物布局保持同步
            {
                "waypoints_xy": (
                    "[-3.45,0.7, -4.95,0.3, -4.95,-2.7, "
                    "-3.6,-3.45, -2.55,-2.85, -2.55,-0.2]"
                )
            },
            {"z": 0.38},
            # 恒速兜底；speed_profile_enable=true 时用慢/快梯形剖面
            {"speed_mps": 0.14},
            {"speed_profile_enable": True},
            {"speed_slow_mps": 0.09},
            {"speed_fast_mps": 0.23},
            {"speed_accel_mps2": 0.18},
            {"speed_hold_sec": 5.0},
            {"rate_hz": 8.0},
            {"startup_delay_sec": 6.0},
            {"wait_for_follow_enable": True},
            {"follow_start_delay_sec": 2.0},
            {"ping_pong": False},
            {"closed_loop": True},
            {"reposition_robot": True},
            {"robot_x": -3.0},
            {"robot_y": 2.925},
            {"robot_yaw": -1.5708},
        ],
    )

    # ---- 跟随控制器（完整 PID）----
    follow_controller = Node(
        package="mecamind_tools",
        executable="mecamind_vision_follow_controller",
        name="mecamind_vision_follow_controller",
        output="screen",
        parameters=[
            {"output_cmd_topic": "/cmd_vel_follow"},
            # 距离通道按"真实距离"闭环：宽度→距离反算参数与红柱直径
            # (0.2m)、相机 HFOV(1.047rad) 匹配；期望跟随距离 0.8m。
            {"desired_distance": 0.80},
            {"target_real_width": 0.20},
            {"camera_hfov": 1.047},
            # 速度上限要明显高于红柱快速段(0.23)，否则没有追赶余量；
            # kp 按米计：落后 0.4m 即给满速。P 控制跟匀速目标必有稳态
            # 滞后（误差≈目标速度/kp），积分给足力度（ki*i_limit≈0.24m/s）
            # 把它消掉——柱子加速时积分自动顶上去，不再"越快离越远"。
            {"max_linear": 0.35},
            {"max_angular": 1.0},
            {"kp_linear": 0.9},
            {"ki_linear": 0.30},
            {"kd_linear": 0.10},
            {"kp_angular": 2.2},
            {"ki_angular": 0.15},
            {"kd_angular": 0.08},
            {"i_limit_angular": 0.45},
            {"i_limit_linear": 0.80},
            {"search_on_loss": True},
            {"search_angular": 0.45},
            # 激光避障融合：侧前方障碍转成转向偏置+麦轮横移，防止拐弯切角蹭箱子；
            # 关掉（false）即可课堂对比"无避障纯 PID"的卡死现象
            {"scan_avoidance_enable": True},
            {"scan_topic": "/scan"},
            {"avoid_clear_dist": 0.90},
            {"avoid_stop_dist": 0.30},
            {"avoid_max_turn": 0.65},
            {"avoid_max_lateral": 0.12},
            # 红柱回波豁免：正前回波不比目标近 0.3m 以上就不当障碍，
            # 否则车会把要追的柱子当墙、越近越慢永远追不上
            {"avoid_target_margin": 0.30},
            # 丢失后先朝目标消失方向弧线绕行 4s（绕过遮挡箱），再原地旋转兜底
            {"search_arc_sec": 4.0},
            {"search_arc_linear": 0.10},
        ],
    )

    # ~10s 后自动打开跟随（连发，避免丢消息）
    auto_enable_follow = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-lc",
                    "for i in 1 2 3 4 5; do ros2 topic pub --once /mecamind/follow_enable std_msgs/msg/Bool '{data: true}'; sleep 0.8; done",
                ],
                output="screen",
            )
        ],
    )

    # ---- 速度仲裁器：estop > teleop > follow > nav ----
    arbiter = Node(
        package="mecamind_perception",
        executable="mecamind_cmd_vel_arbiter",
        name="mecamind_cmd_vel_arbiter",
        output="screen",
        parameters=[
            {"output_topic": "/cmd_vel_nav"},
            {"teleop_topic": "/cmd_vel_teleop"},
            {"follow_topic": "/cmd_vel_follow"},
            {"nav_topic": "/cmd_vel_nav_in"},
        ],
    )

    # ---- 安全门：仲裁之后仍要过避障关（与第三课同一节点、同一接线）----
    safety_gate = Node(
        package="mecamind_tools",
        executable="mecamind_safety_gate",
        name="mecamind_follow_safety_gate",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"scan_topic": "/scan"},
            {"input_cmd_topic": "/cmd_vel_nav"},
            {"output_cmd_topic": "/cmd_vel"},
            {"scan_timeout_sec": 2.0},
            {"cmd_timeout_sec": 1.5},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend",
                default_value="lite",
                description="Simulation backend: gazebo or lite.",
            ),
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument(
                "source",
                default_value="synthetic",
                description="Detector image source: camera / video / synthetic.",
            ),
            DeclareLaunchArgument(
                "infer_backend",
                default_value="auto",
                description="Inference backend: auto / ultralytics / onnx / hsv.",
            ),
            DeclareLaunchArgument("model_path", default_value="yolov8n.pt"),
            DeclareLaunchArgument("video_path", default_value=""),
            DeclareLaunchArgument("target_label", default_value="person"),
            DeclareLaunchArgument(
                "moving_target",
                default_value="true",
                description="Gazebo only: drive the red follow_target cylinder left/right.",
            ),
            gazebo_backend,
            lite_backend,
            detector,
            json_bridge,
            perception_filter,
            follow_target_mover,
            follow_controller,
            arbiter,
            safety_gate,
            auto_enable_follow,
        ]
    )
