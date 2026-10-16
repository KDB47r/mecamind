"""MecaMind 轻量 2D 仿真器 ROS 2 节点（自研仿真后端的"入口"）。

本文件是自研轻量 2D 仿真器与 ROS 2 世界之间的桥梁。它不做具体的物理/几何
计算（那些在 `sim_core.py` 和 `world_geometry.py` 里），只负责：

1. 把仿真状态"翻译"成标准 ROS 消息并周期性发布：
   - ``/clock``       (rosgraph_msgs/Clock)     —— 仿真时钟，供全系统 use_sim_time 使用
   - ``/odom``        (nav_msgs/Odometry)       —— 里程计（可选注入噪声）
   - ``/scan_raw``    (sensor_msgs/LaserScan)   —— 原始激光雷达数据（可选噪声/丢包）
   - ``/imu/data_raw``(sensor_msgs/Imu)         —— 简化的 IMU 数据
   - TF: ``odom -> base_footprint``（动态）以及 base_link、lidar_link、imu_link（静态）

2. 订阅速度指令并转给仿真模型：
   - ``/controller/cmd_vel`` (geometry_msgs/Twist) —— 上游控制器（如手动遥控节点或
     Nav2 输出经过滤后）发来的目标速度，支持模拟指令延迟。

在整个系统中的位置：本节点扮演的角色等价于 Gazebo Harmonic + 各类传感器插件
的组合——下游的 SLAM、定位、导航节点完全不需要知道数据是 Gazebo 产生的还是
本节点产生的，只要 topic 名称和坐标系约定一致即可。这就是"接口一致、后端可换"
的设计思想。

初学者建议重点阅读的函数：
- ``__init__``            —— 学习 ROS 2 参数声明、发布者/订阅者/定时器的创建套路
- ``_tick``               —— 仿真主循环：推进物理 -> 打时间戳 -> 发布各类消息
- ``_publish_tf``         —— TF 广播的最小示例（odom -> base_footprint）
- ``_publish_scan``       —— LaserScan 消息各字段的含义，以及传感器噪声注入
- ``_cmd_cb``             —— 订阅回调 + 模拟指令延迟的队列技巧
"""

from __future__ import annotations

# deque（双端队列）用来存放"延迟生效"的速度指令，先进先出
from collections import deque
import math
from pathlib import Path
import random

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
# TF 广播器：TransformBroadcaster 发布随时间变化的坐标变换（每帧都要发），
# StaticTransformBroadcaster 发布固定不变的变换（只发一次，QoS 为 TransientLocal，
# 即"锁存"——后加入的订阅者也能收到这条历史消息）
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .nav_utils import quaternion_from_yaw
from .sim_core import SimPose, SimTwist, MecaMindSimModel
from .world_geometry import load_world_geometry


def _default_world_path() -> str:
    """返回默认世界文件（SDF）的路径。

    优先从 ament 索引里查找本包安装后的 share 目录——这是 ROS 2 包定位
    资源文件的标准做法（对应 setup.py 里 data_files 的安装规则）。
    如果包还没安装（比如直接用 python 跑源码），就退回相对路径，
    方便开发调试。
    """
    try:
        from ament_index_python.packages import get_package_share_directory

        return str(Path(get_package_share_directory("mecamind_tools")) / "worlds" / "three_room_house.world")
    except Exception:
        return "worlds/three_room_house.world"


def _to_time(seconds: float):
    """把浮点秒数转换成 ROS 的 builtin_interfaces/Time 消息。

    ROS 的时间戳由"整数秒 + 整数纳秒"两个字段组成（避免浮点精度问题），
    所以这里要手动拆分：整数部分给 sec，小数部分乘 1e9 给 nanosec。
    仿真器内部用 float 累计仿真时间，发布消息时统一经过这个函数转换。
    """
    sec = int(seconds)
    nanosec = int((seconds - sec) * 1_000_000_000)
    from builtin_interfaces.msg import Time

    msg = Time()
    msg.sec = sec
    msg.nanosec = nanosec
    return msg


class MecaMindSimulatorNode(Node):
    """轻量 2D 仿真器节点：持有仿真模型，按固定频率发布传感器数据。

    为什么把"物理模型"（MecaMindSimModel）和"ROS 节点"分成两层？
    —— 这样 sim_core 里的数学可以脱离 ROS 单独做单元测试，
    而本类只关心 ROS 通信细节（消息格式、topic、TF、时钟）。
    这是 ROS 项目里常见的"核心逻辑与框架解耦"模式。
    """

    def __init__(self) -> None:
        # 节点名为 mecamind_simulator，会出现在 `ros2 node list` 里
        super().__init__("mecamind_simulator")
        # ---- 参数声明区 ----
        # ROS 2 要求参数必须先 declare 才能用，declare 时给出默认值。
        # 这些参数都可以在 launch 文件或命令行里覆盖，不用改代码。
        self.declare_parameter("world_path", _default_world_path())
        self.declare_parameter("update_rate_hz", 20.0)          # 仿真主循环频率
        self.declare_parameter("initial_x", -3.2)               # 机器人出生点（米）
        self.declare_parameter("initial_y", -2.4)
        self.declare_parameter("initial_yaw", 0.0)              # 出生朝向（弧度）
        self.declare_parameter("robot_radius", 0.22)            # 碰撞检测用的机器人半径
        # 麦轮底盘是全向的：x（前后）、y（左右平移）、z（自转）三个自由度
        # 都可以独立限速/限加速度
        self.declare_parameter("max_linear_x", 0.45)
        self.declare_parameter("max_linear_y", 0.35)
        self.declare_parameter("max_angular_z", 1.5)
        self.declare_parameter("max_accel_x", 0.9)
        self.declare_parameter("max_accel_y", 0.9)
        self.declare_parameter("max_accel_z", 1.8)
        # 雷达相对底盘中心的安装偏移（雷达装在车头前方 0.16m 处）
        self.declare_parameter("sensor_offset_x", 0.16)
        self.declare_parameter("sensor_offset_y", 0.0)
        self.declare_parameter("lidar_range_max", 8.0)          # 雷达最大量程
        self.declare_parameter("lidar_beams", 360)              # 每圈激光束数量
        self.declare_parameter("lidar_angle_min", -math.pi)     # 扫描起始角（-180°）
        self.declare_parameter("lidar_angle_max", math.pi)      # 扫描结束角（+180°）
        self.declare_parameter("publish_clock", True)           # 是否发布 /clock
        self.declare_parameter("random_seed", 8)                # 固定种子 => 噪声可复现
        # 默认带 1cm 高斯噪声：零噪声时命中值恰好落在栅格边界，
        # SLAM 的取整会把北/东方向墙面推到栅格外，导致墙体无法标记占据。
        self.declare_parameter("lidar_noise_std", 0.01)
        self.declare_parameter("lidar_dropout_prob", 0.0)       # 单束激光丢包概率
        self.declare_parameter("odom_xy_noise_std", 0.0)        # 里程计位置噪声
        self.declare_parameter("odom_yaw_noise_std", 0.0)       # 里程计朝向噪声
        self.declare_parameter("cmd_latency_sec", 0.0)          # 模拟指令传输延迟

        # ---- 构建仿真模型 ----
        # 从 SDF 世界文件解析出墙体/障碍的几何盒子（与 Gazebo 使用同一个文件，
        # 保证两个仿真后端看到的世界完全一致）
        world_path = str(self.get_parameter("world_path").value)
        geometry = load_world_geometry(world_path)
        # 把所有参数塞给纯 Python 的仿真核心；此后物理相关的一切都由它负责
        self.model = MecaMindSimModel(
            geometry=geometry,
            initial_pose=SimPose(
                float(self.get_parameter("initial_x").value),
                float(self.get_parameter("initial_y").value),
                float(self.get_parameter("initial_yaw").value),
            ),
            robot_radius=float(self.get_parameter("robot_radius").value),
            max_linear_x=float(self.get_parameter("max_linear_x").value),
            max_linear_y=float(self.get_parameter("max_linear_y").value),
            max_angular_z=float(self.get_parameter("max_angular_z").value),
            max_accel_x=float(self.get_parameter("max_accel_x").value),
            max_accel_y=float(self.get_parameter("max_accel_y").value),
            max_accel_z=float(self.get_parameter("max_accel_z").value),
            sensor_offset_x=float(self.get_parameter("sensor_offset_x").value),
            sensor_offset_y=float(self.get_parameter("sensor_offset_y").value),
            lidar_range_max=float(self.get_parameter("lidar_range_max").value),
            lidar_beams=int(self.get_parameter("lidar_beams").value),
            lidar_angle_min=float(self.get_parameter("lidar_angle_min").value),
            lidar_angle_max=float(self.get_parameter("lidar_angle_max").value),
        )

        # ---- 运行时状态 ----
        self.update_rate = float(self.get_parameter("update_rate_hz").value)
        # dt 是每个仿真步的时长（秒），20Hz 即 0.05s
        self.dt = 1.0 / self.update_rate
        self.publish_clock = bool(self.get_parameter("publish_clock").value)
        self._last_command = SimTwist()          # 最近收到的指令（仅用于日志展示）
        self._last_debug_log = 0.0               # 上次打印调试日志的仿真时间
        # 用独立的 Random 实例而不是全局 random，保证噪声序列只受本节点种子控制
        self._rng = random.Random(int(self.get_parameter("random_seed").value))
        # 延迟指令队列：元素为 (生效时刻, 指令)，按时间先后排队
        self._pending_commands: deque[tuple[float, SimTwist]] = deque()
        # "上报位姿" = 真实位姿 + 里程计噪声；odom/TF 都用它，
        # 而不是直接用 model.pose，这样才能模拟真实机器人里程计的漂移
        self._reported_pose = self.model.pose

        # ---- 发布者 / 订阅者 ----
        # 第三个参数 10 是 QoS 队列深度（history depth）：网络拥堵时最多缓存
        # 10 条消息，默认可靠传输（RELIABLE），对教学场景足够用
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        # 注意发布的是 /scan_raw 而非 /scan：真实系统里原始雷达数据往往还要
        # 经过滤波/裁剪节点处理后才给 SLAM 用，这里保留同样的管线结构
        self.scan_pub = self.create_publisher(LaserScan, "/scan_raw", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data_raw", 10)
        self.cmd_sub = self.create_subscription(Twist, "/controller/cmd_vel", self._cmd_cb, 10)

        # ---- TF 广播 ----
        # 动态 TF：每个仿真周期发布一次 odom->base_footprint
        self.tf_broadcaster = TransformBroadcaster(self)
        # 静态 TF：机器人本体上各传感器的固定安装位置，只需发布一次
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_transforms()
        # 定时器驱动仿真主循环：每 dt 秒调用一次 _tick
        self.timer = self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f"MecaMind simulator ready: {geometry.name} "
            f"({len(geometry.boxes)} boxes, {self.model.lidar_beams} beams)"
        )

    def _publish_static_transforms(self) -> None:
        """发布机器人内部的静态 TF 树（传感器安装位置）。

        TF 树结构：base_footprint -> base_link -> {lidar_link, imu_link}。
        - base_footprint 是"投影到地面"的坐标系（z=0），导航栈以它为基准；
        - base_link 是底盘本体中心（离地 8cm）；
        - lidar_link / imu_link 是传感器坐标系，传感器消息的 frame_id 必须
          与之对应，下游（如 SLAM）才能通过 TF 把测距点变换到地图坐标系。

        这些相对位置在机器人运动时不会变，所以用 StaticTransformBroadcaster
        （TransientLocal QoS，锁存最后一条消息）：只发一次，任何时候启动的
        订阅者都能拿到，省带宽也不怕时序问题。
        """
        static_transforms = []

        def make_transform(parent: str, child: str, x: float, y: float, z: float, yaw: float = 0.0):
            # 小工厂函数：按"父坐标系 -> 子坐标系 + 平移/偏航"生成一条 TF 消息
            msg = TransformStamped()
            msg.header.frame_id = parent
            msg.child_frame_id = child
            msg.transform.translation.x = x
            msg.transform.translation.y = y
            msg.transform.translation.z = z
            # 2D 机器人只有绕 z 轴的旋转，用工具函数把 yaw 角转成四元数
            msg.transform.rotation = quaternion_from_yaw(yaw)
            return msg

        # 数值与 URDF/Gazebo 模型保持一致：底盘中心离地 8cm，
        # 雷达在车头前 16cm、高 16cm 处，IMU 在底盘中心上方 10cm
        static_transforms.append(make_transform("base_footprint", "base_link", 0.0, 0.0, 0.08))
        static_transforms.append(make_transform("base_link", "lidar_link", 0.16, 0.0, 0.16))
        static_transforms.append(make_transform("base_link", "imu_link", 0.0, 0.0, 0.10))
        self.static_tf_broadcaster.sendTransform(static_transforms)

    def _cmd_cb(self, msg: Twist) -> None:
        """速度指令订阅回调：把 Twist 转成仿真指令，可选模拟传输延迟。

        麦轮全向底盘用到 Twist 的三个分量：linear.x（前后）、
        linear.y（横向平移，普通差速车没有这项！）、angular.z（自转）。
        """
        command = SimTwist(msg.linear.x, msg.linear.y, msg.angular.z)
        self._last_command = command
        # 每次回调都实时读参数，这样运行中用 `ros2 param set` 改延迟立即生效
        latency = max(0.0, float(self.get_parameter("cmd_latency_sec").value))
        if latency <= 1.0e-6:
            # 无延迟：直接下发给仿真模型
            self.model.set_command(command.vx, command.vy, command.wz)
            return
        # 有延迟：先进队列，记下"应当生效的仿真时刻"，
        # 由 _tick 里的 _apply_delayed_commands 到点再取出执行。
        # 这样可以教学演示"指令延迟对闭环控制的影响"
        self._pending_commands.append((self.model.sim_time + latency, command))

    def _tick(self) -> None:
        """仿真主循环（由定时器以 update_rate_hz 频率触发）。

        每一步的顺序很重要：
        1. 先把到期的延迟指令下发；
        2. 推进物理仿真一个 dt（速度斜坡 + 位姿积分 + 碰撞检测）；
        3. 生成"带噪声的上报位姿"；
        4. 用同一个仿真时间戳发布 clock/odom/scan/imu/TF——
           所有消息共享同一 stamp，下游做 TF 查询、消息同步才不会出错。
        """
        self._apply_delayed_commands()
        self.model.step(self.dt)
        self._reported_pose = self._make_reported_pose()
        # 每 5 秒仿真时间打印一次状态日志，方便观察而不刷屏
        if self.model.sim_time - self._last_debug_log >= 5.0:
            self._last_debug_log = self.model.sim_time
            self.get_logger().info(
                "motion: pose=(%.2f, %.2f, %.2f) "
                "cmd=(%.2f, %.2f, %.2f) vel=(%.2f, %.2f, %.2f)"
                % (
                    self.model.pose.x,
                    self.model.pose.y,
                    self.model.pose.yaw,
                    self._last_command.vx,
                    self._last_command.vy,
                    self._last_command.wz,
                    self.model.velocity.vx,
                    self.model.velocity.vy,
                    self.model.velocity.wz,
                )
            )
        # 时间戳来自"仿真时间"而非墙上时钟：整个系统运行在 use_sim_time 模式下，
        # 时间由本节点发布的 /clock 定义，这样仿真可以随意加速/暂停而不破坏
        # 下游算法对时间的假设（Gazebo 也是这么做的）
        stamp = _to_time(self.model.sim_time)
        self._publish_clock(stamp)
        self._publish_odom(stamp)
        self._publish_scan(stamp)
        self._publish_imu(stamp)
        self._publish_tf(stamp)

    def _apply_delayed_commands(self) -> None:
        """把队列里"生效时刻已到"的延迟指令依次下发给仿真模型。

        队列按入队顺序（也即时间顺序）排列，所以只要队头没到期就可以停止检查。
        后到期的指令会覆盖先到期的，与真实系统"新指令覆盖旧指令"一致。
        """
        while self._pending_commands and self._pending_commands[0][0] <= self.model.sim_time:
            _, command = self._pending_commands.popleft()
            self.model.set_command(command.vx, command.vy, command.wz)

    def _make_reported_pose(self) -> SimPose:
        """生成用于 odom/TF 的"上报位姿" = 真实位姿 + 高斯噪声。

        为什么要区分"真实位姿"和"上报位姿"？真实机器人的里程计存在打滑、
        编码器量化等误差，SLAM/定位算法的存在意义正是纠正这些误差。
        把噪声参数调大，就能直观演示"纯里程计漂移"以及 SLAM 的纠偏效果。
        """
        xy_std = max(0.0, float(self.get_parameter("odom_xy_noise_std").value))
        yaw_std = max(0.0, float(self.get_parameter("odom_yaw_noise_std").value))
        if xy_std <= 1.0e-9 and yaw_std <= 1.0e-9:
            # 噪声为零时直接返回真值，跳过随机数生成（也保证结果完全确定）
            return self.model.pose
        return SimPose(
            self.model.pose.x + self._rng.gauss(0.0, xy_std),
            self.model.pose.y + self._rng.gauss(0.0, xy_std),
            self.model.pose.yaw + self._rng.gauss(0.0, yaw_std),
        )

    def _publish_clock(self, stamp) -> None:
        """发布 /clock 仿真时钟。

        当其他节点设置了 use_sim_time:=true 时，它们的 `node.get_clock()`
        不再读系统时间，而是订阅 /clock 并以此为"当前时间"。
        因此本节点就是整个仿真系统的"时间源"，角色等同于 Gazebo。
        """
        if not self.publish_clock:
            return
        msg = Clock()
        msg.clock = stamp
        self.clock_pub.publish(msg)

    def _publish_tf(self, stamp) -> None:
        """广播动态 TF：odom -> base_footprint。

        TF 是 ROS 里维护坐标系关系的机制：每条消息声明"子坐标系相对
        父坐标系的位姿"。odom->base_footprint 表达的正是里程计估计出的
        机器人位姿；SLAM 再补上 map->odom，整棵树就是
        map -> odom -> base_footprint -> base_link -> 传感器。
        注意这里用的是带噪声的 _reported_pose，必须与 /odom 消息一致，
        否则下游会看到互相矛盾的两套位姿。
        """
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = self._reported_pose.x
        transform.transform.translation.y = self._reported_pose.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = quaternion_from_yaw(self._reported_pose.yaw)
        self.tf_broadcaster.sendTransform(transform)

    def _publish_odom(self, stamp) -> None:
        """发布 /odom 里程计消息。

        Odometry 包含两部分：pose（在 odom 坐标系下的位姿，用上报位姿）
        和 twist（在 child_frame_id 即机器人本体坐标系下的速度，用真实
        速度——真实机器人的速度通常由轮速直接算出，噪声特性与位置不同，
        教学上简化为无噪声）。
        """
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"           # 位姿所在的参考系
        msg.child_frame_id = "base_footprint"  # 速度所在的参考系（机器人本体）
        msg.pose.pose.position.x = self._reported_pose.x
        msg.pose.pose.position.y = self._reported_pose.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = quaternion_from_yaw(self._reported_pose.yaw)
        msg.twist.twist.linear.x = self.model.velocity.vx
        msg.twist.twist.linear.y = self.model.velocity.vy
        msg.twist.twist.angular.z = self.model.velocity.wz
        self.odom_pub.publish(msg)

    def _publish_scan(self, stamp) -> None:
        """发布 /scan_raw 激光雷达消息（含噪声与丢包模拟）。

        真实雷达的测距存在测量噪声（近似高斯分布）和偶发无回波（丢包）。
        在仿真里主动注入这些缺陷，能让学生体会 SLAM/避障算法对
        "不完美传感器"的鲁棒性要求。
        """
        # 先由几何模型做精确的射线投射，得到每束激光的"真值"距离
        ranges = self.model.scan_ranges()
        noise_std = max(0.0, float(self.get_parameter("lidar_noise_std").value))
        dropout_prob = max(0.0, min(1.0, float(self.get_parameter("lidar_dropout_prob").value)))
        if noise_std > 1.0e-9 or dropout_prob > 1.0e-9:
            noisy_ranges = []
            for value in ranges:
                if self._rng.random() < dropout_prob:
                    # 丢包：按 LaserScan 规范，无有效回波用 inf 表示
                    noisy_ranges.append(float("inf"))
                    continue
                # 叠加零均值高斯噪声后，夹回 [range_min, range_max] 合法区间
                noisy = value + self._rng.gauss(0.0, noise_std)
                noisy_ranges.append(max(0.12, min(self.model.lidar_range_max, noisy)))
            ranges = noisy_ranges
        msg = LaserScan()
        msg.header.stamp = stamp
        # frame_id 必须是雷达自己的坐标系；下游靠 TF 把测距点变换到别的坐标系
        msg.header.frame_id = "lidar_link"
        msg.angle_min = self.model.lidar_angle_min       # 第一束激光的角度
        msg.angle_max = self.model.lidar_angle_max       # 最后一束激光的角度
        msg.angle_increment = self.model.scan_increment()  # 相邻两束的角度差
        # time_increment：相邻两束激光之间的时间差（模拟机械旋转雷达一圈内
        # 逐束采样的时间流逝）；scan_time：完整一圈的耗时
        msg.time_increment = self.dt / max(1, self.model.lidar_beams)
        msg.scan_time = self.dt
        msg.range_min = 0.12   # 小于此值的读数无效（雷达盲区/机器人自身遮挡）
        msg.range_max = self.model.lidar_range_max
        msg.ranges = [float(value) for value in ranges]
        self.scan_pub.publish(msg)

    def _publish_imu(self, stamp) -> None:
        """发布 /imu/data_raw IMU 消息（教学用的极简版本）。

        真实 IMU 输出加速度计+陀螺仪原始数据，姿态需要滤波融合得到；
        这里直接给出理想姿态（含里程计噪声）和角速度，加速度置零，
        足够演示"IMU 提供朝向信息"这一用途。
        """
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = "imu_link"
        # 2D 机器人姿态只有 yaw 一个自由度，转成四元数填入
        msg.orientation = quaternion_from_yaw(self._reported_pose.yaw)
        msg.angular_velocity.z = self.model.velocity.wz
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 0.0
        self.imu_pub.publish(msg)


def main(args=None) -> None:
    """入口函数：初始化 rclpy、创建节点并进入事件循环。

    rclpy.spin 会阻塞并不断处理定时器/订阅回调，直到 Ctrl+C。
    finally 里的层层保护确保无论怎样退出都能干净地销毁节点、
    关闭 rclpy（try_shutdown 在已关闭时不会报错）。
    """
    rclpy.init(args=args)
    node = MecaMindSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        finally:
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
