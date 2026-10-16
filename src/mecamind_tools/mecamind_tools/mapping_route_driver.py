"""半自动建图路线驱动器（MecaMind 第二课：SLAM 建图的"自动导游"）。

这个文件是干什么的？
    在 slam_toolbox 建图过程中，机器人需要有人"带着"它把整个场地走一遍，
    激光雷达才能扫到所有的墙和障碍物。手动遥控对初学者来说很难走得均匀，
    所以本节点按照 YAML 路点文件里预设的一串路点（waypoint），自动发布速度
    指令带机器人绕场一圈；路线走完后，把 slam_toolbox 发布在 /map 上的
    OccupancyGrid 消息直接写成 nav2 兼容的 PGM/YAML 地图文件。

订阅的 topic：
    /odom (nav_msgs/Odometry)         —— 机器人当前位姿，用来算离路点还有多远。
    /scan (sensor_msgs/LaserScan)     —— 激光雷达，用来做"前方有障碍就停"的保护。
    /map  (nav_msgs/OccupancyGrid)    —— slam_toolbox 实时发布的栅格地图，缓存
                                          最新一帧，路线结束时直接存盘。

发布的 topic：
    /cmd_vel (geometry_msgs/Twist)            —— 速度指令（麦克纳姆轮支持横移，
                                                  所以 linear.y 也会被用到）。
    /mecamind/mapping_status (std_msgs/String) —— JSON 格式的进度状态，供教学
                                                  脚本和评测工具订阅。

提供的服务：
    ~/save_map (std_srvs/Trigger) —— 手动触发存图（自动路线结束后可人工补扫再存）。
    ~/status   (std_srvs/Trigger) —— 查询当前状态（JSON）。

在建图流程中的角色：
    slam_toolbox 负责"画地图"，本节点负责"带路 + 存图"。它不做路径规划，
    只用简单的比例控制（P 控制）朝当前路点直线开，所以路点文件必须保证
    相邻路点之间没有障碍。

两个重要的设计决策（初学者容易踩的坑）：
    1. 所有超时/停滞判断都用 **仿真时间**（get_clock().now()，配合 use_sim_time
       参数），而不是 Python 的 time.time() 墙钟。因为 Gazebo 的实时因子
       （RTF）经常小于 1，即仿真里 1 秒可能对应现实 2 秒；如果用墙钟判断
       "18 秒没有进展就跳过路点"，机器人其实只走了 9 个仿真秒，会被冤枉地
       判成停滞。
    2. 存图时优先把缓存的 /map 消息 **直接写成 PGM/YAML**，而不是调用官方的
       map_saver_cli 工具。因为 map_saver_cli 是一个临时启动的小节点，在
       sim time 下它有时来不及收到带 TRANSIENT_LOCAL QoS 的 /map 消息就超时
       退出了（订阅时机问题），导致存图偶发失败。自己写文件则完全不依赖
       订阅时机。map_saver_cli 只作为兜底方案保留。

初学者应该重点看的函数：
    - write_occupancy_grid_pair() —— OccupancyGrid 如何变成 PGM 图片（像素值映射）。
    - MecaMindMappingRouteDriver._tick() —— 路点推进状态机的核心，50ms 跑一次。
    - _skip_reason() —— 超时/停滞的判断逻辑。
    - _try_save_map() —— "直接写文件优先，map_saver_cli 兜底"的存图策略。
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import time
from pathlib import Path
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml

from .nav_utils import load_yaml


def write_occupancy_grid_pair(save_base: Path, msg: OccupancyGrid) -> None:
    """Write nav2-compatible PGM/YAML from an OccupancyGrid (map_saver Y-flip).

    把一帧 OccupancyGrid 消息直接写成 nav2 地图服务器能读的 PGM + YAML 文件对，
    效果等价于官方 map_saver_cli 的输出。自己实现的原因见模块 docstring：
    绕开 map_saver_cli 在 sim time 下偶发的订阅失败问题。

    需要理解的两个约定：
    1. OccupancyGrid.data 是 row-major（按行存储）的一维数组，第 0 行对应
       地图 origin 处（世界坐标 y 最小的一行），行号越大 y 越大。
       每个格子的值：-1 = 未知，0~100 = 被占据的概率百分比。
    2. PGM 图片的第 0 行是"图片顶部"。map_saver 的惯例是让图片顶部对应
       世界坐标 y 最大处（这样图片看起来方向"正"），所以写 PGM 时必须
       把 OccupancyGrid 的行序上下翻转（Y-flip）。后续读图工具（比如
       map_quality_analyzer）加载 PGM 时要再翻回来。
    """
    width = int(msg.info.width)
    height = int(msg.info.height)
    # 防御性检查：SLAM 刚启动时可能发布过空地图，写出去只会得到废文件。
    if width <= 0 or height <= 0 or len(msg.data) < width * height:
        raise ValueError("OccupancyGrid is empty or incomplete")

    pixels = bytearray(width * height)
    for row in range(height):
        src_row = height - 1 - row  # map_saver: image row 0 = world y_max
        # ^ 上面就是 Y-flip：PGM 的第 row 行取自 OccupancyGrid 的倒数第 row 行。
        for col in range(width):
            value = int(msg.data[src_row * width + col])
            # 像素值映射（trinary 三值模式，与 map_saver 默认行为一致）：
            #   -1（未知）      -> 205（灰色）
            #   >= 65（占据）   -> 0（黑色，occupied_thresh=0.65 的对应值）
            #   <= 25（自由）   -> 254（接近白色，free_thresh=0.25 的对应值）
            #   26~64（模糊区） -> 按线性插值给一个中间灰度，仅供人眼参考
            if value < 0:
                pixel = 205
            elif value >= 65:
                pixel = 0
            elif value <= 25:
                pixel = 254
            else:
                # Keep mid values visually between free/occupied.
                pixel = int(round(255 - value * 255 / 100.0))
            pixels[row * width + col] = pixel

    pgm_path = save_base.with_suffix(".pgm")
    yaml_path = save_base.with_suffix(".yaml")
    # P5 是二进制格式的 PGM：ASCII 头（魔数、宽高、最大灰度）后紧跟原始像素字节。
    pgm_path.write_bytes(
        f"P5\n{width} {height}\n255\n".encode("ascii") + bytes(pixels)
    )
    # YAML 元数据：nav2 map_server 加载地图时靠它知道图片对应的分辨率和原点。
    # origin 是地图左下角（第 0 行第 0 列格子）在世界坐标系中的位置。
    meta = {
        "image": pgm_path.name,
        "mode": "trinary",
        "resolution": float(msg.info.resolution),
        "origin": [
            float(msg.info.origin.position.x),
            float(msg.info.origin.position.y),
            0.0,
        ],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    yaml_path.write_text(yaml.dump(meta, default_flow_style=False, sort_keys=False), encoding="utf-8")


def _default_route_file() -> str:
    """返回默认路点文件的路径。

    优先从已安装的 mecamind_tools 包的 share 目录里找（ros2 run 场景）；
    如果包没安装（比如直接用 python 跑源码），退回到相对路径。
    """
    try:
        from ament_index_python.packages import get_package_share_directory

        return str(Path(get_package_share_directory("mecamind_tools")) / "config" / "mecamind_mapping_route.yaml")
    except Exception:
        return "config/mecamind_mapping_route.yaml"


def _clamp(value: float, limit: float) -> float:
    """把 value 限制在 [-limit, +limit] 区间内（速度限幅用）。"""
    return max(-limit, min(limit, value))


def _angle_diff(a: float, b: float) -> float:
    """计算角度差 a - b，并归一化到 (-pi, pi]。

    直接相减会出现"359° 和 1° 差 358°"的问题；用 atan2(sin, cos) 归一化后
    结果永远是最短转向方向，转向控制才不会绕远路。
    """
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


def _as_bool(value: object) -> bool:
    """把 ROS 参数值宽容地转成 bool。

    launch 文件传参有时会把布尔写成字符串（如 "true"、"1"），
    直接 bool("false") 会得到 True，所以字符串要单独处理。
    """
    if isinstance(value, str):
        return value.lower() in ("1", "true", "yes", "on")
    return bool(value)


def build_route_report(
    route_file: str,
    phase: str,
    route_size: int,
    waypoint_results: List[dict],
    robot_pose: dict,
    auto_map_path: str,
    manual_map_path: str,
    auto_map_saved: bool,
    manual_map_saved: bool,
) -> dict:
    """Build the machine-readable evidence emitted by a mapping route run.

    把一次建图路线的执行结果汇总成机器可读的 dict（JSON 报告）。
    这个报告既会周期性地发到状态 topic 上，也会在路线结束时写成
    route_report.json 文件，作为"这次建图跑没跑完、哪些路点没到"的证据，
    方便教学评测脚本自动打分。
    """
    # 逐条统计路点结果：reached = 成功到达，skipped = 超时/停滞被跳过。
    reached = sum(item.get("status") == "reached" for item in waypoint_results)
    skipped = sum(item.get("status") == "skipped" for item in waypoint_results)
    return {
        "schema_version": 1,
        "phase": phase,
        "route_file": route_file,
        "route_size": route_size,
        "processed_waypoints": len(waypoint_results),
        "reached_waypoints": reached,
        "skipped_waypoints": skipped,
        "all_waypoints_reached": route_size > 0 and reached == route_size,
        "auto_map_path": auto_map_path,
        "manual_map_path": manual_map_path,
        "auto_map_saved": auto_map_saved,
        "manual_map_saved": manual_map_saved,
        "robot_pose": robot_pose,
        "waypoints": waypoint_results,
    }


class MecaMindMappingRouteDriver(Node):
    """Drive a safe waypoint route through the three-room house for SLAM mapping.

    半自动建图路线驱动节点。核心是一个 50ms 周期的定时器回调 _tick()，
    它实现了一个简单的"路点推进状态机"：

        等待传感器就绪 -> 启动延时 -> 逐个路点用 P 控制驶向目标
        -> 到达（或超时被跳过）后短暂停留 -> 下一个路点
        -> 全部走完 -> 存图 -> 写报告 ->（可选）自动关闭

    为什么用这么简单的控制而不用 Nav2？因为建图阶段还没有地图，Nav2 的
    全局规划器无从规划；而路点文件是人工按场地布置精心设计过的，相邻
    路点之间保证无障碍，直线 P 控制 + 激光"前方急停"保护就足够安全。
    """

    def __init__(self) -> None:
        super().__init__("mecamind_mapping_route_driver")
        # ---- 参数声明区：全部可以在 launch 文件或命令行里覆盖 ----
        # 路线与到达判定
        self.declare_parameter("route_file", _default_route_file())
        self.declare_parameter("startup_delay_sec", 8.0)      # 开跑前等 SLAM/传感器稳定
        self.declare_parameter("goal_tolerance", 0.18)         # 离路点多近算"到了"（米）
        self.declare_parameter("yaw_tolerance", 0.20)          # 朝向误差容忍（弧度）
        self.declare_parameter("hold_sec", 0.4)                # 到达后原地停留，让激光多扫几帧
        # P 控制增益与速度上限（麦克纳姆轮可以横移，所以 x/y 分别限幅）
        self.declare_parameter("linear_gain", 0.7)
        self.declare_parameter("angular_gain", 1.4)
        self.declare_parameter("max_linear_x", 0.28)
        self.declare_parameter("max_linear_y", 0.22)
        self.declare_parameter("max_angular_z", 1.0)
        self.declare_parameter("obstacle_stop_dist", 0.28)     # 前方障碍近于此距离就禁止前进
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        # 存图相关
        self.declare_parameter("auto_save_map", True)
        self.declare_parameter("checkpoint_save_interval_sec", 90.0)  # 途中定期存"检查点"地图
        self.declare_parameter("checkpoint_keep_snapshots", False)    # True 则每个检查点存独立文件
        # 超时/停滞判定（单位都是仿真秒，见模块 docstring 的说明）
        self.declare_parameter("waypoint_timeout_sec", 50.0)   # 单个路点最长允许时间
        self.declare_parameter("waypoint_stall_sec", 18.0)     # 连续多久"没有靠近"算停滞
        self.declare_parameter("waypoint_min_progress", 0.08)  # 至少靠近多少米才算"有进展"
        self.declare_parameter("shutdown_after_route", True)
        self.declare_parameter("manual_save_path", "")
        self.declare_parameter("route_report_path", "")
        self.declare_parameter("status_topic", "/mecamind/mapping_status")
        self.declare_parameter("map_save_path", "maps/mecamind_three_room_map")

        # ---- 读取路点文件 ----
        self.route_file = str(self.get_parameter("route_file").value)
        config = load_yaml(self.route_file)
        self.route: List[dict] = list(config.get("waypoints", []))
        if not self.route:
            raise ValueError(f"No waypoints found in route file: {self.route_file}")

        # ---- 把参数缓存成普通属性，避免每个控制周期都查参数服务器 ----
        self.startup_delay_sec = float(self.get_parameter("startup_delay_sec").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
        self.hold_sec = float(self.get_parameter("hold_sec").value)
        self.linear_gain = float(self.get_parameter("linear_gain").value)
        self.angular_gain = float(self.get_parameter("angular_gain").value)
        self.max_linear_x = float(self.get_parameter("max_linear_x").value)
        self.max_linear_y = float(self.get_parameter("max_linear_y").value)
        self.max_angular_z = float(self.get_parameter("max_angular_z").value)
        self.obstacle_stop_dist = float(self.get_parameter("obstacle_stop_dist").value)
        self.checkpoint_save_interval_sec = float(self.get_parameter("checkpoint_save_interval_sec").value)
        self.checkpoint_keep_snapshots = _as_bool(
            self.get_parameter("checkpoint_keep_snapshots").value
        )
        self.waypoint_timeout_sec = float(self.get_parameter("waypoint_timeout_sec").value)
        self.waypoint_stall_sec = float(self.get_parameter("waypoint_stall_sec").value)
        self.waypoint_min_progress = float(self.get_parameter("waypoint_min_progress").value)
        self.shutdown_after_route = _as_bool(self.get_parameter("shutdown_after_route").value)
        # 自动存图路径（不带扩展名的"基名"，写文件时补 .pgm/.yaml）
        self.auto_map_path = Path(
            os.path.expanduser(str(self.get_parameter("map_save_path").value))
        ).expanduser()
        # 手动存图路径：没指定就在自动路径后面加 "_manual" 后缀，避免覆盖自动图
        manual_save_path = str(self.get_parameter("manual_save_path").value).strip()
        self.manual_map_path = (
            Path(os.path.expanduser(manual_save_path)).expanduser()
            if manual_save_path
            else self.auto_map_path.with_name(f"{self.auto_map_path.name}_manual")
        )
        report_path = str(self.get_parameter("route_report_path").value).strip()
        self.route_report_path = (
            Path(os.path.expanduser(report_path)).expanduser()
            if report_path
            else self.auto_map_path.with_name(f"{self.auto_map_path.name}_route_report.json")
        )

        # ---- 传感器缓存：由订阅回调持续刷新，控制循环直接读 ----
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.laser_ranges: Optional[List[float]] = None  # None 表示还没收到过激光
        self.scan_angle_min = -math.pi
        self.scan_angle_increment = 0.0

        # ---- 状态机内部变量 ----
        # 使用仿真时钟做路线超时；Gazebo RTF < 1 时墙钟会误判 no_progress。
        self._boot_time = -1.0            # 首次拿到有效仿真时间的时刻（-1 = 还没拿到）
        self._running = False             # 是否已开始沿路线行驶
        self._route_idx = 0               # 当前目标路点在 self.route 里的下标
        self._hold_until = 0.0            # 到达路点后原地停留的截止时刻
        self._map_saved = False           # 自动存图是否已成功
        self._manual_map_saved = False    # 手动存图是否已成功
        self._route_finished = False      # 路线是否已结束（结束后 _tick 只发状态）
        self._phase = "waiting_for_sensors"  # 对外汇报的阶段名，见 _finish_route 等处
        self._waypoint_results: List[dict] = []  # 每个路点的到达/跳过记录
        self._last_status_publish = 0.0   # 状态 topic 的节流时间戳（这个用墙钟即可）
        self._last_checkpoint_save = -1.0
        self._checkpoint_index = 0
        # 下面三个变量服务于"超时/停滞"判断：
        self._active_route_idx = -1       # 当前正在计时的路点下标（-1 表示需要重新计时）
        self._waypoint_started_at = -1.0  # 开始驶向当前路点的时刻
        self._last_waypoint_progress = -1.0  # 上一次"距离显著缩短"的时刻
        self._best_waypoint_dist = float("inf")  # 到当前路点的历史最近距离

        # ---- 订阅/发布/服务 ----
        self.odom_sub = self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, str(self.get_parameter("scan_topic").value), self._scan_cb, 10
        )
        # /map 必须用 TRANSIENT_LOCAL（"锁存"）QoS 订阅：slam_toolbox 只在地图
        # 更新时才发布，锁存 QoS 能让晚加入的订阅者也立刻收到最后一帧地图。
        # QoS 不匹配（比如用默认 VOLATILE 去订锁存的发布者）会导致收不到消息，
        # 这是 ROS 2 初学者最常见的坑之一。
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._latest_map: Optional[OccupancyGrid] = None
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self._map_cb, map_qos
        )
        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("cmd_topic").value), 10)
        # 状态 topic 同样用锁存 QoS 发布，让评测脚本随时订阅都能拿到最新状态。
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            status_qos,
        )
        self.save_map_service = self.create_service(Trigger, "~/save_map", self._save_map_cb)
        self.status_service = self.create_service(Trigger, "~/status", self._status_cb)
        # 主控制循环：50ms 一次（20Hz），足以平滑地跟踪路点。
        self.timer = self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f"Mapping route driver ready: {len(self.route)} waypoints from {self.route_file}"
        )
        self._publish_status(force=True)

    def _odom_cb(self, msg: Odometry) -> None:
        """里程计回调：缓存机器人平面位姿 (x, y, yaw)。"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # 四元数 -> 偏航角（yaw）。因为机器人只在平面上运动（roll/pitch 恒为 0），
        # 可以用这个简化公式直接提取绕 Z 轴的旋转，不必引入完整的 TF 变换库。
        self.robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _scan_cb(self, msg: LaserScan) -> None:
        """激光回调：缓存整帧测距和角度参数，供 _front_distance 使用。"""
        self.laser_ranges = list(msg.ranges)
        self.scan_angle_min = float(msg.angle_min)
        self.scan_angle_increment = float(msg.angle_increment)

    def _map_cb(self, msg: OccupancyGrid) -> None:
        """/map 回调：只缓存最新一帧，存图时直接用它写文件。"""
        self._latest_map = msg

    def _front_distance(self) -> float:
        """从激光帧里提取"机器人正前方"最近的障碍距离。

        取正前方约 ±0.10 弧度（或至少覆盖 1/24 圈的激光束）扇区内的
        最小有效距离。0.02m 以下多为传感器噪声/机体自遮挡，12m 以上
        视为无穷远，都被过滤掉。没有激光数据时返回 +inf（即不刹车），
        因为启动检查已保证开跑前必然有激光。
        """
        if not self.laser_ranges:
            return float("inf")
        increment = abs(self.scan_angle_increment)
        if increment < 1e-9:
            # 有些驱动不填 angle_increment，按整圈均分兜底。
            increment = (2.0 * math.pi) / len(self.laser_ranges)
        values = []
        for index, value in enumerate(self.laser_ranges):
            angle = self.scan_angle_min + index * increment
            delta = _angle_diff(angle, 0.0)
            if abs(delta) <= max(0.10, (len(self.laser_ranges) // 24) * increment):
                if 0.02 < value < 12.0:
                    values.append(value)
        clean = values
        return min(clean) if clean else float("inf")

    def _now_sec(self) -> float:
        """返回当前时间（秒）。

        注意用的是 self.get_clock()：当节点参数 use_sim_time=true 时，
        它读的是 /clock topic 上的仿真时间而不是系统墙钟。所有路线
        计时都必须经过这个函数，原因见模块 docstring。
        """
        return self.get_clock().now().nanoseconds * 1e-9

    def _start_if_ready(self, now: float) -> None:
        """状态机的"起跑闸门"：条件全满足才把 _running 置 True。

        依次检查：还没在跑、还没存过图、激光已经来数据、仿真时钟已经
        开始走（Gazebo 未启动时 sim time 一直是 0，now <= 0 说明时钟
        还没就绪，不能拿它当基准）、启动延时已过。启动延时是为了给
        slam_toolbox 和传感器一点稳定时间，避免机器人一动地图就歪。
        """
        if self._running:
            return
        if self._map_saved:
            return
        if self.laser_ranges is None:
            return
        if self._boot_time < 0.0:
            if now <= 0.0:
                # 仿真时钟还没开始走，无法计时，下个周期再看。
                return
            # 第一次拿到有效时间：以此为基准初始化所有计时器。
            self._boot_time = now
            self._last_checkpoint_save = now
            self._waypoint_started_at = now
            self._last_waypoint_progress = now
        if now - self._boot_time < self.startup_delay_sec:
            return
        self._running = True
        self._phase = "automatic_mapping"
        self.get_logger().info("Starting mapping route traversal")
        self._publish_status(force=True)

    def _tick(self) -> None:
        """主控制循环（20Hz），路点推进状态机的全部逻辑都在这里。

        每个周期按顺序做：
        1. 路线已结束 -> 只维持状态发布，不再动。
        2. 检查是否满足起跑条件（_start_if_ready）。
        3. 到点检查是否需要存检查点地图。
        4. 路点全部处理完 -> 收尾（_finish_route）。
        5. 处于"到达后停留"窗口 -> 发零速度原地等待。
        6. 否则：计算到当前路点的距离，更新进展记录，然后三选一：
           a) 距离进入容差 -> （可选对准朝向后）记为 reached，推进到下一个路点；
           b) 超时/停滞 -> 记为 skipped，推进到下一个路点；
           c) 都不是 -> 用 P 控制发一帧速度指令继续开。
        """
        if self._route_finished:
            self._publish_status()
            return
        now = self._now_sec()
        self._publish_status()
        self._start_if_ready(now)
        if not self._running:
            return

        self._try_checkpoint_save(now)

        if self._route_idx >= len(self.route):
            self._finish_route()
            return

        if now < self._hold_until:
            # 到达路点后的短暂停留：发布空 Twist（全零）确保机器人刹停，
            # 让激光在新位置多扫几帧，地图质量更好。
            self.cmd_pub.publish(Twist())
            return

        target = self.route[self._route_idx]
        tx = float(target["x"])
        ty = float(target["y"])
        name = str(target.get("name", f"wp_{self._route_idx}"))

        dx = tx - self.robot_x
        dy = ty - self.robot_y
        dist = math.hypot(dx, dy)
        # ---- 进展跟踪：为超时/停滞判断提供数据 ----
        if self._active_route_idx != self._route_idx:
            # 刚切换到新路点：重置计时器和"历史最近距离"。
            self._active_route_idx = self._route_idx
            self._waypoint_started_at = now
            self._last_waypoint_progress = now
            self._best_waypoint_dist = dist
        elif dist + self.waypoint_min_progress < self._best_waypoint_dist:
            # 比历史最近距离又靠近了至少 waypoint_min_progress 米，才算
            # "有进展"。加这个阈值是为了过滤里程计噪声引起的毫米级抖动，
            # 否则机器人卡在墙角原地打滑也会被误认为在前进。
            self._best_waypoint_dist = dist
            self._last_waypoint_progress = now

        # ---- 分支 a：已进入到达容差 ----
        target_tolerance = float(target.get("tolerance", self.goal_tolerance))
        if dist <= target_tolerance:
            # 路点可以额外指定 yaw：位置到了但朝向没对准时，先原地转向。
            # 常用于让机器人在门口"扭头"扫一眼房间内部。
            target_yaw = target.get("yaw")
            if target_yaw is not None:
                yaw_error = _angle_diff(float(target_yaw), self.robot_yaw)
                if abs(yaw_error) > self.yaw_tolerance:
                    twist = Twist()
                    twist.angular.z = _clamp(yaw_error * self.angular_gain, self.max_angular_z)
                    self.cmd_pub.publish(twist)
                    return
            self.get_logger().info(f"Reached {name} at ({tx:.2f}, {ty:.2f})")
            self._record_waypoint(name, "reached", "goal_tolerance", dist, now)
            self._route_idx += 1
            self._active_route_idx = -1  # 触发下个周期为新路点重置计时
            self._hold_until = now + float(target.get("dwell_sec", self.hold_sec))
            self.cmd_pub.publish(Twist())
            return

        # ---- 分支 b：超时或停滞，放弃当前路点 ----
        # 跳过而不是死等，保证整条路线总能走完、最终一定会存图；
        # 个别路点没到会如实记录在报告里，供事后分析。
        skip_reason = self._skip_reason(now)
        if skip_reason:
            if skip_reason == "timeout":
                self.get_logger().warn(
                    f"Skipping {name}: timeout after {self.waypoint_timeout_sec:.0f}s, dist={dist:.2f}m"
                )
            else:
                self.get_logger().warn(
                    f"Skipping {name}: no progress for {self.waypoint_stall_sec:.0f}s, dist={dist:.2f}m"
                )
            self._record_waypoint(name, "skipped", skip_reason, dist, now)
            self._route_idx += 1
            self._active_route_idx = -1
            self._hold_until = now + self.hold_sec
            self.cmd_pub.publish(Twist())
            return

        # ---- 分支 c：继续向路点行驶（P 控制） ----
        # 把世界坐标系下的位置误差 (dx, dy) 旋转到机器人本体坐标系：
        # body_x = 前后方向误差，body_y = 左右方向误差（旋转矩阵的逆变换）。
        # 麦克纳姆轮可以直接横移，所以 body_y 也能作为 linear.y 输出，
        # 差速小车则只能用 linear.x + angular.z。
        body_x = math.cos(self.robot_yaw) * dx + math.sin(self.robot_yaw) * dy
        body_y = -math.sin(self.robot_yaw) * dx + math.cos(self.robot_yaw) * dy
        heading = math.atan2(dy, dx)
        yaw_error = _angle_diff(heading, self.robot_yaw)

        # 比例控制：误差乘增益再限幅。误差大时全速，接近目标自动减速。
        twist = Twist()
        twist.linear.x = _clamp(body_x * self.linear_gain, self.max_linear_x)
        twist.linear.y = _clamp(body_y * self.linear_gain, self.max_linear_y)
        twist.angular.z = _clamp(yaw_error * self.angular_gain, self.max_angular_z)

        # 安全保护：前方太近就禁止继续前进（只允许零或负方向速度，
        # 即可以后退/停下，不会硬怼上去）。转向不受限，机器人还能转出来。
        front = self._front_distance()
        if front < self.obstacle_stop_dist:
            twist.linear.x = min(0.0, twist.linear.x)
            twist.linear.y = min(0.0, twist.linear.y)

        self.cmd_pub.publish(twist)

    def _skip_reason(self, now: float) -> str:
        """判断当前路点是否应该被放弃，返回原因字符串（空串 = 不放弃）。

        两条独立规则（参数设为 0 可分别禁用）：
        - timeout：从开始驶向该路点起累计超过 waypoint_timeout_sec。
        - no_progress：连续 waypoint_stall_sec 内距离没有显著缩短
          （"显著"的定义见 _tick 里对 _best_waypoint_dist 的更新逻辑）。
        注意 now 来自仿真时钟，不受 Gazebo 实时因子影响。
        """
        if self.waypoint_timeout_sec > 0.0 and now - self._waypoint_started_at > self.waypoint_timeout_sec:
            return "timeout"
        if self.waypoint_stall_sec > 0.0 and now - self._last_waypoint_progress > self.waypoint_stall_sec:
            return "no_progress"
        return ""

    def _record_waypoint(self, name: str, status: str, reason: str, dist: float, now: float) -> None:
        """把单个路点的处理结果（到达/跳过、耗时、剩余距离、当时位姿）记入列表。

        这些记录最终进入 route_report.json，是评测"路线完成质量"的原始数据。
        """
        self._waypoint_results.append(
            {
                "index": self._route_idx,
                "name": name,
                "status": status,
                "reason": reason,
                "remaining_distance": round(dist, 3),
                "elapsed_sec": round(now - self._waypoint_started_at, 3),
                "robot_pose": self._robot_pose(),
            }
        )
        self._publish_status(force=True)

    def _finish_route(self) -> None:
        """路线收尾：刹停、存图、写报告、决定后续阶段。

        存图成功后有两种走向（由 shutdown_after_route 参数决定）：
        - complete：任务完成，节点自动退出；
        - manual_assist_ready：节点保持存活，允许老师/学生手动遥控补扫
          没扫好的角落，再通过 ~/save_map 服务另存一份手动地图。
        """
        if self._route_finished:
            self.cmd_pub.publish(Twist())
            return
        self.get_logger().info("Route complete, saving map")
        self.cmd_pub.publish(Twist())
        self._map_saved = self._try_save_map(self.auto_map_path)
        self._running = False
        self._route_finished = True
        if self._map_saved:
            self._phase = "complete" if self.shutdown_after_route else "manual_assist_ready"
            self.get_logger().info(
                "Mapping route complete"
                if self.shutdown_after_route
                else "Automatic route complete; manual assist is ready"
            )
        else:
            self._phase = "save_failed"
            self.get_logger().error("Mapping route finished, but the automatic map save failed")
        self._write_route_report()
        self._publish_status(force=True)
        if self.shutdown_after_route:
            rclpy.try_shutdown()

    def _try_checkpoint_save(self, now: float) -> None:
        """途中定期存"检查点"地图，防止长路线中途崩溃后一无所获。

        默认模式（checkpoint_keep_snapshots=False）每次都覆盖写主地图路径，
        磁盘上始终只有一份"最新进度"；开启快照模式则每个检查点带序号
        单独保存，可用于课堂上展示地图是如何一步步"长出来"的。
        """
        if self.checkpoint_save_interval_sec <= 0.0:
            return
        if now - self._last_checkpoint_save < self.checkpoint_save_interval_sec:
            return
        self._last_checkpoint_save = now
        self.get_logger().info("Saving route mapping checkpoint")
        save_base = None
        if self.checkpoint_keep_snapshots:
            self._checkpoint_index += 1
            save_base = self.auto_map_path.with_name(
                f"{self.auto_map_path.name}_checkpoint_{self._checkpoint_index:02d}"
            )
        self._try_save_map(save_base)

    def _try_save_map(self, save_base: Optional[Path] = None) -> bool:
        """存图，成功返回 True。策略是"直接写文件优先，map_saver_cli 兜底"。

        第一优先级：把本节点缓存的最新 /map 消息直接写成 PGM/YAML
        （write_occupancy_grid_pair）。这样做不需要再启动任何外部进程，
        也就完全不存在"新订阅者收不到锁存消息"的时序问题。

        兜底方案：如果本节点自己还没收到过 /map（极少见），退回去调用
        官方的 map_saver_cli，最多重试 3 次。它在 sim time 下偶发失败
        的原因是：map_saver_cli 作为一个临时节点启动后要先等 /clock 和
        /map 都就绪，45 秒窗口内没等到就报错退出——这正是本项目改用
        直接写文件的动机。
        """
        if not bool(self.get_parameter("auto_save_map").value):
            return False
        if save_base is None:
            save_base = self.auto_map_path
        save_base.parent.mkdir(parents=True, exist_ok=True)

        # 优先直接写缓存的 /map（不依赖 map_saver_cli 订阅时机）。
        if self._latest_map is not None:
            try:
                write_occupancy_grid_pair(save_base, self._latest_map)
                self.get_logger().info(f"Map saved to {save_base}.pgm/.yaml")
                return True
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Direct map write failed: {exc}")

        # 兜底：调用官方 map_saver_cli（外部子进程），最多试 3 次。
        for attempt in range(1, 4):
            try:
                completed = subprocess.run(
                    [
                        "ros2",
                        "run",
                        "nav2_map_server",
                        "map_saver_cli",
                        "-f",
                        str(save_base),
                        "--ros-args",
                        "-p",
                        # 让 map_saver 也用锁存 QoS 订阅 /map，并跟随仿真时间，
                        # 否则它在仿真环境下几乎必然收不到地图。
                        "map_subscribe_transient_local:=true",
                        "-p",
                        "use_sim_time:=true",
                    ],
                    check=False,
                    capture_output=True,
                    text=True,
                    timeout=45,
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Map save failed to start (try {attempt}): {exc}")
                continue

            if completed.returncode == 0:
                self.get_logger().info(f"Map saved to {save_base}.pgm/.yaml")
                return True
            self.get_logger().warn(
                f"map_saver_cli try {attempt} failed: "
                f"{completed.stderr.strip() or completed.stdout.strip()}"
            )
            time.sleep(1.0)

        self.get_logger().error(f"Map save failed for {save_base}")
        return False

    def _robot_pose(self) -> dict:
        """把当前位姿打包成可 JSON 序列化的 dict（进报告用）。"""
        return {
            "x": round(self.robot_x, 3),
            "y": round(self.robot_y, 3),
            "yaw": round(self.robot_yaw, 3),
        }

    def _status_payload(self) -> dict:
        """组装完整的状态报告 dict（topic、服务、报告文件三处共用）。"""
        report = build_route_report(
            route_file=self.route_file,
            phase=self._phase,
            route_size=len(self.route),
            waypoint_results=self._waypoint_results,
            robot_pose=self._robot_pose(),
            auto_map_path=str(self.auto_map_path),
            manual_map_path=str(self.manual_map_path),
            auto_map_saved=self._map_saved,
            manual_map_saved=self._manual_map_saved,
        )
        report["current_waypoint_index"] = min(self._route_idx, len(self.route))
        report["manual_assist_available"] = self._phase in (
            "manual_assist_ready",
            "manual_map_saved",
        )
        return report

    def _publish_status(self, force: bool = False) -> None:
        """发布 JSON 状态到状态 topic，默认限速为每秒最多一次。

        节流用的是墙钟 time.time() 而不是仿真时钟——这里是对的：状态
        发布频率关心的是"现实中订阅者多久收到一次"，与仿真快慢无关。
        """
        now = time.time()
        if not force and now - self._last_status_publish < 1.0:
            return
        self._last_status_publish = now
        try:
            self.status_pub.publish(String(data=json.dumps(self._status_payload(), ensure_ascii=False)))
        except Exception as exc:  # noqa: BLE001 - ROS context can close during SIGINT
            if rclpy.ok():
                self.get_logger().warn(f"Mapping status publish failed: {exc}")

    def _write_route_report(self) -> None:
        """把状态报告落盘成 JSON 文件，作为本次建图运行的持久化证据。"""
        try:
            self.route_report_path.parent.mkdir(parents=True, exist_ok=True)
            self.route_report_path.write_text(
                json.dumps(self._status_payload(), ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            self.get_logger().info(f"Route report saved to {self.route_report_path}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Route report save failed: {exc}")

    def _save_map_cb(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """~/save_map 服务回调：先刹停再存图。

        存到哪里取决于阶段：自动路线已结束 -> 存手动地图路径（这是
        "人工补扫"流程的产物，与自动地图分开保存便于对比）；路线
        还在跑 -> 存自动地图路径（相当于手动触发一次检查点）。
        """
        self.cmd_pub.publish(Twist())
        save_path = self.manual_map_path if self._route_finished else self.auto_map_path
        response.success = self._try_save_map(save_path)
        if response.success:
            if self._route_finished:
                self._manual_map_saved = True
                self._phase = "manual_map_saved"
            response.message = f"Map saved to {save_path}.pgm/.yaml"
            self._write_route_report()
            self._publish_status(force=True)
        else:
            response.message = f"Map save failed for {save_path}"
        return response

    def _status_cb(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """~/status 服务回调：把状态 JSON 塞进 Trigger 响应的 message 字段。"""
        response.success = True
        response.message = json.dumps(self._status_payload(), ensure_ascii=False)
        return response

    def mark_interrupted(self) -> None:
        """被 Ctrl+C 打断时调用：把阶段标成 interrupted 并留下报告。

        这样评测脚本能区分"跑完了"和"中途被杀"，不会把残缺数据当成功。
        """
        if self._route_finished:
            return
        self._phase = "interrupted"
        self._write_route_report()
        self._publish_status(force=True)


def main(args=None) -> None:
    """入口函数：初始化 ROS、spin 节点，并保证退出路径的健壮性。

    finally 里的多层 try 看起来啰嗦，但每一层都对应一种真实的退出场景：
    - 先发一帧零速度，避免节点死了机器人还带着最后的速度指令继续跑；
    - 如果自动存图还没成功过，抓住最后机会再存一次（哪怕是 Ctrl+C 退出）；
    - destroy_node / try_shutdown 期间再次收到 Ctrl+C 也不能抛异常炸出去。
    """
    rclpy.init(args=args)
    node = MecaMindMappingRouteDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.mark_interrupted()
    finally:
        try:
            if rclpy.ok():
                try:
                    node.cmd_pub.publish(Twist())
                except Exception:
                    pass
            if rclpy.ok() and not node._map_saved:
                node._try_save_map()
        finally:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
            finally:
                try:
                    rclpy.try_shutdown()
                except Exception:
                    pass


if __name__ == "__main__":
    main()
