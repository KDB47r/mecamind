"""MecaMind 安全门（safety gate）节点：速度指令的最后一道安全关卡。

在速度链路 ``Nav2 -> /cmd_vel_raw -> velocity_smoother -> /cmd_vel_nav ->
safety_gate -> /cmd_vel`` 中，本节点位于最末端：无论上游是 Nav2、巡航
脚本还是手动遥控，速度指令都必须经过它"过滤"后才能到达底盘。

工作原理：
- 订阅激光雷达 topic ``/scan``（参数 scan_topic，默认 /scan_raw）；
- 订阅上游速度指令 topic（参数 input_cmd_topic）；
- 把 360° 激光按方向切成前/后/左/右四个扇区，取每个扇区的最近障碍距离，
  按"停止线 / 警告线"两级阈值把状态分为 stop（截断）/ warn（衰减）/
  clear（放行）；
- 处理后的速度发到输出 topic（参数 output_cmd_topic），同时把决策详情
  以 JSON 发到 ``/mecamind/safety_state`` 供调试与验收；
- 两个看门狗：激光数据超时（scan_stale）或上游指令超时（cmd_timeout）
  都会立即发布零速度急停——"传感器失明或大脑失联时，宁可停下"。

初学者建议重点阅读：
1. ``analyze_scan``：安全分区判断的核心，理解两级阈值和线性衰减；
2. ``gate_twist``：决策如何具体作用到 Twist 的各个速度分量上
   （麦克纳姆轮可以横移，所以 y 方向速度也要单独管）；
3. ``_command_watchdog``：为什么需要指令超时看门狗。

注意：本文件上半部分（analyze_scan、gate_twist 等）是不依赖 ROS 的
纯函数，方便单元测试；下半部分 SafetyGateNode 才是 ROS 节点封装。
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
import time
from typing import Iterable, Tuple

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


@dataclass(frozen=True)
class SectorDistances:
    """四个方向扇区各自的最近障碍距离（米）。

    没有障碍（或数据无效）时为正无穷 inf，这样比较逻辑可以统一写成
    "距离 >= 阈值即安全"，不需要对空扇区做特判。
    """

    front: float
    left: float
    right: float
    rear: float


@dataclass(frozen=True)
class SafetyDecision:
    """一次安全评估的完整结论，会序列化成 JSON 发到 /mecamind/safety_state。

    字段含义：
    - level: 安全等级（clear 放行 / warn 减速 / stop 截断）；
    - reason: 触发原因（如 front_stop、side_warn、scan_stale）；
    - scale_x / scale_y: 前进方向、横移方向的速度缩放系数（0.0~1.0）；
    - allow_forward / allow_left / allow_right: 各方向是否允许移动；
    - sectors: 评估依据（四个扇区的实测最近距离），便于事后复盘。
    """

    level: str
    reason: str
    scale_x: float
    scale_y: float
    allow_forward: bool
    allow_left: bool
    allow_right: bool
    sectors: SectorDistances

    def to_json(self) -> str:
        """转成 JSON。嵌套的 sectors 也要手动 asdict 一次才能序列化。"""
        data = asdict(self)
        data["sectors"] = asdict(self.sectors)
        return json.dumps(data, ensure_ascii=False)


def _angle_diff(a: float, b: float) -> float:
    """计算两个角度之差，并归一化到 (-pi, pi] 区间。

    直接相减会有"359° 和 1° 差 358°"的绕圈问题；先取 sin/cos 再用
    atan2 还原，是把角度差折回最短方向的标准技巧。
    """
    return math.atan2(math.sin(a - b), math.cos(a - b))


def _sector_min(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    center: float,
    half_width: float,
    range_min: float = 0.02,
    range_max: float = 20.0,
) -> float:
    """求激光在某个角度扇区内的最近障碍距离。

    LaserScan 消息里 ranges 是一维数组，第 i 个测距值对应的角度是
    ``angle_min + i * angle_increment``（这是 LaserScan 的标准编码方式）。
    本函数遍历所有测距点，挑出落在 [center - half_width, center + half_width]
    扇区内的有效值，返回其中最小者；扇区内无有效数据时返回 inf。

    过滤条件：NaN/inf 是雷达"没打到东西"的表示；小于 range_min 的读数
    通常是打到机器人自身外壳的噪声，也要剔除。
    """
    values = []
    for index, value in enumerate(ranges):
        if not math.isfinite(value) or value < range_min or value > range_max:
            continue
        # 由数组下标还原该测距点的实际角度。
        angle = angle_min + index * angle_increment
        # 用归一化角度差判断是否落在目标扇区内（能正确处理跨 ±pi 的后方扇区）。
        if abs(_angle_diff(angle, center)) <= half_width:
            values.append(float(value))
    return min(values) if values else float("inf")


def analyze_scan(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    front_warn_dist: float = 0.45,
    front_stop_dist: float = 0.25,
    side_warn_dist: float = 0.28,
    side_stop_dist: float = 0.16,
    sector_half_width: float = 0.45,
) -> SafetyDecision:
    """安全分区判断的核心：把一帧激光数据变成一份安全决策。

    分区模型（以机器人坐标系为准，前方为 0 弧度）：
    - 每个方向都有两条线：stop 线（更近，触碰即禁止该方向运动）和
      warn 线（更远，进入即开始减速）；
    - 前方管 scale_x（前进速度），左右两侧管 scale_y（麦轮横移速度）；
    - 前方在 warn 区时按"离 stop 线越近速度越小"线性衰减，但保底 0.25，
      避免机器人在临界处慢到几乎不动、看起来像卡死。
    """
    # 第一步：算出前/左/右/后四个扇区的最近障碍距离。
    # 扇区中心角：前=0，左=+90°，右=-90°，后=180°（弧度制）。
    sectors = SectorDistances(
        front=_sector_min(ranges, angle_min, angle_increment, 0.0, sector_half_width),
        left=_sector_min(ranges, angle_min, angle_increment, math.pi / 2.0, sector_half_width),
        right=_sector_min(ranges, angle_min, angle_increment, -math.pi / 2.0, sector_half_width),
        rear=_sector_min(ranges, angle_min, angle_increment, math.pi, sector_half_width),
    )

    # 第二步：先假设一切正常（clear、全速放行），再逐项收紧。
    allow_forward = sectors.front >= front_stop_dist
    allow_left = sectors.left >= side_stop_dist
    allow_right = sectors.right >= side_stop_dist
    level = "clear"
    reason = "all_sectors_clear"
    scale_x = 1.0
    scale_y = 1.0

    # 第三步：前方判定。stop 区 -> 前进速度直接归零；
    # warn 区 -> 按距离线性插值：刚过 warn 线时接近 1.0，
    # 逼近 stop 线时趋向下限 0.25。
    if not allow_forward:
        level = "stop"
        reason = "front_stop"
        scale_x = 0.0
    elif sectors.front < front_warn_dist:
        level = "warn"
        reason = "front_warn"
        # span 是 warn 区的宽度；max(0.001, ...) 防止两条线配得太近导致除零。
        span = max(0.001, front_warn_dist - front_stop_dist)
        scale_x = max(0.25, min(1.0, (sectors.front - front_stop_dist) / span))

    # 第四步：侧方判定。注意 "if level == clear 才升级" 的写法：
    # 前方已经是 stop/warn 时，保留前方的 level 和 reason（更紧急者优先），
    # 但侧方的缩放系数 scale_y 照常收紧。
    if not allow_left or not allow_right:
        level = "stop" if level == "clear" else level
        reason = "side_stop" if reason == "all_sectors_clear" else reason
        scale_y = 0.0
    elif sectors.left < side_warn_dist or sectors.right < side_warn_dist:
        if level == "clear":
            level = "warn"
            reason = "side_warn"
        # 侧方 warn 用固定减半，比前方的线性衰减简单——横移速度本来就不高。
        scale_y = 0.5

    return SafetyDecision(
        level=level,
        reason=reason,
        scale_x=scale_x,
        scale_y=scale_y,
        allow_forward=allow_forward,
        allow_left=allow_left,
        allow_right=allow_right,
        sectors=sectors,
    )


def gate_twist(command: Twist, decision: SafetyDecision) -> Twist:
    """按安全决策修剪一条速度指令，返回可以安全下发的新 Twist。

    各分量的处理原则：
    - linear.x（前后）：只限制"前进"（x > 0）；倒车不受前方障碍限制，
      被堵住时机器人仍能退出来；
    - linear.y（横移，麦克纳姆轮特有）：y > 0 是向左移，左侧到 stop 线
      就归零；y < 0 向右同理；否则按 scale_y 衰减；
    - angular.z（原地旋转）：不限制——旋转不产生平移位移，且往往是
      脱困所必需的动作。
    """
    gated = Twist()
    # 先原样拷贝，再按需覆盖。不直接改传入的 command，保持函数无副作用。
    gated.linear.x = command.linear.x
    gated.linear.y = command.linear.y
    gated.angular.z = command.angular.z

    if command.linear.x > 0.0:
        # 只有前进分量按 scale_x 缩放（stop 区时 scale_x=0 即截断）。
        gated.linear.x = command.linear.x * decision.scale_x
    # 横移方向：先看"硬禁止"（对应侧已到 stop 线），再看"软衰减"。
    if command.linear.y > 0.0 and not decision.allow_left:
        gated.linear.y = 0.0
    elif command.linear.y < 0.0 and not decision.allow_right:
        gated.linear.y = 0.0
    else:
        gated.linear.y = command.linear.y * decision.scale_y
    return gated


def command_timed_out(
    last_command_time: float,
    now: float,
    timeout_sec: float,
    command_active: bool,
) -> bool:
    """判断上游速度指令是否已超时（应触发急停）。

    只有在"机器人正在动"（command_active）时超时才有意义：静止状态
    下上游安静是正常的，不需要反复发急停。timeout_sec <= 0 表示禁用
    该看门狗。
    """
    return command_active and timeout_sec > 0.0 and now - last_command_time > timeout_sec


class SafetyGateNode(Node):
    """安全门 ROS 节点：把上面的纯函数接到实际的 topic 上。

    数据流是"指令驱动"的：每收到一条上游速度指令（_cmd_cb），就用
    最新一帧激光做一次安全评估，把修剪后的速度转发出去。激光回调
    （_scan_cb）只负责缓存数据，不主动发布——没有指令时机器人本来
    就不动，无需评估。
    """

    def __init__(self) -> None:
        super().__init__("mecamind_safety_gate")
        # ---- 参数：topic 名称与各方向的安全距离阈值，均可在 launch 中覆盖 ----
        self.declare_parameter("scan_topic", "/scan_raw")
        self.declare_parameter("input_cmd_topic", "/controller/cmd_vel_nav")
        self.declare_parameter("output_cmd_topic", "/controller/cmd_vel")
        self.declare_parameter("state_topic", "/mecamind/safety_state")
        self.declare_parameter("front_warn_dist", 0.45)
        self.declare_parameter("front_stop_dist", 0.25)
        self.declare_parameter("side_warn_dist", 0.28)
        self.declare_parameter("side_stop_dist", 0.16)
        # 两个看门狗的超时阈值（秒）。
        self.declare_parameter("scan_timeout_sec", 0.60)
        self.declare_parameter("cmd_timeout_sec", 0.60)

        self._last_scan: LaserScan | None = None
        # 用 time.monotonic()（单调时钟，只增不减、不受系统对时影响）
        # 记录"墙上时间"，看门狗类逻辑用它比用 ROS 时间更稳妥——
        # 仿真暂停时 sim time 会停走，但激光断流的检测不应该跟着停。
        self._last_scan_wall_time = 0.0
        self._last_cmd_wall_time = time.monotonic()
        # 是否"正在执行非零速度指令"——决定指令超时看门狗是否武装。
        self._command_active = False

        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._scan_cb,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("input_cmd_topic").value),
            self._cmd_cb,
            10,
        )
        self.cmd_pub = self.create_publisher(
            Twist,
            str(self.get_parameter("output_cmd_topic").value),
            10,
        )
        self.state_pub = self.create_publisher(
            String,
            str(self.get_parameter("state_topic").value),
            10,
        )
        # 20Hz 的看门狗定时器：即使上游完全沉默也能及时发现并急停。
        self.watchdog_timer = self.create_timer(0.05, self._command_watchdog)
        self.get_logger().info("MecaMind safety gate ready")

    def _scan_cb(self, msg: LaserScan) -> None:
        """缓存最新一帧激光，并记录到达时刻（供 scan_stale 判断）。"""
        self._last_scan = msg
        self._last_scan_wall_time = time.monotonic()

    def _decision(self) -> SafetyDecision:
        """基于最新激光做一次安全评估。

        激光从未到达或已超时（雷达掉线、驱动崩溃）时，返回一份全禁止的
        stop 决策（reason=scan_stale）——传感器失明时绝不放行任何运动。
        此时 sectors 填 inf 表示"没有可信的距离数据"。
        """
        timeout = float(self.get_parameter("scan_timeout_sec").value)
        if self._last_scan is None or time.monotonic() - self._last_scan_wall_time > timeout:
            return SafetyDecision(
                level="stop",
                reason="scan_stale",
                scale_x=0.0,
                scale_y=0.0,
                allow_forward=False,
                allow_left=False,
                allow_right=False,
                sectors=SectorDistances(float("inf"), float("inf"), float("inf"), float("inf")),
            )
        scan = self._last_scan
        # 阈值参数每次都现读，这样运行中用 ros2 param set 调参能立即生效。
        return analyze_scan(
            scan.ranges,
            float(scan.angle_min),
            float(scan.angle_increment),
            front_warn_dist=float(self.get_parameter("front_warn_dist").value),
            front_stop_dist=float(self.get_parameter("front_stop_dist").value),
            side_warn_dist=float(self.get_parameter("side_warn_dist").value),
            side_stop_dist=float(self.get_parameter("side_stop_dist").value),
        )

    def _cmd_cb(self, msg: Twist) -> None:
        """收到上游速度指令：评估 -> 修剪 -> 转发 -> 发布安全状态。

        这是安全门的主通路，每条指令都要走一遍。
        """
        self._last_cmd_wall_time = time.monotonic()
        decision = self._decision()
        gated = gate_twist(msg, decision)
        # 修剪后仍有任一分量非零，才认为"机器人在动"、武装指令看门狗。
        # 用 1e-6 的小阈值而不是 != 0，避免浮点误差导致误判。
        self._command_active = any(
            abs(value) > 1.0e-6
            for value in (gated.linear.x, gated.linear.y, gated.angular.z)
        )
        self.cmd_pub.publish(gated)
        # 决策详情同步发布，rviz/验收脚本可以据此解释"机器人为什么变慢了"。
        state = String()
        state.data = decision.to_json()
        self.state_pub.publish(state)

    def _command_watchdog(self) -> None:
        """指令超时看门狗（20Hz）：上游断流时主动发零速度急停。

        为什么需要它：如果 Nav2 或遥控节点崩溃，最后一条非零速度指令
        会让底盘"锁定"在那个速度一直跑下去。看门狗发现指令流中断超过
        cmd_timeout_sec 就补发一帧零 Twist 把机器人停住，并发布
        reason=cmd_timeout 的 stop 状态说明原因。
        """
        timeout = float(self.get_parameter("cmd_timeout_sec").value)
        if not command_timed_out(
            self._last_cmd_wall_time,
            time.monotonic(),
            timeout,
            self._command_active,
        ):
            return
        # 置回非活跃，保证急停只发一次，不会 20Hz 刷屏。
        self._command_active = False
        self.cmd_pub.publish(Twist())
        # 借用当前决策里的扇区实测值，让超时状态也带上现场距离信息。
        decision = self._decision()
        timeout_state = SafetyDecision(
            level="stop",
            reason="cmd_timeout",
            scale_x=0.0,
            scale_y=0.0,
            allow_forward=False,
            allow_left=False,
            allow_right=False,
            sectors=decision.sectors,
        )
        self.state_pub.publish(String(data=timeout_state.to_json()))


def main(args=None) -> None:
    """节点入口。安全节点的退出清理写得格外谨慎：

    Ctrl+C 可能在 spin、destroy_node 的任意时刻打断进程，多层
    try/finally 保证无论在哪一步被打断，最终都会走到 try_shutdown
    （它与 shutdown 不同：上下文已关闭时不会抛异常）。
    """
    rclpy.init(args=args)
    node = SafetyGateNode()
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
