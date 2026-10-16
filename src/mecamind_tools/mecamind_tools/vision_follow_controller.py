"""视觉跟随控制器：根据感知事件计算速度指令，让机器人跟着目标走。

在系统中的角色：感知 -> 控制 链路的最后一环。
- 订阅 /mecamind/perception_event（JSON）：perception_filter 输出的
  已确认目标事件（含归一化坐标 cx 和目标宽度 width）；
- 订阅 /mecamind/follow_enable（Bool）：跟随功能总开关，默认关闭，
  由任务调度/状态机在用户说"跟随我"之后打开；
- 发布 /cmd_vel_follow（Twist）：速度指令，送往速度仲裁器
  （再经安全门到 /cmd_vel）；勿直接抢 Nav2 的控制器入口。

控制思想（完整 PID）：
- 转向通道：误差 = cx - 0.5，输出 angular = Kp·e + Ki·∫e + Kd·de/dt；
- 距离通道：先用小孔成像把画面宽度反算成真实距离
  （dist ≈ 柱子实际直径 × 焦距系数 / 归一化宽度），再对
  「距离误差 = 实测距离 - 期望距离」做 PID：离得越远输出越大、
  自然加速追赶，输出饱和在 max_linear（最大限速）；过近误差为负，
  自然后退。之所以不直接用宽度误差：宽度 ∝ 1/距离，是强非线性量，
  "远处宽度变化很小"会让远距离时的加速度严重不足。
- 死区、积分限幅、输出限幅防止震荡与积分饱和。

安全设计：
1. 使能开关：follow_enable 不打开则不动；
2. confirmed 门槛：只追过滤节点确认过的目标；
3. 看门狗超时：断流后刹停或搜索；丢目标时清零积分。

激光避障融合（scan_avoidance_enable，默认开）：
- 纯视觉 PID 只认目标、看不见障碍物，目标拐弯时小车会"切角"蹭上
  障碍物内侧；末端安全门又只会减速/截断，从不改变方向——两者叠加
  就是"卡在障碍物旁边"的根因；
- 本节点订阅 /scan，把左前/右前/正前三个扇区的最近距离转成
  「转向偏置 + 麦轮横移」叠加在 PID 输出上（见 compute_avoidance_bias），
  离障碍越近推开得越狠，离得远则完全不干预；
- 关键细节：红柱自己也会被激光看到！若不处理，车追柱子时会把柱子
  当成"前方障碍"而减速——离得越近减得越狠，永远加不上速（表现为
  "柱子快时车远远跟不上"）。因此正前扇区的回波若不比视觉估出的
  目标距离更近，就视为看到的是红柱本身、不产生减速压力
  （见 front_distance_ignoring_target）；真有箱子挡在车和柱子中间时
  回波会明显更近，照常减速；
- 丢失目标时不再原地打转（遮挡场景下原地转永远看不见目标），而是
  按目标消失的画面方向先"弧线绕行"一段（search_arc_sec），一边前进
  一边转弯绕过遮挡物，超时仍没找到再退化为原地旋转。
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from mecamind_tools.safety_layer import _sector_min


@dataclass(frozen=True)
class PidState:
    """单通道 PID 记忆：积分项与上一拍误差。"""

    integral: float = 0.0
    prev_error: float = 0.0


@dataclass(frozen=True)
class FollowCommand:
    """一次控制律计算的输出：线速度、角速度、是否有效及更新后的 PID 状态。"""

    linear_x: float
    angular_z: float
    active: bool
    reason: str
    angular_state: PidState = PidState()
    linear_state: PidState = PidState()


def _pid_step(
    error: float,
    state: PidState,
    kp: float,
    ki: float,
    kd: float,
    dt: float,
    i_limit: float,
) -> tuple[float, PidState]:
    """标准位置式 PID 一步；dt<=0 时退化为纯 P。"""
    if dt <= 1e-6:
        return kp * error, PidState(0.0, error)
    integral = state.integral + error * dt
    if i_limit > 0.0:
        integral = max(-i_limit, min(i_limit, integral))
    derivative = (error - state.prev_error) / dt
    output = kp * error + ki * integral + kd * derivative
    return output, PidState(integral=integral, prev_error=error)


def estimate_target_distance(
    width: float,
    real_width: float = 0.20,
    hfov: float = 1.047,
    max_dist: float = 6.0,
) -> float:
    """小孔成像反算目标距离（米）。

    归一化像宽 width ≈ f_norm × 实际直径 / 距离，其中
    f_norm = 0.5 / tan(hfov/2)（归一化焦距，本项目相机 ≈0.866）。
    远处 width 很小、噪声占比大，距离截断在 max_dist 防止爆表。
    """
    if width <= 1e-3:
        return max_dist
    f_norm = 0.5 / math.tan(hfov / 2.0)
    return min(max_dist, real_width * f_norm / width)


def compute_follow_command(
    cx: float,
    target_width: float,
    desired_distance: float = 0.80,
    target_real_width: float = 0.20,
    camera_hfov: float = 1.047,
    center_deadband: float = 0.05,
    max_linear: float = 0.20,
    max_angular: float = 0.8,
    kp_angular: float = 2.0,
    ki_angular: float = 0.12,
    kd_angular: float = 0.06,
    kp_linear: float = 0.8,
    ki_linear: float = 0.05,
    kd_linear: float = 0.04,
    i_limit_angular: float = 0.45,
    i_limit_linear: float = 0.30,
    dt: float = 0.1,
    angular_state: PidState | None = None,
    linear_state: PidState | None = None,
) -> FollowCommand:
    """跟随控制律：PID 转向 + 距离域 PID 速度（越远越快，饱和于 max_linear）。

    转向通道：
    - 误差 = cx - 0.5；死区内误差置 0，并缓慢衰减积分，减轻中心抖动；
    - 输出取反：图像偏右（正误差）应对应 ROS 顺时针（负 angular.z）。

    距离通道：
    - 先由画面宽度反算真实距离（estimate_target_distance）；
    - 误差 = 实测距离 - desired_distance（正=太远该加速追）；
    - kp×误差随距离线性增大，实现"按距离分档加速"，输出限幅
      到 max_linear 即最大限速；过近误差为负自然后退；
    - 大转向时压低线速度，先对准再加速。
    """
    ang_state = angular_state or PidState()
    lin_state = linear_state or PidState()

    error_x = cx - 0.5
    if abs(error_x) < center_deadband:
        error_x = 0.0
        # 死区内衰减积分，避免饱和后出死区猛打方向
        ang_state = PidState(integral=ang_state.integral * 0.9, prev_error=0.0)

    angular_raw, ang_state = _pid_step(
        error_x,
        ang_state,
        kp=kp_angular,
        ki=ki_angular,
        kd=kd_angular,
        dt=dt,
        i_limit=i_limit_angular,
    )
    angular = max(-max_angular, min(max_angular, -angular_raw))

    if target_width <= 0.0:
        return FollowCommand(0.0, angular, True, "no_width", ang_state, lin_state)

    distance = estimate_target_distance(
        target_width, real_width=target_real_width, hfov=camera_hfov
    )
    dist_error = distance - desired_distance
    linear_raw, lin_state = _pid_step(
        dist_error,
        lin_state,
        kp=kp_linear,
        ki=ki_linear,
        kd=kd_linear,
        dt=dt,
        i_limit=i_limit_linear,
    )
    # 过近时允许接近满速后退，略放宽负向限幅
    linear = max(-max_linear * 0.95, min(max_linear, linear_raw))

    # 转弯缩速：偏得越狠越减速，先对准再冲。地板从 0.25 放宽到 0.40、
    # 斜率放缓——绕圈拐弯时目标常年偏离画面中心，压得太狠会在目标
    # 加速段掉队追不上（避障偏置已负责防切角蹭障碍）。
    turn = abs(cx - 0.5)
    if turn > center_deadband:
        turn_scale = max(0.40, 1.0 - (turn - center_deadband) / 0.60)
        linear *= turn_scale

    return FollowCommand(linear, angular, True, "target_confirmed", ang_state, lin_state)


def should_publish_follow(enabled: bool, confirmed: bool) -> bool:
    """双条件闸门：跟随开关已打开 且 目标已被过滤节点确认，才允许发速度。"""
    return bool(enabled) and bool(confirmed)


@dataclass(frozen=True)
class AvoidanceBias:
    """一次激光避障评估的输出，叠加在视觉 PID 之上。

    - angular: 转向偏置（rad/s，正=向左转），把车头推离障碍物；
    - lateral: 麦轮横移速度（m/s，正=向左平移），差速车没有这个自由度，
      麦克纳姆轮可以"不转头直接让开"，是本项目底盘的天然优势；
    - linear_scale: 前进速度缩放（0~1），正前方越堵越慢，避免一头撞上
      再依赖安全门急刹。
    """

    angular: float
    lateral: float
    linear_scale: float


def compute_avoidance_bias(
    left_dist: float,
    right_dist: float,
    front_dist: float,
    clear_dist: float = 0.90,
    stop_dist: float = 0.30,
    max_turn: float = 0.65,
    max_lateral: float = 0.12,
) -> AvoidanceBias:
    """把左前/右前/正前三个扇区的最近障碍距离变成一份避障偏置。

    思路（简化版势场法，只有斥力没有引力——引力由视觉 PID 提供）：
    - 每个方向算一个"压力"p：距离 >= clear_dist 时 p=0（完全不干预），
      距离 <= stop_dist 时 p=1（最大推力），中间线性过渡；
    - 左右压力差 (p_right - p_left) 决定往哪边推：右侧近 → 差为正 →
      向左转（正 angular）+ 向左横移（正 lateral），左侧近则反向；
    - 正前方有压力时额外朝"更空的一侧"加一份转向，并按压力压低前进
      速度——两侧都空但正前有障碍时，光靠左右差是推不开的。
    """

    def pressure(dist: float) -> float:
        if not math.isfinite(dist) or dist >= clear_dist:
            return 0.0
        span = max(1e-3, clear_dist - stop_dist)
        return min(1.0, max(0.0, (clear_dist - dist) / span))

    p_left = pressure(left_dist)
    p_right = pressure(right_dist)
    p_front = pressure(front_dist)

    side_push = p_right - p_left
    angular = side_push * max_turn
    lateral = side_push * max_lateral
    if p_front > 0.0:
        # 朝更空的一侧转：右边更空 → 向右（负）；左边更空 → 向左（正）。
        prefer = -1.0 if right_dist >= left_dist else 1.0
        angular += prefer * p_front * max_turn
    linear_scale = max(0.0, 1.0 - p_front)
    return AvoidanceBias(angular=angular, lateral=lateral, linear_scale=linear_scale)


def front_distance_ignoring_target(
    front_dist: float,
    target_dist: float | None,
    margin: float = 0.30,
) -> float:
    """把"红柱本身的激光回波"从正前避障里豁免掉。

    红柱有碰撞体，激光看得见它；不豁免的话，车离柱子越近前方压力
    越大、速度被压得越低，形成"永远追不上"的死锁。规则：
    - 正前最近回波 >= 视觉估计的目标距离 - margin：认为回波就是
      红柱（或柱子后面的墙），返回 inf，不产生减速压力；
    - 回波明显比目标更近：说明真有障碍挡在车和柱子中间，照常减速。
    margin 吸收视觉测距误差，必须小于期望跟随距离，否则会把
    "贴脸的真障碍"也豁免掉。
    """
    if (
        target_dist is not None
        and math.isfinite(target_dist)
        and front_dist >= target_dist - margin
    ):
        return math.inf
    return front_dist


def search_turn_direction(last_cx: float) -> float:
    """丢失目标后往哪边找：目标最后出现在画面右半区就向右（顺时针，
    返回 -1），在左半区就向左（逆时针，返回 +1）。

    依据是"目标通常从它最后所在的那一侧离开画面"——被障碍物遮挡时
    尤其如此：目标绕到障碍物背面的方向，就是它消失前在画面里的方向。
    """
    return -1.0 if float(last_cx) >= 0.5 else 1.0


class VisionFollowController(Node):
    """视觉跟随节点：事件驱动发速度 + 定时看门狗兜底刹停。"""

    def __init__(self) -> None:
        super().__init__("mecamind_vision_follow_controller")
        self.declare_parameter("input_topic", "/mecamind/perception_event")
        self.declare_parameter("output_cmd_topic", "/cmd_vel_follow")
        self.declare_parameter("enable_topic", "/mecamind/follow_enable")
        # 距离通道：宽度反算距离所需的目标实际直径 / 相机水平视场角
        self.declare_parameter("desired_distance", 0.80)
        self.declare_parameter("target_real_width", 0.20)
        self.declare_parameter("camera_hfov", 1.047)
        self.declare_parameter("event_timeout_sec", 0.70)
        self.declare_parameter("max_linear", 0.20)
        self.declare_parameter("max_angular", 0.8)
        self.declare_parameter("center_deadband", 0.05)
        self.declare_parameter("kp_angular", 2.0)
        self.declare_parameter("ki_angular", 0.12)
        self.declare_parameter("kd_angular", 0.06)
        self.declare_parameter("kp_linear", 0.8)
        self.declare_parameter("ki_linear", 0.05)
        self.declare_parameter("kd_linear", 0.04)
        self.declare_parameter("i_limit_angular", 0.45)
        self.declare_parameter("i_limit_linear", 0.30)
        self.declare_parameter("search_on_loss", True)
        self.declare_parameter("search_angular", 0.55)
        # ---- 激光避障融合 + 丢失弧线搜索 ----
        self.declare_parameter("scan_avoidance_enable", True)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("avoid_clear_dist", 0.90)
        self.declare_parameter("avoid_stop_dist", 0.30)
        self.declare_parameter("avoid_max_turn", 0.65)
        self.declare_parameter("avoid_max_lateral", 0.12)
        # 正前回波与目标距离差在此范围内视为"看到的是红柱"，不减速
        self.declare_parameter("avoid_target_margin", 0.30)
        # 丢失后先弧线绕行 search_arc_sec 秒（前进+转向），再退化为原地旋转
        self.declare_parameter("search_arc_sec", 4.0)
        self.declare_parameter("search_arc_linear", 0.10)

        self._enabled = False
        self._last_event_time = 0.0
        self._last_cmd_mono = 0.0
        self._searching = False
        self._search_start_mono = 0.0
        self._search_dir = 1.0
        self._last_cx = 0.5
        self._last_scan: LaserScan | None = None
        self._ang_state = PidState()
        self._lin_state = PidState()
        self.create_subscription(String, str(self.get_parameter("input_topic").value), self._event_cb, 10)
        self.create_subscription(Bool, str(self.get_parameter("enable_topic").value), self._enable_cb, 10)
        if bool(self.get_parameter("scan_avoidance_enable").value):
            self.create_subscription(
                LaserScan, str(self.get_parameter("scan_topic").value), self._scan_cb, 10
            )
        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter("output_cmd_topic").value), 10)
        self.timer = self.create_timer(0.1, self._watchdog)
        self.get_logger().info(
            "MecaMind vision follow controller ready (PID; follow disabled by default)"
        )

    def _reset_pid(self) -> None:
        self._ang_state = PidState()
        self._lin_state = PidState()
        self._last_cmd_mono = 0.0

    def _scan_cb(self, msg: LaserScan) -> None:
        """缓存最新激光帧；扇区距离在需要发速度时才现算。"""
        self._last_scan = msg

    def _avoidance_bias(self, target_dist: float | None = None) -> AvoidanceBias:
        """基于最新激光帧计算避障偏置；无激光/未启用时返回零偏置。

        扇区划分：正前 0°、左前 +40°、右前 -40°，各 ±20°。只看前半圆
        ——跟随只会向前走，后方障碍交给安全门兜底即可。
        target_dist：视觉估计的目标距离；给定时正前扇区会豁免
        "红柱本身的回波"，避免把追踪目标当障碍物减速。
        """
        if not bool(self.get_parameter("scan_avoidance_enable").value) or self._last_scan is None:
            return AvoidanceBias(0.0, 0.0, 1.0)
        scan = self._last_scan
        angle_min = float(scan.angle_min)
        angle_inc = float(scan.angle_increment)
        front = _sector_min(scan.ranges, angle_min, angle_inc, 0.0, 0.35)
        front = front_distance_ignoring_target(
            front, target_dist, margin=float(self.get_parameter("avoid_target_margin").value)
        )
        left = _sector_min(scan.ranges, angle_min, angle_inc, 0.70, 0.35)
        right = _sector_min(scan.ranges, angle_min, angle_inc, -0.70, 0.35)
        return compute_avoidance_bias(
            left_dist=left,
            right_dist=right,
            front_dist=front,
            clear_dist=float(self.get_parameter("avoid_clear_dist").value),
            stop_dist=float(self.get_parameter("avoid_stop_dist").value),
            max_turn=float(self.get_parameter("avoid_max_turn").value),
            max_lateral=float(self.get_parameter("avoid_max_lateral").value),
        )

    def _enable_cb(self, msg: Bool) -> None:
        """关闭跟随时立刻发零速，并清零 PID 积分。"""
        was_enabled = self._enabled
        self._enabled = bool(msg.data)
        if was_enabled and not self._enabled:
            self._last_event_time = 0.0
            self._searching = False
            self._reset_pid()
            self.cmd_pub.publish(Twist())
            self.get_logger().info("Vision follow disabled")
        elif not was_enabled and self._enabled:
            self._reset_pid()
            self.get_logger().info("Vision follow enabled (PID)")

    def _event_cb(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Invalid perception event: {exc}")
            return
        if not should_publish_follow(self._enabled, bool(event.get("confirmed", False))):
            return

        now = time.monotonic()
        dt = 0.1 if self._last_cmd_mono <= 0.0 else max(0.02, min(0.5, now - self._last_cmd_mono))
        width = float(event.get("width", 0.0))
        command = compute_follow_command(
            float(event.get("cx", 0.5)),
            width,
            desired_distance=float(self.get_parameter("desired_distance").value),
            target_real_width=float(self.get_parameter("target_real_width").value),
            camera_hfov=float(self.get_parameter("camera_hfov").value),
            center_deadband=float(self.get_parameter("center_deadband").value),
            max_linear=float(self.get_parameter("max_linear").value),
            max_angular=float(self.get_parameter("max_angular").value),
            kp_angular=float(self.get_parameter("kp_angular").value),
            ki_angular=float(self.get_parameter("ki_angular").value),
            kd_angular=float(self.get_parameter("kd_angular").value),
            kp_linear=float(self.get_parameter("kp_linear").value),
            ki_linear=float(self.get_parameter("ki_linear").value),
            kd_linear=float(self.get_parameter("kd_linear").value),
            i_limit_angular=float(self.get_parameter("i_limit_angular").value),
            i_limit_linear=float(self.get_parameter("i_limit_linear").value),
            dt=dt,
            angular_state=self._ang_state,
            linear_state=self._lin_state,
        )
        self._ang_state = command.angular_state
        self._lin_state = command.linear_state
        self._last_cmd_mono = now
        self._last_cx = float(event.get("cx", 0.5))

        # 视觉 PID 结果先过一遍激光避障偏置，再限幅发布：
        # 目标在前方但侧前有障碍时，小车会边跟边"让"，走外弧而不是切角。
        # 传入目标距离估计，把红柱本身的回波从前方压力里豁免掉。
        target_dist = None
        if width > 0.0:
            target_dist = estimate_target_distance(
                width,
                real_width=float(self.get_parameter("target_real_width").value),
                hfov=float(self.get_parameter("camera_hfov").value),
            )
        bias = self._avoidance_bias(target_dist)
        max_angular = float(self.get_parameter("max_angular").value)
        twist = Twist()
        twist.linear.x = command.linear_x * bias.linear_scale
        twist.linear.y = bias.lateral
        twist.angular.z = max(-max_angular, min(max_angular, command.angular_z + bias.angular))
        self._last_event_time = now
        self._searching = False
        self.cmd_pub.publish(twist)

    def _search_twist(self) -> Twist:
        """丢失搜索的速度指令：先弧线绕行，超时退化为原地旋转。

        弧线阶段 = 前进 + 朝目标消失方向转弯，可以绕到遮挡物侧面重新
        看见目标（原地旋转对遮挡场景无效——转 360° 目标仍在障碍物背面）。
        弧线期间避障偏置照常生效，贴近遮挡物时会被推开、堵死时会减速，
        不会为了找目标撞上障碍物。
        """
        search_angular = float(self.get_parameter("search_angular").value)
        arc_sec = float(self.get_parameter("search_arc_sec").value)
        twist = Twist()
        if time.monotonic() - self._search_start_mono < arc_sec:
            bias = self._avoidance_bias()
            max_angular = float(self.get_parameter("max_angular").value)
            twist.linear.x = float(self.get_parameter("search_arc_linear").value) * bias.linear_scale
            twist.linear.y = bias.lateral
            twist.angular.z = max(
                -max_angular,
                min(max_angular, self._search_dir * search_angular + bias.angular),
            )
        else:
            twist.angular.z = self._search_dir * search_angular
        return twist

    def _watchdog(self) -> None:
        if not self._enabled:
            return
        timeout = float(self.get_parameter("event_timeout_sec").value)
        if self._searching:
            if bool(self.get_parameter("search_on_loss").value):
                self.cmd_pub.publish(self._search_twist())
            return
        if self._last_event_time <= 0.0:
            return
        if time.monotonic() - self._last_event_time <= timeout:
            return
        self._last_event_time = 0.0
        self._reset_pid()
        if bool(self.get_parameter("search_on_loss").value):
            self._searching = True
            self._search_start_mono = time.monotonic()
            self._search_dir = search_turn_direction(self._last_cx)
            self.cmd_pub.publish(self._search_twist())
            side = "右" if self._search_dir < 0.0 else "左"
            self.get_logger().info(
                f"Target lost: arc-search toward {side} (last cx={self._last_cx:.2f})",
                throttle_duration_sec=2.0,
            )
        else:
            self.cmd_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionFollowController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
