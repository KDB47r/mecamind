"""MecaMind 任务执行器（mission executor）节点。

这是导航课（第三课）里"把自然语言级指令翻译成 Nav2 导航目标"的桥梁节点：

- 订阅 topic ``/mecamind/task_command``（std_msgs/String，内容是 JSON 字符串，
  形如 ``{"intent": "navigate", "target": "kitchen"}``）；
- 解析 intent（navigate / patrol / follow / stop / cancel），把命名目标
  （如 "kitchen"、"厨房"）查表解析成地图坐标系下的 PoseStamped；
- 通过 ``NavigateToPose`` **action** 把目标发给 Nav2（bt_navigator）；
- 向 topic ``/mecamind/mission_state`` 发布 JSON 格式的任务状态，供
  验收脚本（nav_acceptance）和课堂工具观察；
- 向 ``/mecamind/follow_enable``（Bool）开关跟随模式，向 cmd_vel topic
  发零速度实现急停。

【action 和 topic 的区别】topic 是"发了就不管"的单向广播；action 则适合
导航这类长耗时任务：客户端发送 goal 后，服务端会回复"是否受理"，执行
过程中可以持续给 feedback，结束时给 result，中途还可以 cancel。本节点
就是一个 NavigateToPose 的 action 客户端。

初学者建议重点阅读：
1. ``_task_cb``：JSON 指令解析入口，理解 intent -> 模式的状态机切换；
2. ``_send_pending`` / ``_goal_response`` / ``_goal_finished``：一条 action
   goal 从发送、被受理到拿到结果的完整异步回调链；
3. ``_tick``：0.5 秒一次的定时器，如何驱动"排队中的目标"真正发出去；
4. ``_cancel_nav`` 里 ``_goal_serial`` 的作用——如何安全地作废旧回调。
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from typing import Any, Dict, List, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .nav_utils import load_yaml, pose_stamped_from_dict
from .task_scheduler import parse_move_direction


@dataclass(frozen=True)
class MissionState:
    """任务状态快照，会被序列化成 JSON 发到 /mecamind/mission_state。

    用 frozen dataclass（不可变）而不是普通 dict，好处是字段名固定、
    拼写错误在构造时就会报错，也方便用 asdict() 一键转 JSON。

    字段含义：
    - mode: 当前模式（idle / navigate / patrol / follow）；
    - last_intent: 最近一次收到的指令意图；
    - target: 当前目标名（如 "kitchen"）；
    - detail: 细节状态字符串（如 "queued:kitchen"、"reached:kitchen"、
      "failed:kitchen"），验收脚本主要靠它判断任务进展。
    """

    mode: str
    last_intent: str = ""
    target: str = ""
    detail: str = ""

    def to_json(self) -> str:
        """转成 JSON 字符串。ensure_ascii=False 让中文目标名保持可读。"""
        return json.dumps(asdict(self), ensure_ascii=False)


def normalize_goal_key(name: str) -> str:
    """把目标名归一化成查表用的键：去首尾空格、转小写、空格和连字符统一成下划线。

    这样用户发 "Living Room"、"living-room"、"living_room" 都能命中
    同一个配置项，减少课堂演示时因大小写/分隔符打错导致的失败。
    """
    return name.strip().lower().replace(" ", "_").replace("-", "_")


def load_named_goals(path: str) -> Dict[str, Dict[str, Any]]:
    """从 YAML 文件加载"命名目标点"表。

    文件结构约定为::

        goals:
          kitchen: {x: 1.0, y: 2.0, yaw: 0.0}
          bedroom: {x: ...}

    返回时键都经过 normalize_goal_key 归一化，值是原始坐标字典，
    留到真正发送时再转成 PoseStamped（那时才能填当前时间戳）。
    """
    data = load_yaml(path)
    goals = data.get("goals", {})
    if not isinstance(goals, dict):
        raise ValueError(f"named goals root.goals must be a mapping: {path}")
    result: Dict[str, Dict[str, Any]] = {}
    for key, value in goals.items():
        # 跳过格式不对的条目（比如值写成了字符串），宽容处理配置错误。
        if not isinstance(value, dict):
            continue
        result[normalize_goal_key(str(key))] = value
    return result


def resolve_named_goal(
    goals: Dict[str, Dict[str, Any]],
    target: str,
) -> Optional[Dict[str, Any]]:
    """把用户说的目标名解析成坐标字典；解析不了返回 None。

    解析分两步：
    1. 先按归一化后的英文键直接查表；
    2. 查不到再尝试中文别名表（课堂上语音/文字指令常用中文），
       把"厨房"这类叫法映射到配置文件里的英文键。
    """
    key = normalize_goal_key(target)
    if key in goals:
        return goals[key]
    # 中文别名 -> 配置文件中的英文键。注意"大厅"和"门口"都指向同一个点。
    aliases = {
        "客厅": "living_room",
        "卧室": "bedroom",
        "厨房": "kitchen",
        "大厅": "hall_entry",
        "门口": "hall_entry",
    }
    # 先用原始字符串（去空格）查别名，再用归一化后的键兜底。
    mapped = aliases.get(target.strip()) or aliases.get(key)
    if mapped and mapped in goals:
        return goals[mapped]
    return None


def load_patrol_waypoints(path: str) -> List[Dict[str, Any]]:
    """从 YAML 文件加载巡航路点列表（顶层键为 waypoints 的列表）。

    与命名目标不同，巡航路点是有序的：机器人会按列表顺序依次前往。
    """
    data = load_yaml(path)
    waypoints = data.get("waypoints", [])
    if not isinstance(waypoints, list):
        raise ValueError(f"patrol waypoints must be a list: {path}")
    # 过滤掉不是字典的脏数据，保证后面 item.get(...) 不会崩。
    return [item for item in waypoints if isinstance(item, dict)]


def next_mission_mode(intent: str) -> str:
    """把指令 intent 映射成任务模式（简单状态机的"输入 -> 下一状态"表）。

    - stop / cancel -> idle（停止一切）
    - navigate -> navigate（单点导航）
    - patrol -> patrol（多点巡航）
    - follow -> follow（跟随模式）
    - move -> move（方向点动：前进/后退/左转/右转）
    - 其他未知 intent 一律回 idle，宁可不动也不要乱动。
    """
    normalized = intent.strip().lower()
    if normalized in {"stop", "cancel"}:
        return "idle"
    if normalized == "navigate":
        return "navigate"
    if normalized == "patrol":
        return "patrol"
    if normalized == "follow":
        return "follow"
    if normalized == "move":
        return "move"
    return "idle"


def move_velocity_for_direction(
    direction: str,
    linear_speed: float,
    turn_speed: float,
) -> Optional[tuple]:
    """方向名 -> (linear_x, angular_z) 速度对；未知方向返回 None。

    纯函数便于单测。约定：ROS 坐标系里 x 正方向是前进，
    angular.z 正值是左转（逆时针）。
    """
    table = {
        "forward": (abs(linear_speed), 0.0),
        "backward": (-abs(linear_speed), 0.0),
        "left": (0.0, abs(turn_speed)),
        "right": (0.0, -abs(turn_speed)),
    }
    return table.get(direction.strip().lower())


class MissionExecutorNode(Node):
    """Execute MecaMind task_command intents against Nav2 / follow enable.

    任务执行器节点本体。核心设计思想：

    - **两阶段发送**：收到指令时只把目标放进 ``_pending_send``（排队），
      由 0.5s 定时器 ``_tick`` 在确认 Nav2 action 服务端就绪后才真正发送。
      这样即使 Nav2 还没启动完，指令也不会丢，只会显示 waiting_for_nav2。
    - **序列号防串扰**：每次发送/取消都会让 ``_goal_serial`` 自增，旧 goal
      的异步回调发现序列号对不上就直接返回，避免"上一个已被取消的目标
      的结果回调"覆盖新任务的状态。
    - **心跳状态**：状态不只在变化时发布，定时器每 0.5s 也重发一次，
      让晚启动的观察者（验收脚本、教学工具）随时能拿到当前状态。
    """

    def __init__(self) -> None:
        super().__init__("mecamind_mission_executor")
        # ---- 参数声明：全部做成 ROS 参数，方便 launch 文件按环境覆盖 ----
        self.declare_parameter("task_command_topic", "/mecamind/task_command")
        self.declare_parameter("mission_state_topic", "/mecamind/mission_state")
        self.declare_parameter("follow_enable_topic", "/mecamind/follow_enable")
        # 急停时直接往这个 topic 发零速度（位于 safety_gate 上游）。
        self.declare_parameter("cmd_vel_topic", "/controller/cmd_vel_nav")
        # 方向点动走仲裁器的遥控通道（优先级高于跟随/导航），
        # 停发 0.5s 后仲裁器自动让位，无需显式释放。
        self.declare_parameter("teleop_cmd_topic", "/cmd_vel_teleop")
        self.declare_parameter("move_linear_speed", 0.15)
        self.declare_parameter("move_turn_speed", 0.6)
        # 前进/后退持续时长与转向持续时长分开：0.6rad/s x 1.5s ≈ 转 52°
        self.declare_parameter("move_duration_sec", 2.0)
        self.declare_parameter("turn_duration_sec", 1.5)
        self.declare_parameter("named_goals_file", "")
        self.declare_parameter("waypoints_file", "")
        # 单个导航目标的超时时间；超时会主动 cancel，防止机器人卡死在半路。
        self.declare_parameter("goal_timeout_sec", 120.0)
        self.declare_parameter("navigate_action", "navigate_to_pose")

        goals_file = str(self.get_parameter("named_goals_file").value)
        waypoints_file = str(self.get_parameter("waypoints_file").value)
        # 这两个配置文件没有合理默认值，缺了宁可启动失败也不要带病运行。
        if not goals_file:
            raise RuntimeError("named_goals_file parameter is required")
        if not waypoints_file:
            raise RuntimeError("waypoints_file parameter is required")

        # 启动时一次性加载配置，之后指令处理都是纯内存查表。
        self._goals = load_named_goals(goals_file)
        self._waypoints = load_patrol_waypoints(waypoints_file)
        self._goal_timeout = float(self.get_parameter("goal_timeout_sec").value)

        # ---- 状态机内部变量 ----
        self._mode = "idle"                # 当前模式：idle/navigate/patrol/follow
        self._last_intent = ""             # 最近一次指令的 intent
        self._target = ""                  # 当前目标名
        self._detail = "ready"             # 细节状态字符串（对外可见）
        self._patrol_index = 0             # 巡航进行到第几个路点
        self._goal_handle = None           # 当前活跃的 action goal 句柄（None 表示无）
        self._goal_sent_time: Optional[float] = None  # goal 被受理的时刻，用于超时判断
        self._pending_send: Optional[Dict[str, Any]] = None  # 排队等待发送的目标
        # 序列号：识别并作废"过期的"异步回调（详见 _cancel_nav 的注释）。
        self._goal_serial = 0
        # 需要用户口头确认时缓存的完整 task_command 载荷。
        self._pending_confirm: Optional[Dict[str, Any]] = None
        # 方向点动状态：截止时刻（ROS 时钟秒）与要连发的速度指令。
        self._move_until: Optional[float] = None
        self._move_twist = Twist()

        # ---- ROS 通信接口 ----
        action_name = str(self.get_parameter("navigate_action").value)
        # NavigateToPose 的 action 客户端：Nav2 的 bt_navigator 是服务端。
        self.navigate_client = ActionClient(self, NavigateToPose, action_name)
        # 发布器/订阅器最后一个参数 10 是 QoS 队列深度：网络繁忙时最多
        # 缓存 10 条消息，再多就丢最旧的（默认 reliable + volatile 策略）。
        self.state_pub = self.create_publisher(
            String, str(self.get_parameter("mission_state_topic").value), 10
        )
        self.follow_pub = self.create_publisher(
            Bool, str(self.get_parameter("follow_enable_topic").value), 10
        )
        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.teleop_pub = self.create_publisher(
            Twist, str(self.get_parameter("teleop_cmd_topic").value), 10
        )
        self.reply_pub = self.create_publisher(String, "/mecamind/robot_reply", 10)
        self.create_subscription(
            String,
            str(self.get_parameter("task_command_topic").value),
            self._task_cb,
            10,
        )
        # 0.5s 心跳定时器：发布状态 + 驱动排队目标的发送 + 检查超时。
        self.create_timer(0.5, self._tick)
        # 10Hz 点动定时器：仲裁器新鲜度窗口 0.5s，必须持续发才能保住遥控通道。
        self.create_timer(0.1, self._move_tick)
        # 启动时明确把跟随模式关掉、广播初始 idle 状态，让下游处于确定状态。
        self._publish_follow(False)
        self._publish_state()
        self.get_logger().info(
            f"MecaMind mission executor ready with {len(self._goals)} named goals "
            f"and {len(self._waypoints)} patrol waypoints"
        )

    def _publish_follow(self, enabled: bool) -> None:
        """向 /mecamind/follow_enable 发布跟随模式开关。"""
        msg = Bool()
        msg.data = enabled
        self.follow_pub.publish(msg)

    def _publish_state(self) -> None:
        """把当前内部状态打包成 MissionState 并以 JSON 发布出去。"""
        state = MissionState(
            mode=self._mode,
            last_intent=self._last_intent,
            target=self._target,
            detail=self._detail,
        )
        out = String()
        out.data = state.to_json()
        self.state_pub.publish(out)

    def _set_state(self, mode: str, detail: str, target: str = "") -> None:
        """更新模式与细节并立刻发布——保证每次状态迁移外界都能第一时间看到。

        target 传空字符串表示"保持原有 target 不变"，这样到达目标后
        状态里仍能看到刚才去的是哪里。
        """
        self._mode = mode
        self._detail = detail
        if target:
            self._target = target
        self._publish_state()

    def _publish_reply(self, text: str) -> None:
        if not text:
            return
        msg = String()
        msg.data = text
        self.reply_pub.publish(msg)

    def _dispatch_intent(self, intent: str, target: str) -> None:
        """按 intent 进入对应模式（确认通过后的真正执行入口）。"""
        mode = next_mission_mode(intent)
        if mode == "idle":
            self._stop_all("stopped" if intent == "stop" else "canceled")
            return
        if mode == "follow":
            self._enter_follow()
            return
        if mode == "move":
            self._enter_move(target)
            return
        if mode == "navigate":
            # LLM 有时会把"后退"解析成 navigate target='backward'，
            # 这里兜底：目标名是方向词就转成点动，而不是查导航目标表。
            direction = parse_move_direction(target)
            if direction:
                self._enter_move(direction)
                return
            self._enter_navigate(target)
            return
        if mode == "patrol":
            self._enter_patrol()
            return
        self._set_state("idle", f"unsupported_intent:{intent}", target)

    def _task_cb(self, msg: String) -> None:
        """任务指令入口：解析 JSON，按 intent 驱动状态机迁移。

        这是整个节点的"大脑"。处理顺序刻意安排为：
        1. JSON 解析失败 -> 只告警不崩溃（外部输入永远不可信）；
        2. 若正在 awaiting_confirmation：confirm 执行挂起任务，cancel/stop 清空；
        3. requires_confirmation 为真且不是停止类指令 -> 挂起等确认；
        4. 根据 intent 映射出目标模式，进入对应的 _enter_xxx / _stop_all。
        """
        try:
            data = json.loads(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Invalid task_command JSON: {exc}")
            return
        # intent 统一转小写去空格，容忍上游大小写不规范。
        intent = str(data.get("intent", "unknown")).strip().lower()
        target = str(data.get("target", "")).strip()
        requires_confirmation = bool(data.get("requires_confirmation", False))
        reply = str(data.get("reply", "")).strip()
        self._last_intent = intent
        self._target = target

        # 确认闭环：挂起中优先处理 confirm / cancel / stop。
        if self._pending_confirm is not None or self._detail == "awaiting_confirmation":
            if intent == "confirm":
                pending = dict(self._pending_confirm or {})
                self._pending_confirm = None
                pending_intent = str(pending.get("intent", "unknown")).strip().lower()
                pending_target = str(pending.get("target", "")).strip()
                self._last_intent = pending_intent
                self._target = pending_target
                self._dispatch_intent(pending_intent, pending_target)
                return
            if intent in {"cancel", "stop"}:
                self._pending_confirm = None
                self._stop_all("stopped" if intent == "stop" else "canceled")
                return
            # 挂起期间收到其他指令：提示先确认，不覆盖 pending。
            self._publish_reply("请先说确认或取消。")
            self._set_state("idle", "awaiting_confirmation", self._target)
            return

        # 需要确认的指令先挂起；但 stop/cancel 例外——停止永远无条件执行。
        if requires_confirmation and intent not in {"stop", "cancel", "confirm"}:
            self._pending_confirm = {
                "intent": intent,
                "target": target,
                "reply": reply,
            }
            # 确认话术由 task_scheduler 的 reply 经 TTS 播出，这里不重复发。
            self._set_state("idle", "awaiting_confirmation", target)
            return

        if intent == "confirm":
            self._publish_reply("当前没有待确认的任务。")
            self._set_state("idle", "no_pending_confirm", target)
            return

        self._dispatch_intent(intent, target)

    def _stop_all(self, detail: str) -> None:
        """停止一切活动：关跟随、取消导航、发零速度、清空排队与巡航进度。

        发一帧零 Twist 是为了让机器人立即减速停下，而不是等上游
        速度指令自然超时。
        """
        self._pending_confirm = None
        self._publish_follow(False)
        self._cancel_nav()
        self._move_until = None
        self.cmd_pub.publish(Twist())
        # 遥控通道也发一帧零速：若刚才在点动，立即刹停而不是等新鲜度超时。
        self.teleop_pub.publish(Twist())
        self._patrol_index = 0
        self._pending_send = None
        self._set_state("idle", detail)

    def _enter_follow(self) -> None:
        """切换到跟随模式：先清掉所有导航相关状态，再打开跟随开关。

        各模式互斥——进入任何一个模式前都要把其他模式的"残留"清干净，
        否则可能出现"跟随和导航同时往 cmd_vel 发指令"的危险状况。
        """
        self._cancel_nav()
        self._pending_send = None
        self._patrol_index = 0
        self._publish_follow(True)
        self._set_state("follow", "follow_enabled")

    def _enter_move(self, direction: str) -> None:
        """方向点动：以固定速度朝指定方向动一小段时间，然后自动停。

        实现方式是"限时连发"：_move_tick 定时器以 10Hz 向遥控通道发
        速度指令，超过截止时刻后发零速收尾。遥控通道在仲裁器里优先级
        高于跟随/导航，所以点动期间可以临时"抢"过控制权，动完自动归还。
        """
        velocity = move_velocity_for_direction(
            direction,
            float(self.get_parameter("move_linear_speed").value),
            float(self.get_parameter("move_turn_speed").value),
        )
        if velocity is None:
            self._set_state("idle", f"unknown_direction:{direction}", direction)
            return
        # 点动与跟随/导航互斥：先清干净其他模式的残留。
        self._publish_follow(False)
        self._cancel_nav()
        self._pending_send = None
        twist = Twist()
        twist.linear.x = float(velocity[0])
        twist.angular.z = float(velocity[1])
        duration_param = "turn_duration_sec" if direction in {"left", "right"} else "move_duration_sec"
        duration = float(self.get_parameter(duration_param).value)
        self._move_twist = twist
        self._move_until = self.get_clock().now().nanoseconds / 1e9 + duration
        self._set_state("move", f"moving:{direction}", direction)
        self.get_logger().info(
            f"Move {direction}: linear={twist.linear.x:.2f} angular={twist.angular.z:.2f} "
            f"for {duration:.1f}s"
        )

    def _move_tick(self) -> None:
        """10Hz 点动心跳：时限内连发速度，到时发零速并回 idle。"""
        if self._move_until is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now < self._move_until:
            self.teleop_pub.publish(self._move_twist)
            return
        self._move_until = None
        self.teleop_pub.publish(Twist())
        if self._mode == "move":
            self._set_state("idle", f"moved:{self._target}")

    def _enter_navigate(self, target: str) -> None:
        """处理单点导航指令：解析目标名，把目标放入待发送队列。

        注意这里并不直接发 action goal，而是写入 _pending_send，
        由 _tick 定时器在 Nav2 就绪后发送（两阶段发送，见类 docstring）。
        """
        goal = resolve_named_goal(self._goals, target)
        if goal is None:
            # 目标名查不到：回到 idle 并在 detail 里带上原始目标名，方便排查。
            self._publish_follow(False)
            self._set_state("idle", f"unknown_goal:{target}", target)
            return
        self._publish_follow(False)
        self._cancel_nav()
        self._patrol_index = 0
        self._pending_send = {"kind": "navigate", "pose": goal, "name": normalize_goal_key(target)}
        self._set_state("navigate", f"queued:{normalize_goal_key(target)}", target)

    def _enter_patrol(self) -> None:
        """处理巡航指令：从第 0 个路点开始，把首个路点放入待发送队列。

        后续路点的推进不在这里，而是在 _goal_finished 里：每到达一个
        路点就把下一个排入队列，形成"到点 -> 排队 -> 发送"的循环。
        """
        if not self._waypoints:
            self._set_state("idle", "empty_patrol")
            return
        self._publish_follow(False)
        self._cancel_nav()
        self._patrol_index = 0
        first = self._waypoints[0]
        name = str(first.get("name", "wp0"))
        self._pending_send = {"kind": "patrol", "pose": first, "name": name}
        self._set_state("patrol", f"queued:{name}")

    def _cancel_nav(self) -> None:
        """取消当前导航 goal，并让所有旧回调失效。

        关键在第一行的序列号自增：action 的回调是异步到达的，取消后
        旧 goal 的 response/result 回调可能仍在路上；它们回来时会发现
        自己携带的 serial 与当前 _goal_serial 不一致，从而直接返回，
        不会污染新任务的状态。这也覆盖了"goal 已发出但服务端还没回
        受理响应"的窗口期。
        """
        # Invalidate callbacks from a goal that is being preempted. This also
        # covers a send request whose response has not arrived yet.
        self._goal_serial += 1
        handle = self._goal_handle
        self._goal_handle = None
        self._goal_sent_time = None
        if handle is None:
            return
        try:
            # 异步请求 Nav2 取消该 goal；不等待结果，失败也只是告警。
            handle.cancel_goal_async()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Cancel navigate goal failed: {exc}")

    def _tick(self) -> None:
        """0.5 秒一次的定时器：状态心跳 + 驱动目标发送 + 超时巡检。

        执行顺序：
        1. 无条件重发一次当前状态（心跳，方便晚加入的观察者）；
        2. 只有 navigate/patrol 模式才有后续工作；
        3. 有活跃 goal -> 检查是否超时；
        4. 无活跃 goal 但有排队目标 -> 等 Nav2 action 服务端就绪后发送。
        """
        # Publish a low-rate heartbeat so late-joining classroom tools and smoke
        # tests can always observe the current state, not only transitions.
        self._publish_state()
        if self._mode not in {"navigate", "patrol"}:
            return
        if self._goal_handle is not None:
            self._poll_active_goal()
            return
        if self._pending_send is None:
            return
        # server_is_ready 检查 Nav2 的 action 服务端是否已上线；
        # 没上线就把 detail 置成 waiting_for_nav2，下个周期再试。
        if not self.navigate_client.server_is_ready():
            self._detail = "waiting_for_nav2"
            self._publish_state()
            return
        self._send_pending()

    def _send_pending(self) -> None:
        """把排队中的目标真正作为 action goal 发给 Nav2。

        这是 action 回调链的第一环：send_goal_async 返回一个 future，
        当服务端回复"是否受理"时触发 _goal_response。
        """
        pending = self._pending_send
        if pending is None:
            return
        # 发送时才生成时间戳并转成 PoseStamped——目标可能已经排队了一阵子，
        # 用旧时间戳会导致 TF 查询失败。
        stamp = self.get_clock().now().to_msg()
        pose = pose_stamped_from_dict(pending["pose"], stamp)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._pending_send = None
        # 每次发送都用新的序列号"盖章"，回调回来时凭章验身。
        self._goal_serial += 1
        serial = self._goal_serial
        send_future = self.navigate_client.send_goal_async(goal)
        # lambda 的默认参数写法（name=..., goal_serial=serial）是 Python
        # 闭包的经典技巧：在定义时刻就把当前值捕获进去，避免回调执行时
        # 读到已经变化的变量。
        send_future.add_done_callback(
            lambda future, name=pending["name"], kind=pending["kind"], goal_serial=serial: (
                self._goal_response(future, name, kind, goal_serial)
            )
        )
        self._detail = f"sending:{pending['name']}"
        self._publish_state()

    def _goal_response(self, future, name: str, kind: str, serial: int) -> None:
        """action 回调链第二环：服务端对 goal 的受理响应到达。

        可能的分支：
        - serial 过期 -> 这是被取消的旧 goal 的响应，直接忽略；
        - 发送过程抛异常 / 服务端拒绝（rejected）-> 回 idle 并记录原因；
        - 受理成功 -> 保存 goal 句柄、记录受理时间（供超时判断），
          并注册第三环回调 _goal_finished 等待最终结果。
        """
        if serial != self._goal_serial:
            return
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Navigate goal send failed: {exc}")
            self._set_state("idle", f"send_failed:{name}")
            return
        if handle is None or not handle.accepted:
            # action 服务端有权拒绝 goal（例如参数非法），必须处理这个分支。
            self._set_state("idle", f"rejected:{name}")
            return
        self._goal_handle = handle
        # 用 ROS 时钟（仿真中即 sim time）记录受理时刻，纳秒转秒。
        self._goal_sent_time = self.get_clock().now().nanoseconds / 1e9
        self._detail = f"active:{name}"
        self._publish_state()
        # get_result_async 返回的 future 在导航结束（成功/失败/被取消）时完成。
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda result_future, goal_name=name, goal_kind=kind, goal_serial=serial: self._goal_finished(
                result_future, goal_name, goal_kind, goal_serial
            )
        )

    def _poll_active_goal(self) -> None:
        """检查活跃 goal 是否超时；超时则取消导航并急停回 idle。

        超时保护针对的场景：机器人被困住、规划器反复失败重试等，
        Nav2 自己可能迟迟不返回结果，需要上层兜底。
        """
        if self._goal_sent_time is None or self._goal_timeout <= 0.0:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._goal_sent_time < self._goal_timeout:
            return
        self.get_logger().warn("Navigate goal timed out; canceling")
        self._cancel_nav()
        self._publish_follow(False)
        # 发零速度确保机器人立刻停住，不依赖 Nav2 的取消流程收尾。
        self.cmd_pub.publish(Twist())
        self._set_state("idle", "goal_timeout")

    def _goal_finished(self, future, name: str, kind: str, serial: int) -> None:
        """action 回调链第三环（终点）：导航结果到达。

        处理逻辑：
        1. serial 过期 -> 旧 goal 的结果，忽略；
        2. 已被外部切回 idle（如用户 stop）-> 不再改状态；
        3. 结果不是 SUCCEEDED -> 记 failed 回 idle；
        4. 成功：navigate 模式记 reached 收工；patrol 模式推进到下一个
           路点并重新排队，全部走完则记 patrol_complete。
        """
        if serial != self._goal_serial:
            return
        self._goal_handle = None
        self._goal_sent_time = None
        try:
            result = future.result()
            # GoalStatus 是 action 协议定义的终态枚举：
            # SUCCEEDED / ABORTED / CANCELED 等。
            status = result.status
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Goal result error: {exc}")
            self._set_state("idle", f"result_error:{name}")
            return

        if self._mode == "idle":
            # 结果到达前用户已经 stop/cancel，保持 idle 的 detail 不被覆盖。
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._set_state("idle", f"failed:{name}")
            return

        if kind == "navigate":
            # 单点导航：到点即完成，回 idle。
            self._set_state("idle", f"reached:{name}", name)
            return

        # 巡航模式：推进索引，排队下一个路点（真正发送仍由 _tick 驱动）。
        self._patrol_index += 1
        if self._patrol_index >= len(self._waypoints):
            self._set_state("idle", "patrol_complete")
            return
        nxt = self._waypoints[self._patrol_index]
        nxt_name = str(nxt.get("name", f"wp{self._patrol_index}"))
        self._pending_send = {"kind": "patrol", "pose": nxt, "name": nxt_name}
        self._set_state("patrol", f"queued:{nxt_name}")


def main(args=None) -> None:
    """节点入口：初始化 rclpy，spin 直到进程被终止。

    spin 会阻塞在这里持续处理订阅回调、定时器和 action 回调；
    finally 保证 Ctrl+C 退出时也能正确销毁节点、释放资源。
    """
    rclpy.init(args=args)
    node = MissionExecutorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
