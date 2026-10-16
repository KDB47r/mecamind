"""速度仲裁器：解决多个模块同时想开车的竞争问题（第 5 课核心）。

问题背景：到第 5 课为止，可能给底盘发速度的源头已经有四个——
急停指令、手动遥控、视觉跟随、Nav2 导航。如果它们各发各的，
底盘会收到互相矛盾的指令来回抽搐。谁说了算？必须有明确的规则。

仲裁规则（优先级从高到低，数字越小越优先）：

    0  急停     /mecamind/estop（Bool，锁存）    一旦置 True 输出恒为零速
    1  遥控     /cmd_vel_teleop                  人永远能抢过机器
    2  跟随     /cmd_vel_follow                  视觉跟随控制器
    3  导航     /cmd_vel_nav_in                  Nav2 输出

    仲裁器输出 -> /cmd_vel_nav（安全门入口，沿用既有链路）

实现要点：
- **新鲜度窗口**（source_timeout_sec）：某一路超过该时长没发新指令
  就视为"沉默"，让位给低优先级源。遥控者松手 0.5 秒后 Nav2 自动
  接管，不需要任何显式的模式切换协议；
- **抢占刹车**：高优先级源刚抢走控制权时，先发一帧零速再执行新
  指令，避免上一个源的残留速度和新指令叠出怪异运动；
- 急停是**锁存**的：置 True 后即使消息不再重发也保持急停，
  必须显式发 False 才解除——安全状态绝不能因为"没消息"而自动解除。

注意：本节点位于安全门**上游**，它解决"该听谁的"，安全门解决
"听了之后撞不撞墙"——两层职责不同，缺一不可。

初学者重点阅读：
1. pick_active_source —— 仲裁规则本体（纯函数，可单测）；
2. _estop_cb —— 锁存语义的实现；
3. _tick —— 定时输出 + 抢占刹车。
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, Optional

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import json


# 优先级表：数字越小优先级越高。急停单独处理（不走这张表）。
SOURCE_PRIORITY = ("teleop", "follow", "nav")


@dataclass
class SourceState:
    """一路速度源的最近状态：最后一条指令 + 收到它的单调钟时刻。"""

    twist: Twist
    stamp_mono: float


def pick_active_source(
    sources: Dict[str, SourceState],
    now_mono: float,
    timeout_sec: float,
) -> Optional[str]:
    """仲裁规则：按优先级返回第一个"仍新鲜"的源名，全部沉默返回 None。

    纯函数设计：输入是各源状态快照与当前时间，输出是获胜者名字，
    不碰任何 ROS 对象——单元测试可以毫秒级穷举各种竞争场景。
    """
    for name in SOURCE_PRIORITY:
        state = sources.get(name)
        if state is not None and (now_mono - state.stamp_mono) <= timeout_sec:
            return name
    return None


class CmdVelArbiter(Node):
    """速度仲裁节点：多入单出，定时器驱动输出。

    为什么用定时器统一输出、而不是在各源的回调里直接转发？
    因为"沉默降级"和"急停恒零速"都需要一个持续的心跳来驱动——
    没有新消息恰恰是需要动作（发零速/切换源）的时刻。
    """

    def __init__(self) -> None:
        super().__init__("mecamind_cmd_vel_arbiter")
        self.declare_parameter("output_topic", "/cmd_vel_nav")
        self.declare_parameter("teleop_topic", "/cmd_vel_teleop")
        self.declare_parameter("follow_topic", "/cmd_vel_follow")
        self.declare_parameter("nav_topic", "/cmd_vel_nav_in")
        self.declare_parameter("estop_topic", "/mecamind/estop")
        self.declare_parameter("state_topic", "/mecamind/arbiter_state")
        self.declare_parameter("source_timeout_sec", 0.5)
        self.declare_parameter("output_rate_hz", 20.0)

        self._sources: Dict[str, SourceState] = {}
        self._estop = False
        self._active: Optional[str] = None  # 当前获胜源，用于检测抢占switching

        self.create_subscription(
            Twist, str(self.get_parameter("teleop_topic").value),
            lambda m: self._source_cb("teleop", m), 10,
        )
        self.create_subscription(
            Twist, str(self.get_parameter("follow_topic").value),
            lambda m: self._source_cb("follow", m), 10,
        )
        self.create_subscription(
            Twist, str(self.get_parameter("nav_topic").value),
            lambda m: self._source_cb("nav", m), 10,
        )
        self.create_subscription(
            Bool, str(self.get_parameter("estop_topic").value), self._estop_cb, 10
        )
        self.out_pub = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10
        )
        self.state_pub = self.create_publisher(
            String, str(self.get_parameter("state_topic").value), 5
        )
        rate = max(1.0, float(self.get_parameter("output_rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.create_timer(1.0, self._publish_state)
        self.get_logger().info("MecaMind cmd_vel arbiter ready (estop > teleop > follow > nav)")

    def _source_cb(self, name: str, msg: Twist) -> None:
        """记录某一路的最新指令与时间戳。仲裁本身放在 _tick 里做。"""
        self._sources[name] = SourceState(msg, time.monotonic())

    def _estop_cb(self, msg: Bool) -> None:
        """急停锁存：True 立即刹车并锁定；必须显式发 False 才解除。"""
        engaged = bool(msg.data)
        if engaged and not self._estop:
            self.get_logger().warn("ESTOP engaged: output forced to zero")
            self.out_pub.publish(Twist())  # 不等下一个 tick，立即刹
        elif not engaged and self._estop:
            self.get_logger().info("ESTOP released")
        self._estop = engaged

    def _tick(self) -> None:
        """输出心跳：急停 > 仲裁获胜源 > 全员沉默（发一帧零速后闭嘴）。"""
        if self._estop:
            self.out_pub.publish(Twist())
            self._active = "estop"
            return

        winner = pick_active_source(
            self._sources, time.monotonic(),
            float(self.get_parameter("source_timeout_sec").value),
        )
        if winner is None:
            # 全部沉默：只在"刚变沉默"的那一刻发一帧零速刹车，
            # 之后保持安静，把 topic 让给别的调试工具观察。
            if self._active not in (None, "estop"):
                self.out_pub.publish(Twist())
            self._active = None
            return

        if winner != self._active and self._active is not None:
            # 抢占瞬间先刹一下，清掉上一个源的残留速度
            self.out_pub.publish(Twist())
        self._active = winner
        self.out_pub.publish(self._sources[winner].twist)

    def _publish_state(self) -> None:
        """每秒播报仲裁状态，供调试与自动验收断言。"""
        now = time.monotonic()
        timeout = float(self.get_parameter("source_timeout_sec").value)
        fresh = {
            name: bool(state and (now - state.stamp_mono) <= timeout)
            for name, state in ((n, self._sources.get(n)) for n in SOURCE_PRIORITY)
        }
        payload = {"estop": self._estop, "active": self._active or "none", "fresh": fresh}
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelArbiter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
