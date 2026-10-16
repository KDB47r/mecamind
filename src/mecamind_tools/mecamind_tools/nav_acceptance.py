"""CP3 导航自动验收节点（acceptance runner）。

课程第三课的"自动考官"：不需要人工盯着 rviz，它会自动驱动整套导航
系统跑一遍验收流程并出具 JSON 报告。流程为：

1. 等 mission_executor 和 Nav2 就绪、AMCL 开始输出定位；
2. 依次向 ``/mecamind/task_command`` 发布 navigate 指令，逐个访问
   参数里配置的命名目标点，并通过 ``/mecamind/mission_state`` 的
   detail 字段确认到点（reached:xxx）；
3. 最后做取消测试：发一个导航目标，跑起来 3 秒后发 stop，验证任务
   能被中途取消；
4. 全程记录状态历史；失败时把最后的 AMCL 位姿、安全门状态、
   mission_state 历史尾部一起写进报告，方便课后离线诊断。

订阅/发布的 topic：
- 发布 ``/mecamind/task_command``（String，JSON 指令）；
- 订阅 ``/mecamind/mission_state``（String，JSON 任务状态）、
  ``/amcl_pose``（AMCL 定位结果）、``/mecamind/safety_state``（安全门状态）。

设计要点：本节点只通过 topic 与系统交互（黑盒测试），不直接调用
Nav2 action——这样验收覆盖的是"学生实际使用的完整链路"。它也不用
rclpy.spin 常驻，而是在主流程中反复 spin_once 手动泵回调（同步的
测试脚本风格）。

初学者建议重点阅读：``run``（总流程）、``run_goal``（单目标的
发送-等待-重试逻辑）、``_wait_for_detail``（如何用谓词函数等待
某个状态出现）。
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class NavAcceptanceNode(Node):
    """验收流程的执行者：发指令、收状态、断言结果、写报告。"""

    def __init__(self) -> None:
        super().__init__("mecamind_nav_acceptance")
        # ---- 验收参数 ----
        self.declare_parameter("goals", ["living_room", "bedroom", "hall_entry"])  # 要依次访问的目标
        self.declare_parameter("goal_timeout_sec", 180.0)   # 单个目标最长等待时间
        self.declare_parameter("nav2_wait_sec", 120.0)      # 等系统就绪的最长时间
        self.declare_parameter("settle_sec", 3.0)           # 就绪后额外稳定时间
        self.declare_parameter("goal_retries", 1)           # 每个目标失败后的重试次数
        self.declare_parameter("cancel_test", True)         # 是否做取消测试
        self.declare_parameter("cancel_target", "bedroom")  # 取消测试用的目标
        self.declare_parameter("report_path", "maps/mecamind_cp3_nav_report.json")

        # ---- 观测缓存：各回调只负责把最新数据存起来，主流程轮询读取 ----
        self._mission_state: Optional[Dict[str, Any]] = None   # 最新任务状态
        self._state_history: List[Dict[str, Any]] = []         # 去重后的状态历史（证据用）
        self._last_amcl: Optional[Dict[str, float]] = None     # 最新 AMCL 位姿摘要
        self._last_safety: Optional[Dict[str, Any]] = None     # 最新安全门状态

        self.task_pub = self.create_publisher(String, "/mecamind/task_command", 10)
        self.create_subscription(
            String, "/mecamind/mission_state", self._state_cb, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_cb, 10
        )
        self.create_subscription(
            String, "/mecamind/safety_state", self._safety_cb, 10
        )

    def _state_cb(self, msg: String) -> None:
        """任务状态回调：更新最新状态，并把"变化了的"状态追加进历史。

        mission_executor 每 0.5s 心跳重发一次相同状态，如果不去重，
        历史里会塞满重复条目；所以只有与上一条不同时才记录。
        """
        try:
            data = json.loads(msg.data)
        except Exception:  # noqa: BLE001
            return
        self._mission_state = data
        if not self._state_history or self._state_history[-1] != data:
            self._state_history.append(data)
            self.get_logger().info(f"mission_state: {msg.data}")
            # 限制历史长度，防止长时间运行占用过多内存。
            if len(self._state_history) > 200:
                self._state_history.pop(0)

    def _amcl_cb(self, msg: PoseWithCovarianceStamped) -> None:
        """AMCL 定位回调：只保留报告需要的摘要（位置 + x/y 方差）。

        协方差下标 0 和 7 分别是 x、y 的方差（6x6 矩阵按行展开），
        方差越小说明 AMCL 对自身位置越有信心。
        """
        self._last_amcl = {
            "x": round(msg.pose.pose.position.x, 3),
            "y": round(msg.pose.pose.position.y, 3),
            "cov_x": round(msg.pose.covariance[0], 4),
            "cov_y": round(msg.pose.covariance[7], 4),
        }

    def _safety_cb(self, msg: String) -> None:
        """安全门状态回调：解析 JSON 缓存；解析失败也保留原文当证据。"""
        try:
            self._last_safety = json.loads(msg.data)
        except Exception:  # noqa: BLE001
            self._last_safety = {"raw": msg.data}

    def _send_task(self, intent: str, target: str = "") -> None:
        """构造并发布一条 JSON 任务指令（与真实用户发指令的方式完全一致）。"""
        msg = String()
        msg.data = json.dumps({"intent": intent, "target": target}, ensure_ascii=False)
        self.task_pub.publish(msg)
        self.get_logger().info(f"task_command sent: {msg.data}")

    def _spin_for(self, seconds: float) -> None:
        """边泵回调边等待固定时长。

        不能用裸 time.sleep：睡死期间订阅回调不会执行，状态缓存就
        不更新了。spin_once 让等待期间消息照常处理。
        """
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def _wait_for_detail(
        self, predicate, timeout_sec: float, label: str
    ) -> bool:
        """轮询等待任务状态满足给定谓词（predicate），超时返回 False。

        predicate 是一个接收状态字典、返回 bool 的函数。把"等什么"
        参数化后，各处等待逻辑（等受理、等到点、等停止）都能复用
        这一个循环。label 只用于超时时的日志。
        """
        start = time.monotonic()
        while time.monotonic() - start < timeout_sec and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            state = self._mission_state
            if state is not None and predicate(state):
                return True
        self.get_logger().error(f"{label} timed out after {timeout_sec:.0f}s")
        return False

    def wait_until_executor_ready(self) -> bool:
        """等待整套系统就绪，分三步，缺一不可。"""
        timeout = float(self.get_parameter("nav2_wait_sec").value)
        # 等 mission_executor 上线并且不再处于 waiting_for_nav2。
        if not self._wait_for_detail(
            lambda s: s.get("detail") not in {None, "", "waiting_for_nav2"},
            timeout,
            "wait for mission executor / Nav2",
        ):
            return False
        # 等 AMCL 真正开始发布定位（即 map->odom 存在），否则第一个目标会
        # 因为 bt_navigator 拿不到机器人初始位姿而立即 abort。
        start = time.monotonic()
        while self._last_amcl is None and time.monotonic() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self._last_amcl is None:
            self.get_logger().error("AMCL pose never arrived; is /initialpose set?")
            return False
        # 再稳定一段时间，让 TF buffer 积累足够历史。
        self._spin_for(float(self.get_parameter("settle_sec").value))
        return True

    def run_goal(self, name: str) -> Dict[str, Any]:
        """验收单个目标：发送 navigate 指令，等到点或失败，支持重试。

        返回该目标的结果字典（是否到点、尝试次数、耗时、最终 detail），
        会被汇总进总报告。
        """
        timeout = float(self.get_parameter("goal_timeout_sec").value)
        retries = max(0, int(self.get_parameter("goal_retries").value))
        start = time.monotonic()
        attempts = 0
        reached = False
        state: Dict[str, Any] = {}
        for attempt in range(retries + 1):
            attempts = attempt + 1
            self._send_task("navigate", name)
            # 先等 executor 受理本次任务，避免把上一次尝试遗留的
            # failed:xxx 终态误判成本次结果。
            self._wait_for_detail(
                lambda s: str(s.get("detail", "")).startswith(
                    ("queued:", "sending:", "active:", "waiting_for_nav2")
                ),
                15.0,
                f"goal {name} acknowledgment (attempt {attempts})",
            )
            # 再等终态：要么成功到点（reached:name），要么进入 idle 且
            # detail 是各种失败前缀之一。两类都算"有结果"，区别在下面判断。
            ok = self._wait_for_detail(
                lambda s: s.get("detail") == f"reached:{name}"
                or (s.get("mode") == "idle" and str(s.get("detail", "")).startswith(("failed:", "rejected:", "unknown_goal:", "goal_timeout"))),
                timeout,
                f"goal {name} (attempt {attempts})",
            )
            state = self._mission_state or {}
            # 只有"等到了结果"且结果确实是 reached 才算通过。
            reached = ok and state.get("detail") == f"reached:{name}"
            if reached:
                break
            self.get_logger().warn(
                f"goal {name} attempt {attempts} failed: {state.get('detail')}"
            )
            # 重试前留 2 秒缓冲，让系统从失败状态中恢复。
            self._spin_for(2.0)
        elapsed = round(time.monotonic() - start, 1)
        return {
            "goal": name,
            "reached": bool(reached),
            "attempts": attempts,
            "elapsed_sec": elapsed,
            "final_detail": state.get("detail", "no_state"),
        }

    def run_cancel_test(self) -> Dict[str, Any]:
        """取消测试：验证运动中的导航任务能被 stop 指令中止。

        步骤：发导航指令 -> 确认任务开始执行 -> 跑 3 秒 -> 发 stop ->
        确认回到 idle 且 detail 是 stopped/canceled。这项测试保证课堂
        演示时"喊停"是真的能停。
        """
        target = str(self.get_parameter("cancel_target").value)
        self._send_task("navigate", target)
        active = self._wait_for_detail(
            lambda s: str(s.get("detail", "")).startswith(("active:", "sending:", "queued:")),
            30.0,
            "cancel-test goal activation",
        )
        # 让机器人真正跑起来一小段再取消，验证运动中取消。
        self._spin_for(3.0)
        self._send_task("stop")
        stopped = self._wait_for_detail(
            lambda s: s.get("mode") == "idle"
            and s.get("detail") in {"stopped", "canceled"},
            20.0,
            "cancel-test stop",
        )
        return {"activated": active, "stopped": bool(stopped)}

    def run(self) -> int:
        """总流程：就绪检查 -> 逐个目标 -> 取消测试 -> 出报告。

        采用"快速失败"策略：任一环节失败就带着 failure 原因直接收尾，
        不再继续后面的环节（前面失败了，后面的结果也不可信）。
        返回进程退出码（0=PASS，1=FAIL）。
        """
        goals = [str(g) for g in self.get_parameter("goals").value]
        report: Dict[str, Any] = {
            "goals": [],
            "cancel_test": None,
            "overall_ok": False,
        }

        if not self.wait_until_executor_ready():
            report["failure"] = "mission_executor_or_nav2_not_ready"
            return self._finish(report)

        for name in goals:
            result = self.run_goal(name)
            report["goals"].append(result)
            if not result["reached"]:
                report["failure"] = f"goal_failed:{name}"
                return self._finish(report)
            # 目标之间稍作停顿，模拟真实使用节奏、也让状态心跳刷新。
            self._spin_for(1.0)

        if bool(self.get_parameter("cancel_test").value):
            cancel_result = self.run_cancel_test()
            report["cancel_test"] = cancel_result
            if not cancel_result["stopped"]:
                report["failure"] = "cancel_test_failed"
                return self._finish(report)

        report["overall_ok"] = True
        return self._finish(report)

    def _finish(self, report: Dict[str, Any]) -> int:
        """收尾：失败时附加现场证据，把报告写成 JSON 文件并打印结论。

        证据包含最后的 AMCL 位姿（定位是否漂了？）、安全门状态（是不是
        被安全门拦停的？）、最近 15 条任务状态（失败前发生了什么？），
        足够课后离线还原故障场景。
        """
        if not report.get("overall_ok"):
            report["evidence"] = {
                "last_amcl_pose": self._last_amcl,
                "last_safety_state": self._last_safety,
                "mission_state_tail": self._state_history[-15:],
            }
        path = Path(str(self.get_parameter("report_path").value)).expanduser()
        # 确保目录存在，避免因 maps/ 未创建而写文件失败。
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        self.get_logger().info(f"CP3 report written to {path}")
        self.get_logger().info(
            "CP3 RESULT: " + ("PASS" if report["overall_ok"] else "FAIL")
        )
        return 0 if report["overall_ok"] else 1


def main(args=None) -> None:
    """入口：跑一轮验收后以对应的退出码结束进程（供 CI/脚本判断结果）。"""
    rclpy.init(args=args)
    node = NavAcceptanceNode()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
