"""感知结果过滤节点：从原始检测结果中筛出稳定可信的目标事件。

在系统中的角色：位于"检测器"和"行为控制"之间的过滤层。
- 订阅 /mecamind/raw_detections（JSON 字符串）：上游检测器
  （真实的 YOLO 节点，或教学用的 fake_detection_publisher）发布的原始检测框；
- 发布 /mecamind/perception_event（JSON 字符串）：只包含"我们关心的那一个
  目标"的事件，附带 confirmed 标志，供 vision_follow_controller 等下游消费。

为什么需要这一层过滤？原始检测结果有两个问题：
1. 噪声多——一帧里可能有很多目标、置信度参差不齐；
2. 会闪烁——某个目标可能只在个别帧误检出现。
本节点通过"标签 + 置信度阈值筛选"和"连续 N 帧确认"（confirm_frames 参数）
两道关卡，把闪烁的误检挡在下游控制器之前，避免机器人追着幽灵目标跑。

初学者重点阅读：
1. parse_detections —— 如何宽容地解析格式不完全统一的 JSON 检测结果；
2. select_detection —— 按标签和置信度挑出最优候选；
3. PerceptionFilterNode._cb —— 连续帧确认（去抖）逻辑的实现。
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from typing import Any, Dict, Iterable, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


@dataclass(frozen=True)
class Detection:
    """一条检测结果的标准化表示。

    坐标约定：cx / cy / width / height 都是归一化到 [0, 1] 的图像相对坐标
    （相对于画面宽高的比例），与图像分辨率无关，这样下游控制律不需要
    知道相机分辨率。cx=0.5 表示目标在画面水平正中。
    """

    label: str
    confidence: float
    cx: float
    cy: float
    width: float = 0.0
    height: float = 0.0


def parse_detections(payload: str) -> List[Detection]:
    """把 JSON 字符串解析成 Detection 列表，兼容多种上游字段命名。

    不同检测器输出的字段名并不统一，这里做了三层兼容：
    - 顶层结构：既接受 {"detections": [...]} 也接受直接的 [...] 列表；
    - 标签字段：label 或 class；置信度字段：confidence 或 score；
    - 坐标字段：既接受顶层的 cx/cy，也接受嵌套在 bbox 里的 cx/center_x 等。
    非法的列表元素（不是 dict）直接跳过而不是报错，保证一帧里个别坏数据
    不影响其他检测结果。
    """
    data = json.loads(payload)
    items = data.get("detections", data if isinstance(data, list) else [])
    result = []
    for item in items:
        if not isinstance(item, dict):
            continue
        bbox = item.get("bbox", {})
        result.append(
            Detection(
                label=str(item.get("label", item.get("class", ""))),
                confidence=float(item.get("confidence", item.get("score", 0.0))),
                # 坐标兜底值 0.5（画面中心）：即使上游漏发坐标，
                # 下游控制器算出的偏差也是 0，不会产生危险的转向指令。
                cx=float(item.get("cx", bbox.get("cx", bbox.get("center_x", 0.5)))),
                cy=float(item.get("cy", bbox.get("cy", bbox.get("center_y", 0.5)))),
                width=float(item.get("width", bbox.get("width", 0.0))),
                height=float(item.get("height", bbox.get("height", 0.0))),
            )
        )
    return result


def select_detection(
    detections: Iterable[Detection],
    target_label: str,
    min_confidence: float,
) -> Detection | None:
    """从一帧检测结果里挑出"目标标签且置信度达标"的最佳候选。

    同时有多个候选时取置信度最高的那个（教学简化：真实系统通常还会
    结合上一帧位置做目标关联/跟踪 ID）。没有合格候选时返回 None，
    让调用方明确知道"这一帧没看到目标"。
    """
    candidates = [
        item
        for item in detections
        if item.label == target_label and item.confidence >= min_confidence
    ]
    if not candidates:
        return None
    candidates.sort(key=lambda item: item.confidence, reverse=True)
    return candidates[0]


class PerceptionFilterNode(Node):
    """感知过滤节点：标签/置信度筛选 + 连续多帧确认后才输出事件。

    关键参数：
    - target_label：只关心这个类别（默认 person，人跟随场景）；
    - min_confidence：低于该置信度的检测直接忽略；
    - confirm_frames：连续多少帧都看到目标才把事件标记为 confirmed=True。
      下游控制器只在 confirmed 时才动作，因此这个参数直接决定了
      "响应速度"与"抗误检能力"之间的权衡。
    """

    def __init__(self) -> None:
        super().__init__("mecamind_perception_filter")
        self.declare_parameter("input_topic", "/mecamind/raw_detections")
        self.declare_parameter("output_topic", "/mecamind/perception_event")
        self.declare_parameter("target_label", "person")
        self.declare_parameter("min_confidence", 0.55)
        self.declare_parameter("confirm_frames", 2)

        # 连续命中计数器：目标连续出现的帧数，一旦丢失就清零重来。
        self._confirm_count = 0
        self._last_target: Detection | None = None
        self.create_subscription(String, str(self.get_parameter("input_topic").value), self._cb, 10)
        self.pub = self.create_publisher(String, str(self.get_parameter("output_topic").value), 10)
        self.get_logger().info("MecaMind perception filter ready")

    def _cb(self, msg: String) -> None:
        """每收到一帧原始检测就执行：解析 -> 筛选 -> 去抖计数 -> 发布事件。"""
        # 步骤 1：解析 JSON。坏数据只警告不崩溃（上游可能是学生手敲的消息）。
        try:
            detections = parse_detections(msg.data)
        except Exception as exc:  # noqa: BLE001 - teaching diagnostics
            self.get_logger().warn(f"Invalid detection payload: {exc}")
            return
        # 步骤 2：按目标标签 + 置信度阈值挑出本帧最佳候选。
        target = select_detection(
            detections,
            str(self.get_parameter("target_label").value),
            float(self.get_parameter("min_confidence").value),
        )
        # 步骤 3：本帧没看到目标 -> 连续计数清零，且不发布任何事件。
        # "不发布"本身就是信号：下游控制器靠事件超时来判断目标丢失。
        if target is None:
            self._confirm_count = 0
            self._last_target = None
            return

        # 步骤 4：看到了目标 -> 计数加一；达到 confirm_frames 门槛才算"确认"。
        # 注意 confirmed=False 的事件也会发布，这样调试时可以在 topic 上
        # 观察到"目标出现但还没确认"的中间状态。
        self._last_target = target
        self._confirm_count += 1
        confirmed = self._confirm_count >= int(self.get_parameter("confirm_frames").value)
        event: Dict[str, Any] = {
            "label": target.label,
            "confidence": target.confidence,
            "cx": target.cx,
            "cy": target.cy,
            "width": target.width,
            "height": target.height,
            "confirmed": confirmed,
            "confirm_count": self._confirm_count,
        }
        output = String()
        output.data = json.dumps(event, ensure_ascii=False)
        self.pub.publish(output)


def main(args=None) -> None:
    """节点入口：标准的 init -> spin -> 清理 生命周期。"""
    rclpy.init(args=args)
    node = PerceptionFilterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
