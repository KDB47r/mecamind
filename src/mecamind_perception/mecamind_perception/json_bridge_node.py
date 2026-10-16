"""检测消息 JSON 适配器：把结构化 Detection2DArray 翻译回旧 JSON 契约。

在系统中的角色：新旧接口之间的兼容层。
- 第 4 课引入了结构化消息 mecamind_interfaces/Detection2DArray（新契约）；
- 但第 3 课之前的下游（perception_filter、vision_follow_controller）
  消费的是 JSON String 契约（/mecamind/raw_detections）。

两种升级策略：a) 把所有下游一次性改成新消息；b) 加一个适配器，
新旧并行、逐步迁移。本项目选 b)——这是工程里处理接口演进的常规
做法（Adapter 模式），一次课只动一层，出问题也容易定位。

订阅：/mecamind/detections（Detection2DArray）
发布：/mecamind/raw_detections（String JSON，与 fake_detection_publisher
      输出格式完全一致，下游无感知）
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from mecamind_interfaces.msg import Detection2DArray


def detection_array_to_json(msg: Detection2DArray) -> str:
    """把 Detection2DArray 转成旧 JSON 契约字符串。

    输出结构与 fake_detection_publisher.build_detection_payload 一致：
    {"detections": [{"label", "confidence", "bbox": {cx, cy, width, height}}]}
    """
    return json.dumps(
        {
            "detections": [
                {
                    "label": det.label,
                    "confidence": round(float(det.confidence), 4),
                    "bbox": {
                        "cx": round(float(det.cx), 4),
                        "cy": round(float(det.cy), 4),
                        "width": round(float(det.width), 4),
                        "height": round(float(det.height), 4),
                    },
                }
                for det in msg.detections
            ]
        },
        ensure_ascii=False,
    )


class DetectionJsonBridge(Node):
    """结构化检测 -> JSON 契约的单向适配节点。"""

    def __init__(self) -> None:
        super().__init__("mecamind_detection_json_bridge")
        self.declare_parameter("input_topic", "/mecamind/detections")
        self.declare_parameter("output_topic", "/mecamind/raw_detections")
        self.create_subscription(
            Detection2DArray, str(self.get_parameter("input_topic").value), self._cb, 10
        )
        self.pub = self.create_publisher(
            String, str(self.get_parameter("output_topic").value), 10
        )
        self.get_logger().info("MecaMind detection JSON bridge ready")

    def _cb(self, msg: Detection2DArray) -> None:
        out = String()
        out.data = detection_array_to_json(msg)
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectionJsonBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
