"""假目标检测发布器：课堂上用来替代真实 YOLO 检测器的"演员"节点。

在系统中的角色：以固定频率向 /mecamind/raw_detections 发布格式与真实
检测器完全一致的 JSON 检测消息，让感知过滤（perception_filter）和
视觉跟随（vision_follow_controller）整条链路在没有相机、没有 GPU 的
教室环境里也能完整跑通。这体现了 ROS 开发的一个重要方法论：
**只要 topic 消息契约（名称 + 格式）一致，任何节点都可以被替身替换**。

- 发布 /mecamind/raw_detections：假的检测结果（label / confidence / bbox）；
- 订阅 /mecamind/fake_detection_cmd：JSON 控制指令，可以在运行时改变
  假目标的位置、大小、置信度或开关发布。上课时老师用一条
  `ros2 topic pub` 命令就能模拟"目标向左移动了"，观察机器人反应。

初学者重点阅读：
1. build_detection_payload —— 检测消息的 JSON 契约长什么样；
2. _cmd_cb —— 如何通过 topic 在运行时热更新节点内部状态；
3. _tick —— 定时器驱动的周期性发布模式（ROS 里最常见的模式之一）。
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def build_detection_payload(
    label: str,
    confidence: float,
    cx: float,
    cy: float,
    width: float,
    height: float,
) -> str:
    """按检测消息契约构造 JSON 字符串（单个检测框）。

    结构必须与真实检测器输出保持一致，这样下游的 parse_detections
    不需要知道数据是真是假。坐标均为 [0, 1] 归一化图像坐标。
    """
    return json.dumps(
        {
            "detections": [
                {
                    "label": label,
                    "confidence": confidence,
                    "bbox": {
                        "cx": cx,
                        "cy": cy,
                        "width": width,
                        "height": height,
                    },
                }
            ]
        },
        ensure_ascii=False,
    )


class FakeDetectionPublisher(Node):
    """Classroom stand-in for a YOLO detector (JSON contract only).

    默认每秒 10 次发布一个"站在画面中间偏右（cx=0.55）的 person"，
    置信度 0.82（高于过滤节点默认阈值 0.55，保证事件能通过筛选）。
    所有假目标属性既可以启动时用 ROS parameter 设置，也可以运行中
    通过 cmd topic 动态修改。
    """

    def __init__(self) -> None:
        super().__init__("mecamind_fake_detection_publisher")
        self.declare_parameter("output_topic", "/mecamind/raw_detections")
        self.declare_parameter("cmd_topic", "/mecamind/fake_detection_cmd")
        self.declare_parameter("label", "person")
        self.declare_parameter("confidence", 0.82)
        self.declare_parameter("cx", 0.55)
        self.declare_parameter("cy", 0.50)
        self.declare_parameter("width", 0.22)
        self.declare_parameter("height", 0.40)
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("enabled", True)

        # 把参数值缓存成实例属性，是因为这些值随后可能被 cmd topic 动态
        # 覆盖——ROS parameter 只提供"初始值"，运行期真值以实例属性为准。
        self._label = str(self.get_parameter("label").value)
        self._confidence = float(self.get_parameter("confidence").value)
        self._cx = float(self.get_parameter("cx").value)
        self._cy = float(self.get_parameter("cy").value)
        self._width = float(self.get_parameter("width").value)
        self._height = float(self.get_parameter("height").value)
        self._enabled = bool(self.get_parameter("enabled").value)

        self.pub = self.create_publisher(String, str(self.get_parameter("output_topic").value), 10)
        self.create_subscription(String, str(self.get_parameter("cmd_topic").value), self._cmd_cb, 10)
        # 频率下限 0.5 Hz：防止用户误传 0 或负数导致除零/定时器周期异常。
        rate_hz = max(0.5, float(self.get_parameter("rate_hz").value))
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info("MecaMind fake detection publisher ready")

    def _cmd_cb(self, msg: String) -> None:
        """处理运行时控制指令：JSON 里出现哪个键就更新哪个属性。

        采用"部分更新"语义——比如只发 {"cx": 0.3} 就只改水平位置，
        其余属性保持不变。这让课堂演示可以一次只动一个变量，
        方便学生观察因果关系。
        """
        try:
            data = json.loads(msg.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Invalid fake detection cmd: {exc}")
            return
        if "enabled" in data:
            self._enabled = bool(data["enabled"])
        if "label" in data:
            self._label = str(data["label"])
        if "confidence" in data:
            self._confidence = float(data["confidence"])
        if "cx" in data:
            self._cx = float(data["cx"])
        if "cy" in data:
            self._cy = float(data["cy"])
        if "width" in data:
            self._width = float(data["width"])
        if "height" in data:
            self._height = float(data["height"])

    def _tick(self) -> None:
        """定时器回调：enabled 时按当前属性发布一帧假检测。

        发布被关闭（enabled=False）时下游会因收不到消息而触发
        "目标丢失"逻辑——这正好可以用来演示跟随控制器的看门狗行为。
        """
        if not self._enabled:
            return
        out = String()
        out.data = build_detection_payload(
            self._label,
            self._confidence,
            self._cx,
            self._cy,
            self._width,
            self._height,
        )
        self.pub.publish(out)


def main(args=None) -> None:
    """节点入口：标准的 init -> spin -> 清理 生命周期。"""
    rclpy.init(args=args)
    node = FakeDetectionPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
