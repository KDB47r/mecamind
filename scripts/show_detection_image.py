#!/usr/bin/env python3
"""简单 OpenCV 窗口：订阅 /mecamind/detection_image 并实时显示。"""

from __future__ import annotations

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class DetectionViewer(Node):
    def __init__(self) -> None:
        super().__init__("mecamind_detection_viewer")
        self.bridge = CvBridge()
        self.frame = None
        self.create_subscription(Image, "/mecamind/detection_image", self._cb, 10)

    def _cb(self, msg: Image) -> None:
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def main() -> None:
    rclpy.init()
    node = DetectionViewer()
    cv2.namedWindow("MecaMind Detection", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("MecaMind Detection", 800, 600)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if node.frame is not None:
                cv2.imshow("MecaMind Detection", node.frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
