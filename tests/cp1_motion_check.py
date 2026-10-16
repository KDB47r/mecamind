#!/usr/bin/env python3
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_from_odom(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_delta(after: float, before: float) -> float:
    return math.atan2(math.sin(after - before), math.cos(after - before))


class MotionCheck(Node):
    def __init__(self) -> None:
        super().__init__("mecamind_cp1_motion_check")
        self.latest_odom: Odometry | None = None
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self._odom_callback, 10)

    def _odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def wait_for_odom(self, timeout_sec: float = 15.0) -> Odometry:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_odom is not None:
                return self.latest_odom
        raise RuntimeError("在规定时间内没有收到 /odom")

    def drive(self, vx: float, vy: float, wz: float, duration_sec: float) -> None:
        command = Twist()
        command.linear.x = vx
        command.linear.y = vy
        command.angular.z = wz
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            self.publisher.publish(command)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        self.publisher.publish(Twist())
        stop_deadline = time.monotonic() + 0.4
        while rclpy.ok() and time.monotonic() < stop_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)


def pose_tuple(msg: Odometry) -> tuple[float, float, float]:
    return (
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        yaw_from_odom(msg),
    )


def main() -> None:
    rclpy.init()
    node = MotionCheck()
    try:
        start = pose_tuple(node.wait_for_odom())
        node.drive(0.20, 0.0, 0.0, 1.5)
        after_forward = pose_tuple(node.wait_for_odom())
        node.drive(0.0, 0.20, 0.0, 1.5)
        after_strafe = pose_tuple(node.wait_for_odom())
        node.drive(0.0, 0.0, 0.55, 1.5)
        after_rotate = pose_tuple(node.wait_for_odom())

        forward_delta = math.hypot(
            after_forward[0] - start[0], after_forward[1] - start[1]
        )
        strafe_delta = math.hypot(
            after_strafe[0] - after_forward[0],
            after_strafe[1] - after_forward[1],
        )
        rotate_delta = abs(angle_delta(after_rotate[2], after_strafe[2]))

        print(f"forward_delta={forward_delta:.3f} m")
        print(f"strafe_delta={strafe_delta:.3f} m")
        print(f"rotate_delta={rotate_delta:.3f} rad")

        if forward_delta < 0.08:
            raise RuntimeError("前进运动不足")
        if strafe_delta < 0.08:
            raise RuntimeError("横向运动不足")
        if rotate_delta < 0.15:
            raise RuntimeError("旋转运动不足")
        print("MECAMIND_CP1_MOTION_OK")
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
