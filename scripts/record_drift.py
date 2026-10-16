#!/usr/bin/env python3
"""Record map->odom (AMCL/SLAM drift correction) and odom->base_footprint over sim time to CSV."""
import csv
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DriftRecorder(Node):
    def __init__(self, out_path: str) -> None:
        super().__init__("mecamind_drift_recorder")
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.rows = []
        self.out_path = out_path
        self.file = open(out_path, "w", newline="", buffering=1)
        self.writer = csv.writer(self.file)
        self.writer.writerow(["t", "mo_x", "mo_y", "mo_yaw", "ob_x", "ob_y", "ob_yaw"])
        self.timer = self.create_timer(0.5, self.sample)

    def sample(self) -> None:
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        if t <= 0.0:
            return
        try:
            mo = self.buffer.lookup_transform("map", "odom", rclpy.time.Time())
            ob = self.buffer.lookup_transform("odom", "base_footprint", rclpy.time.Time())
        except Exception:
            return
        row = [
            round(t, 2),
            round(mo.transform.translation.x, 4),
            round(mo.transform.translation.y, 4),
            round(yaw_of(mo.transform.rotation), 4),
            round(ob.transform.translation.x, 4),
            round(ob.transform.translation.y, 4),
            round(yaw_of(ob.transform.rotation), 4),
        ]
        self.writer.writerow(row)

    def save(self) -> None:
        self.file.close()
        print(f"SAVED {self.out_path}")


def main() -> None:
    rclpy.init()
    node = DriftRecorder(sys.argv[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()


if __name__ == "__main__":
    main()
