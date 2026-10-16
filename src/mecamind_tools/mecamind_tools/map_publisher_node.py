"""地图发布节点（MecaMind：不跑 SLAM 也能有一张 /map 上的地图）。

这个文件是干什么的？
    读取 Gazebo 世界文件（.world）里的墙体几何，直接把它栅格化成一张
    "完美地图"，周期性发布到 /map topic 上。也就是说，这张地图不是
    SLAM 扫出来的，而是从仿真世界的"标准答案"直接生成的。

发布的 topic：
    /map (nav_msgs/OccupancyGrid) —— 由世界几何栅格化得到的静态地图。
    （本节点不订阅任何 topic。）

在建图流程中的角色：
    - 教学对照：先用它发布"完美地图"跑通 RViz 显示和导航流程，再换成
      slam_toolbox 建的真实地图，学生可以直观对比二者差异；
    - 替代 map_server：调试导航栈时不必依赖上一课建好的地图文件。
    注意：真正的建图课（第二课）里 /map 由 slam_toolbox 发布，本节点
    与 slam_toolbox 不能同时运行，否则两个发布者会在 /map 上打架。

初学者应该重点看的函数：
    - MecaMindMapPublisherNode.__init__() —— 尤其是 QoS 的配置：为什么
      地图这种"低频大消息"要用 TRANSIENT_LOCAL（锁存）+ RELIABLE。
    - _publish() —— OccupancyGrid 消息各字段（resolution/origin/data）的含义。
"""

from __future__ import annotations

from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from .world_geometry import load_world_geometry


def _default_world_path() -> str:
    """返回默认世界文件路径。

    优先从已安装的 mecamind_tools 包的 share 目录里找（ros2 run 场景）；
    包没安装时（比如直接跑源码）退回到相对路径。
    """
    try:
        from ament_index_python.packages import get_package_share_directory

        return str(Path(get_package_share_directory("mecamind_tools")) / "worlds" / "three_room_house.world")
    except Exception:
        return "worlds/three_room_house.world"


class MecaMindMapPublisherNode(Node):
    """把世界文件栅格化成 OccupancyGrid 并周期发布到 /map 的节点。

    地图内容在启动时算好一次就固定不变（世界不会变），之后只是定期
    重发同一份数据。之所以还要周期重发而不是只发一次，是给不支持
    锁存语义的工具（以及重启后的 RViz）一个兜底的刷新机会。
    """

    def __init__(self) -> None:
        super().__init__("mecamind_map_publisher")
        # 可调参数：世界文件路径、地图坐标系名、栅格分辨率（米/格）、
        # 地图四周留白（米）、重发周期（秒）。
        self.declare_parameter("world_path", _default_world_path())
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("padding", 0.20)
        self.declare_parameter("publish_period_sec", 5.0)

        # 启动时一次性完成"世界几何 -> 栅格"的转换：解析 .world 里的
        # 墙体盒子，按分辨率画到栅格上。spec 里保存了宽高、origin 和
        # 展平后的占据数据，_publish 直接搬进消息即可。
        world_path = str(self.get_parameter("world_path").value)
        self.geometry = load_world_geometry(world_path)
        self.spec = self.geometry.build_occupancy_grid(
            resolution=float(self.get_parameter("resolution").value),
            padding=float(self.get_parameter("padding").value),
        )

        # 地图的标准 QoS 配置（与 map_server、slam_toolbox 一致）：
        # - TRANSIENT_LOCAL（锁存）：发布者替晚加入的订阅者保留最后一条
        #   消息。地图更新极少，订阅者（如 RViz）往往在发布之后才启动，
        #   没有锁存它们就要干等下一个周期；
        # - RELIABLE：地图丢一帧影响很大，要求可靠传输；
        # - KEEP_LAST + depth=1：地图只有"最新一张"有意义，不需要排队历史。
        # 注意：订阅方的 QoS 必须与此兼容（尤其是 durability），否则收不到。
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(OccupancyGrid, "/map", qos)
        self.timer = self.create_timer(float(self.get_parameter("publish_period_sec").value), self._publish)
        # 构造时立刻发一次，不等第一个定时器周期到来。
        self._publish()
        self.get_logger().info(
            f"MecaMind map publisher ready: {self.geometry.name} "
            f"({self.spec.width}x{self.spec.height} @ {self.spec.resolution:.2f}m)"
        )

    def _publish(self) -> None:
        """组装并发布 OccupancyGrid 消息。

        字段速览（初学者常问）：
        - header.frame_id：地图所在的 TF 坐标系（约定为 "map"）；
        - info.resolution：每个格子的边长（米）；
        - info.origin：第 0 行第 0 列格子在 map 坐标系中的位姿，
          orientation.w=1.0 表示单位四元数（不旋转）；
        - data：row-major 一维数组，第 0 行对应 origin 处（y 最小的
          一行），值域 -1（未知）/ 0~100（占据概率）。
        每次都盖上当前时间戳，让订阅者知道这份数据是"新鲜"的。
        """
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("frame_id").value)
        msg.info.resolution = self.spec.resolution
        msg.info.width = self.spec.width
        msg.info.height = self.spec.height
        msg.info.origin.position.x = self.spec.origin_x
        msg.info.origin.position.y = self.spec.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = list(self.spec.data)
        self.publisher.publish(msg)


def main(args=None) -> None:
    """入口函数：初始化 ROS、spin 节点、退出时清理资源。"""
    rclpy.init(args=args)
    node = MecaMindMapPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
