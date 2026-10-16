"""前沿点（frontier）探索节点 —— 基于 Nav2 的自动建图示例。

【这个文件是干什么的】
机器人建图时，地图上会同时存在三种栅格：
  - 自由（free，值 0~20 左右）：激光已经扫过、确认没有障碍的区域；
  - 占据（occupied，值接近 100）：确认是障碍物（墙、桌腿等）；
  - 未知（unknown，值 -1）：激光还没扫到的区域。
"frontier"（前沿点）指的是**自由区域与未知区域的交界处**。只要机器人不断
开往 frontier，就能把未知区域逐渐"照亮"，直到整张地图没有可达的 frontier
为止——这就是经典的 frontier-based exploration 自动建图算法。

【订阅 / 发布】
  - 订阅  /map (nav_msgs/OccupancyGrid)：SLAM 输出的占据栅格地图；
  - 通过 TF 查询 map->base_footprint，得到机器人当前位姿；
  - 通过 Nav2 的 navigate_to_pose action 发送导航目标（不直接发速度，
    路径规划与避障全部交给 Nav2 完成，本节点只负责"决定去哪"）。

【总体流程】（定时器 _tick 每 2 秒执行一次）
  1. 等待地图和 Nav2 action server 就绪；
  2. 若当前有导航目标在执行 → 只检查是否超时；
  3. 否则：在地图上找出所有 frontier 栅格 → 聚类 → 过滤太小的簇和
     黑名单点 → 选最近的簇质心作为目标 → 发给 Nav2；
  4. 找不到任何可用 frontier 时认为探索完成，按需保存地图。

【初学者重点阅读】
  - _choose_frontier：frontier 检测 + 聚类 + 打分选择的核心；
  - _has_unknown_neighbor：一个栅格是不是 frontier 的判定条件；
  - _cluster_frontiers：用 BFS 把相邻 frontier 栅格聚成簇；
  - _cluster_centroid_world：栅格坐标 → 世界坐标的换算公式。
"""

from __future__ import annotations

import math
import subprocess
import time
from collections import deque
from typing import List, Optional, Set, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener

from .nav_utils import quaternion_from_yaw


# 栅格坐标类型别名：(x 列号, y 行号)，注意与 numpy 的 (行, 列) 顺序相反
GridCell = Tuple[int, int]


class FrontierExplorer(Node):
    """Frontier-based exploration node for MecaMind mapping demos.

    基于前沿点的自动探索节点。它是一个"决策层"：只挑选目标点，
    真正的路径规划、避障、速度控制都委托给 Nav2 的 navigate_to_pose。
    这样代码量小，适合教学演示 frontier 探索的核心思想。
    """

    def __init__(self) -> None:
        super().__init__("mecamind_frontier_explorer")
        # ── 可调参数 ──
        # min_frontier_cells：一个 frontier 簇至少要有多少栅格才值得去。
        #   太小的簇往往是激光噪声或墙缝，去了也扫不出多少新区域。
        # goal_timeout_sec：单个导航目标的最长等待时间，防止 Nav2 卡死。
        # goal_reached_radius：离目标多近算"已经到了"（无需再派目标）。
        # frontier_blacklist_radius：失败目标周围多大范围内不再选点，
        #   避免反复选到同一个到不了的地方。
        # map_save_basename：探索完成后自动保存地图的文件名前缀（留空则不保存）。
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("min_frontier_cells", 12)
        self.declare_parameter("goal_timeout_sec", 90.0)
        self.declare_parameter("goal_reached_radius", 0.45)
        self.declare_parameter("frontier_blacklist_radius", 0.65)
        self.declare_parameter("map_save_basename", "")

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.min_frontier_cells = int(self.get_parameter("min_frontier_cells").value)
        self.goal_timeout = float(self.get_parameter("goal_timeout_sec").value)
        self.goal_reached_radius = float(self.get_parameter("goal_reached_radius").value)
        self.blacklist_radius = float(self.get_parameter("frontier_blacklist_radius").value)

        # ── 运行时状态 ──
        self.latest_map: Optional[OccupancyGrid] = None   # 最新一帧地图
        self.goal_handle = None                            # 非 None 表示 Nav2 正在执行目标
        self.goal_sent_time = None                         # 目标发出时刻，用于超时判断
        self.current_goal_xy: Optional[Tuple[float, float]] = None
        self.blacklist: List[Tuple[float, float]] = []     # 失败目标点黑名单
        self._map_saved = False                            # 防止重复保存地图

        # TF 监听器：用于查询机器人在 map 坐标系下的位置
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Nav2 导航 action 客户端：目标点通过它异步下发
        self.navigate_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        map_topic = str(self.get_parameter("map_topic").value)
        # 地图 QoS 必须用 TRANSIENT_LOCAL（锁存）：SLAM 通常低频发布地图，
        # 锁存能保证本节点晚启动时也能立刻拿到最后一帧地图。
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, map_topic, self._map_callback, map_qos)
        # 主循环：每 2 秒决策一次。探索不需要高频决策，
        # 低频还能给 SLAM 留出更新地图的时间。
        self.timer = self.create_timer(2.0, self._tick)
        self.get_logger().info("Frontier explorer waiting for map and Nav2 action server")

    def _map_callback(self, msg: OccupancyGrid):
        """缓存最新地图。真正的处理放在定时器里做，回调只存不算。"""
        self.latest_map = msg

    def _tick(self):
        """探索主循环（每 2 秒一次）：等待依赖 → 监督当前目标 → 选新 frontier。"""
        # 前置条件不满足时只打印提示，等下一个周期再试
        if self.latest_map is None:
            self.get_logger().info("Waiting for /map ...")
            return
        if not self.navigate_client.server_is_ready():
            self.get_logger().info("Waiting for navigate_to_pose action server ...")
            return
        # 已有目标在执行：不派发新目标，只盯着它是否超时
        if self.goal_handle is not None:
            self._check_goal_timeout()
            return

        robot_xy = self._robot_xy()
        if robot_xy is None:
            return
        frontier = self._choose_frontier(robot_xy)
        if frontier is None:
            # 地图上再也找不到可用的 frontier ⇒ 可达区域都已探索完毕
            self.get_logger().info("No usable frontier found; exploration appears complete")
            self._save_map_if_requested()
            return
        self._send_goal(frontier, robot_xy)

    def _robot_xy(self) -> Optional[Tuple[float, float]]:
        """通过 TF 查询机器人在 map 坐标系下的 (x, y)。

        用 TF 而不用 /odom，是因为 odom 存在累积漂移；
        map->base 的变换由 SLAM 持续校正，才是"地图上的真实位置"。
        查询失败（如 SLAM 尚未发布 TF）时返回 None，本轮跳过。
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(f"TF {self.map_frame}->{self.base_frame} unavailable: {exc}")
            return None
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def _choose_frontier(self, robot_xy: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """核心算法：检测 frontier → 聚类 → 过滤 → 选出下一个目标点。

        分四步（初学者建议按步骤对照代码阅读）：
          1. 逐格扫描地图，找出所有"自由且紧邻未知区"的栅格；
          2. 把相邻的 frontier 栅格聚成簇（一段连续的探索边界）；
          3. 过滤：太小的簇、黑名单附近的簇、离机器人太近的簇；
          4. 打分排序：优先最近的簇（省时间），同距离时优先更大的簇。
        """
        grid = self.latest_map
        assert grid is not None
        width = grid.info.width
        height = grid.info.height
        # OccupancyGrid.data 是一维数组，按行优先存储：
        # 索引 = y * width + x，值域为 -1（未知）/ 0~100（占据概率）
        data = grid.data

        # 第 1 步：frontier 检测。
        # 条件：本格是自由空间（0~20 表示占据概率很低），
        #       且四邻域中至少有一个未知格（-1）。
        # 从 1 到 size-2 遍历是为了避开地图边缘（省去邻居越界判断）。
        frontiers: Set[GridCell] = set()
        for y in range(1, height - 1):
            row = y * width
            for x in range(1, width - 1):
                value = data[row + x]
                if 0 <= value <= 20 and self._has_unknown_neighbor(data, width, x, y):
                    frontiers.add((x, y))

        # 第 2 步：把散点聚成簇；第 3 步：逐簇过滤
        clusters = self._cluster_frontiers(frontiers)
        candidates = []
        for cluster in clusters:
            if len(cluster) < self.min_frontier_cells:
                continue  # 簇太小：多半是噪声或门缝，不值得跑一趟
            world_xy = self._cluster_centroid_world(cluster)
            if self._is_blacklisted(world_xy):
                continue  # 之前在这附近失败过，跳过
            distance = math.hypot(world_xy[0] - robot_xy[0], world_xy[1] - robot_xy[1])
            if distance < self.goal_reached_radius:
                continue  # 已经站在这个 frontier 上了，去了也没意义
            candidates.append((distance, len(cluster), world_xy))

        if not candidates:
            return None
        # 第 4 步：排序取最优。key=(距离升序, 簇大小降序)：
        # 先去最近的边界（贪心策略，路径短），距离相同再挑信息量大的。
        candidates.sort(key=lambda item: (item[0], -item[1]))
        return candidates[0][2]

    @staticmethod
    def _has_unknown_neighbor(data, width: int, x: int, y: int) -> bool:
        """判断 (x, y) 的上下左右四邻域中是否存在未知格（-1）。

        这就是 frontier 的定义：站在自由区、脚边就是未知区的栅格。
        """
        offsets = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        return any(data[(y + dy) * width + (x + dx)] == -1 for dx, dy in offsets)

    @staticmethod
    def _cluster_frontiers(frontiers: Set[GridCell]) -> List[List[GridCell]]:
        """把相邻的 frontier 栅格聚成簇（连通分量），使用 BFS 洪泛。

        为什么要聚类：单个 frontier 栅格没有导航价值，一段连续的
        边界才代表"一片没探索过的区域的入口"。聚类后还能用簇的大小
        过滤噪声、用簇的质心作为导航目标。

        实现：每次从集合里任取一个种子点，BFS 向四邻域扩散，把所有
        连通的 frontier 归入同一簇；集合取空即完成。已访问的点会从
        frontiers 集合中移除，所以每个点只处理一次，整体是 O(N)。
        """
        clusters: List[List[GridCell]] = []
        while frontiers:
            start = frontiers.pop()
            cluster = [start]
            queue = deque([start])
            while queue:
                x, y = queue.popleft()
                for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                    item = (nx, ny)
                    if item in frontiers:
                        frontiers.remove(item)
                        cluster.append(item)
                        queue.append(item)
            clusters.append(cluster)
        return clusters

    def _cluster_centroid_world(self, cluster: List[GridCell]) -> Tuple[float, float]:
        """把簇的质心从栅格坐标换算成 map 坐标系下的世界坐标（米）。

        换算公式：世界坐标 = 地图原点 + (栅格索引 + 0.5) × 分辨率。
        origin 是地图左下角第 (0,0) 格的世界坐标；+0.5 是为了取
        栅格的中心点而不是左下角；resolution 是每格的边长（米/格）。
        """
        grid = self.latest_map
        assert grid is not None
        # 先在栅格坐标系里求均值（质心），再统一换算到世界坐标
        mean_x = sum(cell[0] for cell in cluster) / len(cluster)
        mean_y = sum(cell[1] for cell in cluster) / len(cluster)
        origin = grid.info.origin.position
        resolution = grid.info.resolution
        return (
            origin.x + (mean_x + 0.5) * resolution,
            origin.y + (mean_y + 0.5) * resolution,
        )

    def _is_blacklisted(self, point: Tuple[float, float]) -> bool:
        """检查候选点是否落在任一失败目标的黑名单半径内。

        黑名单机制防止机器人对同一个到不了的目标反复尝试
        （例如玻璃墙后的 frontier，激光穿透导致看似可达实则不可达）。
        """
        return any(
            math.hypot(point[0] - bad[0], point[1] - bad[1]) < self.blacklist_radius
            for bad in self.blacklist
        )

    def _send_goal(self, point: Tuple[float, float], robot_xy: Tuple[float, float]):
        """把选中的 frontier 打包成 NavigateToPose 目标发给 Nav2（异步）。

        目标朝向设为"从机器人指向目标点"的方向：到达后激光正好
        朝着未知区域，能扫出更多新地图。
        """
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = point[0]
        goal.pose.pose.position.y = point[1]
        # atan2(Δy, Δx) 得到机器人→目标的方位角，再转成四元数
        yaw = math.atan2(point[1] - robot_xy[1], point[0] - robot_xy[0])
        goal.pose.pose.orientation = quaternion_from_yaw(yaw)

        self.current_goal_xy = point
        self.goal_sent_time = self.get_clock().now()
        self.get_logger().info(f"Sending frontier goal: x={point[0]:.2f}, y={point[1]:.2f}")
        # 异步发送 + 回调链：send_goal_async → 响应回调 → 结果回调。
        # 不能在回调里阻塞等待，否则会卡死整个节点的 executor。
        future = self.navigate_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """Nav2 对目标"接受/拒绝"的第一层回复。被拒则拉黑该点。"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Frontier goal rejected")
            if self.current_goal_xy:
                self.blacklist.append(self.current_goal_xy)
            self.goal_handle = None
            self.current_goal_xy = None
            return
        self.goal_handle = goal_handle
        # 目标被接受后，再注册第二层回调等待最终执行结果
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        """Nav2 执行完毕（成功/失败/被取消）后的最终回调。

        失败的目标点加入黑名单；无论成败都清空目标状态，
        让 _tick 在下个周期挑选新的 frontier。
        """
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Frontier goal reached")
        else:
            self.get_logger().warn(f"Frontier goal failed with status={result.status}")
            if self.current_goal_xy:
                self.blacklist.append(self.current_goal_xy)
        self.goal_handle = None
        self.goal_sent_time = None
        self.current_goal_xy = None

    def _check_goal_timeout(self):
        """看门狗：目标执行超过 goal_timeout 秒就主动取消并拉黑。

        Nav2 偶尔会在难走的地方反复重规划而不上报失败，
        没有这层超时保护整个探索流程可能永远停在一个目标上。
        """
        if self.goal_sent_time is None or self.goal_handle is None:
            return
        elapsed = (self.get_clock().now() - self.goal_sent_time).nanoseconds / 1e9
        if elapsed <= self.goal_timeout:
            return
        self.get_logger().warn("Frontier goal timed out; canceling and blacklisting")
        if self.current_goal_xy:
            self.blacklist.append(self.current_goal_xy)
        self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.goal_sent_time = None
        self.current_goal_xy = None

    def _save_map_if_requested(self) -> None:
        """探索完成后按需保存地图（调用 nav2_map_server 的命令行工具）。

        只有设置了 map_save_basename 参数才会保存；_map_saved 标志
        确保只保存一次（_tick 在完成后仍会每 2 秒进入这里）。
        """
        basename = str(self.get_parameter("map_save_basename").value).strip()
        if not basename or self._map_saved:
            return
        self.get_logger().info(f"Saving exploration map to {basename}.pgm/.yaml")
        try:
            completed = subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", basename],
                check=False,
                text=True,
                capture_output=True,
                timeout=15,
            )
        except Exception as exc:  # noqa: BLE001 - logged for teaching diagnostics
            self.get_logger().error(f"Map save command failed to start: {exc}")
            return
        if completed.returncode == 0:
            self.get_logger().info("Map save completed")
            self._map_saved = True
        else:
            self.get_logger().error(completed.stderr.strip() or completed.stdout.strip())


def main(args=None):
    """节点入口：初始化 rclpy，spin 直到 Ctrl+C，退出时清理资源。"""
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
