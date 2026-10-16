#!/usr/bin/env python3
'''
auto_explore v4.1 -- Wall-follow + Boustrophedon coverage (vacuum-robot style)
Deterministic exploration: trace walls first, then zigzag-cover the interior.
Optimized for Mecanum chassis (strafing for wall distance + stuck escape).
Anti-spin: never rotate in place for more than 2s, always combine with lateral.
Physical-stuck detection: if wheels spin but robot doesn't move, lateral escape.

════════════════════════ 中文导读（写给初学者） ════════════════════════

【这个文件是干什么的】
这是 MecaMind 项目的"自动探索建图"节点，思路类似扫地机器人：
不依赖 Nav2，自己直接根据激光雷达 + SLAM 地图计算速度指令(cmd_vel)，
让麦克纳姆轮机器人自动把整个室内环境走一遍并建出完整地图。
麦克纳姆轮可以横向平移（linear.y），本文件大量利用这一点来
贴墙、避障和脱困——这是它区别于普通差速小车探索算法的核心。

【订阅 / 发布 / 服务】
  订阅：
    /map      (nav_msgs/OccupancyGrid)  SLAM 输出的占据栅格地图
    /scan_raw (sensor_msgs/LaserScan)   激光雷达原始扫描
    /odom     (nav_msgs/Odometry)       里程计（取机器人位姿）
  发布：
    /controller/cmd_vel (geometry_msgs/Twist)  直接输出速度指令
  服务（Trigger）：
    ~/start ~/stop ~/save_map ~/init_finish    手动控制与状态查询

【状态机：探索分为几个阶段（phase）依次推进】
  INIT → WALL_FOLLOW → CROSS_EXPLORE → PLAN_COVERAGE
       → BOUSTROPHEDON → GAP_FILL → COMPLETE
  1. WALL_FOLLOW   贴墙走一圈：先把房间外轮廓建出来（闭环检测退出）；
  2. CROSS_EXPLORE 冲向最远的 frontier：保证地图两侧都被打开；
  3. PLAN_COVERAGE 根据当前地图规划"弓字形"(boustrophedon)覆盖路径；
  4. BOUSTROPHEDON 逐个航点执行弓字形扫描，把房间内部扫满；
  5. GAP_FILL      收尾：逐个访问剩余 frontier 补掉未知区域，
                   frontier 用完后还会做边界回访(boundary revisit)；
  6. COMPLETE      保存地图，停车。
  每个阶段都有超时/停滞检测，防止在任何一步卡死。

【初学者重点阅读】
  - control_loop        10Hz 主循环，状态机的总调度入口；
  - _wall_follow_step   贴墙 PD 控制 + 闭环判定；
  - _find_frontier_goal frontier 检测、聚类与信息量打分（核心算法）；
  - _plan_coverage      弓字形覆盖路径的生成；
  - _navigate_to_point  点到点运动控制（含防原地打转逻辑）；
  - astar_grid          栅格地图上的 BFS 路径搜索；
  - _check_physical_stuck / _do_physical_escape  物理卡死检测与脱困。
'''
import math, time, os, subprocess, rclpy
from pathlib import Path
import numpy as np
from collections import deque
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def angle_diff(a, b):
    '''返回角度差 a-b 并规整到 (-π, π]。

    直接相减会有 2π 跳变问题（比如 179° 与 -179° 其实只差 2°）。
    先取 sin/cos 再 atan2 是把角度"绕回"最短夹角的标准技巧。
    '''
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


def quat_to_yaw(q):
    '''从四元数提取绕 z 轴的偏航角 yaw（弧度）。

    机器人在平面上运动，只关心 yaw。这里是四元数→欧拉角
    公式在仅有 yaw 时的化简形式，避免引入额外的依赖库。
    '''
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def astar_grid(grid, start_rc, goal_rc, inflation=2):
    '''BFS on occupancy grid with inflated obstacles. Returns [(row,col),...] or None.

    在占据栅格上做路径搜索，输入/输出都用 (row, col) 栅格索引。
    名字叫 astar 但实现其实是 BFS（广度优先）——在均匀栅格上
    BFS 找到的就是最少格数路径，对教学演示足够且更易懂。

    关键设计（初学者注意）：
      1. 障碍膨胀(inflation)：把障碍物向外"膨胀"若干格，相当于
         给机器人体积留出安全余量，规划出的路径不会紧贴墙面。
         未知格(-1)也当作障碍，避免规划路径穿过没探明的区域。
      2. 起点/终点落在障碍里的兜底：膨胀后如果起点或终点被"埋"进
         障碍，就退化为只用真实障碍(>50)重试；仍失败才返回 None。
      3. 终点判定放宽到 3×3 邻域(abs<=1)：目标常是 frontier 边缘、
         紧挨未知格，精确到达往往不可能，靠近即可。
    '''
    h, w = grid.shape
    # 把起点/终点夹到地图范围内，防止越界索引
    sr, sc = int(np.clip(start_rc[0], 0, h-1)), int(np.clip(start_rc[1], 0, w-1))
    gr, gc = int(np.clip(goal_rc[0], 0, h-1)), int(np.clip(goal_rc[1], 0, w-1))

    # 障碍 = 占据(>50) 或 未知(-1)；再按 inflation 膨胀出安全边界
    obstacles = (grid > 50) | (grid == -1)
    if inflation > 0:
        from scipy.ndimage import binary_dilation
        obstacles = binary_dilation(obstacles, iterations=inflation)

    # 兜底：膨胀把起/终点吞掉时，放宽为只考虑确定障碍再试一次
    if obstacles[sr, sc] or obstacles[gr, gc]:
        obstacles = (grid > 50)
        if obstacles[sr, sc] or obstacles[gr, gc]:
            return None

    # 标准 BFS：visited 防重复入队，parent 记录来路以便回溯路径
    visited = np.zeros((h, w), dtype=bool)
    visited[sr, sc] = True
    parent = {}
    queue = deque([(sr, sc)])
    found = False
    # 8 邻域（含斜向），让路径能走对角线、更短更自然
    dirs = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

    while queue:
        r, c = queue.popleft()
        # 到达终点附近（3×3 邻域内）即算成功
        if abs(r - gr) <= 1 and abs(c - gc) <= 1:
            parent[(gr, gc)] = (r, c)
            found = True
            break
        for dr, dc in dirs:
            nr, nc = r + dr, c + dc
            if 0 <= nr < h and 0 <= nc < w and not visited[nr, nc] and not obstacles[nr, nc]:
                visited[nr, nc] = True
                parent[(nr, nc)] = (r, c)
                queue.append((nr, nc))

    if not found:
        return None

    # 从终点沿 parent 链回溯到起点，再反转得到正向路径
    path = []
    cur = (gr, gc)
    while cur != (sr, sc):
        path.append(cur)
        cur = parent.get(cur)
        if cur is None:  # 回溯链断裂（理论上不该发生）时安全退出
            break
    path.reverse()
    return path


# ─── Phases ───
# 状态机的各个阶段常量（用字符串便于日志直接打印、服务查询）。
# 探索按 INIT→WALL_FOLLOW→CROSS_EXPLORE→PLAN_COVERAGE→
# BOUSTROPHEDON→GAP_FILL→COMPLETE 顺序推进，详见文件顶部导读。
PHASE_INIT = 'INIT'                    # 初始/等待地图
PHASE_WALL_FOLLOW = 'WALL_FOLLOW'      # 贴墙走一圈建外轮廓
PHASE_CROSS_EXPLORE = 'CROSS_EXPLORE'  # 冲向最远 frontier 打开另一侧
PHASE_PLAN_COVERAGE = 'PLAN_COVERAGE'  # 规划弓字形覆盖路径
PHASE_BOUSTROPHEDON = 'BOUSTROPHEDON'  # 执行弓字形扫描内部
PHASE_GAP_FILL = 'GAP_FILL'            # 收尾补扫剩余未知区
PHASE_COMPLETE = 'COMPLETE'            # 完成，停车保存地图


class AutoExploreNode(Node):
    """自动探索建图节点：以 10Hz 主循环驱动的状态机。

    与 frontier_explorer 不同，本节点**不用 Nav2**，而是自己根据
    激光和地图直接算出 /cmd_vel。这样能完整展示"感知→决策→控制"
    的闭环，也便于针对麦克纳姆轮做横移贴墙、横移脱困等特殊动作。

    构造函数里声明了大量可调参数，并初始化各阶段用到的状态变量
    （带 _wf_/_cross_/_gap_/_boustro_/_phys_ 前缀，分别对应贴墙、
    冲刺、补扫、弓字形、物理脱困等子状态）。
    """

    def __init__(self, name):
        # allow_undeclared_parameters：允许通过命令行/launch 覆盖
        # 未显式声明的参数，方便调参而不必改代码
        super().__init__(name, allow_undeclared_parameters=True)
        # ── 参数默认值 (名称, 默认值) ──
        # 速度类单位为 m/s 或 rad/s，距离类为米，时间类为秒。
        # 关键几项：obstacle_stop_dist 紧急停障距离；wall_follow_*
        # 是贴墙 PD 控制器的目标距离与增益；row_spacing 弓字形行间距；
        # 各种 *_timeout_sec/*_stall_sec 是防卡死的超时/停滞阈值。
        params = [
            ('linear_speed', 0.10), ('angular_speed', 0.6),
            ('linear_speed_slow', 0.05), ('angular_speed_slow', 0.35),
            ('lateral_speed', 0.08),
            ('obstacle_stop_dist', 0.07), ('obstacle_slow_dist', 0.15),
            ('obstacle_side_dist', 0.05),
            ('goal_tolerance', 0.20),
            ('wall_follow_dist', 0.25),
            ('wall_follow_kp', 1.0),
            ('wall_follow_kd', 0.3),
            ('wall_follow_side', 'right'),
            ('wall_loop_close_dist', 0.4),
            ('wall_min_duration', 20.0),
            ('row_spacing', 0.30),
            ('waypoint_tolerance', 0.15),
            ('auto_save_map', True),
            ('map_save_path', str(Path('~/.ros/mecamind_three_room_map').expanduser())),
            ('startup_wait_sec', 8.0),
            ('progress_report_interval', 10.0),
            ('frontier_cluster_dist', 0.3),
            ('min_frontier_size', 3),
            ('min_frontier_distance', 0.5),
            ('frontier_blacklist_dist', 0.55),
            ('max_gap_fill_failures', 18),
            ('gap_fill_timeout_sec', 240.0),
            ('min_unknown_reduction', 8),
            ('cross_goal_timeout_sec', 150.0),
            ('cross_goal_stall_sec', 35.0),
            ('cross_goal_min_progress', 0.25),
            ('gap_goal_timeout_sec', 120.0),
            ('gap_goal_stall_sec', 40.0),
            ('boustrophedon_stall_sec', 35.0),
            ('boustrophedon_min_coverage_gain', 0.2),
            ('boustrophedon_max_skips', 6),
            ('boundary_revisit_enabled', True),
            ('boundary_revisit_margin', 0.45),
            ('max_boundary_revisits', 4),
            ('cmd_topic', '/controller/cmd_vel'),
            ('checkpoint_save_interval_sec', 90.0),
        ]
        for n, v in params:
            self.declare_parameter(n, v)

        # ── 传感器最新数据缓存 ──
        self.map_data = None      # numpy 二维数组：占据栅格 (int8, -1/0..100)
        self.map_info = None      # 地图元信息：分辨率、原点、宽高
        self.laser_ranges = None  # 最新激光测距数组
        self.scan_angle_min = -math.pi
        self.scan_angle_increment = 0.0
        # ── 机器人当前位姿（来自 /odom）──
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        # ── 总体运行标志与计时 ──
        self.running = False           # 是否已开始探索
        self._map_received = False     # 是否收到过至少一帧地图
        self._map_saved = False        # 最终地图是否已保存（防重复）
        self._start_time = None
        self._last_progress_report = 0.0
        self._last_checkpoint_save = 0.0
        self._node_start_time = time.time()

        self.phase = PHASE_INIT

        # ── 贴墙(WALL_FOLLOW)阶段状态 ──
        self._wf_start_pos = None            # 起点位置，用于闭环检测
        self._wf_start_time = 0.0
        self._wf_prev_error = 0.0            # PD 控制上一次的距离误差（算微分项）
        self._wf_corner_count = 0            # 转角计数（仅用于日志观察）
        self._wf_last_coverage = 0.0         # 上次记录的覆盖率，用于停滞判断
        self._wf_coverage_stall_time = 0.0

        # ── 弓字形(BOUSTROPHEDON)阶段状态 ──
        self._coverage_path = []             # 规划出的航点列表
        self._coverage_idx = 0               # 当前正在前往的航点下标
        self._wp_start_time = 0.0            # 到达当前航点开始尝试的时刻
        self._wp_start_dist = 999.0          # 尝试之初到航点的距离（判断是否在靠近）
        self._boustro_last_coverage = 0.0
        self._boustro_last_gain_time = 0.0
        self._boustro_skip_count = 0         # 连续跳过的航点数（过多则提前收尾）

        # ── A* 局部路径缓存（各阶段共用）──
        self._local_path = []                # astar_grid 规划出的中间航点
        self._local_path_idx = 0

        # ── 冲刺探索(CROSS_EXPLORE)阶段状态 ──
        self._cross_goal = None              # 当前的远端目标点
        self._cross_attempts = 0
        self._cross_round = 0                # 已完成的冲刺轮数
        self._cross_goal_started = 0.0
        self._cross_goal_last_progress = 0.0
        self._cross_goal_best_dist = float('inf')  # 历史最近距离（判断是否在进展）

        # ── 补扫(GAP_FILL)阶段状态 ──
        self._gap_goal = None
        self._gap_attempts = 0
        self._gap_failed_goals = []          # 失败/无收益目标黑名单
        self._gap_started_time = 0.0
        self._gap_unknown_at_goal = None     # 派目标时的未知格数（用于评估收益）
        self._gap_goal_started = 0.0
        self._gap_goal_last_progress = 0.0
        self._gap_goal_best_dist = float('inf')
        self._completion_reason = ''         # 探索结束原因（写入日志）
        self._boundary_revisit_goals = []    # 已做过的边界回访点

        # ── 防原地打转 & 物理卡死跟踪 ──
        # anti-spin：麦轮机器人若长时间纯旋转很容易在原地空转，
        # 这些变量用来监测"是否在自转"以便强制掺入前进/横移。
        self._cmd_time = 0.0
        self._cmd_pos = (0.0, 0.0)
        self._spin_start = 0.0
        self._spinning = False
        # physical-stuck：发了速度指令但里程计几乎不动 = 被物理卡住
        self._phys_stuck_time = 0.0
        self._phys_stuck_pos = (0.0, 0.0)
        self._phys_stuck_escape = False      # 是否正处于脱困动作中
        self._phys_escape_start = 0.0
        self._phys_escape_dir = 1.0          # 脱困横移方向(+1 左/-1 右)
        self._phys_stuck_count = 0           # 连续卡死次数（脱困力度随之升级）
        self._phys_last_stuck_time = 0.0
        # 通用导航停滞检测（各阶段判断"目标够不着"时复用）
        self._nav_stuck_time = 0.0
        self._nav_stuck_pos = (0.0, 0.0)

        # 地图 QoS 用锁存(TRANSIENT_LOCAL)，晚订阅也能拿到最后一帧地图
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.scan_sub = self.create_subscription(LaserScan, '/scan_raw', self.scan_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.cmd_pub = self.create_publisher(Twist, str(self._p('cmd_topic')), 1)
        # 四个 Trigger 服务：手动启动/停止/存图/查询当前阶段
        self.create_service(Trigger, '~/start', self.start_cb)
        self.create_service(Trigger, '~/stop', self.stop_cb)
        self.create_service(Trigger, '~/save_map', self.save_map_cb)
        self.create_service(Trigger, '~/init_finish', self.state_cb)
        # 主控制循环 10Hz（0.1s）；另有一个 1Hz 定时器负责自动启动
        self.timer = self.create_timer(0.1, self.control_loop)
        self.auto_start_timer = self.create_timer(1.0, self._check_auto_start)
        self.get_logger().info(
            '\033[1;36mAutoExplore v4.1 ready (wall-follow + boustrophedon + anti-spin)\033[0m')

    def _p(self, n):
        """读取参数值的简写。全文件用 self._p('xxx') 取参数，比
        self.get_parameter('xxx').value 短很多。"""
        return self.get_parameter(n).value

    def _coverage_percent(self):
        """地图覆盖率(%) = (已知自由格 + 占据格) / 总格数。

        未知格(-1)不计入。这是衡量探索进度、判断是否停滞的核心指标。
        """
        if self.map_data is None:
            return 0.0
        g = self.map_data
        return (np.sum(g == 0) + np.sum(g > 50)) / g.size * 100.0

    # ─── ROS Callbacks ───

    def _check_auto_start(self):
        """自动启动逻辑（1Hz）：收到地图就立刻开跑；否则等够
        startup_wait_sec 秒后也强行开跑——机器人一动，激光扫描
        变化能帮助 SLAM 更快初始化并产出第一帧地图。"""
        if self.running:
            return

        now = time.time()
        if self._map_received:
            self.get_logger().info('\033[1;32mMap received - auto-starting!\033[0m')
        elif now - self._node_start_time < self._p('startup_wait_sec'):
            return  # 还没到等待上限，继续等地图
        else:
            self.get_logger().warn(
                '\033[1;33mMap not received yet - starting exploration to warm up SLAM.\033[0m')

        self.running = True
        self._start_time = now
        self._last_checkpoint_save = now
        # 进入第一个工作阶段：贴墙。记录起点用于后续闭环检测
        self.phase = PHASE_WALL_FOLLOW
        self._wf_start_pos = (self.robot_x, self.robot_y)
        self._wf_start_time = now
        self._phys_stuck_time = now
        self._phys_stuck_pos = (self.robot_x, self.robot_y)
        self.auto_start_timer.cancel()  # 已启动，关掉自动启动定时器

    def start_cb(self, req, resp):
        """服务 ~/start：手动（重新）开始探索，重置到贴墙阶段。"""
        self.running = True
        self._map_saved = False
        self._start_time = time.time()
        self._last_checkpoint_save = self._start_time
        self.phase = PHASE_WALL_FOLLOW
        self._wf_start_pos = (self.robot_x, self.robot_y)
        self._wf_start_time = time.time()
        resp.success = True
        return resp

    def stop_cb(self, req, resp):
        """服务 ~/stop：停止探索、立刻发零速停车并保存当前地图。"""
        self.running = False
        self.cmd_pub.publish(Twist())  # 空 Twist 即全 0 速度 = 停车
        self._try_save_map('stop_requested')
        resp.success = True
        return resp

    def save_map_cb(self, req, resp):
        """服务 ~/save_map：手动触发保存地图，返回是否成功。"""
        ok = self._try_save_map('manual')
        resp.success = ok
        resp.message = 'map saved' if ok else 'save failed'
        return resp

    def state_cb(self, req, resp):
        """服务 ~/init_finish：返回当前所处阶段名（供外部脚本查询进度）。"""
        resp.success = True
        resp.message = self.phase
        return resp

    def map_callback(self, msg):
        """地图回调：把一维 data 重塑成 (height, width) 的二维 numpy 数组。

        OccupancyGrid.data 是行优先的一维数组，reshape 后 [row, col]
        对应地图上的一格，值 -1=未知 / 0=自由 / ~100=占据。
        """
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        self.map_info = msg.info
        if not self._map_received:
            self.get_logger().info('\033[1;32m[Map] First map received.\033[0m')
        self._map_received = True

    def scan_callback(self, msg):
        """激光回调：缓存测距数组及扫描的起始角与角分辨率。

        后续 _scan_sector 用 angle_min + i*angle_increment 反推每束
        激光的角度，所以这两个量必须一并保存。
        """
        self.laser_ranges = np.array(msg.ranges, dtype=np.float32)
        self.scan_angle_min = float(msg.angle_min)
        self.scan_angle_increment = float(msg.angle_increment)

    def _scan_sector(self, center_angle, half_width):
        """Return scan values in a robot-relative angular sector.

        取出以 center_angle 为中心、±half_width 范围内的所有激光测距值
        （角度都是机器人本体坐标系：0=正前，+π/2=正左，-π/2=正右）。
        用途：判断前方/左侧/右侧是否有障碍。做法是把每束激光的角度
        与中心角作差并规整到 (-π,π]，保留角差在半宽内的那些测距。
        """
        if self.laser_ranges is None or len(self.laser_ranges) == 0:
            return np.array([], dtype=np.float32)

        increment = self.scan_angle_increment
        if abs(increment) < 1e-9:  # 保护：角分辨率异常时按均匀分布估算
            increment = (2.0 * math.pi) / len(self.laser_ranges)

        # 每束激光的绝对角度 = 起始角 + 序号 × 角增量
        indices = np.arange(len(self.laser_ranges), dtype=np.float32)
        angles = self.scan_angle_min + indices * increment
        # atan2(sin, cos) 把角差规整到 (-π,π]，避免 ±π 附近漏选
        delta = np.arctan2(
            np.sin(angles - center_angle),
            np.cos(angles - center_angle),
        )
        return self.laser_ranges[np.abs(delta) <= half_width]

    def _clean_scan_sector(self, center_angle, half_width):
        """在 _scan_sector 基础上滤掉无效测距。

        <0.02m 多为传感器噪声/自身遮挡，>12m 超出可信量程或表示
        无回波(inf)。只保留有效值，避免用 0 或 inf 误判障碍距离。
        """
        values = self._scan_sector(center_angle, half_width)
        return values[(values > 0.02) & (values < 12.0)]

    def odom_callback(self, msg):
        """里程计回调：更新机器人位姿 (x, y, yaw)。yaw 由四元数换算。"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = quat_to_yaw(msg.pose.pose.orientation)

    # ─── Main Control Loop ───

    def control_loop(self):
        """主控制循环（10Hz）：状态机的总调度中枢。

        每次执行的固定顺序（优先级从高到低）：
          1. 未启动或没激光数据 → 直接返回；
          2. 汇报进度（含定期存图检查点）；
          3. 已完成 → 持续发停车指令；
          4. 物理脱困最高优先：正在脱困或刚检测到卡死时先处理，
             其它阶段逻辑一律让路；
          5. 按当前 phase 分派到对应的阶段处理函数。
        """
        if not self.running or self.laser_ranges is None:
            return
        now = time.time()
        self._report_progress(now)

        if self.phase == PHASE_COMPLETE:
            self.cmd_pub.publish(Twist())
            return

        # 物理脱困优先于一切阶段逻辑：正在做脱困动作就继续做完
        if self._phys_stuck_escape:
            self._do_physical_escape(now)
            return

        # 检测是否物理卡死（在发指令但里程计不动）；触发则本轮交给脱困
        if self._check_physical_stuck(now):
            return

        # 根据当前阶段分派处理
        if self.phase == PHASE_WALL_FOLLOW:
            self._wall_follow_step(now)
        elif self.phase == PHASE_CROSS_EXPLORE:
            self._cross_explore_step(now)
        elif self.phase == PHASE_PLAN_COVERAGE:
            self._plan_coverage()
        elif self.phase == PHASE_BOUSTROPHEDON:
            self._boustrophedon_step(now)
        elif self.phase == PHASE_GAP_FILL:
            self._gap_fill_step(now)
        elif self.phase == PHASE_COMPLETE:
            self.cmd_pub.publish(Twist())

    # ─── Physical Stuck Detection ───

    def _check_physical_stuck(self, now):
        '''Detect: wheels spinning but robot not moving (odom unchanged).

        原理：记录一个参考位置 _phys_stuck_pos 和时刻 _phys_stuck_time。
        只要机器人相对参考点移动超过 8cm，就认为"在正常前进"，刷新参考
        并返回 False；若持续 5 秒都没挪动 8cm，则判定被卡住，进入脱困。

        _phys_stuck_count 记录连续卡死次数，脱困力度会随之升级；
        但若距上次卡死已超过 20 秒（说明中间正常走了一段），则清零重来。
        '''
        moved = math.hypot(
            self.robot_x - self._phys_stuck_pos[0],
            self.robot_y - self._phys_stuck_pos[1])

        if moved > 0.08:
            # 正常移动：刷新参考点与计时
            self._phys_stuck_pos = (self.robot_x, self.robot_y)
            self._phys_stuck_time = now
            if now - self._phys_last_stuck_time > 20.0:
                self._phys_stuck_count = 0  # 久未卡死，重置升级计数
            return False

        # 5 秒内位移不足 8cm → 判定物理卡死，启动脱困
        if now - self._phys_stuck_time > 5.0:
            self._phys_stuck_count += 1
            self._phys_last_stuck_time = now
            self.get_logger().warn(
                f'\033[1;31m[STUCK] Wheels spinning but not moving! '
                f'Escape #{self._phys_stuck_count}...\033[0m')
            self._phys_stuck_escape = True
            self._phys_escape_start = now
            # 选择横移脱困方向：比较左右两侧激光中位距离，往更空的一侧躲。
            # 反复卡死时用 count 的奇偶交替左右，避免总往同一边撞死。
            side_half_width = math.pi / 4.0
            left_values = self._clean_scan_sector(math.pi / 2.0, side_half_width)
            right_values = self._clean_scan_sector(-math.pi / 2.0, side_half_width)
            left_clear = float(np.median(left_values)) if len(left_values) else 0.0
            right_clear = float(np.median(right_values)) if len(right_values) else 0.0
            if self._phys_stuck_count % 2 == 1:
                self._phys_escape_dir = 1.0 if left_clear > right_clear else -1.0
            else:
                self._phys_escape_dir = -1.0 if left_clear > right_clear else 1.0
            self._phys_stuck_time = now
            self._phys_stuck_pos = (self.robot_x, self.robot_y)
            return True
        return False

    def _do_physical_escape(self, now):
        '''Escalating escape: longer and more aggressive with repeated stucks.

        分两阶段的脱困动作（充分利用麦轮的横移能力）：
          阶段1（前半程）：后退 + 大幅横移 + 小转向，先脱离接触点；
          阶段2（后半程）：斜向前进（前进 + 横移），从新角度绕开障碍。
        持续时长 esc_duration 随连续卡死次数增加（2s 起，封顶 6s），
        卡得越狠、挣脱动作越久越猛。时间到即结束脱困、清空局部路径、
        恢复正常阶段逻辑。
        '''
        twist = Twist()
        elapsed = now - self._phys_escape_start
        lat_speed = self._p('lateral_speed')
        lin_speed = self._p('linear_speed')

        # 脱困时长随连续卡死次数升级（每多卡一次 +1s，最长 6s）
        esc_duration = min(2.0 + self._phys_stuck_count * 1.0, 6.0)
        phase1_end = esc_duration * 0.5
        phase2_end = esc_duration

        if elapsed < phase1_end:
            # 阶段1：强力后退 + 横移（linear.y 即麦轮横向平移）
            twist.linear.y = self._phys_escape_dir * lat_speed * 2.0
            twist.linear.x = -lin_speed * 0.8
            twist.angular.z = self._phys_escape_dir * 0.3
        elif elapsed < phase2_end:
            # 阶段2：斜向前冲（前进+横移），换个方向脱身
            twist.linear.x = lin_speed * 1.2
            twist.linear.y = self._phys_escape_dir * lat_speed * 1.0
            twist.angular.z = -self._phys_escape_dir * 0.2
        else:
            # 脱困完成：复位状态，回到正常探索
            self._phys_stuck_escape = False
            self._phys_stuck_time = now
            self._phys_stuck_pos = (self.robot_x, self.robot_y)
            self._local_path = []  # 旧局部路径已失效，清空重新规划
            self.get_logger().info(
                f'[STUCK] Escape #{self._phys_stuck_count} done '
                f'({esc_duration:.1f}s), resuming...')
            self.cmd_pub.publish(Twist())
            return

        self.cmd_pub.publish(twist)

    # ─── Phase 1: Wall Follow ───

    def _wall_follow_step(self, now):
        """贴墙走一圈把房间外轮廓先建出来（阶段1）。

        采用经典的"右手/左手法则"贴墙：默认贴右墙(follow_right)，用
        PD 控制器把机器人到侧墙的距离稳定在 target_d。麦轮的横移
        (linear.y) 让机器人能在不改变朝向的情况下微调离墙距离。

        退出本阶段的三个条件（任一满足即切到 CROSS_EXPLORE）：
          - 闭环：走够最短时长后回到出发点附近（绕完了一圈）；
          - 覆盖率停滞：40 秒内覆盖率涨幅不足 1%（贴墙已无新收益）；
          - 硬超时：贴墙超过 5 分钟。
        """
        twist = Twist()
        ranges = self.laser_ranges
        n = len(ranges)

        target_d = self._p('wall_follow_dist')  # 期望离墙距离
        kp = self._p('wall_follow_kp')           # PD 比例增益
        kd = self._p('wall_follow_kd')           # PD 微分增益
        follow_right = (self._p('wall_follow_side') == 'right')

        if n == 0:
            return

        # 前方扇区半宽：随激光点数自适应，夹在 [0.12, 60°] 之间
        increment = abs(self.scan_angle_increment)
        if increment < 1e-9:
            increment = (2.0 * math.pi) / n
        sector_half_width = max(0.12, min(math.pi / 3.0, (n // 8) * increment / 2.0))
        front_values = self._clean_scan_sector(0.0, sector_half_width)
        front_min = float(np.min(front_values)) if len(front_values) else np.inf

        # 取"侧向"和"侧前方"两个扇区：
        #   side  ≈ 正侧方(±90°)，用于测量离墙距离；
        #   fr    ≈ 侧前方(±45°)，提前发现墙拐角好转弯。
        if follow_right:
            side_range = self._clean_scan_sector(-math.pi / 2.0, math.pi / 4.0)
            fr_range = self._clean_scan_sector(-math.pi / 4.0, math.pi / 8.0)
        else:
            side_range = self._clean_scan_sector(math.pi / 2.0, math.pi / 4.0)
            fr_range = self._clean_scan_sector(math.pi / 4.0, math.pi / 8.0)

        side_min = float(np.min(side_range)) if len(side_range) else np.inf
        fr_min = float(np.min(fr_range)) if len(fr_range) else np.inf

        stop_d = self._p('obstacle_stop_dist')
        speed = self._p('linear_speed')
        ang_speed = self._p('angular_speed')

        # ── 退出条件① 闭环检测 ──
        # 必须先走够 wall_min_duration，否则一起步就"在起点附近"会误判
        elapsed = now - self._wf_start_time
        if elapsed > self._p('wall_min_duration') and self._wf_start_pos:
            dist_to_start = math.hypot(
                self.robot_x - self._wf_start_pos[0],
                self.robot_y - self._wf_start_pos[1])
            if dist_to_start < self._p('wall_loop_close_dist'):
                self.get_logger().info(
                    f'\033[1;32m[WallFollow] Loop closed! '
                    f'({elapsed:.0f}s, corners={self._wf_corner_count})\033[0m')
                self.phase = PHASE_CROSS_EXPLORE
                self._cross_goal = None
                self._cross_attempts = 0
                self.cmd_pub.publish(Twist())
                return

        # ── 退出条件② 覆盖率停滞 ──
        # 每当覆盖率涨幅超过 1% 就刷新"最近增长时刻"；若 40 秒都没长
        # 且已跑够 30 秒，说明贴墙再走也扫不出新东西，提前转下一阶段。
        if self.map_data is not None:
            g = self.map_data
            coverage = (np.sum(g == 0) + np.sum(g > 50)) / g.size * 100.0
            if coverage - self._wf_last_coverage > 1.0:
                self._wf_last_coverage = coverage
                self._wf_coverage_stall_time = now
            elif now - self._wf_coverage_stall_time > 40.0 and elapsed > 30.0:
                self.get_logger().info(
                    f'[WallFollow] Coverage stalled at {coverage:.1f}%, '
                    f'navigating to far side...')
                self.phase = PHASE_CROSS_EXPLORE
                self._cross_goal = None
                self._cross_attempts = 0
                self.cmd_pub.publish(Twist())
                return

        # ── 退出条件③ 硬超时（保险丝，防止贴墙无限循环）──
        if elapsed > 300.0:
            self.get_logger().warn('[WallFollow] Timeout (5min), navigating to far side...')
            self.phase = PHASE_CROSS_EXPLORE
            self._cross_goal = None
            self._cross_attempts = 0
            self.cmd_pub.publish(Twist())
            return

        # ─── 贴墙运动控制（按优先级从急到缓）───
        # 情形A：正前方被挡 → 朝内转 + 横移躲开（绝不纯自转，防空转）。
        # 贴右墙时向左横移(lat_dir=-1)拉开距离，并带一点点后退解卡。
        if front_min < stop_d * 2.5:
            turn_dir = 1.0 if follow_right else -1.0
            twist.angular.z = turn_dir * ang_speed * 0.7
            # 始终掺入横移，避免退化为原地打转
            lat_dir = -1.0 if follow_right else 1.0
            twist.linear.y = lat_dir * self._p('lateral_speed')
            twist.linear.x = -0.02  # 轻微后退帮助脱离
            self._wf_corner_count += 1
            self.cmd_pub.publish(twist)
            return

        # 情形B：侧前方逼近（快到内墙角）→ 边慢速前进边朝内轻转
        if fr_min < target_d * 0.8:
            turn_dir = 1.0 if follow_right else -1.0
            twist.angular.z = turn_dir * self._p('angular_speed_slow')
            twist.linear.x = speed * 0.7
            self.cmd_pub.publish(twist)
            return

        # 情形C：常规巡墙——对"侧墙距离误差"做 PD 控制。
        # error>0 表示离墙太远(side_min<target)，需要靠近；反之远离。
        # correction = kp*误差 + kd*误差变化率，是本阶段的核心公式。
        error = target_d - side_min
        d_error = error - self._wf_prev_error
        self._wf_prev_error = error
        correction = kp * error + kd * d_error

        twist.linear.x = speed

        if side_min > target_d * 3.0:
            # 侧墙丢失（拐过外墙角）→ 朝墙的方向转并前进去重新找到墙
            turn_dir = -1.0 if follow_right else 1.0
            twist.angular.z = turn_dir * self._p('angular_speed_slow')
            twist.linear.x = speed * 0.6
        else:
            # 主要靠横移(linear.y)纠偏，辅以极小的转向；符号取决于贴哪侧。
            # 贴右墙时 correction>0(需靠近右墙)对应向右横移，故取负号。
            lat_speed = self._p('lateral_speed')
            if follow_right:
                twist.linear.y = -np.clip(correction, -lat_speed, lat_speed)
            else:
                twist.linear.y = np.clip(correction, -lat_speed, lat_speed)
            twist.angular.z = -correction * 0.5 if follow_right else correction * 0.5
            twist.angular.z = np.clip(twist.angular.z, -0.3, 0.3)

        # 靠近前方障碍时按距离线性降速（越近越慢，最低降到 30%）
        if front_min < self._p('obstacle_slow_dist'):
            twist.linear.x *= max(0.3, (front_min - stop_d) /
                                   (self._p('obstacle_slow_dist') - stop_d + 0.001))

        self.cmd_pub.publish(twist)

    # ─── Phase 1.5: Cross-Explore (go to far side) ───

    def _cross_explore_step(self, now):
        '''Navigate to the farthest frontier cluster to ensure both sides explored.

        阶段1.5（贴墙之后、覆盖规划之前）：主动冲向"最远的 frontier"。

        为什么需要它：贴墙只能建出机器人所在这一侧的轮廓，房间另一头
        （如穿过门后的邻室）可能完全没被打开。先跑到最远的未知边界，
        能保证地图两端都被"点亮"，后续弓字形覆盖才不会漏掉半间屋。

        结构：无目标时先挑一个最远 frontier；有目标时持续导航过去，
        同时用"到达/超时/无进展/卡死"多重判据决定何时放弃并进入
        PLAN_COVERAGE。前两轮到达后还会顺势再找下一个远端 frontier。
        '''
        if self._cross_goal is not None:
            dist = math.hypot(
                self._cross_goal[0] - self.robot_x,
                self._cross_goal[1] - self.robot_y)
            # 到达判定放宽为 2 倍容差：这里只求"大致到达那一侧"
            if dist < self._p('goal_tolerance') * 2.0:
                self.get_logger().info(
                    f'[CrossExplore] Reached far side, round {self._cross_round+1} done!')
                self._cross_goal = None
                self._cross_round += 1
                self._local_path = []
                self.cmd_pub.publish(Twist())
                # 前两轮：到达后立刻再找一个更远的 frontier 继续冲，
                # 尽量把各个方向的未知区都打开；之后才转去规划覆盖。
                if self._cross_round <= 2:
                    next_goal = self._find_farthest_frontier()
                    if next_goal is not None:
                        dist_to_next = math.hypot(
                            next_goal[0] - self.robot_x,
                            next_goal[1] - self.robot_y)
                        if dist_to_next > 0.8:
                            self._cross_goal = next_goal
                            self._cross_attempts = 0
                            self._cross_goal_started = now
                            self._cross_goal_last_progress = now
                            self._cross_goal_best_dist = dist_to_next
                            self._nav_stuck_time = now
                            self._nav_stuck_pos = (self.robot_x, self.robot_y)
                            self.get_logger().info(
                                f'[CrossExplore] Round {self._cross_round+1}: '
                                f'next frontier at ({next_goal[0]:.1f},{next_goal[1]:.1f}), '
                                f'd={dist_to_next:.1f}m')
                            return
                self.phase = PHASE_PLAN_COVERAGE
                return

            # 放弃判据① 时间预算耗尽：这个远端目标花的时间太久
            if (
                self._cross_goal_started > 0.0
                and now - self._cross_goal_started > self._p('cross_goal_timeout_sec')
            ):
                self.get_logger().warn(
                    '[CrossExplore] Goal time budget exhausted, '
                    'planning coverage from the current map')
                self._cross_goal = None
                self._local_path = []
                self.phase = PHASE_PLAN_COVERAGE
                self.cmd_pub.publish(Twist())
                return

            # 放弃判据② 距离不再缩小：只要比历史最近距离又近了
            # cross_goal_min_progress，就刷新进展计时；否则若停滞超过
            # cross_goal_stall_sec 说明卡在半路（绕不过障碍），放弃。
            if dist < self._cross_goal_best_dist - self._p('cross_goal_min_progress'):
                self._cross_goal_best_dist = dist
                self._cross_goal_last_progress = now
            elif (
                self._cross_goal_last_progress > 0.0
                and now - self._cross_goal_last_progress > self._p('cross_goal_stall_sec')
            ):
                self.get_logger().warn(
                    '[CrossExplore] Goal distance is not improving, '
                    'planning coverage from the current map')
                self._cross_goal = None
                self._local_path = []
                self.phase = PHASE_PLAN_COVERAGE
                self.cmd_pub.publish(Twist())
                return

            # 放弃判据③ 物理原地不动：每 12 秒检查一次位移，若基本没动
            # (<8cm) 累计 3 次仍脱不了身，就放弃这个目标改去规划覆盖。
            moved = math.hypot(
                self.robot_x - self._nav_stuck_pos[0],
                self.robot_y - self._nav_stuck_pos[1])
            if now - self._nav_stuck_time > 12.0:
                if moved < 0.08:
                    self._cross_attempts += 1
                    if self._cross_attempts >= 3:
                        self.get_logger().warn(
                            '[CrossExplore] Cannot reach far side, '
                            'planning coverage from here...')
                        self._cross_goal = None
                        self._local_path = []
                        self.phase = PHASE_PLAN_COVERAGE
                        self.cmd_pub.publish(Twist())
                        return
                self._nav_stuck_time = now
                self._nav_stuck_pos = (self.robot_x, self.robot_y)

            # 正常情况：用 A* 取下一个局部航点，再驱动机器人过去
            nav_target = self._get_next_local_waypoint(self._cross_goal)
            twist = self._navigate_to_point(nav_target)
            self.cmd_pub.publish(twist)
            return

        # 当前没有目标：挑选最远的 frontier 簇作为新目标
        goal = self._find_farthest_frontier()
        if goal is None:
            self.get_logger().info('[CrossExplore] No far frontiers, planning coverage...')
            self.phase = PHASE_PLAN_COVERAGE
            return

        self._cross_goal = goal
        self._cross_attempts = 0
        self._cross_goal_started = now
        self._local_path = []
        self._nav_stuck_time = now
        self._nav_stuck_pos = (self.robot_x, self.robot_y)
        dist = math.hypot(goal[0] - self.robot_x, goal[1] - self.robot_y)
        self._cross_goal_last_progress = now
        self._cross_goal_best_dist = dist
        self.get_logger().info(
            f'\033[1;35m[CrossExplore] Navigating to far frontier '
            f'({goal[0]:.1f},{goal[1]:.1f}), d={dist:.1f}m\033[0m')

    def _find_farthest_frontier(self):
        '''Find the farthest large frontier cluster from current position.

        与 _find_frontier_goal（补扫阶段用）思路相同，但打分相反：
        这里挑**最远**的可达 frontier，目的是把地图另一端撑开。
        '''
        if self.map_data is None:
            return None
        g = self.map_data
        h, w = g.shape
        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y

        # 统计每格周围 8 邻域里有多少个未知格。用 np.roll 把整张地图
        # 向 8 个方向各平移一格再累加，是"每格邻域未知数"的向量化算法，
        # 比逐格 for 循环快得多。unknown_neighbors>0 即代表该格紧邻未知区。
        unk = (g == -1)
        unknown_neighbors = np.zeros_like(g, dtype=np.int16)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy == 0 and dx == 0:
                    continue
                unknown_neighbors += (
                    np.roll(np.roll(unk, dy, axis=0), dx, axis=1)
                ).astype(np.int16)

        # 关键：目标取"紧邻未知区的已知自由格"，而不是未知格本身。
        # 若直接把目标设在未知格里，A* 会因终点是障碍而拒绝，导致机器人
        # 对着同一堵墙反复尝试。fm[边缘]=False 是去掉贴边的伪 frontier。
        fm = (g == 0) & (unknown_neighbors > 0)
        fm[:2, :] = fm[-2:, :] = fm[:, :2] = fm[:, -2:] = False

        if np.sum(fm) < 5:
            return None

        # 栅格索引 → 世界坐标（+0.5 取格中心），再聚成簇
        fy, fx = np.where(fm)
        wx = ox + (fx + 0.5) * res
        wy = oy + (fy + 0.5) * res
        pts = np.column_stack([wx, wy])

        clusters = self._grid_cluster(pts, 0.4)
        if not clusters:
            return None

        rx, ry = self.robot_x, self.robot_y
        # 只保留"离机器人 >0.5m 且 A* 可达"的簇，再按距离降序取最远
        scored = []
        for cx, cy in clusters:
            dd = math.hypot(cx - rx, cy - ry)
            if dd > 0.5 and self._is_reachable_goal(cx, cy):
                scored.append((dd, cx, cy))

        if not scored:
            return None

        scored.sort(key=lambda s: -s[0])  # 最远者优先
        return (scored[0][1], scored[0][2])

    # ─── Phase 2: Plan Coverage Path ───

    def _plan_coverage(self):
        """阶段2：规划"弓字形(boustrophedon)"覆盖路径，把房间内部扫满。

        boustrophedon = 像耕地/扫地机那样一行一行来回蛇形走位。做法：
          1. 求出可覆盖区域的世界坐标包围盒（自由格 + frontier 格）；
          2. 从离机器人最近的角落开始，沿 y 方向按 row_spacing 分行；
          3. 每一行在 x_min↔x_max 之间来回走（奇偶行方向相反），
             首尾相接形成连续的"弓"字；
          4. 过滤掉落在墙体实心区里的航点，得到最终航点列表。
        """
        if self.map_data is None:
            return

        g = self.map_data
        h, w = g.shape
        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y

        free_mask = (g == 0)
        if np.sum(free_mask) < 10:  # 已知自由区太小，没什么可覆盖，直接收尾
            self.phase = PHASE_GAP_FILL
            return

        # 把 frontier 格(紧邻自由区的未知格)也纳入覆盖范围，
        # 让弓字形边界略微外扩，覆盖时顺带把边缘未知区扫开。
        unk_mask = (g == -1)
        frontier_mask = np.zeros_like(g, dtype=bool)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy == 0 and dx == 0:
                    continue
                shifted_free = np.roll(np.roll(free_mask, dy, axis=0), dx, axis=1)
                frontier_mask |= (unk_mask & shifted_free)
        coverage_mask = free_mask | frontier_mask

        fy, fx = np.where(coverage_mask)
        row_spacing = self._p('row_spacing')

        # 计算覆盖区世界坐标包围盒，四边各向内缩 row_spacing 一行，
        # 避免航点压在最外圈的墙上
        x_min_w = ox + float(np.min(fx)) * res + row_spacing
        x_max_w = ox + float(np.max(fx)) * res - row_spacing
        y_min_w = oy + float(np.min(fy)) * res + row_spacing
        y_max_w = oy + float(np.max(fy)) * res - row_spacing

        if x_max_w <= x_min_w or y_max_w <= y_min_w:  # 缩完就没空间了 → 收尾
            self.phase = PHASE_GAP_FILL
            return

        # 从离机器人最近的角落起步，减少空驶。going_right/start_from_bottom
        # 决定第一行朝哪走、从上还是下开始，让路径起点贴近当前位置。
        corners = [
            (x_min_w, y_min_w), (x_max_w, y_min_w),
            (x_min_w, y_max_w), (x_max_w, y_max_w)]
        corners.sort(key=lambda c: math.hypot(c[0]-self.robot_x, c[1]-self.robot_y))
        start_x, start_y = corners[0]
        going_right = (start_x == x_min_w)
        start_from_bottom = (start_y == y_min_w)

        # 按 row_spacing 生成一系列 y 值（行），必要时反转使其从起点侧开始
        path = []
        ys = np.arange(y_min_w, y_max_w + 0.01, row_spacing)
        if not start_from_bottom:
            ys = ys[::-1]

        # 逐行生成两个端点；going_right 每行翻转，形成来回蛇形
        for y_val in ys:
            if going_right:
                path.append((x_min_w, float(y_val)))
                path.append((x_max_w, float(y_val)))
            else:
                path.append((x_max_w, float(y_val)))
                path.append((x_min_w, float(y_val)))
            going_right = not going_right

        # 过滤穿墙航点：检查每个航点周围 5×5 栅格窗口，若窗口里有
        # 自由格或未知格就保留（说明该点附近是可达/待探索的空间）；
        # 若整片都是障碍，则丢弃这个航点。
        filtered_path = []
        for wx, wy in path:
            mx = int(np.clip((wx - ox) / res, 0, w - 1))
            my = int(np.clip((wy - oy) / res, 0, h - 1))
            y_lo = max(0, my - 2); y_hi = min(h, my + 3)
            x_lo = max(0, mx - 2); x_hi = min(w, mx + 3)
            region = g[y_lo:y_hi, x_lo:x_hi]
            if np.any(region == 0) or np.any(region == -1):
                filtered_path.append((wx, wy))

        self._coverage_path = filtered_path
        self._coverage_idx = 0
        self._local_path = []
        self._nav_stuck_time = time.time()
        self._nav_stuck_pos = (self.robot_x, self.robot_y)
        self._boustro_last_coverage = self._coverage_percent()
        self._boustro_last_gain_time = time.time()
        self._boustro_skip_count = 0

        self.get_logger().info(
            f'\033[1;34m[Coverage] Planned {len(filtered_path)} waypoints '
            f'({len(ys)} rows)\033[0m')

        # 有航点就去执行弓字形；一个都没有(全被墙过滤掉)则直接收尾
        self.phase = PHASE_BOUSTROPHEDON if filtered_path else PHASE_GAP_FILL

    # ─── Phase 3: Boustrophedon Execution ───

    def _boustrophedon_step(self, now):
        """阶段3：逐个执行弓字形航点。附带停滞检测与"跳过卡住航点"。

        对每个航点：到达则前进到下一个；长时间靠近不了(15s 未缩短
        30%)或彻底不动(6s 没挪 5cm)就跳过它并计数。整条路径走完，
        或覆盖率长时间不再增长/跳过次数过多时，转入 GAP_FILL 收尾。
        """
        # 所有航点都处理完了 → 覆盖阶段结束
        if self._coverage_idx >= len(self._coverage_path):
            self.get_logger().info(
                '\033[1;32m[Boustrophedon] Coverage path complete!\033[0m')
            self.phase = PHASE_GAP_FILL
            self._local_path = []
            self.cmd_pub.publish(Twist())
            return

        # 覆盖率停滞监控：有明显增长就刷新计时并清零跳过计数；
        # 若长时间无增长，或连续跳过太多航点，说明覆盖已无收益，收尾。
        coverage = self._coverage_percent()
        if coverage - self._boustro_last_coverage >= self._p('boustrophedon_min_coverage_gain'):
            self._boustro_last_coverage = coverage
            self._boustro_last_gain_time = now
            self._boustro_skip_count = 0
        elif (
            now - self._boustro_last_gain_time > self._p('boustrophedon_stall_sec')
            or self._boustro_skip_count >= int(self._p('boustrophedon_max_skips'))
        ):
            self.get_logger().warn(
                '[Boustrophedon] Coverage stalled, switching to gap fill...')
            self.phase = PHASE_GAP_FILL
            self._gap_goal = None
            self._local_path = []
            self._nav_stuck_time = now
            self._nav_stuck_pos = (self.robot_x, self.robot_y)
            self.cmd_pub.publish(Twist())
            return

        goal = self._coverage_path[self._coverage_idx]
        dist = math.hypot(goal[0] - self.robot_x, goal[1] - self.robot_y)

        # 到达当前航点：推进到下一个
        if dist < self._p('waypoint_tolerance'):
            self._advance_waypoint(now)
            return

        # 记录本航点的起始时刻与起始距离（用于"是否在靠近"的判断）
        if self._wp_start_time == 0.0 or self._wp_start_dist == 999.0:
            self._wp_start_time = now
            self._wp_start_dist = dist

        # 进展检查：尝试 15 秒后若距离仍大于起始的 70%（没缩短 30%），
        # 认为到不了，跳过；否则重置检查窗口继续尝试。
        wp_elapsed = now - self._wp_start_time
        if wp_elapsed > 15.0:
            if dist > self._wp_start_dist * 0.7:
                self.get_logger().warn(
                    f'[Boustrophedon] No progress toward wp {self._coverage_idx} '
                    f'(d={dist:.2f}m, was {self._wp_start_dist:.2f}m), skipping...')
                self._boustro_skip_count += 1
                self._advance_waypoint(now)
                self.cmd_pub.publish(Twist())
                return
            # 有进展：开启下一个检查窗口
            self._wp_start_time = now
            self._wp_start_dist = dist

        # 绝对卡死：6 秒内几乎没动(<5cm)也直接跳过该航点
        moved = math.hypot(
            self.robot_x - self._nav_stuck_pos[0],
            self.robot_y - self._nav_stuck_pos[1])
        if now - self._nav_stuck_time > 6.0:
            if moved < 0.05:
                self.get_logger().warn(
                    f'[Boustrophedon] Stuck at wp {self._coverage_idx}, skipping...')
                self._boustro_skip_count += 1
                self._advance_waypoint(now)
                self.cmd_pub.publish(Twist())
                return
            self._nav_stuck_time = now
            self._nav_stuck_pos = (self.robot_x, self.robot_y)

        # 正常前往航点：A* 取局部路径下一点 → 运动控制
        nav_target = self._get_next_local_waypoint(goal)
        twist = self._navigate_to_point(nav_target)
        self.cmd_pub.publish(twist)

    def _advance_waypoint(self, now):
        """前进到下一个覆盖航点，并复位与"到达/卡死判定"相关的计时状态。"""
        self._coverage_idx += 1
        self._local_path = []  # 换目标了，旧的 A* 局部路径作废
        self._nav_stuck_time = now
        self._nav_stuck_pos = (self.robot_x, self.robot_y)
        self._wp_start_time = 0.0
        self._wp_start_dist = 999.0

    def _get_next_local_waypoint(self, final_goal):
        '''A* path planning to goal, returns next intermediate waypoint.

        把"最终目标"拆成一串可通行的局部小目标：先看是否已有缓存的
        局部路径且没走完——若快到当前局部点就推进到下一个并返回；
        没有缓存时用 astar_grid 在栅格地图上规划一条路径，按约 0.25m
        间隔抽稀成航点缓存起来。A* 失败(如目标被墙围住)则退回直接朝
        final_goal 走，交给 _navigate_to_point 的反应式避障处理。

        为什么要分局部航点：直接朝最终目标直线冲会撞墙；沿 A* 路径
        逐点走既能绕开已知障碍，又不必每帧都重规划，省算力。
        '''
        # 已有未走完的局部路径：判断是否该切换到下一个局部航点
        if self._local_path and self._local_path_idx < len(self._local_path):
            wp = self._local_path[self._local_path_idx]
            dist = math.hypot(wp[0] - self.robot_x, wp[1] - self.robot_y)
            if dist < self._p('waypoint_tolerance') * 1.5:
                self._local_path_idx += 1
                if self._local_path_idx < len(self._local_path):
                    return self._local_path[self._local_path_idx]
                return final_goal
            return wp

        if self.map_data is None or self.map_info is None:
            return final_goal

        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        h, w = self.map_data.shape

        # 世界坐标 → 栅格索引：(x-ox)/res 得列(col)，(y-oy)/res 得行(row)
        sc = int((self.robot_x - ox) / res)
        sr = int((self.robot_y - oy) / res)
        gc = int((final_goal[0] - ox) / res)
        gr = int((final_goal[1] - oy) / res)

        path_cells = astar_grid(self.map_data, (sr, sc), (gr, gc), inflation=2)
        if path_cells is None or len(path_cells) < 2:
            return final_goal

        # 抽稀：每隔约 0.25m 取一个路径点（step 是对应的栅格步数），
        # 把 A* 的密集格路径转成稀疏的世界坐标航点，末尾补上最终目标。
        step = max(1, int(0.25 / res))
        self._local_path = []
        for i in range(0, len(path_cells), step):
            r, c = path_cells[i]
            wx = ox + (c + 0.5) * res
            wy = oy + (r + 0.5) * res
            self._local_path.append((wx, wy))
        self._local_path.append(final_goal)
        self._local_path_idx = 0

        return self._local_path[0] if self._local_path else final_goal

    # ─── Phase 4: Gap Fill ───

    def _gap_fill_step(self, now):
        """阶段4：收尾补扫。逐个访问剩余 frontier，把零散未知区补掉。

        与前面阶段最大的不同：这里会**评估每次前往的实际收益**——
        到达目标后比较"未知格减少了多少"，收益不足的目标点会被拉黑，
        避免在无意义的 frontier 上耗时间。frontier 都用完后再尝试
        边界回访(boundary revisit)；仍无目标则判定探索结束。

        整体预算：总时长超 gap_fill_timeout_sec 或失败目标数达上限
        即强制收尾，保证探索一定会在有限时间内结束。
        """
        if self._gap_started_time == 0.0:
            self._gap_started_time = now

        # 预算耗尽（超时或失败目标过多）→ 带着当前"尽力而为"的地图收尾
        if (
            now - self._gap_started_time > self._p('gap_fill_timeout_sec')
            or len(self._gap_failed_goals) >= self._p('max_gap_fill_failures')
        ):
            self._finish_exploration('gap_fill_budget_exhausted')
            return

        if self._gap_goal is not None:
            dist = math.hypot(
                self._gap_goal[0] - self.robot_x,
                self._gap_goal[1] - self.robot_y)
            if dist < self._p('goal_tolerance'):
                # 到达目标：用"到达后未知格数"对比"派目标时未知格数"，
                # 差值 unknown_delta 就是这趟补扫的实际收益（探明了多少格）。
                unknown_now = int(np.sum(self.map_data == -1)) if self.map_data is not None else 0
                unknown_delta = (
                    self._gap_unknown_at_goal - unknown_now
                    if self._gap_unknown_at_goal is not None
                    else 0
                )
                if (
                    self._gap_unknown_at_goal is not None
                    and unknown_delta >= self._p('min_unknown_reduction')
                ):
                    self.get_logger().info(
                        f'[GapFill] Frontier visit reduced unknown cells by '
                        f'{unknown_delta}')
                else:
                    # 收益太小：把该点拉黑，防止之后又选到它白跑一趟
                    self._gap_failed_goals.append(self._gap_goal)
                    self.get_logger().warn(
                        f'[GapFill] Frontier yielded limited gain '
                        f'({unknown_delta} unknown cells), blacklisting it '
                        f'({len(self._gap_failed_goals)}/'
                        f'{int(self._p("max_gap_fill_failures"))})')
                self._gap_goal = None
                self._gap_attempts = 0
                self._local_path = []
                return

            # 进展/超时判据：距离比历史最近再缩 0.15m 才算有进展；
            # 否则单目标超时或停滞过久就拉黑换下一个 frontier。
            if dist < self._gap_goal_best_dist - 0.15:
                self._gap_goal_best_dist = dist
                self._gap_goal_last_progress = now
            elif (
                now - self._gap_goal_started > self._p('gap_goal_timeout_sec')
                or now - self._gap_goal_last_progress > self._p('gap_goal_stall_sec')
            ):
                self._gap_failed_goals.append(self._gap_goal)
                self.get_logger().warn(
                    f'[GapFill] Goal made insufficient progress, switching '
                    f'frontier ({len(self._gap_failed_goals)}/'
                    f'{int(self._p("max_gap_fill_failures"))})')
                self._gap_goal = None
                self._gap_attempts = 0
                self._local_path = []
                self._nav_stuck_time = now
                self._nav_stuck_pos = (self.robot_x, self.robot_y)
                self.cmd_pub.publish(Twist())
                return

            # 物理卡死判据：每 10 秒查一次，位移不足 8cm 累计 3 次即拉黑放弃
            moved = math.hypot(
                self.robot_x - self._nav_stuck_pos[0],
                self.robot_y - self._nav_stuck_pos[1])
            if now - self._nav_stuck_time > 10.0:
                if moved < 0.08:
                    self._gap_attempts += 1
                    if self._gap_attempts >= 3:
                        self._gap_failed_goals.append(self._gap_goal)
                        self.get_logger().warn(
                            f'[GapFill] Goal unreachable, skipping '
                            f'({len(self._gap_failed_goals)}/'
                            f'{int(self._p("max_gap_fill_failures"))})')
                        self._gap_goal = None
                        self._gap_attempts = 0
                        self._local_path = []
                        self._nav_stuck_time = now
                        self._nav_stuck_pos = (self.robot_x, self.robot_y)
                        self.cmd_pub.publish(Twist())
                        return
                self._nav_stuck_time = now
                self._nav_stuck_pos = (self.robot_x, self.robot_y)

            # 正常前往补扫目标
            nav_target = self._get_next_local_waypoint(self._gap_goal)
            twist = self._navigate_to_point(nav_target)
            self.cmd_pub.publish(twist)
            return

        # 当前没有补扫目标：先找新的 frontier；找不到再退而求其次做边界回访
        goal = self._find_frontier_goal()
        if goal is None:
            goal = self._find_boundary_revisit_goal()
            if goal is None:
                # frontier 和边界回访都没了 ⇒ 可达区域全部探明，正常完成
                self._finish_exploration('no_reachable_frontiers')
                return
            self._boundary_revisit_goals.append(goal)
            self.get_logger().warn(
                f'[BoundaryRevisit] No frontier selected; revisiting map edge '
                f'at ({goal[0]:.2f},{goal[1]:.2f}) before accepting map.')

        # 锁定新目标，并记下此刻的未知格数作为收益评估基准
        self._gap_goal = goal
        self._gap_attempts = 0
        self._local_path = []
        self._nav_stuck_time = now
        self._nav_stuck_pos = (self.robot_x, self.robot_y)
        self._gap_unknown_at_goal = (
            int(np.sum(self.map_data == -1)) if self.map_data is not None else None
        )
        self._gap_goal_started = now
        self._gap_goal_last_progress = now
        self._gap_goal_best_dist = math.hypot(
            goal[0] - self.robot_x, goal[1] - self.robot_y)

    def _finish_exploration(self, reason):
        """Stop exploration once the frontier budget is exhausted or closed.

        探索的唯一出口：切到 COMPLETE 阶段、保存地图、停车。
        reason 区分两种结局：'no_reachable_frontiers' 是理想的
        "全部探完"（打绿色 INFO）；其它 reason 表示预算耗尽的
        "尽力而为地图"（打黄色 WARN 提醒图可能不完整）。
        """
        self._completion_reason = reason
        unknown = int(np.sum(self.map_data == -1)) if self.map_data is not None else -1
        if reason == 'no_reachable_frontiers':
            self.get_logger().info(
                f'\033[1;32m=== Exploration COMPLETE! === '
                f'(reason={reason}, unknown={unknown})\033[0m')
        else:
            self.get_logger().warn(
                f'\033[1;33m=== Exploration STOPPED WITH BEST-EFFORT MAP === '
                f'(reason={reason}, unknown={unknown})\033[0m')
        self.phase = PHASE_COMPLETE
        self._try_save_map(f'exploration_{reason}')
        self.cmd_pub.publish(Twist())

    # ─── Navigation ───

    def _is_reachable_goal(self, goal_x, goal_y):
        """Check reachability using only known free cells in the current map.

        用 A* 在当前地图上试算一条路径来判断目标是否可达，
        避免把车派去一个被墙隔死的目标。
        - 世界坐标 -> 栅格下标的换算：col=(x-ox)/res, row=(y-oy)/res
          （origin 是栅格 (0,0) 角点的世界坐标）；
        - 只把"已知空闲"格当可走（未知格不算路），所以结论偏保守；
        - inflation=2 表示把障碍向外膨胀 2 格，给车身留安全余量；
        - 地图还没建立时返回 True（先放行，探索初期没有依据可查）。
        """
        if self.map_data is None or self.map_info is None:
            return True
        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        h, w = self.map_data.shape
        # numpy 数组按 (行, 列) 索引，行对应 y、列对应 x，注意顺序
        start = (
            int((self.robot_y - oy) / res),
            int((self.robot_x - ox) / res),
        )
        goal = (
            int((goal_y - oy) / res),
            int((goal_x - ox) / res),
        )
        path = astar_grid(self.map_data, start, goal, inflation=2)
        return path is not None

    def _navigate_to_point(self, goal):
        '''Navigate to point. NEVER pure rotation > 2s; always add lateral.

        底层"开往一个点"的反应式控制律（不走全局路径，纯激光反应）。
        设计核心是防呆：麦轮车原地纯旋转时激光视角变化剧烈，SLAM 匹配
        容易失败，所以任何情况下都不允许长时间纯旋转，卡壳时优先
        用麦轮特有的横移（linear.y）脱困。分三种情形：
          1. 正前方近距离有障碍 -> 横移躲避（不旋转）；
          2. 朝向偏差大 -> 限时旋转（超 2 秒强制加前进+横移打破僵局）；
          3. 正常 -> 比例转向 + 直行，接近目标和接近障碍时减速。
        '''
        twist = Twist()
        # 目标方向与当前朝向的偏差（angle_diff 已归一化到 [-pi, pi]）
        dx = goal[0] - self.robot_x
        dy = goal[1] - self.robot_y
        dist = math.hypot(dx, dy)
        goal_angle = math.atan2(dy, dx)
        heading_error = angle_diff(goal_angle, self.robot_yaw)

        # 取"正前方约 ±(n/16 束)"的扇区求最近障碍距离，
        # 扇区宽度按激光角分辨率折算，夹在 [0.10, pi/6] 弧度之间
        n = len(self.laser_ranges)
        front_half_width = max(
            0.10,
            min(math.pi / 6.0, (n // 16) * max(abs(self.scan_angle_increment), 1e-6)),
        )
        front_values = self._clean_scan_sector(0.0, front_half_width)
        front_min = float(np.min(front_values)) if len(front_values) else np.inf

        stop_d = self._p('obstacle_stop_dist')
        slow_d = self._p('obstacle_slow_dist')
        speed = self._p('linear_speed')
        ang_speed = self._p('angular_speed')
        lat_speed = self._p('lateral_speed')

        # 情形 1：前方受阻 -> 横移躲避（绝不原地旋转）。
        # 比较左右两侧 90° 扇区的中位距离，往更空旷的一侧横移，
        # 同时带一点微小转向和 -0.02 的轻微倒车帮助脱离。
        if front_min < stop_d * 2.0:
            left_range = self._clean_scan_sector(math.pi / 2.0, math.pi / 4.0)
            right_range = self._clean_scan_sector(-math.pi / 2.0, math.pi / 4.0)
            # 用中位数而不是最小值：抗单束噪声
            left_clear = float(np.median(left_range)) if len(left_range) else 0.0
            right_clear = float(np.median(right_range)) if len(right_range) else 0.0

            # 横移 + 缓转（麦轮专属逃生方式，永不纯旋转）
            if left_clear > right_clear:
                twist.linear.y = lat_speed
                twist.angular.z = self._p('angular_speed_slow') * 0.5
            else:
                twist.linear.y = -lat_speed
                twist.angular.z = -self._p('angular_speed_slow') * 0.5
            twist.linear.x = -0.02
            self._spinning = False
            return twist

        # 情形 2：朝向偏差 > 0.5rad（约 29°）-> 需要旋转，但限时。
        # 用 _spinning/_spin_start 记录连续旋转时长
        if abs(heading_error) > 0.5:
            now = time.time()
            if not self._spinning:
                self._spinning = True
                self._spin_start = now

            spin_duration = now - self._spin_start

            if spin_duration > 2.0:
                # 已连续旋转超 2 秒：叠加前进+横移强行打破"原地打转"，
                # 超 4 秒重置计时器重新给一轮机会
                self.get_logger().debug('Anti-spin: adding forward+lateral')
                twist.linear.x = speed * 0.5
                twist.linear.y = lat_speed * (1.0 if heading_error > 0 else -1.0)
                twist.angular.z = ang_speed * 0.3 * (1.0 if heading_error > 0 else -1.0)
                if spin_duration > 4.0:
                    self._spinning = False
                    self._spin_start = now
            else:
                # 2 秒以内的正常转向；linear.x 保持 20% 慢速前进，
                # 让激光视角持续平移，SLAM 匹配更稳
                twist.angular.z = ang_speed * (1.0 if heading_error > 0 else -1.0)
                twist.linear.x = speed * 0.2  # always creep forward
            return twist

        self._spinning = False

        # 情形 3：正常行驶——比例转向（P 控制，增益 2.0）+ 全速直行
        twist.linear.x = speed
        twist.angular.z = heading_error * 2.0
        twist.angular.z = np.clip(twist.angular.z, -ang_speed, ang_speed)

        # 距目标 0.5m 内线性减速（最低保留 40%），避免冲过头
        if dist < 0.5:
            twist.linear.x *= max(0.4, dist / 0.5)

        # 前方障碍进入减速带：按 (当前距离-停止距离)/带宽 比例降速
        if front_min < slow_d:
            ratio = max(0.2, (front_min - stop_d) / (slow_d - stop_d + 0.001))
            twist.linear.x *= ratio

        return twist

    # ─── Frontier Detection ───

    def _find_boundary_revisit_goal(self):
        """Pick a reachable free cell near under-observed map edges.

        边界回访：frontier 都探完后的"查漏补缺"环节。
        动机：SLAM 地图边缘常出现"该有墙却没画上"的薄弱带（激光
        太远命中数不足），如果就此收图，质量分析会报 missing_walls。
        做法：检查地图四边各一条 margin 宽的条带，若某条带里
        "占据格太少或未知格太多"（说明墙没画实），就在带内挑一个
        可达的空闲格作为回访点，开过去补扫。
        评分 = 紧迫度（缺墙程度）+ 越靠地图边缘越优 + 轻微偏好远点。
        """
        if not self._p('boundary_revisit_enabled'):
            return None
        if self.map_data is None or self.map_info is None:
            return None
        if len(self._boundary_revisit_goals) >= int(self._p('max_boundary_revisits')):
            return None

        g = self.map_data
        h, w = g.shape
        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        # 条带宽度：参数按米给出，换算成格数；上限为地图短边的 1/3，
        # 防止小地图时条带互相覆盖
        margin_cells = max(3, int(float(self._p('boundary_revisit_margin')) / max(res, 1.0e-6)))
        margin_cells = min(margin_cells, max(3, min(h, w) // 3))

        # 四边条带的 (行起, 行止, 列起, 列止)。行号小 = y 小 = 南
        sides = {
            'north': (max(0, h - margin_cells), h, 0, w),
            'south': (0, min(h, margin_cells), 0, w),
            'east': (0, h, max(0, w - margin_cells), w),
            'west': (0, h, 0, min(w, margin_cells)),
        }

        candidates = []
        for side, (r0, r1, c0, c1) in sides.items():
            region = g[r0:r1, c0:c1]
            if region.size == 0:
                continue
            # 该条带里 占据格（>50 视为墙）与 未知格（-1）的占比
            occupied_ratio = float(np.sum(region > 50)) / float(region.size)
            unknown_ratio = float(np.sum(region == -1)) / float(region.size)
            # 墙已画实（占据 >=4%）且几乎没未知区 -> 这条边没问题，跳过
            if occupied_ratio >= 0.04 and unknown_ratio < 0.04:
                continue

            free_rows, free_cols = np.where(region == 0)
            if len(free_rows) == 0:
                continue
            # 空闲格可能上千个，等间隔抽样约 100 个当候选就够了
            step = max(1, len(free_rows) // 100)
            # 紧迫度：占据比越低于 6% 越紧迫，未知比越高越紧迫（封顶 0.6）
            urgency = max(0.0, 0.06 - occupied_ratio) * 8.0 + min(0.6, unknown_ratio)
            for local_r, local_c in zip(free_rows[::step], free_cols[::step]):
                # 条带内局部下标 -> 全图下标 -> 世界坐标（+0.5 取格中心）
                row = int(r0 + local_r)
                col = int(c0 + local_c)
                wx = ox + (col + 0.5) * res
                wy = oy + (row + 0.5) * res
                # 排除三类点：离车太近（<0.45m 扫不出新信息）、
                # 离历史回访点/失败点太近（避免重复劳动）、A* 不可达
                if math.hypot(wx - self.robot_x, wy - self.robot_y) < 0.45:
                    continue
                if any(
                    math.hypot(wx - old_x, wy - old_y) < 0.65
                    for old_x, old_y in self._boundary_revisit_goals + self._gap_failed_goals
                ):
                    continue
                if not self._is_reachable_goal(wx, wy):
                    continue

                # 该点到所属地图边缘的格数：越贴边越值得去
                if side == 'north':
                    edge_distance = h - 1 - row
                elif side == 'south':
                    edge_distance = row
                elif side == 'east':
                    edge_distance = w - 1 - col
                else:
                    edge_distance = col

                distance = math.hypot(wx - self.robot_x, wy - self.robot_y)
                # 总分 = 缺墙紧迫度 + 贴边程度 + 轻微偏好远点（一次跑远点顺路多扫）
                score = urgency + 1.0 / (1.0 + edge_distance) + 0.04 * distance
                candidates.append((score, side, wx, wy, occupied_ratio, unknown_ratio))

        if not candidates:
            return None
        # 取总分最高的候选作为回访目标
        candidates.sort(key=lambda item: -item[0])
        score, side, wx, wy, occupied_ratio, unknown_ratio = candidates[0]
        self.get_logger().info(
            f'[BoundaryRevisit] {side} edge target ({wx:.2f},{wy:.2f}) '
            f'score={score:.2f} occ={occupied_ratio:.3f} unk={unknown_ratio:.3f}')
        return (wx, wy)

    def _find_frontier_goal(self):
        """检测 frontier 并选出信息收益最高的探索目标。

        frontier（前沿）= "已知空闲"与"未知"的交界格。往 frontier
        开就能把激光探进未知区，这是自动探索的核心思想。
        流程：逐格标记 -> 网格聚类 -> 打分选优。
        """
        if self.map_data is None:
            return None
        g = self.map_data
        h, w = g.shape
        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y

        # 第一步：统计每个格子 8 邻域里未知格的数量。
        # 技巧：对"未知掩码"做 8 个方向的 np.roll（整体平移）再累加，
        # 等价于 8 邻域卷积，纯向量化、无 Python 循环，快得多
        unk = (g == -1)
        unknown_neighbors = np.zeros_like(g, dtype=np.int16)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy == 0 and dx == 0:
                    continue
                unknown_neighbors += (
                    np.roll(np.roll(unk, dy, axis=0), dx, axis=1)
                ).astype(np.int16)
        # frontier 定义：本格已知空闲(=0) 且 邻域有未知格。
        # 目标点选在"已知空闲"一侧（车能安全到达），而非未知格里。
        # 地图最外 2 圈强制清掉：np.roll 是循环平移，边缘会"绕回"产生假 frontier
        fm = (g == 0) & (unknown_neighbors > 0)
        fm[:2, :] = fm[-2:, :] = fm[:, :2] = fm[:, -2:] = False

        if np.sum(fm) < self._p('min_frontier_size'):
            return None

        # 第二步：把零散 frontier 格聚成簇。按 frontier_cluster_dist
        # 为边长划分网格桶，同桶的点归为一簇（O(n) 的简易聚类，
        # 比 DBSCAN 之类快，精度对选目标足够）
        fy, fx = np.where(fm)
        wx = ox + (fx + 0.5) * res
        wy = oy + (fy + 0.5) * res
        strengths = unknown_neighbors[fy, fx]
        cs = self._p('frontier_cluster_dist')

        buckets = {}
        for px, py, strength in zip(wx, wy, strengths):
            key = (int(px / cs), int(py / cs))
            buckets.setdefault(key, []).append((px, py, int(strength)))

        # 每簇取质心；strength（邻域未知格总数）代表这簇背后未知区的大小
        clusters = []
        for members in buckets.values():
            if len(members) < self._p('min_frontier_size'):
                continue
            cx = sum(p[0] for p in members) / len(members)
            cy = sum(p[1] for p in members) / len(members)
            total_strength = sum(p[2] for p in members)
            clusters.append((cx, cy, len(members), total_strength))

        if not clusters:
            return None

        # 第三步：打分选目标。过滤掉太近的（去了也扫不出新东西）、
        # 黑名单附近的（之前去不了的地方别再撞）、A* 不可达的；
        # 得分 = 未知强度 + 0.5*簇大小 - 0.25*距离（收益大且顺路者优先）
        rx, ry = self.robot_x, self.robot_y
        scored = []
        for cx, cy, cluster_size, total_strength in clusters:
            dd = math.hypot(cx - rx, cy - ry)
            if dd < self._p('min_frontier_distance'):
                continue
            if any(
                math.hypot(cx - fx, cy - fy) < self._p('frontier_blacklist_dist')
                for fx, fy in self._gap_failed_goals
            ):
                continue
            if not self._is_reachable_goal(cx, cy):
                continue
            info_score = total_strength + 0.5 * cluster_size - 0.25 * dd
            scored.append((info_score, dd, cx, cy, cluster_size, total_strength))

        if not scored:
            return None

        # 同分时取更近的
        scored.sort(key=lambda item: (-item[0], item[1]))
        _, dist, bx, by, cluster_size, total_strength = scored[0]
        self.get_logger().info(
            f'[GapFill] Free approach: ({bx:.2f},{by:.2f}) '
            f'dist={dist:.2f}m size={cluster_size} strength={total_strength}')
        return (bx, by)

    def _grid_cluster(self, pts, cs):
        """通用网格聚类：按 cs 边长分桶取质心，桶内点数不足则丢弃。"""
        if len(pts) == 0:
            return []
        grid = {}
        for px, py in pts:
            k = (int(px / cs), int(py / cs))
            grid.setdefault(k, []).append((px, py))
        return [(sum(p[0] for p in v) / len(v), sum(p[1] for p in v) / len(v))
                for v in grid.values() if len(v) >= self._p('min_frontier_size')]

    # ─── Progress Reporting ───

    def _report_progress(self, now):
        """按固定间隔打印探索进度（覆盖率、阶段、位置），并触发检查点存图。

        coverage = (空闲 + 占据) / 总格数，即"已知区域"的占比；
        每个阶段附带各自的关键指标（覆盖路点进度 / 目标距离 / 失败数），
        方便在无 GUI 的终端里判断探索是否健康推进。
        """
        if now - self._last_progress_report < self._p('progress_report_interval'):
            return
        self._last_progress_report = now
        if self.map_data is None:
            return
        g = self.map_data
        total = g.size
        free = int(np.sum(g == 0))
        occupied = int(np.sum(g > 50))
        unknown = int(np.sum(g == -1))
        coverage = (free + occupied) / total * 100.0
        elapsed = now - self._start_time if self._start_time else 0
        mins = int(elapsed // 60)
        secs = int(elapsed % 60)

        phase_info = self.phase
        if self.phase == PHASE_BOUSTROPHEDON:
            phase_info += f' ({self._coverage_idx}/{len(self._coverage_path)})'
        elif self.phase == PHASE_CROSS_EXPLORE and self._cross_goal:
            d = math.hypot(self._cross_goal[0] - self.robot_x,
                           self._cross_goal[1] - self.robot_y)
            phase_info += f' (d={d:.1f}m, r={self._cross_round})'
        elif self.phase == PHASE_GAP_FILL:
            phase_info += f' (failed={len(self._gap_failed_goals)})'

        self.get_logger().info(
            f'\033[1;33m[{phase_info}] {mins:02d}:{secs:02d} | '
            f'coverage={coverage:.1f}% | free={free} occ={occupied} unk={unknown} | '
            f'pos=({self.robot_x:.2f},{self.robot_y:.2f})\033[0m')
        self._try_checkpoint_save(now)

    # ─── Map Saving ───

    def _try_checkpoint_save(self, now):
        """周期性检查点存图：长时间探索中途崩溃也能保住阶段成果。

        与最终存图的区别是 mark_saved=False——检查点不算"已保存"，
        探索结束时仍会再存一次最终版。
        """
        interval = float(self._p('checkpoint_save_interval_sec'))
        if interval <= 0.0 or not self._p('auto_save_map'):
            return False
        if self.map_data is None:
            return False
        if now - self._last_checkpoint_save < interval:
            return False
        self._last_checkpoint_save = now
        return self._save_map_to_disk('checkpoint', mark_saved=False)

    def _try_save_map(self, reason='unknown'):
        """幂等的最终存图入口：已存过就直接返回成功（可被多处安全调用）。"""
        if self._map_saved:
            return True
        return self._save_map_to_disk(reason, mark_saved=True)

    def _save_map_to_disk(self, reason='unknown', mark_saved=True):
        """调用 nav2 的 map_saver_cli 子进程把 /map 存成 PGM/YAML。

        以子进程方式运行而不是自己写文件：复用官方工具的格式保证。
        map_subscribe_transient_local:=true 让它能收到 slam_toolbox
        以锁存 QoS 发布的最后一帧地图；30 秒超时防止子进程卡死拖住探索。
        """
        if not self._p('auto_save_map'):
            return False
        save_path = Path(os.path.expanduser(str(self._p('map_save_path')))).expanduser()
        save_dir = save_path.parent
        save_dir.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(
            f'\033[1;35m[MapSave] Saving map ({reason}) -> {save_path}\033[0m')
        try:
            env = os.environ.copy()
            env['ROS_DOMAIN_ID'] = env.get('ROS_DOMAIN_ID', '0')
            result = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', str(save_path),
                 '--ros-args', '-p', 'map_subscribe_transient_local:=true'],
                timeout=30, capture_output=True, text=True, env=env)
            if result.returncode == 0:
                if mark_saved:
                    self._map_saved = True
                self.get_logger().info(
                    f'\033[1;32m[MapSave] SUCCESS: {save_path}.pgm + .yaml\033[0m')
                return True
            else:
                self.get_logger().error(f'[MapSave] FAILED: {result.stderr}')
                return False
        except subprocess.TimeoutExpired:
            self.get_logger().error('[MapSave] Timeout (30s)')
            return False
        except Exception as e:
            self.get_logger().error(f'[MapSave] Error: {e}')
            return False


def main(args=None):
    """入口：spin 主循环，Ctrl+C 退出前先停车并抢救性存一次图。"""
    rclpy.init(args=args)
    node = AutoExploreNode('auto_explore')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C received, saving map before exit...')
    finally:
        # 退出兜底：先发零速度让车停下，再尝试保存地图，
        # 保证手动中断的探索成果不会白跑
        if rclpy.ok():
            try:
                node.cmd_pub.publish(Twist())
                node._try_save_map('shutdown')
            except Exception as exc:
                node.get_logger().warn(f'Shutdown save skipped: {exc}')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
