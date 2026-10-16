"""MecaMind 轻量 2D 仿真器的核心数学模型（不依赖 ROS）。

本文件实现仿真器的"物理引擎"部分：麦克纳姆轮全向底盘的运动学积分、
速度/加速度限幅、简单碰撞检测，以及 2D 激光雷达的射线投射采样。

它刻意不 import 任何 ROS 模块——纯 Python + math，好处是：
1. 可以脱离 ROS 环境直接写 pytest 单元测试（改一个公式立刻能验证）；
2. 与 `simulator_node.py`（负责 ROS 消息收发）职责清晰分离。

与系统中其他模块的关系：
- 被 `simulator_node.py` 持有并驱动：节点每个定时器周期调用 ``step(dt)``
  推进物理，再调用 ``scan_ranges()`` 生成激光数据；
- 依赖 `world_geometry.py` 的 ``WorldGeometry``：碰撞检测和射线求交
  这些"世界相关"的几何运算都委托给它。

初学者建议重点阅读的函数：
- ``MecaMindSimModel.step``               —— 仿真一步做了什么（速度斜坡 + 积分）
- ``MecaMindSimModel._integrate_candidate``—— 本体坐标系速度如何旋转到世界坐标系
                                             （这是移动机器人运动学最核心的公式）
- ``MecaMindSimModel.scan_ranges``        —— 激光雷达每束光的角度如何计算
- ``_ramp``                                —— 用加速度限幅模拟电机"不能瞬间变速"
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import random
from typing import Iterable, List

from .world_geometry import WorldGeometry


@dataclass
class SimPose:
    """机器人在世界坐标系下的 2D 位姿。

    x/y 单位为米，yaw 为绕 z 轴的偏航角（弧度，逆时针为正，
    0 表示朝向世界坐标系 +x 方向）。2D 平面机器人只需要这三个量
    就能完全描述位置和朝向（即 SE(2) 位姿）。
    """

    x: float
    y: float
    yaw: float


@dataclass
class SimTwist:
    """机器人本体坐标系下的速度指令/状态。

    注意参考系是"机器人自己"：vx 沿车头方向（m/s），vy 沿车身左侧
    （m/s，麦轮底盘才有的横移能力！普通差速车 vy 恒为 0），
    wz 为自转角速度（rad/s，逆时针为正）。与 ROS 的
    geometry_msgs/Twist 中 linear.x / linear.y / angular.z 一一对应。
    """

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


def _normalize_angle(angle: float) -> float:
    """把任意角度归一化到 (-pi, pi] 区间。

    机器人持续旋转时 yaw 会无限累加（比如转 10 圈变成 62.8），
    而下游算法（TF、SLAM）都假设角度在 ±pi 内。用 atan2(sin, cos)
    归一化是数值上最稳的写法：sin/cos 天然消除了整圈数，
    atan2 再根据象限还原出等价的最小角度，不需要写 while 循环减 2*pi。
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def _ramp(current: float, target: float, limit: float) -> float:
    """把 current 朝 target 移动，但单步变化量不超过 ±limit（斜坡限幅）。

    物理意义：电机不可能让速度瞬间跳变，加速度有限。每个仿真步允许的
    最大速度增量就是 limit = 最大加速度 * dt。若目标与当前差距在限幅内
    则一步到位，否则按最大加速度逐步逼近——这就是"梯形速度曲线"的由来。
    """
    delta = max(-limit, min(limit, target - current))
    return current + delta


class MecaMindSimModel:
    """麦克纳姆轮机器人的 2D 运动学仿真模型。

    只做"运动学"仿真（速度直接决定位移），不做"动力学"仿真
    （不算力、摩擦、轮子打滑）——对室内导航教学来说，运动学层面
    加上加速度限幅已经足够逼真，而且代码量小、便于讲解。

    状态量三件套：
    - ``pose``     当前真实位姿（世界坐标系）
    - ``velocity`` 当前实际速度（本体坐标系，受加速度限幅影响滞后于指令）
    - ``command``  最新目标速度（本体坐标系，已做速度限幅）

    使用流程：外部先 ``set_command()`` 设定目标速度，然后周期性调用
    ``step(dt)`` 推进仿真，随时读取 ``pose``/``velocity`` 或调用
    ``scan_ranges()`` 采样激光。
    """

    def __init__(
        self,
        geometry: WorldGeometry,
        initial_pose: SimPose | None = None,
        robot_radius: float = 0.22,
        max_linear_x: float = 0.45,
        max_linear_y: float = 0.35,
        max_angular_z: float = 1.5,
        max_accel_x: float = 0.9,
        max_accel_y: float = 0.9,
        max_accel_z: float = 1.8,
        sensor_offset_x: float = 0.16,
        sensor_offset_y: float = 0.0,
        lidar_range_max: float = 8.0,
        lidar_beams: int = 360,
        lidar_angle_min: float = -math.pi,
        lidar_angle_max: float = math.pi,
        lidar_noise_std: float = 0.0,
        lidar_range_bias: float = 0.0,
        lidar_dropout_prob: float = 0.0,
        random_seed: int = 8,
    ) -> None:
        """初始化模型；各参数含义见 simulator_node 中同名 ROS 参数的注释。

        这里也内置了一套激光噪声参数（noise_std / range_bias / dropout），
        供脱离 ROS 的单元测试使用；在完整系统中节点层还会再注入一次噪声，
        所以节点默认把这里的噪声设为 0，避免双重叠加。
        """
        self.geometry = geometry
        self.pose = initial_pose or SimPose(-3.2, -2.4, 0.0)
        self.velocity = SimTwist()
        self.command = SimTwist()
        # 碰撞检测把机器人简化为半径 robot_radius 的圆——
        # 判断"圆是否碰墙"等价于"圆心到墙的距离是否小于半径"
        self.robot_radius = robot_radius
        self.max_linear_x = max_linear_x
        self.max_linear_y = max_linear_y
        self.max_angular_z = max_angular_z
        self.max_accel_x = max_accel_x
        self.max_accel_y = max_accel_y
        self.max_accel_z = max_accel_z
        # 雷达在本体坐标系中的安装偏移（车头前方 16cm）
        self.sensor_offset_x = sensor_offset_x
        self.sensor_offset_y = sensor_offset_y
        self.lidar_range_max = lidar_range_max
        self.lidar_beams = max(1, lidar_beams)
        self.lidar_angle_min = lidar_angle_min
        self.lidar_angle_max = lidar_angle_max
        self.lidar_noise_std = max(0.0, lidar_noise_std)
        self.lidar_range_bias = lidar_range_bias
        self.lidar_dropout_prob = min(1.0, max(0.0, lidar_dropout_prob))
        # 固定随机种子 => 每次运行噪声序列相同，实验/测试可复现
        self._random = random.Random(random_seed)
        # 仿真时间从 0 开始累计，由外部节点转换成 /clock 发布
        self.sim_time = 0.0

    def set_command(self, vx: float, vy: float, wz: float) -> None:
        """设置目标速度，同时按底盘物理极限做速度限幅。

        限幅在"指令入口"处做，保证不管上游发来多离谱的数值，
        仿真机器人也不会超过真实底盘的能力（与真实底盘驱动板行为一致）。
        """
        self.command.vx = max(-self.max_linear_x, min(self.max_linear_x, vx))
        self.command.vy = max(-self.max_linear_y, min(self.max_linear_y, vy))
        self.command.wz = max(-self.max_angular_z, min(self.max_angular_z, wz))

    def _sensor_origin(self) -> tuple[float, float]:
        """计算雷达在世界坐标系中的位置（激光射线的出发点）。

        雷达装在机器人身上，随机器人一起平移和旋转。把本体坐标系下的
        安装偏移 (offset_x, offset_y) 变换到世界坐标系，用的是标准的
        2D 刚体变换（旋转矩阵 + 平移）：
            world_x = robot_x + offset_x*cos(yaw) - offset_y*sin(yaw)
            world_y = robot_y + offset_x*sin(yaw) + offset_y*cos(yaw)
        这正是 TF 在幕后帮我们做的事，这里手写一遍便于理解原理。
        """
        offset_x = self.sensor_offset_x
        offset_y = self.sensor_offset_y
        cos_yaw = math.cos(self.pose.yaw)
        sin_yaw = math.sin(self.pose.yaw)
        return (
            self.pose.x + offset_x * cos_yaw - offset_y * sin_yaw,
            self.pose.y + offset_x * sin_yaw + offset_y * cos_yaw,
        )

    def _collision(self, x: float, y: float) -> bool:
        """判断机器人中心位于 (x, y) 时是否与墙体/障碍碰撞。

        margin=robot_radius 的含义：把所有障碍向外"膨胀"一个机器人半径，
        再把机器人当成一个点来检测——这是运动规划里经典的
        "配置空间（C-space）膨胀"技巧。
        """
        return self.geometry.is_occupied(x, y, margin=self.robot_radius)

    def _integrate_candidate(self, dt: float) -> None:
        """按当前速度积分一个 dt，得到新位姿，并处理碰撞。

        核心数学（本体坐标系速度 -> 世界坐标系位移）：
        机器人速度 (vx, vy) 是相对"车头朝向"定义的，要得到世界坐标系下的
        位移，需要按当前 yaw 做旋转变换：
            dx = (vx*cos(yaw) - vy*sin(yaw)) * dt
            dy = (vx*sin(yaw) + vy*cos(yaw)) * dt
        这是 2D 旋转矩阵 R(yaw) 乘速度向量的展开式。yaw 本身按
        yaw += wz*dt 积分（欧拉积分，dt 足够小的时候精度够用）。

        碰撞处理用"候选位姿列表 + 滑墙"策略：优先尝试完整位移；
        若撞墙则退化为只走 x 或只走 y（实现沿墙滑动的手感，
        而不是一碰墙就整个卡死）；四个候选全部碰撞则原地停下并清零速度。
        """
        base_x = self.pose.x
        base_y = self.pose.y
        base_yaw = self.pose.yaw
        # 本体速度旋转到世界坐标系（注意用积分前的 base_yaw）
        dx = (
            self.velocity.vx * math.cos(base_yaw) - self.velocity.vy * math.sin(base_yaw)
        ) * dt
        dy = (
            self.velocity.vx * math.sin(base_yaw) + self.velocity.vy * math.cos(base_yaw)
        ) * dt
        # 朝向积分后立刻归一化，防止长时间旋转导致角度无界增长
        yaw = _normalize_angle(base_yaw + self.velocity.wz * dt)

        # 候选按"完整移动 > 只沿 x 滑动 > 只沿 y 滑动 > 原地转向"降级尝试
        candidates = [
            (base_x + dx, base_y + dy, yaw),
            (base_x + dx, base_y, yaw),
            (base_x, base_y + dy, yaw),
            (base_x, base_y, yaw),
        ]
        for new_x, new_y, new_yaw in candidates:
            if not self._collision(new_x, new_y):
                self.pose = SimPose(new_x, new_y, new_yaw)
                return

        # 连"原地不动只转向"都碰撞（理论上只在出生点就在墙里时发生）：
        # 清零速度，模拟被完全卡死
        self.velocity = SimTwist()

    def step(self, dt: float) -> None:
        """推进一个仿真步：时间 += dt，速度做斜坡逼近，然后积分位姿。

        速度不是瞬间跳到指令值，而是以 max_accel * dt 为单步上限
        逐渐逼近（见 _ramp）——这让仿真机器人有真实的加减速过程，
        学生调 PID/前瞻控制时的体验会更接近实车。
        """
        self.sim_time += dt
        self.velocity.vx = _ramp(self.velocity.vx, self.command.vx, self.max_accel_x * dt)
        self.velocity.vy = _ramp(self.velocity.vy, self.command.vy, self.max_accel_y * dt)
        self.velocity.wz = _ramp(self.velocity.wz, self.command.wz, self.max_accel_z * dt)
        self._integrate_candidate(dt)

    def scan_ranges(self) -> List[float]:
        """模拟一帧完整的激光雷达扫描，返回每束激光的测距值（米）。

        步骤：
        1. 算出雷达在世界坐标系中的位置（射线起点）；
        2. 生成每束激光的世界朝向角。关键点：LaserScan 的角度是相对
           雷达自身坐标系定义的（angle_min 到 angle_max），所以要
           加上机器人当前 yaw 才是世界坐标系下的射线方向：
               heading = yaw + angle_min + i * increment
           分母用 (beams - 1) 是让首尾两束恰好落在 angle_min/angle_max 上；
        3. 对每束激光调用几何模块的 raycast 求"最近命中距离"；
        4. 可选注入传感器缺陷：丢包（返回最大量程）、高斯噪声、系统偏差，
           最后夹回 [0.12, range_max] 合法区间。
        """
        origin_x, origin_y = self._sensor_origin()
        if self.lidar_beams == 1:
            # 退化情况：只有一束激光时直接朝车头方向发射
            headings = [self.pose.yaw]
        else:
            increment = (self.lidar_angle_max - self.lidar_angle_min) / (self.lidar_beams - 1)
            headings = [
                self.pose.yaw + self.lidar_angle_min + index * increment
                for index in range(self.lidar_beams)
            ]
        ranges = []
        for heading in headings:
            # 射线投射：返回该方向上最近障碍的距离，无命中则为 max_range
            value = self.geometry.raycast(origin_x, origin_y, heading, self.lidar_range_max)
            if self.lidar_dropout_prob > 0.0 and self._random.random() < self.lidar_dropout_prob:
                # 模拟无回波：这里用最大量程表示（节点层的丢包则用 inf）
                ranges.append(self.lidar_range_max)
                continue
            if self.lidar_noise_std > 0.0:
                # 零均值高斯噪声：模拟测距的随机抖动
                value += self._random.gauss(0.0, self.lidar_noise_std)
            # 固定偏差：模拟雷达标定不准导致的系统性偏移
            value += self.lidar_range_bias
            ranges.append(max(0.12, min(self.lidar_range_max, value)))
        return ranges

    def scan_increment(self) -> float:
        """返回相邻两束激光的角度间隔（rad），供 LaserScan.angle_increment 使用。

        与 scan_ranges 里的 increment 计算保持一致（分母是 beams - 1），
        两处若不一致，下游按角度还原激光点云时会整体错位。
        """
        if self.lidar_beams <= 1:
            return 0.0
        return (self.lidar_angle_max - self.lidar_angle_min) / (self.lidar_beams - 1)

    def linear_acceleration(self) -> tuple[float, float, float]:
        """返回 IMU 加速度计读数（教学简化版：恒为零）。

        真实 IMU 会测到运动加速度和重力分量；本项目的定位/导航链路
        只用到 IMU 的姿态和角速度，所以加速度直接返回零占位，
        保留这个接口是为了消息字段完整。
        """
        return 0.0, 0.0, 0.0
