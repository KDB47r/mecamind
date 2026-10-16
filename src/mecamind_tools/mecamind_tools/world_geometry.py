"""三室户世界的 2D 几何表示：从 Gazebo 世界文件解析墙体/障碍并提供几何查询。

本文件不含任何 ROS 通信代码，它解决一个问题：**让轻量 2D 仿真器和
Gazebo 看到完全一样的世界**。做法是直接解析 Gazebo 使用的 SDF 世界文件
（three_room_house.world），把其中每个 box 碰撞体提取成 2D 轴对齐矩形
（AxisAlignedBox），组成 WorldGeometry。这样墙体位置只在 .world 文件里
维护一份，两个仿真后端永远不会"各说各话"。

对外提供三类几何查询（都是仿真器的基础运算）：
- ``WorldGeometry.is_occupied``       —— 某点是否在障碍内（碰撞检测用）
- ``WorldGeometry.raycast``           —— 射线最近命中距离（模拟激光雷达用）
- ``WorldGeometry.build_occupancy_grid`` —— 栅格化成占据栅格（生成真值地图用）

与其他模块的关系：
- `sim_core.py` 的碰撞检测和激光采样全部委托给本模块；
- `simulator_node.py` 启动时调用 ``load_world_geometry()`` 加载世界。

初学者建议重点阅读的函数：
- ``AxisAlignedBox.ray_distance`` —— 经典的 slab（平板）法射线-矩形求交，
                                     光线追踪和激光仿真的数学核心
- ``load_world_geometry``         —— 如何用 XML 解析从 SDF 提取几何信息
- ``WorldGeometry.build_occupancy_grid`` —— 占据栅格地图（OccupancyGrid）
                                     是怎么从连续几何"采样"出来的
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET


# frozen=True 使实例不可变（创建后字段不能改），几何定义天然是只读数据，
# 不可变还能避免被意外修改、可安全共享
@dataclass(frozen=True)
class AxisAlignedBox:
    """轴对齐矩形障碍（AABB, Axis-Aligned Bounding Box）的 2D 表示。

    "轴对齐"指矩形的边平行于世界坐标系 x/y 轴，不考虑旋转——
    本项目的墙和家具在 SDF 里都没有 yaw 旋转，所以这个简化成立，
    换来的是极其简单快速的包含判断与射线求交。

    用"中心点 + 宽(x 方向)/深(y 方向)"存储，与 SDF 中
    <pose>(中心) 和 <box><size>(尺寸) 的表达方式一一对应。
    height 只是记录用（2D 仿真不关心高度）。
    """

    name: str
    center_x: float
    center_y: float
    width: float
    depth: float
    height: float = 0.0

    # 下面四个属性把"中心+尺寸"换算成"最小/最大坐标"表示，
    # 后者在包含判断和射线求交里更好用
    @property
    def min_x(self) -> float:
        return self.center_x - self.width * 0.5

    @property
    def max_x(self) -> float:
        return self.center_x + self.width * 0.5

    @property
    def min_y(self) -> float:
        return self.center_y - self.depth * 0.5

    @property
    def max_y(self) -> float:
        return self.center_y + self.depth * 0.5

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        """判断点 (x, y) 是否落在矩形内（可外扩 margin）。

        margin > 0 相当于把矩形四周各膨胀 margin 米——配合
        "机器人半径"使用就实现了圆形机器人的碰撞检测
        （把障碍膨胀一个半径后，机器人可当质点处理）。
        """
        return (
            self.min_x - margin <= x <= self.max_x + margin
            and self.min_y - margin <= y <= self.max_y + margin
        )

    def ray_distance(
        self,
        origin_x: float,
        origin_y: float,
        dx: float,
        dy: float,
        max_range: float,
    ) -> float | None:
        """slab 法求射线与矩形的最近交点距离；不相交返回 None。

        射线用参数方程表示：P(t) = origin + t * (dx, dy)，t >= 0，
        (dx, dy) 是单位方向向量，因此参数 t 就等于沿射线走过的"距离"。

        slab 法思路：把矩形看成两组"平板"的交集——
        x 方向平板 [min_x, max_x] 和 y 方向平板 [min_y, max_y]。
        分别算出射线"停留在每个平板内"的参数区间，两区间的交集
        非空则射线穿过矩形，交集起点就是进入矩形的距离。
        """
        eps = 1e-9

        # ---- 第一步：射线在 x 平板内的参数区间 [tx_min, tx_max] ----
        if abs(dx) < eps:
            # 射线几乎平行于 y 轴（x 分量为 0）：不能除以 dx。
            # 此时若起点 x 不在平板内则永远进不去；在平板内则
            # "全程都在"，区间取 (-inf, +inf)
            if origin_x < self.min_x or origin_x > self.max_x:
                return None
            tx_min = -math.inf
            tx_max = math.inf
        else:
            # 解 origin_x + t*dx = min_x / max_x 得到穿过两条边界线的 t 值；
            # dx 可能为负（射线朝 -x 方向），所以要用 min/max 排序
            tx1 = (self.min_x - origin_x) / dx
            tx2 = (self.max_x - origin_x) / dx
            tx_min = min(tx1, tx2)
            tx_max = max(tx1, tx2)

        # ---- 第二步：同样方法算 y 平板的参数区间 ----
        if abs(dy) < eps:
            if origin_y < self.min_y or origin_y > self.max_y:
                return None
            ty_min = -math.inf
            ty_max = math.inf
        else:
            ty1 = (self.min_y - origin_y) / dy
            ty2 = (self.max_y - origin_y) / dy
            ty_min = min(ty1, ty2)
            ty_max = max(ty1, ty2)

        # ---- 第三步：两个区间求交集 ----
        # 进入矩形的时刻 = 较晚进入的那个平板；离开时刻 = 较早离开的那个
        enter = max(tx_min, ty_min)
        exit = min(tx_max, ty_max)
        # 三种"不命中"：整个矩形在射线反方向（exit < 0）、
        # 区间为空即射线从矩形旁边掠过（enter > exit）、
        # 交点超出雷达量程（enter > max_range）
        if exit < 0.0 or enter > exit or enter > max_range:
            return None
        if enter < 0.0 <= exit:
            # enter < 0 <= exit 说明射线起点已在矩形内部——距离记为 0
            # （对应雷达被贴在障碍上的极端情况）
            return 0.0
        hit = enter if enter >= 0.0 else exit
        if 0.0 <= hit <= max_range:
            return hit
        return None


@dataclass(frozen=True)
class OccupancyGridSpec:
    """占据栅格的纯数据描述，字段与 ROS 的 nav_msgs/OccupancyGrid 对应。

    data 按"行优先"（先 x 后 y）排列，每格取值 100（占据）或 0（空闲），
    origin_x/origin_y 是栅格左下角在世界坐标系中的位置。
    独立定义这个 dataclass 而不直接用 ROS 消息，是为了保持本模块
    无 ROS 依赖，转换成真正的 OccupancyGrid 消息由调用方完成。
    """

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: tuple[int, ...]


@dataclass(frozen=True)
class WorldGeometry:
    """整个世界的 2D 几何：一组矩形障碍 + 整体包围盒。

    min/max_x/y 是所有障碍的联合包围盒，用于确定占据栅格的尺寸。
    所有查询都是对 boxes 的线性遍历——教学世界只有几十个盒子，
    简单直接比空间索引（四叉树等）更利于理解。
    """

    name: str
    boxes: tuple[AxisAlignedBox, ...]
    min_x: float
    max_x: float
    min_y: float
    max_y: float

    def is_occupied(self, x: float, y: float, margin: float = 0.0) -> bool:
        """点 (x, y) 是否被任一障碍占据（margin 为障碍外扩量）。"""
        return any(box.contains(x, y, margin) for box in self.boxes)

    def raycast(self, origin_x: float, origin_y: float, heading: float, max_range: float) -> float:
        """从 (origin_x, origin_y) 沿 heading 方向投射射线，返回最近命中距离。

        这就是激光雷达单束光的数学模型：对每个障碍做 slab 求交，
        取所有命中里最近的一个；全都没命中则返回最大量程。
        小优化：把当前最近距离 best 作为 max_range 传给下一次求交，
        比 best 更远的交点会被提前剪掉。
        """
        # 朝向角 -> 单位方向向量，(cos, sin) 的模长恒为 1，
        # 保证求交得到的参数 t 直接就是米为单位的距离
        dx = math.cos(heading)
        dy = math.sin(heading)
        best = max_range
        for box in self.boxes:
            hit = box.ray_distance(origin_x, origin_y, dx, dy, best)
            if hit is not None and hit < best:
                best = hit
        return best

    def build_occupancy_grid(
        self,
        resolution: float = 0.05,
        padding: float = 0.20,
    ) -> OccupancyGridSpec:
        """把连续几何栅格化成占据栅格（真值地图）。

        用途：生成"标准答案"地图，可以与 SLAM 建出的地图做对比评估。
        方法是最朴素的中心点采样：把包围盒（外扩 padding 米防止边缘
        被截断）划分成 resolution 大小的方格，对每个格子的**中心点**
        做一次 is_occupied 查询，占据记 100、空闲记 0
        （OccupancyGrid 规范：0~100 表示占据概率，-1 表示未知）。
        """
        origin_x = self.min_x - padding
        origin_y = self.min_y - padding
        # ceil 保证栅格完全覆盖（宁多勿少），max(1,...) 防御退化成 0 格
        width = max(1, int(math.ceil((self.max_x - self.min_x + padding * 2.0) / resolution)))
        height = max(1, int(math.ceil((self.max_y - self.min_y + padding * 2.0) / resolution)))
        data: list[int] = []
        # 外层 row（y 方向）、内层 col（x 方向）的顺序与
        # OccupancyGrid 的行优先存储约定一致
        for row in range(height):
            # +0.5 取格子中心而非左下角，采样更公平
            y = origin_y + (row + 0.5) * resolution
            for col in range(width):
                x = origin_x + (col + 0.5) * resolution
                data.append(100 if self.is_occupied(x, y) else 0)
        return OccupancyGridSpec(
            width=width,
            height=height,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
            data=tuple(data),
        )


def _parse_pose(pose_text: str | None) -> tuple[float, float]:
    """从 SDF 的 <pose> 文本中解析出 x、y 坐标。

    SDF 的 pose 格式是 "x y z roll pitch yaw" 六个数字，
    2D 仿真只需要前两个；文本缺失或格式不对时退回原点 (0, 0)，
    让个别写得不规范的 link 不至于让整个加载失败。
    """
    if not pose_text:
        return 0.0, 0.0
    parts = pose_text.split()
    if len(parts) < 2:
        return 0.0, 0.0
    return float(parts[0]), float(parts[1])


def _parse_box_size(link_element: ET.Element) -> tuple[float, float, float] | None:
    """从 SDF link 元素中提取 box 尺寸 (宽, 深, 高)；不是 box 则返回 None。

    优先读 collision（物理碰撞体，仿真应以它为准），没有再退回
    visual（显示模型）——有些世界文件只写了 visual。
    返回 None 表示该 link 不含 box 几何（例如球体、网格模型），
    调用方会直接跳过它。
    """
    size_text = link_element.findtext("collision/geometry/box/size")
    if not size_text:
        size_text = link_element.findtext("visual/geometry/box/size")
    if not size_text:
        return None
    parts = size_text.split()
    if len(parts) < 2:
        return None
    width = float(parts[0])
    depth = float(parts[1])
    # SDF 的 size 是 "x y z" 三个数；z（高度）缺省按 0 处理
    height = float(parts[2]) if len(parts) > 2 else 0.0
    return width, depth, height


def load_world_geometry(world_path: str | Path) -> WorldGeometry:
    """解析 Gazebo SDF 世界文件，构建 2D WorldGeometry。

    这是本模块的入口函数。SDF 的层级结构是：
        <sdf><world name="..."><model name="..."><link name="...">
            <pose>x y z r p y</pose>
            <collision><geometry><box><size>x y z</size>...
    我们遍历每个 model 的每个 link，凡是带 box 几何的都提取成一个
    AxisAlignedBox。注意本项目的世界文件把 pose 写在 link 上
    （model 的 pose 为原点），所以只读 link 的 pose 即可。
    """
    path = Path(world_path)
    root = ET.parse(path).getroot()
    # 世界名用于日志展示；文件里没写就用文件名兜底
    world_name = root.find("world").get("name", path.stem)

    boxes: list[AxisAlignedBox] = []
    for model in root.findall("world/model"):
        model_name = model.get("name", "model")
        for link in model.findall("link"):
            size = _parse_box_size(link)
            if size is None:
                # 非 box 几何（或没有几何）的 link 与 2D 仿真无关，跳过
                continue
            width, depth, height = size
            pose_x, pose_y = _parse_pose(link.findtext("pose"))
            link_name = link.get("name", "link")
            boxes.append(
                AxisAlignedBox(
                    # 名字拼成 "模型名/link名"，调试时能一眼定位到 SDF 里的元素
                    name=f"{model_name}/{link_name}",
                    center_x=pose_x,
                    center_y=pose_y,
                    width=width,
                    depth=depth,
                    height=height,
                )
            )

    if not boxes:
        # 一个障碍都解析不出来通常意味着文件路径错误或格式不符，
        # 与其让仿真器在"空无一物"的世界里静默运行，不如尽早报错
        raise ValueError(f"No box geometry found in {path}")

    # 计算所有盒子的联合包围盒，供占据栅格确定覆盖范围
    min_x = min(box.min_x for box in boxes)
    max_x = max(box.max_x for box in boxes)
    min_y = min(box.min_y for box in boxes)
    max_y = max(box.max_y for box in boxes)

    return WorldGeometry(
        name=world_name,
        boxes=tuple(boxes),
        min_x=min_x,
        max_x=max_x,
        min_y=min_y,
        max_y=max_y,
    )
