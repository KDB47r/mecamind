"""三室户教学世界的"配置档案"：用纯数据描述房间布局与障碍清单。

本文件与 `world_geometry.py` 的分工要分清楚：
- `world_geometry.py` 解析 SDF 文件得到**精确的碰撞几何**（每一面墙的盒子），
  供仿真器做碰撞检测和激光射线求交；
- 本文件则是**语义层面的世界描述**——"这个世界有哪几个房间、放了哪些
  家具障碍"，数值与 Gazebo 世界文件 three_room_house.world 保持一致，
  但只用于任务简报、文档生成和测试校验，不参与物理计算。

被谁使用：
- `mission_brief_node.py` 启动时调用 ``three_room_blueprint()`` 并用
  ``validate_three_room_blueprint()`` 自检，然后向学生播报世界概况；
- `runbook.py` 用 ``summarize_blueprint()`` 生成实验手册里的世界摘要；
- `test/test_mecamind.py` 用这些函数做单元测试。

本文件不依赖 ROS，也不依赖本包其他模块，是最容易读懂的一个文件——
初学者可以从这里入手，先建立对世界布局的整体印象（坐标单位均为米，
世界原点在户型中央），再去看几何和仿真代码。
"""

from dataclasses import dataclass
from typing import List


# frozen=True 让配置数据不可变：档案一旦创建就不该被运行时修改，
# 意外改动会让"档案"与实际 SDF 世界不一致
@dataclass(frozen=True)
class RoomSpec:
    """单个房间的规格：中心位置 + 平面尺寸。

    width 是 x 方向长度、depth 是 y 方向长度（与 SDF box 的
    size 前两维同一约定）。房间信息主要用于向学生描述
    "机器人现在大概在哪个房间"，不用于碰撞。
    """

    name: str
    center_x: float
    center_y: float
    width: float
    depth: float


@dataclass(frozen=True)
class ObstacleSpec:
    """单个家具/障碍物的规格：位置 + 三维尺寸。

    与 RoomSpec 相比多了 height（高度，米）——虽然 2D 仿真用不到高度，
    但 Gazebo 世界里的家具是有高度的，记录下来保证两边描述一致。
    """

    name: str
    x: float
    y: float
    width: float
    depth: float
    height: float


@dataclass(frozen=True)
class WorldBlueprint:
    """一份完整的世界档案：外墙尺寸 + 房间列表 + 障碍列表。

    outer_width / outer_depth 是外墙围合的整体尺寸（约 11.7m x 8.85m），
    机器人所有活动都发生在这个矩形之内。
    """

    name: str
    outer_width: float
    outer_depth: float
    rooms: List[RoomSpec]
    obstacles: List[ObstacleSpec]


def three_room_blueprint() -> WorldBlueprint:
    """构造"三室户"世界的标准档案（数值与 Gazebo 世界文件一一对应）。

    布局速览（世界原点在户型中央，x 向右、y 向上）：
    - entry_hall（门厅）在左下，机器人默认出生点就在这附近；
    - living_room（客厅）在中部偏上，是面积最大的活动区域；
    - bedroom（卧室）在右下。
    四个障碍（桌子、柜子、柱子、椅子）分散在各房间，
    用来给建图/避障教学制造"绕行"场景。

    注意：改这里的数值**不会**改变仿真世界！真正的墙体来自
    .world 文件；若要调整布局，必须两边同步修改。
    """
    rooms = [
        RoomSpec("entry_hall", -4.2, -2.7, 3.6, 3.3),
        RoomSpec("living_room", 0.0, 0.6, 5.1, 3.9),
        RoomSpec("bedroom", 3.75, -2.55, 3.3, 3.0),
    ]
    obstacles = [
        ObstacleSpec("table", -3.9, 2.25, 1.0, 1.0, 0.36),
        ObstacleSpec("cabinet", 4.05, -2.7, 0.75, 1.2, 0.5),
        ObstacleSpec("pillar", 0.75, -3.75, 0.45, 0.45, 0.8),
        ObstacleSpec("chair", 4.2, 1.5, 0.5, 0.5, 0.45),
    ]
    return WorldBlueprint(
        name="three_room_house",
        outer_width=11.7,
        outer_depth=8.85,
        rooms=rooms,
        obstacles=obstacles,
    )


def summarize_blueprint(blueprint: WorldBlueprint) -> str:
    """把档案压缩成一行人类可读的摘要字符串。

    输出形如 "world=...; rooms=3 [...]; obstacles=4 [...]"，
    供任务简报节点的日志和实验手册引用——学生不用打开源码
    就能知道当前世界的构成。
    """
    room_names = ", ".join(room.name for room in blueprint.rooms)
    obstacle_names = ", ".join(obj.name for obj in blueprint.obstacles)
    return (
        f"world={blueprint.name}; "
        f"rooms={len(blueprint.rooms)} [{room_names}]; "
        f"obstacles={len(blueprint.obstacles)} [{obstacle_names}]"
    )


def validate_three_room_blueprint(blueprint: WorldBlueprint) -> bool:
    """粗粒度自检：档案是否满足"三室 + 至少三个障碍"的课程要求。

    这是一道"防呆"检查：如果有人误删了房间/障碍配置，
    mission_brief_node 会在启动时立刻报错，而不是带着残缺的
    世界描述继续上课。
    """
    return len(blueprint.rooms) >= 3 and len(blueprint.obstacles) >= 3
