"""导航通用工具函数集合。

本模块不是 ROS 2 节点，而是被本包其他节点复用的"纯函数"工具箱：

- ``quaternion_from_yaw``：把平面上的朝向角 yaw 转成 ROS 消息里的四元数；
- ``load_yaml``：读取 YAML 配置文件（命名目标点、巡航路点等）；
- ``pose_stamped_from_dict`` / ``initial_pose_from_dict``：把 YAML 里的
  ``{x, y, yaw}`` 字典转成 Nav2 需要的 ``PoseStamped`` 消息。

在导航系统中的角色：mission_executor（任务执行器）、waypoint_patrol（巡航
示例）、lifecycle_activator（生命周期激活器）都依赖这里的函数来加载配置、
构造目标位姿。初学者建议重点阅读 ``quaternion_from_yaw``（理解 yaw 与
四元数的换算公式）和 ``pose_stamped_from_dict``（理解 PoseStamped 消息
的组成：header 的时间戳 + 坐标系名 + 位置 + 姿态）。
"""

import math
import os
from typing import Any, Dict

import yaml
from geometry_msgs.msg import PoseStamped, Quaternion


def quaternion_from_yaw(yaw: float) -> Quaternion:
    """把绕 Z 轴的旋转角 yaw（弧度）转成四元数。

    ROS 中所有姿态（orientation）都用四元数 (x, y, z, w) 表示，而不是
    直接用角度，因为四元数没有万向节死锁问题、插值也更平滑。

    对于室内地面机器人，只会绕竖直的 Z 轴旋转（即只有 yaw），此时
    通用的"轴角转四元数"公式退化为：

        q = (x=0, y=0, z=sin(yaw/2), w=cos(yaw/2))

    注意角度要除以 2，这是四元数定义决定的（四元数覆盖角度的一半）。
    反过来，从四元数恢复 yaw 的公式是 yaw = 2 * atan2(z, w)。
    """
    q = Quaternion()
    # Quaternion 消息的 x、y 字段默认就是 0.0（绕 Z 轴旋转时不涉及），
    # 所以这里只需要填 z 和 w 两个分量。
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def load_yaml(path: str) -> Dict[str, Any]:
    """读取一个 YAML 文件并返回顶层字典。

    为什么单独封装：本包多个节点都要加载 YAML 配置（目标点、路点），
    统一在这里处理"~ 展开成家目录"、"空文件当作空字典"、"顶层必须是
    映射（mapping）"这三件事，避免每个节点各写一份且行为不一致。
    """
    # 支持用户在参数里写 "~/maps/goals.yaml" 这类以 ~ 开头的路径。
    expanded = os.path.expanduser(path)
    with open(expanded, "r", encoding="utf-8") as stream:
        # safe_load 只解析普通数据类型，不会执行任意 Python 对象构造，
        # 比 load 更安全；文件为空时返回 None，这里用 `or {}` 兜底。
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        # 顶层如果是列表或标量，后续 data.get(...) 会直接崩，
        # 提前抛出带文件路径的错误方便定位配置问题。
        raise ValueError(f"YAML root must be a mapping: {path}")
    return data


def pose_stamped_from_dict(item: Dict[str, Any], stamp) -> PoseStamped:
    """把 YAML 中的一条目标点字典转成 ``PoseStamped`` 消息。

    输入字典形如 ``{"x": 1.0, "y": 2.0, "yaw": 1.57}``，可选键：
    ``frame_id``（默认 "map"）、``z``（默认 0.0）。

    PoseStamped = Pose（位置 + 姿态）+ Header（时间戳 + 参考坐标系）。
    "Stamped"（带戳）在 ROS 里非常重要：同一个坐标 (1, 2) 在 map 坐标系
    和 odom 坐标系下是完全不同的物理位置，所以必须声明 frame_id；
    时间戳则让 TF 系统知道该用哪个时刻的坐标变换。

    参数 stamp 由调用方传入（通常是 ``node.get_clock().now().to_msg()``），
    这样仿真中使用 sim time（/clock 话题的仿真时间）时也能拿到正确时间。
    """
    pose = PoseStamped()
    pose.header.stamp = stamp
    # 导航目标一般写在 map（全局地图）坐标系下，所以默认 "map"。
    pose.header.frame_id = str(item.get("frame_id", "map"))
    # x、y 是必填项，缺了就直接抛 KeyError，让配置错误尽早暴露。
    pose.pose.position.x = float(item["x"])
    pose.pose.position.y = float(item["y"])
    # 地面机器人 z 恒为 0，留可选项是为了字典结构完整。
    pose.pose.position.z = float(item.get("z", 0.0))
    # YAML 里用人类友好的 yaw 角（弧度）描述朝向，这里统一转成四元数。
    pose.pose.orientation = quaternion_from_yaw(float(item.get("yaw", 0.0)))
    return pose


def initial_pose_from_dict(item: Dict[str, Any], stamp) -> PoseStamped:
    """把 YAML 字典转成"初始位姿"的 ``PoseStamped``。

    实现与 ``pose_stamped_from_dict`` 相同，单独保留一个函数名是为了
    让调用处语义更清晰：一个用于"导航目标点"，一个用于"告诉 AMCL
    机器人现在在哪"（例如 waypoint_patrol 里的 setInitialPose）。
    """
    pose = PoseStamped()
    pose.header.stamp = stamp
    pose.header.frame_id = str(item.get("frame_id", "map"))
    pose.pose.position.x = float(item["x"])
    pose.pose.position.y = float(item["y"])
    pose.pose.position.z = float(item.get("z", 0.0))
    pose.pose.orientation = quaternion_from_yaw(float(item.get("yaw", 0.0)))
    return pose
