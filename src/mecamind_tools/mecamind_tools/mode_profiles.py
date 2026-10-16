"""模式档案：sim（仿真）/ real（实机）两种运行模式的配置数据。

这是一个不依赖 ROS 的纯数据模块，被 mission_brief_node、runbook 等使用。
它回答一个贯穿整个课程的问题："仿真和实机到底差在哪？"
答案被浓缩成一个 ModeProfile 数据类：时钟来源（use_sim_time）、
世界名称、注意事项不同，而 **topic 名称完全相同**（两种模式共用同一个
默认 TopicMap）——这正是本项目的核心设计：算法节点面对的 topic 接口
在仿真和实机之间保持一致，代码不改一行就能从仿真切换到真机。

模块内容：
- TopicMap：全系统关键 topic 名的"一处定义"（single source of truth）；
- ModeProfile：一种模式的完整档案（时钟、世界、topic 表、注意事项）；
- build_mode_profile()：工厂函数，按 "sim"/"real" 生成对应档案；
- render_profile_summary()：把档案渲染成多行文本，用于日志打印。

初学者重点阅读 build_mode_profile：注意 sim 与 real 两个分支的差异
（use_sim_time、world_name、notes），以及相同点（topic_map）。
"""

from dataclasses import dataclass, asdict
from typing import Dict, List


@dataclass(frozen=True)
class TopicMap:
    """全系统关键 topic 名称表。

    把 topic 名集中定义在一处，而不是散落在各节点里硬编码，
    好处是改名时只改这里，且简报里能把当前约定广播给所有人。
    命名习惯说明：
    - scan/imu 带 _raw 后缀：表示是传感器原始数据，滤波后的数据
      会用不带后缀的名字；
    - cmd_vel 在 /controller 命名空间下：速度指令先进混控/安全层，
      不直接打到底盘。
    """

    scan: str = "/scan_raw"
    imu: str = "/imu/data_raw"
    odom: str = "/odom"
    cmd_vel: str = "/controller/cmd_vel"
    map_topic: str = "/map"
    tf: str = "/tf"
    tf_static: str = "/tf_static"


@dataclass(frozen=True)
class ModeProfile:
    """一种运行模式的完整档案。

    字段说明：
    - name: "sim" 或 "real"；
    - mission: 本次任务类型（mapping / navigation 等），元信息；
    - use_sim_time: 是否使用仿真时钟 /clock。这是仿真与实机最重要的
      区别之一——所有节点必须统一时钟来源，否则 TF 会因时间戳
      对不上而大面积报错；
    - expects_gazebo: 是否需要 Gazebo（本项目用自带 2D 仿真器，恒为 False）；
    - topic_map: 该模式下的 topic 名称表；
    - world_name: 世界/场景标识；
    - notes: 给人看的注意事项，会出现在任务简报里。
    """

    name: str
    mission: str
    use_sim_time: bool
    expects_gazebo: bool
    topic_map: TopicMap
    world_name: str
    notes: List[str]

    def to_dict(self) -> Dict[str, object]:
        """转成可 JSON 序列化的普通 dict（dataclass 本身不能直接 json.dumps）。

        notes 用 list() 拷贝一份，避免调用方拿到 dict 后改动列表
        反过来影响到（本应不可变的）档案对象。
        """
        return {
            "name": self.name,
            "mission": self.mission,
            "use_sim_time": self.use_sim_time,
            "expects_gazebo": self.expects_gazebo,
            "topic_map": asdict(self.topic_map),
            "world_name": self.world_name,
            "notes": list(self.notes),
        }


def build_mode_profile(mode: str, mission: str = "mapping") -> ModeProfile:
    """按模式名生成对应的 ModeProfile（工厂函数）。

    mode 先做归一化（去空白、转小写），再用白名单校验——非法值直接
    抛 ValueError 让调用方启动失败，而不是悄悄退回某个默认模式。

    注意两个分支都用 TopicMap()（同一套默认 topic 名）：仿真与实机
    的接口一致性就是在这里保证的。
    """
    normalized = mode.strip().lower()
    if normalized not in {"sim", "real"}:
        raise ValueError(f"Unsupported mode: {mode}")

    if normalized == "sim":
        return ModeProfile(
            name="sim",
            mission=mission,
            # 仿真模式：时间由仿真器的 /clock 驱动，所有节点须开 use_sim_time。
            use_sim_time=True,
            expects_gazebo=False,
            topic_map=TopicMap(),
            world_name="three_room_house.world",
            notes=[
                "Use MecaMind built-in 2D simulator (not Gazebo) with three_room_house.world.",
                "Prefer slower motion and visible obstacles.",
            ],
        )

    return ModeProfile(
        name="real",
        mission=mission,
        # 实机模式：用系统真实时钟（墙上时钟）。
        use_sim_time=False,
        expects_gazebo=False,
        topic_map=TopicMap(),
        world_name="real_robot",
        notes=[
            "Keep the same topic names as simulation.",
            "Use conservative velocity and strict safety checks.",
        ],
    )


def render_profile_summary(profile: ModeProfile) -> str:
    """把档案渲染成"key: value"多行文本，供节点启动日志打印。

    每行一个字段的格式便于人眼快速核对，也便于在日志里用 grep 检索。
    """
    lines = [
        f"mode: {profile.name}",
        f"mission: {profile.mission}",
        f"use_sim_time: {profile.use_sim_time}",
        f"expects_gazebo: {profile.expects_gazebo}",
        f"world_name: {profile.world_name}",
        f"scan_topic: {profile.topic_map.scan}",
        f"imu_topic: {profile.topic_map.imu}",
        f"odom_topic: {profile.topic_map.odom}",
        f"cmd_vel_topic: {profile.topic_map.cmd_vel}",
    ]
    lines.extend(f"note: {item}" for item in profile.notes)
    return "\n".join(lines)
