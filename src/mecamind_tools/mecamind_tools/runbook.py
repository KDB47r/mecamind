"""运行手册（runbook）工具：生成开机检查清单和任务简报数据。

"runbook" 是运维领域的术语，指一份"照着做就能把系统跑起来/查清问题"
的标准操作手册。本模块把这份手册从纸面文档变成代码生成的数据：
根据当前的模式档案（ModeProfile）和世界蓝图（WorldBlueprint），
自动生成一份逐项可执行的开机检查清单——每一项都对应一条真实的
ros2 诊断命令，学生在终端里逐条执行即可确认系统就绪。

不依赖 ROS 的纯数据模块，主要被 mission_brief_node 调用：
- build_boot_checklist()：生成检查项列表（结构化数据）；
- render_boot_checklist() / render_mission_summary()：渲染成给人看的文本；
- build_mission_brief_payload()：把以上内容打包成一个 dict，
  由简报节点序列化成 JSON 发布到 /mecamind/mission_brief。

对初学者的价值：build_boot_checklist 里的 7 条命令本身就是一套
标准的 ROS 2 系统排查流程（先查环境变量、再查各传感器 topic、
再查 TF 树、最后查控制通道），值得背下来。
"""

from dataclasses import dataclass
from typing import List

from .mode_profiles import ModeProfile
from .world_profiles import WorldBlueprint, summarize_blueprint


@dataclass(frozen=True)
class ChecklistItem:
    """一条检查项：名称 + 可执行的验证命令 + 是否必查。

    command 存的是完整可复制的 shell 命令，学生直接粘贴到终端执行；
    required 预留了"可选项"的表达能力（当前清单里全部为必查）。
    """

    name: str
    command: str
    required: bool = True


def build_boot_checklist(profile: ModeProfile, blueprint: WorldBlueprint) -> List[ChecklistItem]:
    """按当前模式档案生成开机检查清单。

    检查顺序有讲究——按依赖链从底向上排查：
    1. ros_domain_id：先确认 ROS_DOMAIN_ID，串域是"看不到 topic"的头号原因；
    2-4. scan/imu/odom：逐个确认传感器数据在流动（hz 看频率，echo --once
       看内容）；
    5. tf_tree：传感器有数据后，确认坐标变换树完整（建图导航都依赖 TF）；
    6. world_assets：确认世界资源就位；
    7. cmd_vel：最后确认控制通道存在（topic info 看有没有订阅者）。
    命令里的 topic 名从 profile.topic_map 取，保证清单永远与当前
    模式的实际配置一致，不会出现"手册和系统对不上"的经典事故。
    """
    return [
        ChecklistItem("ros_domain_id", "echo $ROS_DOMAIN_ID", True),
        ChecklistItem("scan_topic", f"ros2 topic hz {profile.topic_map.scan}", True),
        ChecklistItem("imu_topic", f"ros2 topic echo --once {profile.topic_map.imu}", True),
        ChecklistItem("odom_topic", f"ros2 topic echo --once {profile.topic_map.odom}", True),
        ChecklistItem("tf_tree", "ros2 run tf2_tools view_frames", True),
        ChecklistItem("world_assets", f"check world {blueprint.name}", True),
        ChecklistItem("cmd_vel", f"ros2 topic info {profile.topic_map.cmd_vel}", True),
    ]


def render_boot_checklist(items: List[ChecklistItem]) -> str:
    """把检查清单渲染成带序号的多行文本，形如：

        1. ros_domain_id [required] -> echo $ROS_DOMAIN_ID

    供打印到终端或显示在教学 UI 上，学生照着逐条执行。
    """
    lines = []
    for index, item in enumerate(items, start=1):
        suffix = "required" if item.required else "optional"
        lines.append(f"{index}. {item.name} [{suffix}] -> {item.command}")
    return "\n".join(lines)


def render_mission_summary(profile: ModeProfile, blueprint: WorldBlueprint) -> str:
    """生成一段简短的任务概要文本：世界摘要 + 模式关键参数。

    放在简报 payload 的 "summary" 字段里，让人不展开完整 JSON
    也能一眼看清"现在跑的是什么模式、什么任务"。
    """
    return "\n".join(
        [
            "MecaMind mission brief",
            summarize_blueprint(blueprint),
            f"mode={profile.name}",
            f"mission={profile.mission}",
            f"use_sim_time={profile.use_sim_time}",
        ]
    )


def build_mission_brief_payload(
    profile: ModeProfile,
    blueprint: WorldBlueprint,
    world_path: str,
) -> dict:
    """组装完整的任务简报 payload（发布到 topic 前的最后一步）。

    同一份检查清单同时以两种形式放进 payload：
    - "checklist"：结构化的 dict 列表，给程序（UI、自动检查脚本）消费；
    - "checklist_text"：渲染好的文本，给人直接阅读。
    数据和展示分离、但一起下发，订阅方按需取用。
    """
    checklist = build_boot_checklist(profile, blueprint)
    return {
        "summary": render_mission_summary(profile, blueprint),
        "world_path": world_path,
        "profile": profile.to_dict(),
        "checklist": [
            {"name": item.name, "command": item.command, "required": item.required}
            for item in checklist
        ],
        "checklist_text": render_boot_checklist(checklist),
    }
