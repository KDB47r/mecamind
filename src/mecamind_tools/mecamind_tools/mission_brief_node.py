"""任务简报节点：周期性广播"本次任务的配置概要 + 开机检查清单"。

在系统中的角色：一个纯信息类节点，不控制任何硬件。它把三份静态信息
组装成一份 JSON"任务简报"，每 2 秒发布一次：
1. 模式档案（mode_profiles.build_mode_profile）：sim/real 模式下
   应该用哪些 topic、是否用仿真时间等；
2. 世界蓝图（world_profiles.three_room_blueprint）：三房间教学世界
   的房间/路点布局；
3. 开机检查清单（runbook.build_boot_checklist）：一步步验证系统
   是否就绪的 ros2 命令列表。

- 发布 /mecamind/mission_brief（JSON String），2 秒一次。
  用"周期发布"而不是"只发一次"，是因为 ROS 里订阅者可能比发布者
  晚启动，周期重发保证晚来的节点（如 Web 面板、教学 UI）也能拿到简报。

对初学者的价值：这是理解"配置即数据"思想的入口——把课程里所有
需要口头交代的约定（用哪个 topic、开机先查什么）沉淀成机器可读、
可通过 topic 自描述的数据。重点看 __init__ 里参数如何驱动
build_mode_profile，以及 _publish_brief 的组装流程。
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .mode_profiles import build_mode_profile, render_profile_summary
from .runbook import build_mission_brief_payload
from .world_profiles import three_room_blueprint, validate_three_room_blueprint


class MissionBriefNode(Node):
    """任务简报节点：启动时校验配置，之后定时发布 JSON 简报。

    参数：
    - mode: "sim" 或 "real"，决定加载哪套模式档案；
    - mission: 任务类型（如 mapping / navigation），仅作为元信息透传；
    - world_path: 世界文件路径，透传给简报供下游定位资源；
    - output_topic: 简报发布的 topic 名。
    """

    def __init__(self) -> None:
        super().__init__("mecamind_mission_brief")
        self.declare_parameter("mode", "sim")
        self.declare_parameter("mission", "mapping")
        self.declare_parameter("world_path", "")
        self.declare_parameter("output_topic", "/mecamind/mission_brief")
        mode = self.get_parameter("mode").value
        mission = self.get_parameter("mission").value
        world_path = self.get_parameter("world_path").value
        output_topic = self.get_parameter("output_topic").value

        # 配置在构造阶段就组装并校验：mode 非法或蓝图不自洽时节点直接
        # 启动失败——"快速失败"比带着坏配置运行到一半才出错好排查得多。
        self.profile = build_mode_profile(mode, mission=mission)
        self.blueprint = three_room_blueprint()
        if not validate_three_room_blueprint(self.blueprint):
            raise ValueError("three_room_blueprint is not valid")
        self.world_path = world_path

        self.publisher = self.create_publisher(String, output_topic, 10)
        # 2 秒周期重发，保证晚启动的订阅者也能收到简报。
        self.timer = self.create_timer(2.0, self._publish_brief)
        # 启动时把档案摘要打进日志，终端里就能直接核对本次运行配置。
        self.get_logger().info(render_profile_summary(self.profile))

    def _publish_brief(self) -> None:
        """定时器回调：组装完整简报 payload 并以 JSON 发布。

        每次都重新组装（而不是缓存字符串）——数据量很小，重算的开销
        可忽略，而且未来若有字段变成动态的也不需要改这里。
        """
        payload = build_mission_brief_payload(
            self.profile,
            self.blueprint,
            self.world_path,
        )
        self.publisher.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main(args=None) -> None:
    """节点入口。

    比其他节点多了对 KeyboardInterrupt 的显式处理：Ctrl+C 可能在 spin
    期间、也可能在清理期间到来，两处都捕获才能保证退出时不打印
    吓人的堆栈；rclpy.try_shutdown() 则容忍"上下文已经关闭"的情况
    （普通 shutdown 在重复调用时会抛异常）。
    """
    rclpy.init(args=args)
    node = MissionBriefNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        finally:
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
