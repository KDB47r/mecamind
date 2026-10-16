"""Nav2 生命周期激活器：按顺序把各 lifecycle 节点带到 active 状态。

【什么是 lifecycle node（生命周期节点）】普通 ROS 2 节点一启动就开始
工作；而 lifecycle 节点是"受管节点"，有一套标准状态机：

    unconfigured --configure--> inactive --activate--> active
    （未配置）                 （已配置未激活）        （工作中）

好处是启动顺序可控：Nav2 由多个节点组成（AMCL 定位、planner 规划、
controller 控制……），必须等地图、TF 等依赖就绪后按序激活，否则会
互相报错。每个 lifecycle 节点自动提供两个服务：
``/<节点名>/get_state``（查询当前状态）和 ``/<节点名>/change_state``
（请求状态迁移）。本节点就是逐个调用这两个服务的"点火器"。

Nav2 官方自带 lifecycle_manager 做同样的事；本项目自己实现一个简化版，
是为了让初学者看清 lifecycle 机制的每一步。

本节点是"一次性"程序：不常驻 spin，做完激活就退出（进程返回码
0=成功，1=失败）。额外职责：AMCL 激活后向 ``/initialpose`` topic 发布
初始位姿——AMCL 是粒子滤波定位，必须有人告诉它机器人大概在地图哪里，
它才能开始输出 map->odom 的 TF 变换。

初学者建议重点阅读：
1. ``configure_and_activate``：状态机迁移的核心逻辑（先查状态再决定
   需要哪些 transition）；
2. ``publish_initial_pose``：初始位姿消息的构造（协方差的含义）；
3. ``get_state`` / ``change_state``：ROS 2 service 客户端的标准写法
   （wait_for_service -> call_async -> spin_until_future_complete）。
"""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from .nav_utils import quaternion_from_yaw


class LifecycleActivator(Node):
    """把参数列表中的 lifecycle 节点依次 configure + activate 的工具节点。"""

    def __init__(self) -> None:
        super().__init__("mecamind_lifecycle_activator")
        # 要激活的 Nav2 节点清单。顺序有讲究：amcl 先激活（提供定位），
        # bt_navigator 等依赖 TF 的节点排在后面。
        self.declare_parameter(
            "node_names",
            [
                "amcl",
                "controller_server",
                "smoother_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
                "waypoint_follower",
                "velocity_smoother",
            ],
        )
        # 等待每个 service 上线及响应的超时时间。
        self.declare_parameter("service_timeout_sec", 20.0)
        # 整体激活前先睡一会儿，给 Nav2 各进程留出启动时间。
        self.declare_parameter("startup_delay_sec", 1.0)
        # ---- 初始位姿相关参数（发给 AMCL） ----
        self.declare_parameter("publish_initial_pose", True)
        self.declare_parameter("initial_pose_frame_id", "map")
        self.declare_parameter("initial_pose_x", -3.2)
        self.declare_parameter("initial_pose_y", -2.4)
        self.declare_parameter("initial_pose_yaw", 0.0)
        # 重复发布多次 + 间隔时间：单发一次可能在 AMCL 尚未完全就绪时被错过。
        self.declare_parameter("initial_pose_publish_count", 5)
        self.declare_parameter("initial_pose_period_sec", 0.5)
        # 发完初始位姿后再等一段，让 AMCL 建立 map->odom 变换。
        self.declare_parameter("post_initial_pose_delay_sec", 2.0)

        # QoS（服务质量）配置：TRANSIENT_LOCAL（暂态本地）意味着发布器
        # 会为后来的订阅者保留最后一条消息——即使 AMCL 在我们发布之后
        # 才完成订阅，也能补收到初始位姿。默认的 VOLATILE 则"错过就没了"。
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", qos)

    def activate_all(self) -> bool:
        """主流程：按清单顺序激活所有节点，返回是否全部成功。

        注意 ``ok = node_ok and ok`` 的写法：某个节点失败后不会提前退出，
        而是继续尝试激活余下节点——这样一次运行就能看到所有故障点，
        而不是修一个跑一次。
        """
        delay = float(self.get_parameter("startup_delay_sec").value)
        if delay > 0:
            self.get_logger().info(f"Waiting {delay:.1f}s before lifecycle activation")
            time.sleep(delay)
        node_names = [str(name) for name in self.get_parameter("node_names").value]
        ok = True
        for node_name in node_names:
            node_ok = self.configure_and_activate(node_name)
            ok = node_ok and ok
            # AMCL 刚激活就立刻喂初始位姿——它不定位，后面依赖 TF 的
            # 节点（bt_navigator 等）激活后也无法正常工作。
            if node_ok and node_name == "amcl" and bool(self.get_parameter("publish_initial_pose").value):
                self.publish_initial_pose()
        if ok:
            self.get_logger().info("All requested lifecycle nodes are active")
        else:
            self.get_logger().error("One or more lifecycle nodes failed to activate")
        return ok

    def publish_initial_pose(self) -> None:
        """向 /initialpose 重复发布初始位姿，告诉 AMCL 机器人在哪。

        消息类型是 PoseWithCovarianceStamped：位姿 + 协方差（不确定度）。
        协方差矩阵是 6x6 按行展开的 36 个数，对应 (x, y, z, roll, pitch, yaw)：
        - 下标 0  = x 方向方差（0.25 即标准差 0.5 米）；
        - 下标 7  = y 方向方差（第 2 行第 2 列，7 = 1*6+1）；
        - 下标 35 = yaw 方差（第 6 行第 6 列，0.0685 约等于 15° 的平方）。
        给一点不确定度而不是 0，AMCL 的粒子才会撒开、通过激光匹配收敛
        到真实位置，而不是死板地相信给定坐标。
        """
        count = int(self.get_parameter("initial_pose_publish_count").value)
        period = float(self.get_parameter("initial_pose_period_sec").value)
        x = float(self.get_parameter("initial_pose_x").value)
        y = float(self.get_parameter("initial_pose_y").value)
        yaw = float(self.get_parameter("initial_pose_yaw").value)
        frame_id = str(self.get_parameter("initial_pose_frame_id").value)

        for index in range(count):
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = frame_id
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            # yaw 角转四元数（见 nav_utils.quaternion_from_yaw 的公式说明）。
            msg.pose.pose.orientation = quaternion_from_yaw(yaw)
            msg.pose.covariance[0] = 0.25
            msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = 0.0685
            self.initial_pose_pub.publish(msg)
            self.get_logger().info(
                f"Published initial pose {index + 1}/{count}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
            )
            time.sleep(period)

        # AMCL 收到初始位姿后需要处理几帧激光才会开始发布 map->odom TF，
        # 这里额外等待，避免下游节点激活时 TF 还不存在而报错。
        delay = float(self.get_parameter("post_initial_pose_delay_sec").value)
        if delay > 0:
            self.get_logger().info(f"Waiting {delay:.1f}s for AMCL map->odom transform")
            time.sleep(delay)

    def configure_and_activate(self, node_name: str) -> bool:
        """把单个 lifecycle 节点推进到 active 状态。

        核心思路是"先查当前状态，再补齐缺少的迁移"，而不是盲目地
        configure + activate 各来一遍——节点可能已经被别人激活过：
        - 已经 active：什么都不用做；
        - unconfigured：先 CONFIGURE（加载参数、建立发布订阅），
          成功后重新查询状态再走下一步；
        - inactive：只差最后一步 ACTIVATE（真正开始工作）；
        - 其他状态（如迁移中、finalized）：不在预期内，报错返回。
        """
        state = self.get_state(node_name)
        if state is None:
            # 连状态都查不到（service 超时），说明节点没起来。
            return False
        if state == State.PRIMARY_STATE_ACTIVE:
            self.get_logger().info(f"{node_name} already active")
            return True
        if state == State.PRIMARY_STATE_UNCONFIGURED:
            if not self.change_state(node_name, Transition.TRANSITION_CONFIGURE):
                return False
            # configure 完成后重新查询，确认确实进入了 inactive。
            state = self.get_state(node_name)
        if state == State.PRIMARY_STATE_INACTIVE:
            return self.change_state(node_name, Transition.TRANSITION_ACTIVATE)
        self.get_logger().error(f"{node_name} is in unsupported state id={state}")
        return False

    def get_state(self, node_name: str):
        """调用 /<node>/get_state service 查询节点当前生命周期状态。

        ROS 2 service 客户端的标准三步：wait_for_service 等服务端上线、
        call_async 发出异步请求、spin_until_future_complete 阻塞等响应。
        返回状态 id（int），失败返回 None。
        """
        client = self.create_client(GetState, f"/{node_name}/get_state")
        timeout = float(self.get_parameter("service_timeout_sec").value)
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f"Timed out waiting for /{node_name}/get_state")
            return None
        future = client.call_async(GetState.Request())
        # 本节点没有常驻 spin，需要手动 spin 直到这个 future 完成。
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            self.get_logger().error(f"Failed to get lifecycle state for {node_name}")
            return None
        state = future.result().current_state.id
        # 同时打印人类可读的 label（如 "inactive"）和数字 id，方便对照。
        self.get_logger().info(f"{node_name} state={future.result().current_state.label} [{state}]")
        return state

    def change_state(self, node_name: str, transition_id: int) -> bool:
        """调用 /<node>/change_state service 请求一次状态迁移。

        transition_id 用 lifecycle_msgs 里的常量（如 TRANSITION_CONFIGURE、
        TRANSITION_ACTIVATE）。目标节点会执行对应的回调（on_configure /
        on_activate），返回的 success 表示迁移是否被接受并成功完成。
        """
        client = self.create_client(ChangeState, f"/{node_name}/change_state")
        timeout = float(self.get_parameter("service_timeout_sec").value)
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f"Timed out waiting for /{node_name}/change_state")
            return False
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        result = future.result()
        if result is not None and result.success:
            self.get_logger().info(f"{node_name} transition {transition_id} succeeded")
            return True
        self.get_logger().error(f"{node_name} transition {transition_id} failed")
        return False


def main(args=None) -> None:
    """入口：执行一轮激活后立即退出，用进程返回码告知结果。

    SystemExit(0/1) 让 launch 文件或 shell 脚本可以据此判断 Nav2
    是否成功拉起（0=全部激活成功，1=有失败）。
    """
    rclpy.init(args=args)
    node = LifecycleActivator()
    try:
        ok = node.activate_all()
        raise SystemExit(0 if ok else 1)
    finally:
        node.destroy_node()
        # activate_all 抛异常时 rclpy 可能已被信号处理关闭，先判断再 shutdown。
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
