"""基于 nav2_simple_commander 的多点巡航示例脚本。

这是一个"极简版"巡航程序，与 mission_executor 的 patrol 模式对照着看
很有教学价值：

- mission_executor 自己写 ActionClient、自己管理异步回调链，代码复杂
  但可以随时响应新指令；
- 本脚本用 Nav2 官方封装 ``BasicNavigator``：goToPose 发目标、
  isTaskComplete 轮询、getResult 拿结果，把 action 的异步细节全部藏起来，
  代码是简单的顺序流程（发目标 -> 等结果 -> 下一个），代价是执行期间
  无法接收外部指令。写一次性演示脚本时这种同步风格往往就够用。

流程：读 YAML 配置（初始位姿、路点列表、循环次数等）-> setInitialPose
告诉 AMCL 起点 -> waitUntilNav2Active 等 Nav2 全部激活 -> 按路点顺序
循环导航 -> 结束时 lifecycleShutdown 关闭 Nav2。

与外界的交互都由 BasicNavigator 代劳：内部发布 /initialpose，调用
navigate_to_pose action，查询 lifecycle 状态等。

初学者建议重点阅读：``_wait_for_task``（同步等待 action 结果 + 超时
取消的写法）和 ``main`` 里的整体流程。
"""

from __future__ import annotations

import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from .nav_utils import initial_pose_from_dict, load_yaml, pose_stamped_from_dict


def _default_waypoints_file() -> str:
    """返回包内自带的默认路点配置文件路径。

    get_package_share_directory 会定位到已安装包的 share 目录
    （install/mecamind_tools/share/mecamind_tools），这是 ROS 2 中
    查找包内资源文件的标准做法，比写死绝对路径可移植得多。
    """
    return (
        get_package_share_directory("mecamind_tools")
        + "/config/mecamind_patrol_waypoints.yaml"
    )


def _wait_for_task(navigator: BasicNavigator, timeout_sec: float, label: str) -> bool:
    """同步等待当前导航任务结束，带超时保护；返回是否成功到点。

    BasicNavigator 把 action 的异步性封装成了轮询接口：
    - isTaskComplete()：goal 是否已经结束（成功/失败/取消都算结束）；
    - cancelTask()：请求取消当前 goal；
    - getResult()：结束后取最终结果（TaskResult 枚举）。

    超时后主动 cancelTask，防止机器人被困时脚本永远卡在这个路点。
    """
    start = time.monotonic()
    while not navigator.isTaskComplete():
        if timeout_sec > 0 and time.monotonic() - start > timeout_sec:
            navigator.get_logger().warn(f"{label} timed out after {timeout_sec:.1f}s; canceling")
            navigator.cancelTask()
            return False
        # 0.2s 轮询一次即可；BasicNavigator 内部有自己的 spin 线程，
        # 这里 sleep 不会阻塞消息处理。
        time.sleep(0.2)
    # 任务结束后区分三种结局：成功 / 被取消 / 失败（如规划不出路径）。
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info(f"{label} succeeded")
        return True
    if result == TaskResult.CANCELED:
        navigator.get_logger().warn(f"{label} canceled")
    else:
        navigator.get_logger().error(f"{label} failed with result={result}")
    return False


def main(args=None):
    """巡航主流程：加载配置 -> 设初始位姿 -> 等 Nav2 -> 循环走点 -> 收尾。"""
    rclpy.init(args=args)
    # BasicNavigator 本身就是一个 ROS 节点，创建即完成各种客户端的初始化。
    navigator = BasicNavigator()
    navigator.declare_parameter("waypoints_file", "")

    # 参数没给就落回包内默认配置（`or` 利用了空字符串为假值的特性）。
    path = str(navigator.get_parameter("waypoints_file").value) or _default_waypoints_file()
    config = load_yaml(path)
    now = navigator.get_clock().now().to_msg()

    # 先告诉 AMCL 机器人当前在哪（内部发布到 /initialpose），
    # 否则 AMCL 无法定位，waitUntilNav2Active 会一直等不到就绪。
    initial_pose = initial_pose_from_dict(config["initial_pose"], now)
    navigator.setInitialPose(initial_pose)
    navigator.get_logger().info(f"Loaded patrol waypoints from {path}")

    # 阻塞等待 Nav2 各 lifecycle 节点全部 active 且定位收敛。
    # try/except 是版本兼容：旧版 API 不接受 localizer 关键字参数
    # 会抛 TypeError，新版则需要显式指定用 amcl 做定位器。
    try:
        navigator.waitUntilNav2Active()
    except TypeError:
        navigator.waitUntilNav2Active(localizer="amcl")

    # ---- 从配置读取巡航行为参数（都有默认值，配置可省略） ----
    loop_count = int(config.get("loop_count", 1))            # 整个路线循环几圈
    timeout_sec = float(config.get("goal_timeout_sec", 120.0))  # 单点超时
    wait_sec = float(config.get("wait_between_goals_sec", 0.0))  # 到点后停留时间
    waypoints = config.get("waypoints", [])

    # 双层循环：外层控制圈数，内层按顺序访问每个路点。
    for loop_index in range(loop_count):
        navigator.get_logger().info(f"Starting patrol loop {loop_index + 1}/{loop_count}")
        for item in waypoints:
            name = item.get("name", "unnamed")
            # 时间戳要用"当下"的时间重新生成，不能复用启动时的 now。
            goal = pose_stamped_from_dict(item, navigator.get_clock().now().to_msg())
            navigator.get_logger().info(
                f"Navigating to {name}: x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}"
            )
            # goToPose 内部就是向 navigate_to_pose action 发送 goal。
            navigator.goToPose(goal)
            # 注意：单点失败不中断巡航，继续尝试下一个路点
            # （返回值被有意忽略），演示场景下这样更皮实。
            _wait_for_task(navigator, timeout_sec, f"Waypoint {name}")
            if wait_sec > 0:
                # 模拟"巡逻到点停留观察"的行为。
                time.sleep(wait_sec)

    navigator.get_logger().info("Patrol complete")
    # 演示结束顺手把 Nav2 的 lifecycle 节点关掉（deactivate + cleanup），
    # 与 lifecycle_activator 的激活过程正好互为逆操作。
    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
