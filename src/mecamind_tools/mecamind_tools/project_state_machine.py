"""项目级状态机：管理整个机器人系统"当前处于哪个工作阶段"。

这是一个不依赖 ROS 的纯 Python 状态机（方便单元测试和课堂讲解），
描述的不是某个节点的内部状态，而是整个项目的宏观运行模式：
启动（BOOT）-> 仿真/实机建图 -> 仿真/实机导航 -> 演示完成（DEMO），
外加两个"异常态"：故障（FAULT）和急停（ESTOP）。

状态图（正常流）：
    BOOT --START_SIM--> SIM_MAPPING --BEGIN_NAVIGATION--> SIM_NAVIGATION
    BOOT --START_REAL--> REAL_MAPPING --BEGIN_NAVIGATION--> REAL_NAVIGATION
    *_MAPPING --DEMO_COMPLETE--> DEMO
    任意状态 --STOP--> BOOT（复位）

异常流（优先级最高，注意 handle() 里的判断顺序就是优先级顺序）：
    任意状态 --ESTOP--> ESTOP（锁存，只有 RESET_ESTOP 能解除，回 BOOT）
    非急停状态 --SENSOR_FAULT--> FAULT（只有 CLEAR_FAULT 能解除，回 BOOT）

设计要点（初学者重点体会）：
1. 用枚举而不是裸字符串表示状态/事件——拼写错误在开发期就能被发现；
2. handle() 返回 TransitionResult 而不是抛异常——"事件被拒绝"是
   状态机的正常行为（例如导航中收到 START_SIM），不是程序错误；
3. 急停是"锁存"（latched）语义：进入 ESTOP 后除了显式复位，任何
   事件都推不动它，这是安全系统的标准做法。

初学者重点阅读：ProjectStateMachine.handle()，注意各 if 块的先后顺序
如何实现"急停 > 故障 > 正常流"的优先级。
"""

from dataclasses import dataclass
from enum import Enum


class ProjectMode(str, Enum):
    """系统宏观工作模式。

    继承 str 让枚举值可以直接当字符串用（如塞进 JSON、写日志），
    不用到处写 .value。
    """

    BOOT = "boot"                        # 刚启动/已复位，等待选择仿真或实机
    SIM_MAPPING = "sim_mapping"          # 仿真环境建图
    SIM_NAVIGATION = "sim_navigation"    # 仿真环境导航
    REAL_MAPPING = "real_mapping"        # 真实机器人建图
    REAL_NAVIGATION = "real_navigation"  # 真实机器人导航
    DEMO = "demo"                        # 演示任务已完成
    FAULT = "fault"                      # 传感器故障，需人工排查后清除
    ESTOP = "estop"                      # 急停锁存，需显式复位


class ProjectEvent(str, Enum):
    """驱动状态迁移的事件（来自用户指令、监控节点或安全系统）。"""

    START_SIM = "start_sim"              # 启动仿真流程
    START_REAL = "start_real"            # 启动实机流程
    BEGIN_MAPPING = "begin_mapping"      # 开始建图（预留事件）
    BEGIN_NAVIGATION = "begin_navigation"  # 建图完成，切换到导航
    DEMO_COMPLETE = "demo_complete"      # 演示任务完成
    SENSOR_FAULT = "sensor_fault"        # 检测到传感器故障
    CLEAR_FAULT = "clear_fault"          # 故障已排除
    ESTOP = "estop"                      # 触发急停
    RESET_ESTOP = "reset_estop"          # 复位急停
    STOP = "stop"                        # 停止当前任务并回到 BOOT


@dataclass
class TransitionResult:
    """一次事件处理的完整结果，供调用方记日志/发 topic/做决策。

    accepted=False 表示事件在当前状态下被拒绝（状态未变化），
    reason 用一句话解释接受或拒绝的原因——把"为什么"随结果一起
    返回，调试时不用再翻状态机源码。
    """

    previous: ProjectMode
    current: ProjectMode
    accepted: bool
    reason: str


class ProjectStateMachine:
    """项目状态机本体：保存当前模式，逐个处理事件。

    实现风格是"平铺的 if 链"而不是查表（迁移字典）——对于状态数不多
    的教学项目，if 链能让"判断顺序 = 安全优先级"这一点一目了然。
    """

    def __init__(self) -> None:
        # 初始状态固定为 BOOT：系统上电后必须先经过检查/选择阶段。
        self.mode = ProjectMode.BOOT

    def handle(self, event: ProjectEvent) -> TransitionResult:
        """处理一个事件，返回迁移结果（可能是"拒绝"）。

        下面各代码块的先后顺序就是优先级，从高到低依次为：
        急停触发 > 急停锁存检查 > 故障触发 > 故障锁存检查 >
        正常业务迁移 > 通用 STOP 复位 > 默认拒绝。
        """
        previous = self.mode

        # 优先级 1：急停事件无条件生效——不管现在处于什么状态，
        # 安全永远是第一位，这个判断必须放在所有逻辑之前。
        if event == ProjectEvent.ESTOP:
            self.mode = ProjectMode.ESTOP
            return TransitionResult(previous, self.mode, True, "estop latched")

        # 优先级 2：处于急停锁存中——除 RESET_ESTOP 外一律拒绝。
        # 复位后回 BOOT 而不是回急停前的状态：强制重新走一遍启动检查，
        # 因为触发急停的原因可能还没排除。
        if self.mode == ProjectMode.ESTOP:
            if event == ProjectEvent.RESET_ESTOP:
                self.mode = ProjectMode.BOOT
                return TransitionResult(previous, self.mode, True, "estop reset")
            return TransitionResult(previous, self.mode, False, "estop latched")

        # 优先级 3：传感器故障在任何非急停状态下都立即生效。
        if event == ProjectEvent.SENSOR_FAULT:
            self.mode = ProjectMode.FAULT
            return TransitionResult(previous, self.mode, True, "sensor fault")

        # 优先级 4：故障锁存——必须显式 CLEAR_FAULT 才能离开，同样回 BOOT。
        if self.mode == ProjectMode.FAULT:
            if event == ProjectEvent.CLEAR_FAULT:
                self.mode = ProjectMode.BOOT
                return TransitionResult(previous, self.mode, True, "fault cleared")
            return TransitionResult(previous, self.mode, False, "fault requires clear")

        # ---- 以下是正常业务流 ----

        # BOOT 状态：选择进入仿真还是实机流程，二者都从建图阶段开始。
        if self.mode == ProjectMode.BOOT:
            if event == ProjectEvent.START_SIM:
                self.mode = ProjectMode.SIM_MAPPING
                return TransitionResult(previous, self.mode, True, "simulation start")
            if event == ProjectEvent.START_REAL:
                self.mode = ProjectMode.REAL_MAPPING
                return TransitionResult(previous, self.mode, True, "real robot start")

        # 建图状态（仿真或实机）：可以转入对应的导航状态，或宣告演示完成。
        # 仿真建图只能进仿真导航、实机建图只能进实机导航——用条件表达式
        # 保持 sim/real 两条轨道互不串线。
        if self.mode in {ProjectMode.SIM_MAPPING, ProjectMode.REAL_MAPPING}:
            if event == ProjectEvent.BEGIN_NAVIGATION:
                self.mode = (
                    ProjectMode.SIM_NAVIGATION
                    if self.mode == ProjectMode.SIM_MAPPING
                    else ProjectMode.REAL_NAVIGATION
                )
                return TransitionResult(previous, self.mode, True, "switch to navigation")
            if event == ProjectEvent.DEMO_COMPLETE:
                self.mode = ProjectMode.DEMO
                return TransitionResult(previous, self.mode, True, "mission done")

        # STOP 是通用复位：在任何正常业务状态下都能把系统拉回 BOOT。
        # 注意它排在急停/故障检查之后——锁存状态下 STOP 也会被拒绝。
        if event == ProjectEvent.STOP:
            self.mode = ProjectMode.BOOT
            return TransitionResult(previous, self.mode, True, "stop and reset")

        # 默认分支：事件在当前状态下没有意义，保持原状态并明确拒绝。
        # "白名单式"迁移（没写的都拒绝）比"黑名单式"更安全。
        return TransitionResult(previous, self.mode, False, "event refused in current mode")
