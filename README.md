# MecaMind —— ROS 2 麦克纳姆轮小车（仿真 · 语音任务导航）

基于 **ROS 2 Jazzy + Gazebo Harmonic** 的麦克纳姆轮全向小车仿真项目，支持半自动建图、Nav2 自主导航、YOLO 目标检测、视觉跟随，以及"唤醒词 + 流式 ASR + 大模型解析 + TTS 播报"的语音任务导航。

## 已有功能

| 功能 | 说明 |
| --- | --- |
| 半自动建图 | 小车按预设路点自动行驶，slam_toolbox 实时建图，生成三房间栅格地图 |
| 自主导航 | Nav2 全栈（AMCL 定位 + 全局规划 + DWB 局部控制 + 恢复行为），支持命名目标：客厅 / 卧室 / 厨房 / 大厅入口 |
| 语音任务导航 | 说"小度小度 去卧室"即可自动导航；云端链路：通义千问（任务解析）+ paraformer（ASR）+ cosyvoice（TTS） |
| 目标检测 | YOLOv8n 多线程推理，检测目标并在图像上可视化 |
| 视觉跟随 | 锁定检测目标（默认 person），按检测框偏移控制小车跟随 |
| 语音播报 | 收到任务、执行状态、确认提示均有语音回复（TTS 合成后播放） |
| 安全机制 | 速度仲裁器（急停 > 遥控 > 跟随 > 导航）+ 激光安全门 + 速度平滑限幅 |
| 任务闭环 | 含糊指令先确认再执行；停止/取消/确认/方向点动走本地规则，零延迟、不依赖网络 |

## 环境要求

- Ubuntu 24.04（WSL2 亦可）+ ROS 2 Jazzy + Gazebo Harmonic
- 依赖安装：`./setup/install_dependencies.sh`；环境自检：`./setup/check_environment.sh`

## 构建

```bash
cd ~/mecamind_ros2
colcon build
source install/setup.bash
```

## 语音功能配置（阿里云 DashScope）

1. 安装 SDK：`pip install --user --break-system-packages -U dashscope`
2. 配置密钥（二选一）：
   - 环境变量：`export DASHSCOPE_API_KEY=sk-...`
   - 或复制模板填密钥：`cp src/mecamind_tools/config/mecamind_aliyun.example.yaml src/mecamind_tools/config/mecamind_aliyun.local.yaml`
   
   `*.local.yaml` 已被 .gitignore 忽略，**不要把真实密钥写进代码或提交到仓库**。
3. 默认模型：ASR `paraformer-realtime-v2`、LLM `qwen-plus`（非思考模式）、TTS `cosyvoice-v3-flash`（音色 longanyang），均可用 launch 参数覆盖。

## 各功能运行方式

### 1. 仿真环境

```bash
ros2 launch mecamind_bringup gazebo.launch.py
```

启动三房间世界 + 麦克纳姆轮小车。

### 2. 半自动建图

```bash
ros2 launch mecamind_bringup mapping.launch.py
```

小车按预设路点自动行驶建图，地图保存后供导航使用。

### 3. 自主导航

```bash
ros2 launch mecamind_bringup navigation.launch.py
```

默认 `backend:=gazebo`，仿真 + RViz + Nav2 一起启动，加载打包地图（三房间）。RViz 里用 2D Goal Pose 点目标即可导航。

### 4. 语音任务导航

```bash
# 终端 1：仿真 + 导航
ros2 launch mecamind_bringup navigation.launch.py
# 终端 2：语音链路
ros2 launch mecamind_tools mecamind_aliyun_voice.launch.py
```

对麦克风说"小度小度 去卧室"（唤醒词默认 `小度小度`，可用 `wake_words` 参数配置）。支持的指令：去某房间 / 巡逻 / 停止 / 取消 / 前进后退左转右转 / 跟随 / 建图；含糊指令会先语音询问确认。

跳过语音的文本直测：

```bash
ros2 topic pub --once /mecamind/voice_command std_msgs/msg/String "{data: '去卧室'}"
```

### 5. 目标检测与视觉跟随

```bash
ros2 launch mecamind_bringup perception.launch.py   # YOLOv8n 目标检测
ros2 launch mecamind_bringup follow.launch.py       # 视觉跟随（默认跟 person）
```

## 语音链路架构

```
麦克风 → voice_listen（能量门限 + 唤醒词）→ 流式 ASR → 指令文本
        → task_scheduler（本地规则 + 大模型解析）→ task_command（JSON）
        → mission_executor → Nav2 → 速度仲裁器 → 激光安全门 → 小车
回复：task_scheduler / mission_executor → robot_reply → TTS → 扬声器
```

- 两级唤醒：本地 RMS 能量门限（启动自动标定）→ ASR 部分结果匹配唤醒词；
- 解析策略：停止/取消/确认/点动走本地规则（零延迟），复杂指令走大模型，LLM 失败自动降级规则解析；
- 解析出的任务统一为 JSON（intent / target / requires_confirmation / reply）。

## 常用 topic

| topic | 内容 |
| --- | --- |
| `/mecamind/voice_command` | ASR 转出的指令文本（LLM 的输入） |
| `/mecamind/task_command` | 解析后的任务 JSON |
| `/mecamind/robot_reply` | 将要语音播报的话术 |
| `/mecamind/mission_state` | 任务执行状态（JSON） |
| `/cmd_vel` | 最终速度指令（经仲裁 + 安全门） |

## 目录结构

```
src/
├── mecamind_description/  机器人 URDF 描述
├── mecamind_gazebo/       仿真世界与模型（SDF）
├── mecamind_interfaces/   自定义 msg/srv 接口
├── mecamind_perception/   YOLO 检测、速度仲裁、视觉跟随
├── mecamind_tools/        建图/导航/语音全链路节点、参数配置、地图
└── mecamind_bringup/      顶层 launch（gazebo / mapping / navigation / perception / follow）
```

## 验收脚本

```bash
./scripts/test_cp1.sh   # CP1 底盘运动
./scripts/test_cp2.sh   # CP2 建图
./scripts/test_cp3.sh   # CP3 导航
./scripts/test_cp4.sh   # CP4 目标检测与跟随
./scripts/test_cp5.sh   # CP5 语音交互
```
