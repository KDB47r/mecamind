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
| 语音闲聊（联网） | 天气、新闻等实时问题调用带联网搜索的大模型如实回答；闲聊只回话、不打断导航/巡逻任务 |
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

对麦克风说"小度小度 去卧室"（唤醒词默认 `小度小度`，可用 `wake_words` 参数配置）。唤醒后的 30 秒内再开口说话无需重复唤醒词（`wake_hold_sec` 参数，0 禁用），可连续多轮对话。支持的指令：去某房间 / 巡逻 / 停止 / 取消 / 前进后退左转右转 / 跟随 / 建图；含糊指令会先语音询问确认。天气、新闻等闲聊问题走联网搜索如实回答（不编造、不打断任务）。

跳过语音的文本直测：

```bash
ros2 topic pub --once /mecamind/voice_command std_msgs/msg/String "{data: '去卧室'}"
```

### 5. 目标检测与视觉跟随

```bash
ros2 launch mecamind_bringup perception.launch.py   # YOLOv8n 目标检测
ros2 launch mecamind_bringup follow.launch.py       # 视觉跟随（默认跟 person）
```

## 节点总览与数据流

### 语音链路节点（`mecamind_aliyun_voice.launch.py` 启动，mecamind_tools 包）

| 节点 | 功能 | 订阅（输入） | 发布（输出） |
| --- | --- | --- | --- |
| `mecamind_voice_listen` | 麦克风采集：能量门限 + 唤醒词匹配 + 30 秒免唤醒追问窗口 + 播报中可喊唤醒词打断（barge-in） | `/mecamind/asr_partial`（唤醒词匹配）、`/mecamind/asr_result`、`/mecamind/tts_status` | `/mecamind/audio_pcm`（base64 PCM 音频帧）、`/mecamind/audio_pcm_end`、`/mecamind/voice_command`（唤醒后的指令文本）、`/mecamind/listen_status`、`/mecamind/robot_reply`（"没听清"等提示）、`/mecamind/tts_stop`（打断播报） |
| `mecamind_aliyun_asr_stream` | 流式语音识别（阿里云 paraformer 实时 websocket，边说边出字） | `/mecamind/audio_pcm`、`/mecamind/audio_pcm_end` | `/mecamind/asr_partial`（实时部分结果）、`/mecamind/asr_result`（最终识别文本）、`/mecamind/voice_error`、`/mecamind/robot_reply` |
| `mecamind_aliyun_task_scheduler` | 指令解析：本地规则（停止/取消/确认/点动，零延迟）+ 大模型 qwen-plus（复杂指令）；闲聊走带联网搜索的二次调用 | `/mecamind/voice_command` | `/mecamind/task_command`（任务 JSON，闲聊不发）、`/mecamind/robot_reply` |
| `mecamind_aliyun_tts` | 语音合成（cosyvoice），把播报文本合成为音频文件 | `/mecamind/robot_reply` | `/mecamind/tts_audio_file`（音频文件路径）、`/mecamind/voice_error` |
| `mecamind_tts_playback` | 播放 TTS 音频；收到打断信号立即停止 | `/mecamind/tts_audio_file`、`/mecamind/tts_stop` | `/mecamind/tts_status`（playing / idle / failed / interrupted） |

按键说话模式（`listen_mode:=ptt`）用 `mecamind_microphone_recorder`（`/mecamind/record_voice` 服务 → `/mecamind/audio_file`）和 `mecamind_aliyun_asr_file`（`/mecamind/audio_file` → `/mecamind/voice_command`）替代上面前两个节点。

### 任务与导航链路节点（`navigation.launch.py` 启动）

| 节点 | 功能 | 订阅（输入） | 发布（输出） |
| --- | --- | --- | --- |
| `mecamind_mission_executor` | 任务执行器：解析 task_command JSON，调用 Nav2 导航、巡逻、开启跟随、点动、停止，并汇报状态 | `/mecamind/task_command` | `/mecamind/mission_state`、`/mecamind/robot_reply`、`/mecamind/follow_enable`（跟随开关）、`/cmd_vel_nav`（点动速度）、`/cmd_vel_teleop`（遥控通道速度） |
| `mecamind_lifecycle_activator` | 启动时按序激活 Nav2 生命周期节点，并向 AMCL 发布初始位姿 (0,0,0) | —（服务调用） | `/initialpose` |
| Nav2 全栈（map_server / amcl / planner_server / controller_server / bt_navigator） | 定位 + 全局规划 + 局部控制 + 行为树 | `/map`、`/scan`、`/odom`、`/initialpose`、NavigateToPose action | `/cmd_vel_nav`（平滑后速度） |
| `mecamind_safety_gate` | 激光安全门：速度链路最后一道关卡，用激光检测前方障碍，必要时衰减或清零速度 | `/scan`、`/cmd_vel_nav` | `/cmd_vel`（小车真正执行的速度）、`/mecamind/safety_state` |
| `mecamind_map_publisher` | 周期性发布打包好的三房间地图 | — | `/map` |

### 感知与跟随链路节点（`perception.launch.py` / `follow.launch.py` 启动）

| 节点 | 功能 | 订阅（输入） | 发布（输出） |
| --- | --- | --- | --- |
| `mecamind_detector` | YOLOv8n 目标检测（camera / 视频 / 合成图像三种源） | `/camera/image_raw`（camera 源） | `/mecamind/detections`（Detection2DArray）、`/mecamind/detection_image`（调试画面）、`/mecamind/perception_metrics` |
| `mecamind_detection_json_bridge` | 结构化检测结果转 JSON 文本 | `/mecamind/detections` | `/mecamind/raw_detections` |
| `mecamind_perception_filter` | 按目标类别、距离等过滤检测结果 | `/mecamind/raw_detections` | `/mecamind/perception_event` |
| `mecamind_vision_follow_controller` | 视觉跟随 PID：按目标框偏移控速，融合激光避障 | `/mecamind/perception_event`、`/mecamind/follow_enable`、`/scan` | `/cmd_vel_follow` |
| `mecamind_cmd_vel_arbiter` | 速度仲裁：急停 > 遥控 > 跟随 > 导航，多路速度源选一路输出 | `/mecamind/estop`、`/cmd_vel_teleop`、`/cmd_vel_follow`、`/cmd_vel_nav_in` | `/cmd_vel_nav`、`/mecamind/arbiter_state` |
| `mecamind_follow_target_mover` | 仿真专用：让被跟随目标按预设路点绕圈移动 | `/mecamind/follow_enable` | `/mecamind/follow_target_speed` |

辅助节点：`mecamind_simulator`（lite 轻量仿真：`/clock`、`/odom`、`/scan_raw`、`/imu/data_raw`）、`mecamind_mapping_route_driver`（半自动建图路点驱动）、`mecamind_frontier_explorer`（边界探索）、`mecamind_nav_acceptance`（导航验收）、`mecamind_mission_brief`（任务简报）。

### 数据流：谁把数据发给谁

**语音 → 任务 → 运动（主链路）**

```
麦克风 ──audio_pcm──► voice_listen ──audio_pcm──► asr_stream（流式 ASR）
                         ▲ asr_partial（唤醒词匹配）
                         │ asr_result（识别文本）
                         ▼
                  组装指令 ──voice_command──► task_scheduler（规则 + qwen-plus）
                                                  │
                        ┌─────────────────────────┤
             task_command（任务 JSON）     robot_reply（回复文本）
                        │                         │
                        ▼                         ▼
              mission_executor                aliyun_tts
                        │                         │ tts_audio_file
           NavigateToPose action                  ▼
                        ▼                    tts_playback ──► 扬声器
                      Nav2                        ▲
                        │ cmd_vel_nav         tts_stop（barge-in 打断，来自 voice_listen）
                        ▼
                  safety_gate ──cmd_vel──► 小车（Gazebo）
点动：mission_executor 直接发 cmd_vel_nav（不经 Nav2 规划）
闲聊：task_scheduler 判定为 chat → 带联网搜索的二次调用 → 只发 robot_reply，task_command 不发 → 不打扰任务
```

**感知 → 跟随（副链路，follow.launch.py）**

```
相机 /camera/image_raw ──► detector（YOLOv8n）──detections──► json_bridge
                                                                 │ raw_detections
                                                                 ▼
                                                         perception_filter
                                                                 │ perception_event
                                                                 ▼
                     mission_executor ──follow_enable──► vision_follow_controller（PID + /scan 避障）
                                                                 │ cmd_vel_follow
                                                                 ▼
  急停 /mecamind/estop、遥控 /cmd_vel_teleop ─► cmd_vel_arbiter（急停>遥控>跟随>导航）
                                                                 │ cmd_vel_nav
                                                                 ▼
                                                            safety_gate ──► /cmd_vel ──► 小车
```

唤醒与解析要点：

- 两级唤醒：本地 RMS 能量门限（启动自动标定）→ ASR 部分结果匹配唤醒词；
- 解析策略：停止/取消/确认/点动走本地规则（零延迟），复杂指令走大模型，LLM 失败自动降级规则解析；
- 解析出的任务统一为 JSON（intent / target / requires_confirmation / reply）。

## 常用 topic

| topic | 类型 | 内容 | 谁发布 → 谁订阅 |
| --- | --- | --- | --- |
| `/mecamind/audio_pcm` | String | base64 PCM 音频帧（含 phase 标记） | voice_listen → asr_stream |
| `/mecamind/asr_partial` | String | ASR 实时部分结果（唤醒词匹配） | asr_stream → voice_listen |
| `/mecamind/asr_result` | String | ASR 最终识别文本（JSON） | asr_stream → voice_listen |
| `/mecamind/voice_command` | String | 唤醒后的指令文本（LLM 的输入） | voice_listen → task_scheduler |
| `/mecamind/task_command` | String | 解析后的任务 JSON | task_scheduler → mission_executor |
| `/mecamind/robot_reply` | String | 将要语音播报的话术（多个节点汇聚于此） | task_scheduler / mission_executor / voice_listen → aliyun_tts |
| `/mecamind/tts_audio_file` | String | TTS 合成音频文件路径 | aliyun_tts → tts_playback |
| `/mecamind/tts_status` | String | 播报状态（playing / idle / failed / interrupted） | tts_playback → voice_listen |
| `/mecamind/tts_stop` | String | 打断播报信号（barge-in） | voice_listen → tts_playback |
| `/mecamind/mission_state` | String | 任务执行状态（JSON） | mission_executor → 调试/验收 |
| `/mecamind/follow_enable` | Bool | 跟随开关 | mission_executor → vision_follow_controller / follow_target_mover |
| `/mecamind/detections` | Detection2DArray | YOLO 检测结果 | detector → json_bridge |
| `/mecamind/perception_event` | String | 过滤后的目标事件 | perception_filter → vision_follow_controller |
| `/cmd_vel_nav` | Twist | 导航/点动速度（安全门输入） | Nav2 / mission_executor / arbiter → safety_gate |
| `/cmd_vel` | Twist | 最终速度指令（经仲裁 + 安全门） | safety_gate → 小车 |

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
