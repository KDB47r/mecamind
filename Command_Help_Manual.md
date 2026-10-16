# MecaMind 语音导航调试手册（多终端版）

按本手册开 5 个终端，即可：全链路跑导航 + 看见每个环节的文字信息 + 不开麦克风用文本调试。

## 前置准备（一次性）

```bash
cd ~/mecamind_ros2
source install/setup.bash            # 每个新终端都要 source
```

语音功能需已配置密钥（`DASHSCOPE_API_KEY` 或 `mecamind_aliyun.local.yaml`）并安装 dashscope SDK，详见 README「语音功能配置」。

## 终端总览

| 终端 | 作用 | 关键命令 |
| --- | --- | --- |
| 1 | 仿真 + 导航总控 | `ros2 launch mecamind_bringup navigation.launch.py` |
| 2 | 语音链路全日志 | `ros2 launch mecamind_tools mecamind_aliyun_voice.launch.py` |
| 3 | **文本指令输入台（不开麦调试）** | `ros2 topic pub --once /mecamind/voice_command std_msgs/msg/String "{data: '去卧室'}"` |
| 4 | 监听：**语音播报文字** / 麦克风识别结果 | `ros2 topic echo /mecamind/robot_reply` |
| 5 | 监听：任务 JSON / 执行状态 | `ros2 topic echo /mecamind/task_command` |

**启动顺序**：先开 1、2（等它们初始化完成，终端 2 出现 `Voice listen continuous ready`），再开 4、5 两个监听台。

## 终端 1：仿真 + 导航总控

```bash
ros2 launch mecamind_bringup navigation.launch.py
```

- 默认 `backend:=gazebo`：Gazebo 仿真 + RViz + Nav2 全栈一次拉起（无需单独开仿真）。
- `backend:=lite` 为无 Gazebo 的轻量 2D 仿真（无头调试 / CI 用）。
- 等 RViz 出现、日志出现 `lifecycle activator` 激活完成、AMCL 收到 initial pose 后再下达任务。
- 定位漂移时：RViz 工具栏「2D Pose Estimate」重新定位。

## 终端 2：语音链路全日志

```bash
ros2 launch mecamind_tools mecamind_aliyun_voice.launch.py
```

此终端汇聚语音链路所有节点的日志（`output="screen"`），正常一次语音指令应依次看到：

```
Energy threshold calibrated to 450    ← 启动能量门限标定
Capture start (rms=xxx)               ← 检测到说话
Wake word heard: '小度小度...'         ← 唤醒词命中
ASR final: '小度小度去卧室'            ← 语音转文字结果
Voice text '去卧室' -> intent=navigate target='卧室'   ← LLM/规则解析结果
Voice text '北京今天天气怎么样' -> intent=chat          ← 闲聊判定（随后带联网搜索二次调用）
TTS synth: '好的，正在前往卧室。'       ← 播报文字合成
Playing TTS audio                     ← 开始播放
```

LLM 调用失败时会看到 `Aliyun LLM task parsing failed, falling back to rules`（自动降级规则，属预期内）。

## 终端 3：文本指令输入台（不开麦调试）

```bash
# 语法：整条命令必须写在一行内，不能换行
ros2 topic pub --once /mecamind/voice_command std_msgs/msg/String "{data: '去卧室'}"
```

- 直接向 `/mecamind/voice_command` 注入文本 = **跳过麦克风、跳过 ASR**，走「任务解析 → 回复 → TTS」后半段链路。
- **不需要唤醒词**（唤醒是 voice_listen 的职责，文本注入点在其下游）。
- 用 ↑ 调出上一条命令改引号里的内容，即可连续调试。
- 注意：任务真正执行（车动起来）需要终端 1 的 mission_executor；只开语音链路时文本测试只能验证「解析 + 播报」。
- **闲聊（chat）不打断任务**：聊天内容只走回复通道，不会下发 task_command，导航/巡逻照常运行。

常用指令集：

```bash
"{data: '去卧室'}"
"{data: '去厨房'}"
"{data: '巡逻'}"
"{data: '停止'}"
"{data: '取消'}"
"{data: '前进'}"
"{data: '左转'}"
"{data: '跟随'}"
"{data: '今天天气怎么样'}"      # 闲聊：基于真实联网搜索回答（chat 意图）
"{data: '北京今天天气怎么样'}"   # 同上，可指定城市
"{data: '今天有什么新闻'}"      # 同上，新闻类也走联网搜索
"{data: '你好'}"               # 问候闲聊
"{data: '打扫一下'}"            # 目标含糊 → 触发"请确认"闭环
```

## 终端 4：监听语音播报文字

```bash
ros2 topic echo /mecamind/robot_reply
```

- 机器人**每句说出口的话**都会先以文字形式出现在这里（TTS 的输入），是核对"它说了什么"最直接的窗口。
- 想同时看麦克风识别文字（开麦时），另开终端监听：

```bash
ros2 topic echo /mecamind/asr_result   # ASR 最终识别结果（JSON，text 字段）
```

## 终端 5：监听任务 JSON 与执行状态

```bash
ros2 topic echo /mecamind/task_command     # 任务 JSON：LLM/规则的解析产物
# 或另开终端监听执行状态：
ros2 topic echo /mecamind/mission_state    # 状态 JSON：mode/detail 字段
```

典型输出：

```json
# /mecamind/task_command
{"intent": "navigate", "target": "卧室", "requires_confirmation": false, "reply": "好的，正在前往卧室。"}
# /mecamind/mission_state（到达后）
{"mode": "idle", "detail": "reached:bedroom", ...}
```

## 一次语音指令的消息流（对照各终端）

```
麦克风 ──> voice_listen（唤醒/采集）[终端2日志]
        --> /mecamind/audio_pcm --> 流式ASR
        --> /mecamind/asr_result [可监听]     开麦路径
        --> /mecamind/voice_command           文本路径 ← 终端3在此注入
        --> task_scheduler（规则+LLM）[终端2日志]
             ├─ 任务意图 --> /mecamind/task_command [终端5] --> mission_executor --> Nav2 --> 小车
             └─ chat 意图 --> 聊天LLM（带联网搜索）--> 只回话，不碰任务管线
回复链：
        task_scheduler/executor --> /mecamind/robot_reply [终端4] --> TTS --> 扬声器
        --> /mecamind/tts_status [可监听：playing/finished]
```

## 最小验证链路（纯文本，不开麦）

只想验证「LLM 解析 + TTS 播报」，只需开 **终端 2、3、4、5**（不启动终端 1）：

```
终端2 语音链路 → 终端3 发 "{data: '去卧室'}"
→ 终端4 看到 "好的，正在前往卧室。"
→ 终端2 日志 TTS synth + Playing TTS audio，扬声器出声

验证聊天：终端3 发 "{data: '北京今天天气怎么样'}"
→ 终端4 收到基于真实搜索结果的回答（约 3~5 秒），终端5 无任何新消息
```

验证通过后再开终端 1，同样文本注入即可看到车真的动起来。

## 常见问题

| 现象 | 原因 / 处理 |
| --- | --- |
| `topic does not appear to be published yet` | 语音链路没启动，或监听台开在了链路之前。先开 1/2，等就绪后再开 echo |
| `Package 'mecamind_tools' not found` | 当前终端没 `source install/setup.bash` |
| 文本命令报"没有这个文件或目录" | `ros2 topic pub` 命令被拆成两行了，必须一整行 |
| 开麦时车"听不到"我 | TTS 播放期间麦克风暂停采集（pause_while_tts），等它说完；或检查终端 2 的能量门限标定值是否过高 |
| 文本注入有回复但车不动 | mission_executor 在终端 1，未启动则只解析不执行 |
| 闲聊回复要等 3~5 秒才播报 | 正常：chat 走两次 LLM 调用（意图判定 + 联网搜索回答） |
| 回复"已改用本地理解…"并要求确认 | LLM 调用失败（密钥没找到/网络问题）自动降级规则；看终端 2 的 `Aliyun LLM task parsing failed` 日志定位原因 |
| 环境有噪音时一直播"语音识别失败" | 已修复：未唤醒（没说唤醒词）的会话失败静默丢弃；鉴权/配额/网络错误最多 30 秒播报一次 |
