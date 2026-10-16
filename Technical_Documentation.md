# MecaMind 语音聊天（联网实时问答）功能技术文档

> 适用版本：本文档描述 chat 功能实现及随后修复的三个线上问题（密钥查找、噪音降噪、确认闭环）。
> 涉及文件：`aliyun_clients.py`、`task_scheduler.py`、`aliyun_speech_nodes.py`、`mission_executor.py`。

---

## 1. 需求背景

扫地机器人使用场景下，语音交互有两条明确诉求：

1. **支持闲聊、且回答必须是真实的**：使用者随口问"今天天气怎么样""今天有什么新闻"，机器人要基于真实数据回答，**不允许编造**；
2. **聊天不能打断正在执行的任务**：导航/巡逻进行中随口聊天，任务照常执行。

最初的想法是"给 chat 单独开一个线程，和导航分开"。分析后的结论是：**不需要多线程**——ROS 2 里每个节点本身就是独立进程（有自己的 executor），任务管线（mission_executor）与语音管线（task_scheduler）天然并行。真正的关键不在于"在哪条线程里跑"，而在于**让 chat 的消息在语义上不进任务通道**。这就是本文档的核心设计：意图分流。

---

## 2. 总体设计思路

### 2.1 意图分流：chat 与任务走不同通道

```
                     ┌─ 任务意图 ──> /mecamind/task_command ──> mission_executor ──> Nav2
voice_command ──> task_scheduler（意图判定）
                     └─ chat 意图 ──> 聊天 LLM（联网搜索）──> 只发 /mecamind/robot_reply ──> TTS
```

- 所有用户话语先经过**任务解析 LLM**做意图分类（原链路不变，只是意图白名单里新增 `chat`）；
- 判定为 `chat` 时，**不再发布 `/mecamind/task_command`**——mission_executor 对聊天完全无感，正在执行的导航/巡逻不受任何影响；
- 聊天回复直接发布到 `/mecamind/robot_reply`，走原有的 TTS 播报链路。

这样"导航的同时聊天"不需要任何并发机制：导航在 mission_executor 进程里跑，聊天在 task_scheduler 进程里跑，两者在话题层面互不相交。

### 2.2 为什么用两个 LLM 客户端，而不是一个

任务解析和聊天对模型的要求是**相反的**：

| | 任务解析客户端 | 聊天客户端 |
| --- | --- | --- |
| 模型 | qwen-plus | qwen-plus（同一个） |
| temperature | 0.0（要确定性） | 0.7（要自然语气） |
| 输出格式 | 强制 JSON（`response_format=json_object`） | 纯口语文本 |
| 联网搜索 | 关闭 | **开启（enable_search）** |
| 提示词角色 | "任务解析器"，白名单意图 + few-shot | "家用扫地机器人聊天人格"，1~2 句口语 |

- **搜索只在聊天时开**：搜索会增加响应延迟、且按模型调用计费，任务解析（"去卧室"）完全用不到；
- 同一模型、同一密钥、不同参数的两个实例，比"一个客户端里塞满分支"清晰得多。

### 2.3 实时信息的实现：DashScope `enable_search`

- `enable_search` 是兼容模式**请求体的一个字段**（`payload["enable_search"] = True`），**不需要**在百炼控制台做任何开关配置、不需要新密钥、不需要换模型；
- 模型只在需要时效性信息（天气/新闻）时才触发搜索，把搜索结果作为上下文参与回答；普通问候不会触发搜索；
- 聊天提示词明确要求："搜索不到的内容不要编造"；
- **降级策略**：聊天二次调用失败（网络抖动等）时，退回任务 LLM 在第一轮已经生成的话术，保证"永远有话可播、链路不瘫痪"。

### 2.4 完整调用时序（一次天气提问）

```
用户："小度小度 今天天气怎么样"
  │
  ├─ voice_listen：唤醒词命中，流式采麦
  ├─ asr_stream：流式 ASR → "今天天气怎么样"
  ├─ task_scheduler._parse_command：
  │     第 1 次调用 任务解析 LLM（无搜索，JSON 模式）
  │     → {"intent":"chat","requires_confirmation":false,"reply":"..."}
  ├─ 判定 intent == "chat"：
  │     第 2 次调用 聊天 LLM（enable_search=True，口语模式）
  │     → "今天北京晴，气温12到24摄氏度，适合开窗通风哦～"
  ├─ 发布 /mecamind/robot_reply（不发布 task_command）
  └─ aliyun_tts → tts_playback → 扬声器
```

---

## 3. 上线后暴露的三个问题与修复

### 3.1 问题一：LLM 调用失败，天气提问被当成"待确认指令"

**现象**：问天气，回复"已改用本地理解"并要求确认；确认后又没有任何结果。

**根因**：`setup.py` 出于安全考虑**故意不把 `*.local.yaml` 打进安装包**（密钥只留在 `src/mecamind_tools/config/`），但安装后运行的代码找密钥时只在 `install/` 目录附近找——**永远找不到**。于是 LLM 调用抛 `Missing DASHSCOPE_API_KEY`，调度器按设计自动降级到本地规则；本地规则不认识天气问题 → 返回 `unknown + requires_confirmation=True` → "请确认是否执行"。

**修复**（`aliyun_clients.py`）：新增工作空间源码树搜索——从"代码文件位置"和"当前工作目录"出发**逐级向上**查找 `<root>/src/mecamind_tools/config/mecamind_aliyun.local.yaml`。源码运行和 colcon 安装后运行都能命中，同时保留了"安装包绝不携带密钥"的安全设计。

**验证**：不带任何环境变量，安装后的代码直接调用：

```
PARSE: chat | requires_confirmation = False
CHAT:  今天北京晴，气温12到24摄氏度，空气质量良，适合开窗通风哦～
```

### 3.2 问题二：环境噪音导致"语音识别失败"反复播报

**现象**：还没说唤醒词，仅环境噪音就触发采麦，终端不停刷"语音识别失败，请再说一遍"。

**根因**（两层）：
1. 密钥缺失期间，每个噪音会话的 ASR 连接建立都失败——把本应偶发的事件变成了高频事件；
2. `asr_stream` 节点的**发送失败**路径没有唤醒门控：`voice_listen` 本来就有"未唤醒会话静默丢弃"的逻辑，但 asr_stream 这条路径绕过了它。

**修复**（`aliyun_speech_nodes.py`）：
- 跟踪 PCM 帧自带的 `phase` 字段（voice_listen 听到唤醒词后把后续帧标为 `command`），维护 `_session_woken` 状态；
- 未唤醒会话的失败**静默丢弃**（噪音误触发，不打扰用户）；
- 鉴权/配额/网络这类"配置级"错误必须让用户知道——**30 秒限频播报一次**，既不错过真正的故障提示，也不会被噪音刷屏。

设计原则：**只有听到唤醒词之后，机器人才"认真听"并对失败做出反馈**。

### 3.3 问题三：确认了未知指令，播完"开始执行"就没有下文

**现象**：对听不懂的话说"确认"后，机器人播报"好的，开始执行。"然后什么都没发生。

**根因**：确认闭环里，`_dispatch_intent` 对 `unknown` 意图走 `mode == "idle"` 分支 → `_stop_all("canceled")`——**静默停机**（如果当时在导航，导航也会被取消）。而"好的，开始执行。"是调度器对 confirm 的无脑回复，与实际行为矛盾。

**修复**（`mission_executor.py`）：`idle` 分支拆成两种情况：

- `stop` / `cancel` → 保持原样，停机；
- 其他无法执行的意图（unknown 等）→ **不动现有任务**，明确播报：

```python
self._publish_reply(f"抱歉，我还不会执行「{target or intent}」，请换个说法。")
self._set_state("idle", f"unsupported_intent:{intent}", target)
```

同时，聊天在 LLM 可用时**根本不会进入确认环节**（chat 意图的 `requires_confirmation` 恒为 False），确认闭环只为真正含糊的指令保留。

---

## 4. 改动清单（代码级）

### 4.1 `src/mecamind_tools/mecamind_tools/aliyun_clients.py`（云服务访问层）

| 改动 | 说明 |
| --- | --- |
| `AliyunLlmClient.__init__` 增加 `enable_search: bool = False` | 联网搜索开关，存为实例属性 |
| 抽取 `_post_chat(messages, temperature, json_mode)` | parse_task 与 chat 共用同一段 HTTP 逻辑（取密钥 → 组装请求 → POST → 解析 choices[0]） |
| `_post_chat` 中按需加 `payload["enable_search"] = True` | 纯请求体字段，无需控制台配置 |
| 重写 `parse_task(text)` | `temperature=0.0` + `json_mode=True`，返回 `AliyunTaskPlan` |
| 新增 `chat(text)` | `temperature=0.7`，返回口语文本；空结果兜底"抱歉，我一时没想好怎么回答。" |
| 新增 `build_chat_messages(text)` | 聊天人格提示词：友好口语、1~2 句、适合语音播报、基于搜索结果如实回答不编造 |
| `build_task_parser_messages` 提示词更新 | 意图白名单加入 `chat`；明确"非任务的闲聊/提问 → intent=chat、requires_confirmation=false、附 1~2 句友好回复" |
| `parse_task_plan_payload` 白名单加入 `"chat"` | 防止 LLM 输出的 chat 意图被强制降级成 unknown |
| 新增 `_workspace_src_config_candidates()` | 从代码位置/工作目录逐级向上搜 `<root>/src/mecamind_tools/config/*.local.yaml` |
| `_credential_config_candidates` 扩展为 5 级查找 | 环境变量指定 → 包旁 config → 工作空间源码树 → cwd 下 config → `~/.config/mecamind` |

### 4.2 `src/mecamind_tools/mecamind_tools/task_scheduler.py`（任务调度器）

| 改动 | 说明 |
| --- | --- |
| `TaskCommand` 意图集合加入 `move/chat` | 文档与类型说明同步 |
| `task_command_from_aliyun_plan` 确认前缀排除集加入 `chat` | chat 永远不生成"请确认是否执行"话术 |
| `__init__` 创建第二个客户端 `self.chat_client` | 同模型同密钥，`enable_search=True` |
| `_cb` 新增 chat 分支 | 见下方代码——判定为 chat 时二次调用聊天客户端，**只发 robot_reply 后直接 return，不发布 task_command**；聊天调用失败则退回任务 LLM 已生成的话术 |

```python
if command.intent == "chat":
    reply = command.reply
    try:
        reply = self.chat_client.chat(msg.data) or reply
    except Exception as exc:
        self.get_logger().warn(f"Aliyun chat LLM failed, using fallback reply: {exc}")
    if reply:
        reply_msg = String()
        reply_msg.data = reply
        self.reply_pub.publish(reply_msg)
    return          # 关键：闲聊只回话，不发布 task_command
```

### 4.3 `src/mecamind_tools/mecamind_tools/aliyun_speech_nodes.py`（流式 ASR 节点）

| 改动 | 说明 |
| --- | --- |
| 新增 `_session_woken` / `_last_failure_announce_at` | 会话唤醒标记 + 配置级错误限频时间戳 |
| `_ensure_session` 创建新会话时重置 `_session_woken` | 每个采麦会话独立判定唤醒 |
| `_pcm_cb` 按帧合并 `phase == "command"` 到 `_session_woken` | voice_listen 听到唤醒词后的帧带 command 标记 |
| `_pcm_cb` 失败路径传 `woken=self._session_woken` | 发送失败播报纳入唤醒门控 |
| `_publish_failure` 增加 `woken` 参数 | 未唤醒：普通失败静默；鉴权/配额/网络错误 30 秒限频播报一次 |

### 4.4 `src/mecamind_tools/mecamind_tools/mission_executor.py`（任务执行器）

| 改动 | 说明 |
| --- | --- |
| `_dispatch_intent` 的 `idle` 分支拆分 | `stop/cancel` → 停机（原行为）；其余意图 → 明确回复"抱歉，我还不会执行「…」"，**不动现有任务**，状态记为 `unsupported_intent:<intent>` |

---

## 5. 完整消息流（含 chat 分支）

```
麦克风 ──> voice_listen（能量门限 + 唤醒词）
        --> /mecamind/audio_pcm（每帧带 phase: wake/command）
        --> asr_stream（流式 ASR；未唤醒会话的失败静默）
        --> /mecamind/voice_command
        --> task_scheduler（规则 + LLM 意图判定）
             ├─ 任务意图 ──> /mecamind/task_command ──> mission_executor ──> Nav2 ──> 小车
             └─ chat 意图 ──> 聊天 LLM（enable_search，二次调用）
                            ──> /mecamind/robot_reply（不碰任务管线）
回复：robot_reply ──> aliyun_tts ──> tts_playback ──> 扬声器
```

**并发性说明**：导航执行在 mission_executor 进程内进行；聊天只发生在 task_scheduler 进程内，两者通过"chat 不发布 task_command"在话题层面解耦——**无需线程、无需锁，天然并行**。

---

## 6. 验证结果

- 密钥修复后（安装后代码、无环境变量）：`parse_task("今天天气怎么样")` → `intent=chat, requires_confirmation=False`；`chat("今天天气怎么样")` → "今天北京晴，气温12到24摄氏度，空气质量良，适合开窗通风哦～"（真实搜索数据）；
- `parse_task("去卧室")` → `navigate | 卧室 | False`（任务解析不受影响）；
- 文本注入调试命令（不开麦）：

```bash
ros2 topic pub --once /mecamind/voice_command std_msgs/msg/String "{data: '今天天气怎么样'}"
ros2 topic pub --once /mecamind/voice_command std_msgs/msg/String "{data: '北京今天天气怎么样'}"
ros2 topic pub --once /mecamind/voice_command std_msgs/msg/String "{data: '今天有什么新闻'}"
```

- 预期：终端 4（`/mecamind/robot_reply`）收到真实回答；终端 5（`/mecamind/task_command`）**无任何新消息**（导航不被干扰）。

---

## 7. 后续可扩展方向

| 方向 | 说明 |
| --- | --- |
| 多轮对话记忆 | chat 客户端维护历史消息列表，支持"那明天呢"这类追问 |
| 到达播报 | mission_executor 到达目标后经 robot_reply 播报"已抵达卧室"（当前为已知空缺） |
| Plan B：function calling | 用 qwen 的 agent 策略 + 工具调用直连天气/新闻 API，响应更快、结果可结构化，但按 agent 策略计费 |
| TTS 播放时采麦优化 | 播报期间麦克风暂停采集（pause_while_tts）的进一步打磨 |
| 闲聊意图的本地兜底 | LLM 完全不可用时，用关键词启发式（天气/新闻/怎么样/为什么…）把问题类文本路由到闲聊而非确认闭环 |
