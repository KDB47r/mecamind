"""语音交互链路启动文件（产品核心：连续听 + 流式 ASR + TTS 播放）。

默认 listen_mode:=continuous：
  voice_listen -> PCM -> asr_stream -> voice_command -> task_scheduler -> TTS -> playback

listen_mode:=ptt 时回退到按键说话：
  record_voice -> audio_file -> asr_file -> ...

密钥优先读环境变量 DASHSCOPE_API_KEY，否则读 mecamind_aliyun.local.yaml（不进 git）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context, *args, **kwargs):
    api_key_env = LaunchConfiguration("api_key_env")
    websocket_url = LaunchConfiguration("websocket_url")
    audio_file_topic = LaunchConfiguration("audio_file_topic")
    pcm_topic = LaunchConfiguration("pcm_topic")
    pcm_end_topic = LaunchConfiguration("pcm_end_topic")
    voice_command_topic = LaunchConfiguration("voice_command_topic")
    asr_result_topic = LaunchConfiguration("asr_result_topic")
    asr_partial_topic = LaunchConfiguration("asr_partial_topic")
    voice_error_topic = LaunchConfiguration("voice_error_topic")
    task_command_topic = LaunchConfiguration("task_command_topic")
    robot_reply_topic = LaunchConfiguration("robot_reply_topic")
    tts_audio_file_topic = LaunchConfiguration("tts_audio_file_topic")
    tts_status_topic = LaunchConfiguration("tts_status_topic")
    listen_status_topic = LaunchConfiguration("listen_status_topic")
    dashscope_base_url = LaunchConfiguration("dashscope_base_url")
    llm_model = LaunchConfiguration("llm_model")
    asr_model = LaunchConfiguration("asr_model")
    tts_model = LaunchConfiguration("tts_model")
    tts_voice = LaunchConfiguration("tts_voice")
    tts_output_dir = LaunchConfiguration("tts_output_dir")
    listen_mode = LaunchConfiguration("listen_mode")
    enable_microphone = LaunchConfiguration("enable_microphone")
    enable_task_scheduler = LaunchConfiguration("enable_task_scheduler")
    enable_tts = LaunchConfiguration("enable_tts")
    enable_tts_playback = LaunchConfiguration("enable_tts_playback")
    llm_provider = LaunchConfiguration("llm_provider")
    microphone_backend = LaunchConfiguration("microphone_backend")
    microphone_device = LaunchConfiguration("microphone_device")
    microphone_duration_sec = LaunchConfiguration("microphone_duration_sec")
    microphone_status_topic = LaunchConfiguration("microphone_status_topic")
    record_voice_service = LaunchConfiguration("record_voice_service")
    microphone_output_dir = LaunchConfiguration("microphone_output_dir")
    wake_words = LaunchConfiguration("wake_words")
    energy_threshold = LaunchConfiguration("energy_threshold")
    playback_backend = LaunchConfiguration("playback_backend")

    continuous = PythonExpression(["'", listen_mode, "' == 'continuous'"])
    ptt = PythonExpression(["'", listen_mode, "' == 'ptt'"])

    nodes = [
        Node(
            package="mecamind_tools",
            executable="mecamind_voice_listen",
            name="mecamind_voice_listen",
            output="screen",
            condition=IfCondition(continuous),
            parameters=[
                {"listen_mode": listen_mode},
                {"pcm_topic": pcm_topic},
                {"pcm_end_topic": pcm_end_topic},
                {"status_topic": listen_status_topic},
                {"asr_partial_topic": asr_partial_topic},
                {"asr_result_topic": asr_result_topic},
                {"voice_command_topic": voice_command_topic},
                {"robot_reply_topic": robot_reply_topic},
                {"tts_status_topic": tts_status_topic},
                {"recorder_backend": microphone_backend},
                {"device": microphone_device},
                {"wake_words": wake_words},
                {
                    "energy_threshold": ParameterValue(
                        energy_threshold,
                        value_type=float,
                    )
                },
            ],
        ),
        Node(
            package="mecamind_tools",
            executable="mecamind_aliyun_asr_stream",
            name="mecamind_aliyun_asr_stream",
            output="screen",
            condition=IfCondition(continuous),
            parameters=[
                {"api_key_env": api_key_env},
                {"websocket_url": websocket_url},
                {"pcm_topic": pcm_topic},
                {"pcm_end_topic": pcm_end_topic},
                {"voice_command_topic": voice_command_topic},
                {"asr_partial_topic": asr_partial_topic},
                {"asr_result_topic": asr_result_topic},
                {"voice_error_topic": voice_error_topic},
                {"robot_reply_topic": robot_reply_topic},
                {"asr_model": asr_model},
                {"publish_voice_command": False},
            ],
        ),
        Node(
            package="mecamind_tools",
            executable="mecamind_microphone_recorder",
            name="mecamind_microphone_recorder",
            output="screen",
            condition=IfCondition(
                PythonExpression(
                    [
                        "'",
                        enable_microphone,
                        "' == 'true' and '",
                        listen_mode,
                        "' == 'ptt'",
                    ]
                )
            ),
            parameters=[
                {"audio_file_topic": audio_file_topic},
                {"status_topic": microphone_status_topic},
                {"record_service": record_voice_service},
                {"recorder_backend": microphone_backend},
                {"device": microphone_device},
                {
                    "duration_sec": ParameterValue(
                        microphone_duration_sec,
                        value_type=float,
                    )
                },
                {"output_dir": microphone_output_dir},
            ],
        ),
        Node(
            package="mecamind_tools",
            executable="mecamind_aliyun_asr_file",
            name="mecamind_aliyun_asr_file",
            output="screen",
            condition=IfCondition(ptt),
            parameters=[
                {"api_key_env": api_key_env},
                {"websocket_url": websocket_url},
                {"audio_file_topic": audio_file_topic},
                {"voice_command_topic": voice_command_topic},
                {"asr_result_topic": asr_result_topic},
                {"voice_error_topic": voice_error_topic},
                {"robot_reply_topic": robot_reply_topic},
                {"asr_model": asr_model},
                {"publish_voice_command": True},
            ],
        ),
        Node(
            package="mecamind_tools",
            executable="mecamind_task_scheduler",
            name="mecamind_aliyun_task_scheduler",
            output="screen",
            condition=IfCondition(enable_task_scheduler),
            parameters=[
                {"provider": llm_provider},
                {"api_key_env": api_key_env},
                {"dashscope_base_url": dashscope_base_url},
                {"llm_model": llm_model},
                {"input_topic": voice_command_topic},
                {"output_topic": task_command_topic},
                {"reply_topic": robot_reply_topic},
            ],
        ),
        Node(
            package="mecamind_tools",
            executable="mecamind_aliyun_tts",
            name="mecamind_aliyun_tts",
            output="screen",
            condition=IfCondition(enable_tts),
            parameters=[
                {"api_key_env": api_key_env},
                {"websocket_url": websocket_url},
                {"input_topic": robot_reply_topic},
                {"output_topic": tts_audio_file_topic},
                {"voice_error_topic": voice_error_topic},
                {"tts_model": tts_model},
                {"tts_voice": tts_voice},
                {"output_dir": tts_output_dir},
            ],
        ),
        Node(
            package="mecamind_tools",
            executable="mecamind_tts_playback",
            name="mecamind_tts_playback",
            output="screen",
            condition=IfCondition(enable_tts_playback),
            parameters=[
                {"input_topic": tts_audio_file_topic},
                {"status_topic": tts_status_topic},
                {"playback_backend": playback_backend},
            ],
        ),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("api_key_env", default_value="DASHSCOPE_API_KEY"),
            DeclareLaunchArgument("websocket_url", default_value=""),
            DeclareLaunchArgument("listen_mode", default_value="continuous"),
            DeclareLaunchArgument("audio_file_topic", default_value="/mecamind/audio_file"),
            DeclareLaunchArgument("pcm_topic", default_value="/mecamind/audio_pcm"),
            DeclareLaunchArgument("pcm_end_topic", default_value="/mecamind/audio_pcm_end"),
            DeclareLaunchArgument("voice_command_topic", default_value="/mecamind/voice_command"),
            DeclareLaunchArgument("asr_result_topic", default_value="/mecamind/asr_result"),
            DeclareLaunchArgument("asr_partial_topic", default_value="/mecamind/asr_partial"),
            DeclareLaunchArgument("voice_error_topic", default_value="/mecamind/voice_error"),
            DeclareLaunchArgument("task_command_topic", default_value="/mecamind/task_command"),
            DeclareLaunchArgument("robot_reply_topic", default_value="/mecamind/robot_reply"),
            DeclareLaunchArgument("tts_audio_file_topic", default_value="/mecamind/tts_audio_file"),
            DeclareLaunchArgument("tts_status_topic", default_value="/mecamind/tts_status"),
            DeclareLaunchArgument("listen_status_topic", default_value="/mecamind/listen_status"),
            DeclareLaunchArgument(
                "dashscope_base_url",
                default_value="https://dashscope.aliyuncs.com/compatible-mode/v1",
            ),
            DeclareLaunchArgument("llm_model", default_value="qwen-plus"),
            DeclareLaunchArgument("llm_provider", default_value="aliyun"),
            DeclareLaunchArgument("asr_model", default_value="paraformer-realtime-v2"),
            DeclareLaunchArgument("tts_model", default_value="cosyvoice-v3-flash"),
            DeclareLaunchArgument("tts_voice", default_value="longanyang"),
            DeclareLaunchArgument("tts_output_dir", default_value="~/.ros/mecamind_tts"),
            DeclareLaunchArgument("enable_microphone", default_value="true"),
            DeclareLaunchArgument("enable_task_scheduler", default_value="true"),
            DeclareLaunchArgument("enable_tts", default_value="true"),
            DeclareLaunchArgument("enable_tts_playback", default_value="true"),
            DeclareLaunchArgument("microphone_backend", default_value="auto"),
            DeclareLaunchArgument("microphone_device", default_value="default"),
            DeclareLaunchArgument("microphone_duration_sec", default_value="4.0"),
            DeclareLaunchArgument(
                "microphone_status_topic",
                default_value="/mecamind/microphone_status",
            ),
            DeclareLaunchArgument(
                "record_voice_service",
                default_value="/mecamind/record_voice",
            ),
            DeclareLaunchArgument(
                "microphone_output_dir",
                default_value="~/.ros/mecamind_mic",
            ),
            DeclareLaunchArgument(
                "wake_words",
                default_value="小度小度,小度,小杜,小渡,小肚",
                description="唤醒词（逗号分隔子串匹配）：小度小度 + 同音误转写兜底",
            ),
            DeclareLaunchArgument("energy_threshold", default_value="450.0"),
            DeclareLaunchArgument("playback_backend", default_value="auto"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
