"""感知-交互链路的启动文件（视觉跟随演示，进阶课内容）。

启动 假目标检测发布器 -> 感知过滤器 -> 视觉跟随控制器 的数据链，
用于在没有真实相机/检测模型时演示"检测->过滤->控制"的完整闭环。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("mecamind_tools")
    follow_cmd_topic = LaunchConfiguration("follow_cmd_topic")
    raw_detection_topic = LaunchConfiguration("raw_detection_topic")
    perception_event_topic = LaunchConfiguration("perception_event_topic")
    voice_command_topic = LaunchConfiguration("voice_command_topic")
    task_command_topic = LaunchConfiguration("task_command_topic")
    robot_reply_topic = LaunchConfiguration("robot_reply_topic")
    mission_state_topic = LaunchConfiguration("mission_state_topic")
    follow_enable_topic = LaunchConfiguration("follow_enable_topic")
    task_provider = LaunchConfiguration("task_provider")
    api_key_env = LaunchConfiguration("api_key_env")
    dashscope_base_url = LaunchConfiguration("dashscope_base_url")
    llm_model = LaunchConfiguration("llm_model")
    named_goals_file = LaunchConfiguration("named_goals_file")
    waypoints_file = LaunchConfiguration("waypoints_file")
    enable_fake_detections = LaunchConfiguration("enable_fake_detections")

    return LaunchDescription(
        [
            DeclareLaunchArgument("follow_cmd_topic", default_value="/controller/cmd_vel_nav"),
            DeclareLaunchArgument("raw_detection_topic", default_value="/mecamind/raw_detections"),
            DeclareLaunchArgument("perception_event_topic", default_value="/mecamind/perception_event"),
            DeclareLaunchArgument("voice_command_topic", default_value="/mecamind/voice_command"),
            DeclareLaunchArgument("task_command_topic", default_value="/mecamind/task_command"),
            DeclareLaunchArgument("robot_reply_topic", default_value="/mecamind/robot_reply"),
            DeclareLaunchArgument("mission_state_topic", default_value="/mecamind/mission_state"),
            DeclareLaunchArgument("follow_enable_topic", default_value="/mecamind/follow_enable"),
            DeclareLaunchArgument("task_provider", default_value="rule"),
            DeclareLaunchArgument("api_key_env", default_value="DASHSCOPE_API_KEY"),
            DeclareLaunchArgument(
                "dashscope_base_url",
                default_value="https://dashscope.aliyuncs.com/compatible-mode/v1",
            ),
            DeclareLaunchArgument("llm_model", default_value="qwen-plus"),
            DeclareLaunchArgument(
                "named_goals_file",
                default_value=f"{package_share}/config/mecamind_named_goals.yaml",
            ),
            DeclareLaunchArgument(
                "waypoints_file",
                default_value=f"{package_share}/config/mecamind_patrol_waypoints.yaml",
            ),
            DeclareLaunchArgument("enable_fake_detections", default_value="true"),
            Node(
                package="mecamind_tools",
                executable="mecamind_perception_filter",
                name="mecamind_perception_filter",
                output="screen",
                parameters=[
                    {"input_topic": raw_detection_topic},
                    {"output_topic": perception_event_topic},
                ],
            ),
            Node(
                package="mecamind_tools",
                executable="mecamind_vision_follow_controller",
                name="mecamind_vision_follow_controller",
                output="screen",
                parameters=[
                    {"input_topic": perception_event_topic},
                    {"output_cmd_topic": follow_cmd_topic},
                    {"enable_topic": follow_enable_topic},
                ],
            ),
            Node(
                package="mecamind_tools",
                executable="mecamind_fake_detection_publisher",
                name="mecamind_fake_detection_publisher",
                output="screen",
                parameters=[
                    {"output_topic": raw_detection_topic},
                    {
                        "enabled": ParameterValue(
                            enable_fake_detections, value_type=bool
                        )
                    },
                ],
            ),
            Node(
                package="mecamind_tools",
                executable="mecamind_task_scheduler",
                name="mecamind_task_scheduler",
                output="screen",
                parameters=[
                    {"input_topic": voice_command_topic},
                    {"output_topic": task_command_topic},
                    {"reply_topic": robot_reply_topic},
                    {"provider": task_provider},
                    {"api_key_env": api_key_env},
                    {"dashscope_base_url": dashscope_base_url},
                    {"llm_model": llm_model},
                ],
            ),
            Node(
                package="mecamind_tools",
                executable="mecamind_mission_executor",
                name="mecamind_mission_executor",
                output="screen",
                parameters=[
                    {"task_command_topic": task_command_topic},
                    {"mission_state_topic": mission_state_topic},
                    {"follow_enable_topic": follow_enable_topic},
                    {"cmd_vel_topic": follow_cmd_topic},
                    {"named_goals_file": named_goals_file},
                    {"waypoints_file": waypoints_file},
                ],
            ),
        ]
    )
