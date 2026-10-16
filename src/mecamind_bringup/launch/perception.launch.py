"""第四课（10.3.4）目标检测链路的统一启动入口。

【这个文件是干什么的】
一条命令拉起"检测器 + JSON 适配器 + 感知过滤"三个节点：

    ros2 launch mecamind_bringup perception.launch.py
    # 默认 with_ui:=true，会弹出 OpenCV 窗口显示带框调试图
    ros2 launch mecamind_bringup perception.launch.py with_ui:=false   # 无界面/CI
    ros2 launch mecamind_bringup perception.launch.py source:=camera backend:=onnx \
        model_path:=/path/to/yolov8n.onnx

【数据流】
    detector（相机/视频/合成画面 -> YOLO/ONNX/HSV 推理）
        │ /mecamind/detections（Detection2DArray 结构化消息，新契约）
        ▼
    json_bridge（新契约 -> 旧 JSON 契约的适配器）
        │ /mecamind/raw_detections（String JSON）
        ▼
    perception_filter（标签/置信度筛选 + 连续帧确认）
        │ /mecamind/perception_event
        ▼
    （第 5 课接跟随控制器）

【三级兜底设计】
- source:=synthetic（默认）：内置合成画面，任何机器都能跑；
- backend:=auto（默认）：ultralytics -> onnx -> hsv 依次降级，
  没装任何 ML 包时自动落到 HSV 颜色检测，链路照样完整。
课堂上先用默认参数保证全班跑通，再按各自环境逐级升级。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_viewer_script(_context, *args, **kwargs):
    """定位项目根下的 scripts/show_detection_image.py（share 旁两级回 workspace）。"""
    # install/mecamind_bringup/share/mecamind_bringup → 项目根（开发态常见布局）
    share = get_package_share_directory("mecamind_bringup")
    candidates = [
        os.path.normpath(os.path.join(share, "..", "..", "..", "scripts", "show_detection_image.py")),
        os.path.expanduser("~/mecamind_ros2/scripts/show_detection_image.py"),
    ]
    script = next((p for p in candidates if os.path.isfile(p)), candidates[-1])
    return [
        ExecuteProcess(
            cmd=["python3", script],
            output="screen",
            additional_env={"DISPLAY": os.environ.get("DISPLAY", ":0")},
            condition=IfCondition(LaunchConfiguration("with_ui")),
        )
    ]


def generate_launch_description():
    source = LaunchConfiguration("source")
    backend = LaunchConfiguration("backend")
    model_path = LaunchConfiguration("model_path")
    video_path = LaunchConfiguration("video_path")
    camera_topic = LaunchConfiguration("camera_topic")
    target_label = LaunchConfiguration("target_label")
    debug_image = LaunchConfiguration("debug_image")

    # 检测器：视觉链路的源头（详见 detector_node.py 的模块注释）。
    detector = Node(
        package="mecamind_perception",
        executable="mecamind_detector",
        name="mecamind_detector",
        output="screen",
        parameters=[
            {"source": source},
            {"backend": backend},
            {"model_path": model_path},
            {"video_path": video_path},
            {"camera_topic": camera_topic},
            {"target_label": target_label},
            {"debug_image": debug_image},
        ],
    )

    # 新旧契约适配器：让第 3 课之前写好的 JSON 下游零改动继续工作。
    json_bridge = Node(
        package="mecamind_perception",
        executable="mecamind_detection_json_bridge",
        name="mecamind_detection_json_bridge",
        output="screen",
    )

    # 感知过滤：筛掉低置信度/闪烁的检测，输出确认过的目标事件。
    perception_filter = Node(
        package="mecamind_tools",
        executable="mecamind_perception_filter",
        name="mecamind_perception_filter",
        output="screen",
        parameters=[{"target_label": target_label}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "source",
                default_value="synthetic",
                description="Image source: camera / video / synthetic.",
            ),
            DeclareLaunchArgument(
                "backend",
                default_value="auto",
                description="Inference backend: auto / ultralytics / onnx / hsv.",
            ),
            DeclareLaunchArgument("model_path", default_value="yolov8n.pt"),
            DeclareLaunchArgument("video_path", default_value=""),
            DeclareLaunchArgument("camera_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument(
                "target_label",
                default_value="person",
                description="Label the filter keeps (hsv backend also tags its blob with it).",
            ),
            DeclareLaunchArgument("debug_image", default_value="true"),
            DeclareLaunchArgument(
                "with_ui",
                default_value="true",
                description="Open OpenCV window showing /mecamind/detection_image.",
            ),
            detector,
            json_bridge,
            perception_filter,
            OpaqueFunction(function=_resolve_viewer_script),
        ]
    )
