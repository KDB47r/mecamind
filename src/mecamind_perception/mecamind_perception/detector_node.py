"""目标检测节点：多输入源、多推理后端、多线程流水线（第 4~5 课核心）。

在系统中的角色：视觉链路的源头。
- 输入（source 参数三选一）：
  camera    订阅 sensor_msgs/Image（Gazebo 相机或真实相机）；
  video     循环读取本地视频文件（无相机时的课堂路径）；
  synthetic 内置合成画面（一个左右移动的红色"目标"），零外部依赖，
            保证任何机器上整条视觉链路都能跑通——最后一级兜底。
- 推理后端（backend 参数）：
  ultralytics  Ultralytics YOLO（装了 ultralytics 包才可用）；
  onnx         ONNX Runtime 跑 YOLOv8 导出模型（装了 onnxruntime 才可用）；
  hsv          经典 HSV 颜色阈值 + 轮廓检测，无任何 ML 依赖，永远可用；
  auto         按 ultralytics -> onnx -> hsv 顺序自动降级（默认）。
- 输出：
  /mecamind/detections           Detection2DArray（结构化检测结果）；
  /mecamind/detection_image      带框调试图（可关）；
  /mecamind/perception_metrics   JSON 指标（fps / 推理耗时 / 队列丢帧数）。

多线程流水线（第 5 课重点）：

    采集（相机回调 / 视频线程 / 合成线程）
        │  put_latest()：队列容量 1，满了就挤掉旧帧
        ▼
    有界队列（maxsize=1）        ← 关键设计：宁可丢帧，不可积压
        │  get(timeout)
        ▼
    推理线程（独立于 ROS 回调线程）→ 发布检测/调试图/指标

为什么队列容量是 1？推理比采集慢时，若队列无限长，帧会越积越多，
机器人看到的是"几秒前的世界"——对控制来说延迟比丢帧危险得多。
容量 1 + 挤掉旧帧 = "永远处理最新一帧"策略，延迟被限制在一个
推理周期内。

初学者重点阅读：
1. LatestFrameQueue —— 有界队列 + 最新帧策略；
2. _inference_loop —— 推理线程主循环（与 ROS 回调解耦）；
3. HsvBackend.detect —— 最朴素的"检测器"长什么样；
4. _pick_backend —— 依赖缺失时的自动降级。
"""

from __future__ import annotations

import json
import math
import queue
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from mecamind_interfaces.msg import Detection2D, Detection2DArray

try:  # cv2 在 ros-jazzy-desktop 环境默认可用；缺失时只有 synthetic+fake 可用
    import cv2
except Exception:  # noqa: BLE001
    cv2 = None


# ---------------------------------------------------------------------------
# 帧队列：容量 1 的"最新帧"队列
# ---------------------------------------------------------------------------


@dataclass
class FramePacket:
    """一帧图像 + 采集时刻（单调钟，用于算端到端延迟）。"""

    frame: np.ndarray
    stamp_mono: float


class LatestFrameQueue:
    """容量为 1 的帧队列：满了就丢弃旧帧，永远保留最新一帧。

    dropped 计数器记录被挤掉的帧数——它不是错误指标，而是"推理跟不上
    采集"的健康信号，会随 metrics 一起发布出去供观察。
    """

    def __init__(self) -> None:
        self._q: "queue.Queue[FramePacket]" = queue.Queue(maxsize=1)
        self.dropped = 0

    def put_latest(self, packet: FramePacket) -> None:
        while True:
            try:
                self._q.put_nowait(packet)
                return
            except queue.Full:
                try:
                    self._q.get_nowait()
                    self.dropped += 1
                except queue.Empty:
                    pass

    def get(self, timeout: float) -> Optional[FramePacket]:
        try:
            return self._q.get(timeout=timeout)
        except queue.Empty:
            return None


# ---------------------------------------------------------------------------
# 推理后端：统一的 detect(frame) -> List[(label, conf, cx, cy, w, h)] 接口
# 坐标一律输出归一化值（0~1），与 Detection2D 消息约定一致。
# ---------------------------------------------------------------------------


DetTuple = Tuple[str, float, float, float, float, float]

# COCO 80 类别名（YOLO 系列模型的默认标签表）
COCO_NAMES = (
    "person bicycle car motorcycle airplane bus train truck boat traffic_light "
    "fire_hydrant stop_sign parking_meter bench bird cat dog horse sheep cow "
    "elephant bear zebra giraffe backpack umbrella handbag tie suitcase frisbee "
    "skis snowboard sports_ball kite baseball_bat baseball_glove skateboard "
    "surfboard tennis_racket bottle wine_glass cup fork knife spoon bowl banana "
    "apple sandwich orange broccoli carrot hot_dog pizza donut cake chair couch "
    "potted_plant bed dining_table toilet tv laptop mouse remote keyboard "
    "cell_phone microwave oven toaster sink refrigerator book clock vase "
    "scissors teddy_bear hair_drier toothbrush"
).split()


class UltralyticsBackend:
    """Ultralytics YOLO 后端：pip install ultralytics 后可用（首选）。"""

    name = "ultralytics"

    def __init__(self, model_path: str, conf_threshold: float) -> None:
        from ultralytics import YOLO  # 缺包时抛 ImportError，由 _pick_backend 捕获

        self._model = YOLO(model_path)
        self._conf = conf_threshold

    def detect(self, frame: np.ndarray) -> List[DetTuple]:
        h, w = frame.shape[:2]
        results = self._model(frame, conf=self._conf, verbose=False)
        out: List[DetTuple] = []
        for r in results:
            names = r.names
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                out.append(
                    (
                        str(names[int(box.cls[0])]),
                        float(box.conf[0]),
                        ((x1 + x2) / 2.0) / w,
                        ((y1 + y2) / 2.0) / h,
                        (x2 - x1) / w,
                        (y2 - y1) / h,
                    )
                )
        return out


class OnnxBackend:
    """ONNX Runtime 后端：跑 YOLOv8 导出的 .onnx 模型（次选）。

    实现了标准的 YOLOv8 前后处理：letterbox 缩放 -> 推理 ->
    置信度过滤 -> NMS 去重 -> 坐标映射回原图并归一化。
    """

    name = "onnx"

    def __init__(self, model_path: str, conf_threshold: float, input_size: int = 640) -> None:
        import onnxruntime  # 缺包时抛 ImportError

        self._sess = onnxruntime.InferenceSession(
            model_path, providers=["CPUExecutionProvider"]
        )
        self._input_name = self._sess.get_inputs()[0].name
        self._size = input_size
        self._conf = conf_threshold

    def detect(self, frame: np.ndarray) -> List[DetTuple]:
        h, w = frame.shape[:2]
        # letterbox：等比缩放 + 灰边填充成正方形，保持长宽比不变形
        scale = self._size / max(h, w)
        nh, nw = int(round(h * scale)), int(round(w * scale))
        canvas = np.full((self._size, self._size, 3), 114, dtype=np.uint8)
        canvas[:nh, :nw] = cv2.resize(frame, (nw, nh))
        blob = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0

        # YOLOv8 输出形状 (1, 84, N)：4 个框参数 + 80 类得分，N 个候选
        pred = self._sess.run(None, {self._input_name: blob})[0][0]
        boxes_xywh = pred[:4].T          # (N, 4) cx cy w h（letterbox 坐标系）
        scores_all = pred[4:].T          # (N, 80)
        cls_ids = scores_all.argmax(axis=1)
        confs = scores_all.max(axis=1)
        keep = confs >= self._conf
        boxes_xywh, cls_ids, confs = boxes_xywh[keep], cls_ids[keep], confs[keep]
        if len(boxes_xywh) == 0:
            return []

        # NMS 去掉互相重叠的重复框
        xy = boxes_xywh.copy()
        xy[:, 0] -= xy[:, 2] / 2
        xy[:, 1] -= xy[:, 3] / 2
        idx = cv2.dnn.NMSBoxes(xy.tolist(), confs.tolist(), self._conf, 0.45)
        out: List[DetTuple] = []
        for i in np.array(idx).flatten():
            cx, cy, bw, bh = boxes_xywh[i]
            label = COCO_NAMES[cls_ids[i]] if cls_ids[i] < len(COCO_NAMES) else str(cls_ids[i])
            # letterbox 坐标 -> 原图像素 -> 归一化
            out.append(
                (
                    label,
                    float(confs[i]),
                    float(cx / scale / w),
                    float(cy / scale / h),
                    float(bw / scale / w),
                    float(bh / scale / h),
                )
            )
        return out


class HsvBackend:
    """HSV 颜色阈值后端：检测画面里最大的红色色块（保底，永远可用）。

    这不是"玩具"：颜色阈值 + 轮廓提取是深度学习之前工业界用了几十年
    的方法，而且它让整条链路（消息契约、下游过滤、跟随控制）在没有
    任何 ML 依赖的机器上都能被完整验证——课堂兜底路径的意义所在。
    置信度用色块面积占画面比例映射（面积越大越"确信"），仅供教学。
    """

    name = "hsv"

    def __init__(self, label: str, conf_threshold: float) -> None:
        self._label = label
        self._conf_threshold = conf_threshold

    def detect(self, frame: np.ndarray) -> List[DetTuple]:
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # 红色在 HSV 色环上跨 0 度，要用两段范围拼起来
        mask = cv2.inRange(hsv, (0, 120, 80), (10, 255, 255)) | cv2.inRange(
            hsv, (170, 120, 80), (180, 255, 255)
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return []
        big = max(contours, key=cv2.contourArea)
        x, y, bw, bh = cv2.boundingRect(big)
        area_ratio = cv2.contourArea(big) / float(w * h)
        confidence = max(0.0, min(0.99, 0.5 + area_ratio * 20.0))
        if confidence < self._conf_threshold:
            return []
        return [
            (
                self._label,
                confidence,
                (x + bw / 2.0) / w,
                (y + bh / 2.0) / h,
                bw / float(w),
                bh / float(h),
            )
        ]


# ---------------------------------------------------------------------------
# 检测节点
# ---------------------------------------------------------------------------


class DetectorNode(Node):
    """检测节点：采集与推理解耦的双线程结构 + 结构化消息发布。"""

    def __init__(self) -> None:
        super().__init__("mecamind_detector")
        # ---- 输入源与后端选择 ----
        self.declare_parameter("source", "synthetic")        # camera / video / synthetic
        self.declare_parameter("backend", "auto")            # auto / ultralytics / onnx / hsv
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("video_path", "")
        self.declare_parameter("model_path", "yolov8n.pt")   # ultralytics 权重或 .onnx 路径
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("target_label", "person")     # hsv 后端给色块贴的标签
        # ---- 输出 ----
        self.declare_parameter("detections_topic", "/mecamind/detections")
        self.declare_parameter("debug_image", True)
        self.declare_parameter("debug_image_topic", "/mecamind/detection_image")
        self.declare_parameter("metrics_topic", "/mecamind/perception_metrics")
        # ---- 合成/视频源的帧率 ----
        self.declare_parameter("capture_rate_hz", 15.0)
        # ---- 合成画面里目标的运动参数（课堂演示"目标左右移动"用） ----
        self.declare_parameter("synthetic_period_sec", 8.0)  # 目标左右往返一圈的时间
        self.declare_parameter("synthetic_amplitude", 0.30)  # 相对画面中心的摆幅

        if cv2 is None:
            raise RuntimeError("mecamind_detector 需要 OpenCV（python3-opencv）")

        self._queue = LatestFrameQueue()
        self._backend = self._pick_backend()
        self._bridge = None  # cv_bridge 仅 camera 源需要，惰性导入

        self.det_pub = self.create_publisher(
            Detection2DArray, str(self.get_parameter("detections_topic").value), 10
        )
        self._debug_enabled = bool(self.get_parameter("debug_image").value)
        self.img_pub = (
            self.create_publisher(Image, str(self.get_parameter("debug_image_topic").value), 2)
            if self._debug_enabled
            else None
        )
        self.metrics_pub = self.create_publisher(
            String, str(self.get_parameter("metrics_topic").value), 5
        )

        # ---- 按 source 启动采集端 ----
        source = str(self.get_parameter("source").value)
        self._capture_stop = threading.Event()
        if source == "camera":
            from cv_bridge import CvBridge

            self._bridge = CvBridge()
            self.create_subscription(
                Image, str(self.get_parameter("camera_topic").value), self._image_cb, 2
            )
        elif source in ("video", "synthetic"):
            self._capture_thread = threading.Thread(
                target=self._capture_loop, args=(source,), daemon=True
            )
            self._capture_thread.start()
        else:
            raise RuntimeError(f"未知 source: {source}（可选 camera/video/synthetic）")

        # ---- 推理线程与指标 ----
        self._frames_done = 0
        self._inference_ms_last = 0.0
        self._metrics_lock = threading.Lock()
        self._infer_thread = threading.Thread(target=self._inference_loop, daemon=True)
        self._infer_thread.start()
        self.create_timer(1.0, self._publish_metrics)
        self.get_logger().info(
            f"MecaMind detector ready: source={source}, backend={self._backend.name}"
        )

    # ------------------------------------------------------------------
    # 后端选择（自动降级）
    # ------------------------------------------------------------------

    def _pick_backend(self):
        """按参数挑选推理后端；auto 模式按可用性依次降级。

        依赖缺失（ImportError）或模型加载失败都会触发降级并打日志，
        保证节点"总能起来"——课堂上绝不因为缺一个 pip 包全场卡住。
        """
        want = str(self.get_parameter("backend").value)
        conf = float(self.get_parameter("conf_threshold").value)
        model = str(self.get_parameter("model_path").value)
        label = str(self.get_parameter("target_label").value)

        # 合成画面里的红色矩形是给 HSV 设计的，真实 YOLO 模型认不出它
        # （不是 COCO 里的任何类别）。auto + synthetic 时直接配 HSV，
        # 避免"装了 ultralytics 反而检不到目标"的反直觉现象。
        source = str(self.get_parameter("source").value)
        if want == "auto" and source == "synthetic":
            self.get_logger().info("source=synthetic 且 backend=auto：使用 hsv 后端")
            return HsvBackend(label, conf)

        order = [want] if want != "auto" else ["ultralytics", "onnx", "hsv"]
        for name in order:
            try:
                if name == "ultralytics":
                    return UltralyticsBackend(model, conf)
                if name == "onnx":
                    return OnnxBackend(model, conf)
                if name == "hsv":
                    return HsvBackend(label, conf)
                raise RuntimeError(f"未知 backend: {name}")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"后端 {name} 不可用（{exc}），尝试下一个")
        # HSV 没有外部依赖，理论上永远走不到这里
        return HsvBackend(label, conf)

    # ------------------------------------------------------------------
    # 采集端
    # ------------------------------------------------------------------

    def _image_cb(self, msg: Image) -> None:
        """camera 源：ROS 图像回调 -> 转 OpenCV -> 入队（挤掉旧帧）。"""
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._queue.put_latest(FramePacket(frame, time.monotonic()))

    def _capture_loop(self, source: str) -> None:
        """video / synthetic 源的采集线程：按固定帧率产帧并入队。"""
        rate = max(1.0, float(self.get_parameter("capture_rate_hz").value))
        period = 1.0 / rate
        cap = None
        if source == "video":
            path = str(self.get_parameter("video_path").value)
            cap = cv2.VideoCapture(path)
            if not cap.isOpened():
                self.get_logger().error(f"视频打不开：{path}，改用 synthetic 合成画面")
                source = "synthetic"
        t0 = time.monotonic()
        while not self._capture_stop.is_set() and rclpy.ok():
            if source == "video":
                ok, frame = cap.read()
                if not ok:  # 播到结尾就回到开头循环播放
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
            else:
                frame = self._synthetic_frame(time.monotonic() - t0)
            self._queue.put_latest(FramePacket(frame, time.monotonic()))
            time.sleep(period)

    def _synthetic_frame(self, t: float) -> np.ndarray:
        """合成一帧 640x480 画面：灰色房间背景 + 左右往返移动的红色目标。

        目标水平位置按正弦摆动，模拟"行人在画面里走来走去"，
        让 HSV 后端能检出一个 cx 随时间变化的目标——第 5 课验证
        "目标左右移动时角速度方向正确"就靠它。
        """
        w, h = 640, 480
        frame = np.full((h, w, 3), 96, dtype=np.uint8)
        # 地平线和"墙角"线，让画面不至于完全空白
        cv2.line(frame, (0, int(h * 0.72)), (w, int(h * 0.72)), (70, 70, 70), 2)
        period = max(1.0, float(self.get_parameter("synthetic_period_sec").value))
        amp = float(self.get_parameter("synthetic_amplitude").value)
        cx_norm = 0.5 + amp * math.sin(2.0 * math.pi * t / period)
        bw, bh = int(w * 0.16), int(h * 0.42)
        cx, cy = int(cx_norm * w), int(h * 0.52)
        # 红色矩形当"人"，头部画个圆增加辨识度
        cv2.rectangle(frame, (cx - bw // 2, cy - bh // 2), (cx + bw // 2, cy + bh // 2), (0, 0, 200), -1)
        cv2.circle(frame, (cx, cy - bh // 2 - 18), 16, (0, 0, 200), -1)
        return frame

    # ------------------------------------------------------------------
    # 推理线程
    # ------------------------------------------------------------------

    def _inference_loop(self) -> None:
        """推理主循环：取最新帧 -> 推理 -> 发布。独立线程，永不阻塞 ROS 回调。"""
        while rclpy.ok():
            packet = self._queue.get(timeout=0.5)
            if packet is None:
                continue
            t0 = time.monotonic()
            try:
                detections = self._backend.detect(packet.frame) # 有可能是有多个目标
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"推理失败：{exc}")
                continue
            infer_ms = (time.monotonic() - t0) * 1000.0

            msg = Detection2DArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_optical_frame"
            msg.backend = self._backend.name
            msg.inference_ms = float(infer_ms)
            for label, conf, cx, cy, bw, bh in detections:  # 分不同的目标画出框框 标注
                det = Detection2D()
                det.label = label # person/bus
                det.confidence = float(conf)
                det.cx, det.cy, det.width, det.height = float(cx), float(cy), float(bw), float(bh)
                msg.detections.append(det)
            self.det_pub.publish(msg)

            if self.img_pub is not None:
                self._publish_debug_image(packet.frame, detections)

            with self._metrics_lock:
                self._frames_done += 1
                self._inference_ms_last = infer_ms

    def _publish_debug_image(self, frame: np.ndarray, detections: List[DetTuple]) -> None:
        """把检测框画到图上发布，供 rqt_image_view / RViz 查看。"""
        canvas = frame.copy()
        h, w = canvas.shape[:2]
        for label, conf, cx, cy, bw, bh in detections:
            x1 = int((cx - bw / 2) * w)
            y1 = int((cy - bh / 2) * h)
            x2 = int((cx + bw / 2) * w)
            y2 = int((cy + bh / 2) * h)
            cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                canvas, f"{label} {conf:.2f}", (x1, max(14, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2,
            )
        out = Image()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "camera_optical_frame"
        out.height, out.width = h, w
        out.encoding = "bgr8"
        out.step = w * 3
        out.data = canvas.tobytes()
        self.img_pub.publish(out)

    def _publish_metrics(self) -> None:
        """每秒发布一次运行指标：处理帧率、单帧耗时、累计丢帧。"""
        with self._metrics_lock:
            fps = self._frames_done
            self._frames_done = 0
            infer_ms = self._inference_ms_last
        payload = {
            "backend": self._backend.name,
            "fps": fps,
            "inference_ms": round(infer_ms, 2),
            "dropped_frames": self._queue.dropped,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.metrics_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
