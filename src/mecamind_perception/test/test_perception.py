"""mecamind_perception 的单元测试：只测纯逻辑，不起 ROS 节点。

覆盖三块核心：
1. LatestFrameQueue —— 有界队列的"最新帧"语义与丢帧计数；
2. HsvBackend —— 颜色检测在合成画面上能找到目标且坐标正确；
3. pick_active_source —— 速度仲裁的优先级与新鲜度规则。
"""

import numpy as np

from geometry_msgs.msg import Twist

from mecamind_perception.cmd_vel_arbiter import SourceState, pick_active_source
from mecamind_perception.detector_node import FramePacket, HsvBackend, LatestFrameQueue


# ---------------------------------------------------------------------------
# LatestFrameQueue
# ---------------------------------------------------------------------------


def _packet(value: int) -> FramePacket:
    frame = np.full((2, 2, 3), value, dtype=np.uint8)
    return FramePacket(frame, float(value))


def test_latest_queue_keeps_newest_and_counts_drops():
    q = LatestFrameQueue()
    q.put_latest(_packet(1))
    q.put_latest(_packet(2))  # 挤掉 1
    q.put_latest(_packet(3))  # 挤掉 2
    got = q.get(timeout=0.1)
    assert got is not None and got.stamp_mono == 3.0
    assert q.dropped == 2
    assert q.get(timeout=0.05) is None  # 取空后没有残留


# ---------------------------------------------------------------------------
# HsvBackend
# ---------------------------------------------------------------------------


def _red_target_frame(cx_norm: float) -> np.ndarray:
    """造一帧 200x200：灰底 + 指定水平位置的红色方块（BGR）。"""
    frame = np.full((200, 200, 3), 96, dtype=np.uint8)
    cx = int(cx_norm * 200)
    frame[60:140, max(0, cx - 20):min(200, cx + 20)] = (0, 0, 200)
    return frame


def test_hsv_backend_finds_red_blob_position():
    backend = HsvBackend(label="person", conf_threshold=0.3)
    detections = backend.detect(_red_target_frame(0.7))
    assert len(detections) == 1
    label, conf, cx, cy, w, h = detections[0]
    assert label == "person"
    assert conf >= 0.3
    assert abs(cx - 0.7) < 0.05      # 水平位置应接近目标真值
    assert 0.3 < cy < 0.7            # 垂直大致居中
    assert w > 0.0 and h > 0.0


def test_hsv_backend_empty_frame_returns_nothing():
    backend = HsvBackend(label="person", conf_threshold=0.3)
    frame = np.full((200, 200, 3), 96, dtype=np.uint8)
    assert backend.detect(frame) == []


# ---------------------------------------------------------------------------
# pick_active_source（速度仲裁规则）
# ---------------------------------------------------------------------------


def _state(stamp: float) -> SourceState:
    return SourceState(Twist(), stamp)


def test_arbiter_priority_teleop_beats_follow_and_nav():
    sources = {"teleop": _state(10.0), "follow": _state(10.0), "nav": _state(10.0)}
    assert pick_active_source(sources, now_mono=10.1, timeout_sec=0.5) == "teleop"


def test_arbiter_falls_back_when_higher_source_goes_stale():
    sources = {"teleop": _state(1.0), "follow": _state(10.0), "nav": _state(10.0)}
    # teleop 已过期 -> follow 接管
    assert pick_active_source(sources, now_mono=10.1, timeout_sec=0.5) == "follow"
    # follow 也过期 -> nav 接管
    sources["follow"] = _state(1.0)
    assert pick_active_source(sources, now_mono=10.1, timeout_sec=0.5) == "nav"


def test_arbiter_all_silent_returns_none():
    sources = {"teleop": _state(1.0), "follow": _state(1.0), "nav": _state(1.0)}
    assert pick_active_source(sources, now_mono=10.0, timeout_sec=0.5) is None
    assert pick_active_source({}, now_mono=10.0, timeout_sec=0.5) is None
