from pathlib import Path
import json

from geometry_msgs.msg import Twist

from mecamind_tools.aliyun_clients import (
    AliyunLlmClient,
    build_task_parser_messages,
    extract_recognition_text,
    load_api_key_from_config,
    parse_task_plan_payload,
)
from mecamind_tools.aliyun_speech_nodes import (
    build_microphone_record_command,
    build_playback_command,
    classify_voice_failure,
    parse_audio_file_payload,
    record_microphone_clip,
    select_microphone_backend,
    select_playback_backend,
)
from mecamind_tools.boundary_revisit_planner import build_revisit_route
from mecamind_tools.mode_profiles import build_mode_profile, render_profile_summary
from mecamind_tools.map_asset_manager import collect_map_assets
from mecamind_tools.map_quality_analyzer import analyze_map
from mecamind_tools.mapping_route_driver import MecaMindMappingRouteDriver, build_route_report
from mecamind_tools.perception_filter import parse_detections, select_detection
from mecamind_tools.runbook import build_mission_brief_payload
from mecamind_tools.project_state_machine import ProjectEvent, ProjectMode, ProjectStateMachine
from mecamind_tools.runbook import build_boot_checklist, render_boot_checklist
from mecamind_tools.safety_layer import analyze_scan, command_timed_out, gate_twist
from mecamind_tools.sim_core import SimPose, MecaMindSimModel
from mecamind_tools.fake_detection_publisher import build_detection_payload
from mecamind_tools.mission_executor import (
    load_named_goals,
    move_velocity_for_direction,
    next_mission_mode,
    resolve_named_goal,
)
from mecamind_tools.task_scheduler import parse_move_direction, parse_task_command
from mecamind_tools.voice_listen_node import (
    pcm_rms,
    calibrated_energy_threshold,
    strip_wake_words,
    text_contains_wake_word,
)
from mecamind_tools.vision_follow_controller import (
    compute_avoidance_bias,
    compute_follow_command,
    estimate_target_distance,
    front_distance_ignoring_target,
    search_turn_direction,
    should_publish_follow,
)
from mecamind_tools.follow_target_mover import (
    FOLLOW_DEMO_OBSTACLES,
    FOLLOW_DEMO_WAYPOINTS,
    profile_cycle_sec,
    profile_speed_at,
)
from mecamind_tools.world_geometry import load_world_geometry
from mecamind_tools.world_profiles import three_room_blueprint, summarize_blueprint, validate_three_room_blueprint

PKG = Path(__file__).resolve().parents[1]


def test_mode_profiles_switch_sim_and_real():
    sim = build_mode_profile("sim", mission="mapping")
    real = build_mode_profile("real", mission="navigation")
    assert sim.use_sim_time
    assert not sim.expects_gazebo
    assert "built-in 2D simulator" in " ".join(sim.notes)
    assert not real.use_sim_time
    assert not real.expects_gazebo
    assert "scan_topic" in render_profile_summary(sim)


def test_three_room_blueprint_is_valid():
    blueprint = three_room_blueprint()
    assert validate_three_room_blueprint(blueprint)
    summary = summarize_blueprint(blueprint)
    assert "rooms=3" in summary
    assert "obstacles=" in summary


def test_state_machine_handles_mapping_navigation_and_estop():
    machine = ProjectStateMachine()
    assert machine.handle(ProjectEvent.START_SIM).current == ProjectMode.SIM_MAPPING
    assert machine.handle(ProjectEvent.BEGIN_NAVIGATION).current == ProjectMode.SIM_NAVIGATION
    assert machine.handle(ProjectEvent.ESTOP).current == ProjectMode.ESTOP
    assert not machine.handle(ProjectEvent.BEGIN_NAVIGATION).accepted
    assert machine.handle(ProjectEvent.RESET_ESTOP).current == ProjectMode.BOOT


def test_boot_checklist_mentions_core_topics():
    profile = build_mode_profile("sim", mission="mapping")
    blueprint = three_room_blueprint()
    items = build_boot_checklist(profile, blueprint)
    text = render_boot_checklist(items)
    assert "/scan_raw" in text
    assert "/imu/data_raw" in text
    assert "/controller/cmd_vel" in text


def test_world_geometry_and_sim_model_step_forward():
    world_path = Path(__file__).resolve().parents[1] / "worlds" / "three_room_house.world"
    geometry = load_world_geometry(world_path)
    assert geometry.name == "three_room_house"
    assert len(geometry.boxes) >= 5
    front_hit = geometry.raycast(-4.0, -1.8, 0.0, 10.0)
    assert front_hit > 0.0

    model = MecaMindSimModel(geometry, initial_pose=SimPose(-3.2, -2.4, 0.0))
    model.set_command(0.30, 0.0, 0.0)
    before_x = model.pose.x
    model.step(0.5)
    assert model.pose.x > before_x
    assert len(model.scan_ranges()) == model.lidar_beams


def test_sim_model_can_degrade_lidar_for_hardware_gap_analysis():
    world_path = Path(__file__).resolve().parents[1] / "worlds" / "three_room_house.world"
    geometry = load_world_geometry(world_path)

    clean = MecaMindSimModel(
        geometry,
        initial_pose=SimPose(-3.2, -2.4, 0.0),
        lidar_beams=12,
        lidar_range_max=8.0,
    ).scan_ranges()
    biased = MecaMindSimModel(
        geometry,
        initial_pose=SimPose(-3.2, -2.4, 0.0),
        lidar_beams=12,
        lidar_range_max=8.0,
        lidar_range_bias=0.10,
    ).scan_ranges()
    dropout = MecaMindSimModel(
        geometry,
        initial_pose=SimPose(-3.2, -2.4, 0.0),
        lidar_beams=12,
        lidar_range_max=8.0,
        lidar_dropout_prob=1.0,
    ).scan_ranges()

    assert any(abs(a - b) > 0.05 for a, b in zip(clean, biased))
    assert all(value == 8.0 for value in dropout)


def test_mission_brief_payload_preserves_world_path():
    profile = build_mode_profile("sim", mission="mapping")
    blueprint = three_room_blueprint()
    payload = build_mission_brief_payload(profile, blueprint, "/tmp/mecamind.world")
    assert payload["world_path"] == "/tmp/mecamind.world"
    assert payload["profile"]["name"] == "sim"
    assert "checklist_text" in payload


def test_map_quality_analyzer_accepts_complete_world_map(tmp_path):
    world_path = Path(__file__).resolve().parents[1] / "worlds" / "three_room_house.world"
    geometry = load_world_geometry(world_path)
    spec = geometry.build_occupancy_grid(resolution=0.05, padding=0.20)

    pgm_path = tmp_path / "complete_world_map.pgm"
    yaml_path = tmp_path / "complete_world_map.yaml"
    pixels = [0 if value == 100 else 254 for value in spec.data]
    pgm_path.write_bytes(
        f"P5\n{spec.width} {spec.height}\n255\n".encode("ascii")
        + bytes(pixels)
    )
    yaml_path.write_text(
        "\n".join(
            [
                "image: complete_world_map.pgm",
                "resolution: 0.05",
                f"origin: [{spec.origin_x}, {spec.origin_y}, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.25",
            ]
        ),
        encoding="utf-8",
    )

    result = analyze_map(yaml_path, world_path)
    assert result["verdict"] == "commercial_ready"
    assert result["missing_walls"] == []
    assert result["coverage"] == 1.0


def test_safety_layer_stops_forward_motion_when_front_blocked():
    ranges = [1.0] * 36
    ranges[18] = 0.15
    decision = analyze_scan(ranges, -3.14159, 2 * 3.14159 / 36)
    cmd = Twist()
    cmd.linear.x = 0.3
    gated = gate_twist(cmd, decision)
    assert decision.level == "stop"
    assert gated.linear.x == 0.0


def test_safety_layer_command_watchdog_only_stops_active_stale_commands():
    assert command_timed_out(10.0, 10.7, 0.6, True)
    assert not command_timed_out(10.0, 10.5, 0.6, True)
    assert not command_timed_out(10.0, 20.0, 0.6, False)


def test_perception_filter_selects_confirmable_target():
    detections = parse_detections(
        '{"detections":[{"label":"chair","confidence":0.9},'
        '{"label":"person","confidence":0.72,"bbox":{"cx":0.6,"cy":0.5,"width":0.2}}]}'
    )
    target = select_detection(detections, "person", 0.55)
    assert target is not None
    assert target.label == "person"
    assert target.cx == 0.6


def test_vision_follow_command_turns_toward_target():
    command = compute_follow_command(cx=0.70, target_width=0.10, dt=0.1)
    assert command.active
    assert command.angular_z < 0.0
    assert command.linear_x > 0.0


def test_vision_follow_pid_integral_grows_on_steady_error():
    state_ang = None
    state_lin = None
    last = None
    for _ in range(5):
        last = compute_follow_command(
            cx=0.70,
            target_width=0.10,
            dt=0.1,
            ki_angular=0.2,
            ki_linear=0.1,
            angular_state=state_ang,
            linear_state=state_lin,
        )
        state_ang = last.angular_state
        state_lin = last.linear_state
    assert last is not None
    assert abs(last.angular_state.integral) > 0.0
    assert abs(last.linear_state.integral) > 0.0


def test_vision_follow_speed_scales_with_distance_and_caps():
    # 直径 0.2m 的柱子：宽度 0.35 ≈ 0.5m（比期望 0.8m 近）；宽度 0.1 ≈ 1.7m（远）
    near = compute_follow_command(cx=0.5, target_width=0.35, dt=0.1, max_linear=0.35)
    mid = compute_follow_command(cx=0.5, target_width=0.15, dt=0.1, max_linear=0.35)
    far = compute_follow_command(cx=0.5, target_width=0.05, dt=0.1, max_linear=0.35)
    # 比期望距离近 → 后退；越远 → 越快；再远也不超过最大限速
    assert near.linear_x < 0.0
    assert far.linear_x >= mid.linear_x > 0.0
    assert far.linear_x <= 0.35


def test_estimate_target_distance_matches_pinhole_model():
    # f_norm≈0.866：宽度 0.2 → 距离≈0.87m；宽度越小距离越远；有上限截断
    assert abs(estimate_target_distance(0.20) - 0.866) < 0.01
    assert estimate_target_distance(0.10) > estimate_target_distance(0.20)
    assert estimate_target_distance(0.0001) == 6.0


def test_front_avoidance_ignores_target_own_echo():
    # 正前回波 0.6m ≈ 视觉估的目标距离 0.65m → 是红柱本身，豁免（inf）
    assert front_distance_ignoring_target(0.60, 0.65) == float("inf")
    # 回波比目标近得多（0.4 < 0.9-0.3）→ 真障碍挡在中间，不豁免
    assert front_distance_ignoring_target(0.40, 0.90) == 0.40
    # 没有目标距离信息 → 原样返回
    assert front_distance_ignoring_target(0.40, None) == 0.40


def test_vision_follow_requires_enable_flag():
    assert not should_publish_follow(enabled=False, confirmed=True)
    assert not should_publish_follow(enabled=True, confirmed=False)
    assert should_publish_follow(enabled=True, confirmed=True)


def test_follow_avoidance_bias_pushes_away_from_side_obstacle():
    # 右前方近障 → 向左转（正 angular）+ 向左横移（正 lateral），前进不减速
    bias = compute_avoidance_bias(left_dist=2.0, right_dist=0.4, front_dist=2.0)
    assert bias.angular > 0.0
    assert bias.lateral > 0.0
    assert bias.linear_scale == 1.0
    # 左前方近障 → 全部反向
    bias = compute_avoidance_bias(left_dist=0.4, right_dist=2.0, front_dist=2.0)
    assert bias.angular < 0.0
    assert bias.lateral < 0.0
    # 三个方向都空 → 完全不干预
    bias = compute_avoidance_bias(left_dist=3.0, right_dist=3.0, front_dist=3.0)
    assert bias.angular == 0.0
    assert bias.lateral == 0.0
    assert bias.linear_scale == 1.0


def test_follow_avoidance_bias_slows_and_turns_when_front_blocked():
    # 正前贴脸、右侧更空 → 明显减速 + 向右转（负 angular）
    bias = compute_avoidance_bias(left_dist=0.5, right_dist=2.0, front_dist=0.35)
    assert bias.linear_scale < 0.5
    assert bias.angular < 0.0
    # 正前到 stop 线 → 前进完全归零
    bias = compute_avoidance_bias(left_dist=2.0, right_dist=2.0, front_dist=0.25)
    assert bias.linear_scale == 0.0


def test_follow_search_direction_follows_last_seen_side():
    # 目标最后在画面右半区 → 顺时针（向右，负方向）弧线找
    assert search_turn_direction(0.8) == -1.0
    # 左半区 → 逆时针
    assert search_turn_direction(0.2) == 1.0


def _point_to_segment_dist(px, py, ax, ay, bx, by):
    """点到线段的最短距离（测试辅助）。"""
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-12:
        return ((px - ax) ** 2 + (py - ay) ** 2) ** 0.5
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len_sq))
    cx, cy = ax + t * dx, ay + t * dy
    return ((px - cx) ** 2 + (py - cy) ** 2) ** 0.5


def test_follow_demo_obstacles_leave_corridor_for_robot():
    """红柱回路到每个障碍物的最小间距必须 >= 0.8m（车宽 + 绕行余量），
    否则小车跟在红柱后面会被挤进过不去的缝里——这正是之前"卡箱子"的教训。"""
    loop = list(FOLLOW_DEMO_WAYPOINTS) + [FOLLOW_DEMO_WAYPOINTS[0]]
    for name, (ox, oy, sx, sy) in FOLLOW_DEMO_OBSTACLES.items():
        half_diag = 0.5 * (sx * sx + sy * sy) ** 0.5
        min_clear = min(
            _point_to_segment_dist(ox, oy, *loop[i], *loop[i + 1]) - half_diag
            for i in range(len(loop) - 1)
        )
        assert min_clear >= 0.8, f"{name} 距红柱路径仅 {min_clear:.2f}m，不够小车通行"


def test_follow_target_speed_profile_trapezoid():
    kwargs = dict(
        slow_mps=0.09, fast_mps=0.23, accel_mps2=0.18, hold_sec=5.0
    )
    cycle = profile_cycle_sec(**kwargs)
    # 慢持 → 加速中点 → 快持 → 减速中点
    assert abs(profile_speed_at(0.5, **kwargs) - 0.09) < 1e-6
    assert abs(profile_speed_at(5.0 + (0.14 / 0.18) * 0.5, **kwargs) - 0.16) < 1e-3
    assert abs(profile_speed_at(5.0 + 0.14 / 0.18 + 1.0, **kwargs) - 0.23) < 1e-6
    mid_down = 5.0 + 0.14 / 0.18 + 5.0 + (0.14 / 0.18) * 0.5
    assert abs(profile_speed_at(mid_down, **kwargs) - 0.16) < 1e-3
    assert abs(profile_speed_at(cycle + 0.5, **kwargs) - 0.09) < 1e-6


def test_named_goals_resolve_aliases_and_reject_unknown():
    goals = load_named_goals(str(PKG / "config" / "mecamind_named_goals.yaml"))
    assert "bedroom" in goals
    assert resolve_named_goal(goals, "bedroom")["x"] == 3.75
    assert resolve_named_goal(goals, "卧室")["x"] == 3.75
    assert resolve_named_goal(goals, "nowhere") is None


def test_mission_mode_transitions_from_intent():
    assert next_mission_mode("navigate") == "navigate"
    assert next_mission_mode("patrol") == "patrol"
    assert next_mission_mode("follow") == "follow"
    assert next_mission_mode("move") == "move"
    assert next_mission_mode("stop") == "idle"
    assert next_mission_mode("cancel") == "idle"
    assert next_mission_mode("mapping") == "idle"


def test_move_velocity_for_direction():
    assert move_velocity_for_direction("forward", 0.15, 0.6) == (0.15, 0.0)
    assert move_velocity_for_direction("backward", 0.15, 0.6) == (-0.15, 0.0)
    # 左转是逆时针（angular.z 正值），右转反之；速度取绝对值防呆
    assert move_velocity_for_direction("left", 0.15, -0.6) == (0.0, 0.6)
    assert move_velocity_for_direction("right", 0.15, 0.6) == (0.0, -0.6)
    assert move_velocity_for_direction("sideways", 0.15, 0.6) is None


def test_fake_detection_payload_is_filter_compatible():
    payload = build_detection_payload("person", 0.9, 0.7, 0.5, 0.25, 0.4)
    detections = parse_detections(payload)
    target = select_detection(detections, "person", 0.55)
    assert target is not None
    assert target.cx == 0.7
    assert target.width == 0.25


def test_task_scheduler_parses_basic_voice_intents():
    assert parse_task_command("stop now").intent == "stop"
    assert parse_task_command("开始巡逻").intent == "patrol"
    nav = parse_task_command("go to bedroom")
    assert nav.intent == "navigate"
    assert nav.target == "bedroom"
    confirm = parse_task_command("确认")
    assert confirm.intent == "confirm"
    cancel = parse_task_command("取消")
    assert cancel.intent == "cancel"
    unknown = parse_task_command("随便做点什么")
    assert unknown.intent == "unknown"
    assert unknown.requires_confirmation
    assert "确认" in unknown.reply


def test_task_scheduler_parses_move_directions():
    # 语音点动指令：前进/后退/左转/右转直接映射 move 意图
    for text, direction in (
        ("前进", "forward"),
        ("向前走", "forward"),
        ("后退", "backward"),
        ("退后一点", "backward"),
        ("倒车", "backward"),
        ("左转", "left"),
        ("向左", "left"),
        ("右转", "right"),
        ("turn right", "right"),
    ):
        cmd = parse_task_command(text)
        assert cmd.intent == "move", text
        assert cmd.target == direction, text
        assert not cmd.requires_confirmation
        assert cmd.reply
    # 停止优先级必须高于方向词："停止后退" 应理解为停止
    assert parse_task_command("停止后退").intent == "stop"
    assert parse_move_direction("随便聊聊") == ""


def test_calibrated_energy_threshold():
    # 底噪中位数 500，margin 2.5 -> 门限 1250（高于 floor 450）
    samples = [400.0, 500.0, 600.0, 480.0, 520.0]
    assert calibrated_energy_threshold(samples, 450.0, 2.5) == 500.0 * 2.5
    # 安静环境：中位数很低时不低于 floor
    assert calibrated_energy_threshold([10.0, 20.0, 30.0], 450.0, 2.5) == 450.0
    # 无样本时退回 floor
    assert calibrated_energy_threshold([], 450.0, 2.5) == 450.0


def test_wake_word_helpers_and_pcm_energy():
    wake_words = ["小度小度", "小度", "小杜", "小渡", "小肚"]
    # 标准叫法与带指令的连读
    assert text_contains_wake_word("小度小度去卧室", wake_words)
    assert strip_wake_words("小度小度去卧室", wake_words) == "去卧室"
    # ASR 常见转写：唤醒词之间带标点，靠单个「小度」子串兜底
    assert text_contains_wake_word("小度，小度，去卧室", wake_words)
    assert strip_wake_words("小度，小度，去卧室", wake_words) == "去卧室"
    # 同音误转写兜底
    assert text_contains_wake_word("小杜小杜前进", wake_words)
    assert strip_wake_words("小杜小杜前进", wake_words) == "前进"
    assert text_contains_wake_word("小渡去客厅", wake_words)
    assert text_contains_wake_word("小肚跟随", wake_words)
    # 未唤醒的普通句子不应误触发
    assert not text_contains_wake_word("这节课讲麦克纳姆轮", wake_words)
    # 静音帧能量应为 0
    assert pcm_rms(b"\x00\x00" * 80) == 0.0


def test_voice_failure_classification_and_playback_command():
    code, reply = classify_voice_failure(RuntimeError("Aliyun ASR returned no recognized text"))
    assert code == "asr_empty"
    assert "再说" in reply
    backend, executable = select_playback_backend(
        "ffplay",
        which={"ffplay": "/usr/bin/ffplay"}.get,
    )
    assert backend == "ffplay"
    command = build_playback_command(backend, executable, "/tmp/a.mp3")
    assert command[0] == "/usr/bin/ffplay"
    assert command[-1] == "/tmp/a.mp3"


def test_aliyun_task_payload_parser_accepts_json_fences():
    plan = parse_task_plan_payload(
        '```json\n{"intent":"navigate","target":"kitchen","requires_confirmation":false,"reply":"OK"}\n```'
    )
    assert plan.intent == "navigate"
    assert plan.target == "kitchen"
    assert not plan.requires_confirmation
    assert plan.reply == "OK"
    messages = build_task_parser_messages("go to bedroom")
    assert messages[0]["role"] == "system"
    assert "Allowed intents" in messages[0]["content"]


def test_aliyun_llm_client_uses_openai_compatible_payload(monkeypatch):
    captured = {}

    class FakeResponse:
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            return False

        def read(self):
            body = {
                "choices": [
                    {
                        "message": {
                            "content": json.dumps(
                                {
                                    "intent": "patrol",
                                    "target": "",
                                    "requires_confirmation": False,
                                    "reply": "Starting patrol.",
                                }
                            )
                        }
                    }
                ]
            }
            return json.dumps(body).encode("utf-8")

    def fake_opener(req, timeout):
        captured["url"] = req.full_url
        captured["headers"] = dict(req.header_items())
        captured["payload"] = json.loads(req.data.decode("utf-8"))
        captured["timeout"] = timeout
        return FakeResponse()

    monkeypatch.setenv("DASHSCOPE_API_KEY", "unit-test-key")
    client = AliyunLlmClient(base_url="https://example.invalid/compatible-mode/v1", opener=fake_opener)
    plan = client.parse_task("start patrol")

    assert plan.intent == "patrol"
    assert captured["url"].endswith("/chat/completions")
    assert captured["payload"]["model"] == "qwen-plus"
    assert captured["payload"]["messages"][1]["content"] == "start patrol"
    assert captured["headers"]["Authorization"] == "Bearer unit-test-key"


def test_aliyun_api_key_loads_from_local_yaml(tmp_path):
    config_file = tmp_path / "mecamind_aliyun.local.yaml"
    config_file.write_text(
        'aliyun:\n  api_key: "local-test-key"\n',
        encoding="utf-8",
    )

    assert load_api_key_from_config(config_file=config_file) == "local-test-key"


def test_aliyun_recognition_extracts_text_from_sentence_list():
    sentences = [
        {"sentence_id": 1, "text": "去卧室"},
        {"sentence_id": 2, "text": "然后停止"},
    ]

    assert extract_recognition_text(sentences) == "去卧室然后停止"


def test_aliyun_audio_file_payload_accepts_plain_path_and_json():
    assert parse_audio_file_payload("/tmp/mecamind.wav") == "/tmp/mecamind.wav"
    assert parse_audio_file_payload('{"path":"/tmp/mecamind.mp3"}') == "/tmp/mecamind.mp3"


def test_microphone_backend_prefers_wslg_pulse_and_builds_wav_command():
    executables = {"ffmpeg": "/usr/bin/ffmpeg", "arecord": "/usr/bin/arecord"}
    backend, executable = select_microphone_backend(
        "auto",
        environment={"PULSE_SERVER": "unix:/mnt/wslg/PulseServer"},
        which=executables.get,
    )
    assert backend == "ffmpeg_pulse"
    command = build_microphone_record_command(
        backend,
        executable,
        "/tmp/mecamind_mic.wav",
        "default",
        4.0,
        16000,
        1,
    )
    assert command[0] == "/usr/bin/ffmpeg"
    assert command[-1] == "/tmp/mecamind_mic.wav"
    assert command[command.index("-f") + 1] == "pulse"
    assert command[command.index("-ar") + 1] == "16000"


def test_microphone_clip_requires_usable_recording(tmp_path):
    output_path = tmp_path / "microphone.wav"
    captured = {}

    def fake_runner(command, **kwargs):
        captured["command"] = command
        captured["timeout"] = kwargs["timeout"]
        output_path.write_bytes(b"RIFF" + bytes(100))
        return None

    result = record_microphone_clip(
        ["fake-recorder", str(output_path)],
        output_path,
        timeout_sec=9.0,
        runner=fake_runner,
    )
    assert result == output_path
    assert captured["command"][0] == "fake-recorder"
    assert captured["timeout"] == 9.0


def test_map_asset_manager_and_boundary_revisit_route(tmp_path):
    world_path = Path(__file__).resolve().parents[1] / "worlds" / "three_room_house.world"
    pgm_path = tmp_path / "empty_map.pgm"
    yaml_path = tmp_path / "empty_map.yaml"
    checkpoint_pgm_path = tmp_path / "empty_map_checkpoint_01.pgm"
    checkpoint_yaml_path = tmp_path / "empty_map_checkpoint_01.yaml"
    base_route_path = tmp_path / "base_route.yaml"
    width = 20
    height = 20
    pgm_bytes = f"P5\n{width} {height}\n255\n".encode("ascii") + bytes([254] * width * height)
    pgm_path.write_bytes(pgm_bytes)
    checkpoint_pgm_path.write_bytes(pgm_bytes)
    yaml_path.write_text(
        "\n".join(
            [
                "image: empty_map.pgm",
                "resolution: 0.05",
                "origin: [-1.0, -1.0, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.25",
            ]
        ),
        encoding="utf-8",
    )
    checkpoint_yaml_path.write_text(
        "\n".join(
            [
                "image: empty_map_checkpoint_01.pgm",
                "resolution: 0.05",
                "origin: [-1.0, -1.0, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.25",
            ]
        ),
        encoding="utf-8",
    )
    base_route_path.write_text(
        "\n".join(
            [
                "waypoints:",
                "  - name: base_start",
                "    x: -3.2",
                "    y: -2.4",
            ]
        ),
        encoding="utf-8",
    )

    manifest = collect_map_assets(tmp_path, world_path)
    manifest_with_checkpoints = collect_map_assets(tmp_path, world_path, include_checkpoints=True)
    route = build_revisit_route(
        yaml_path,
        world_path,
        base_route=base_route_path,
        include_base_route=True,
    )
    assert manifest["count"] == 1
    assert manifest_with_checkpoints["count"] == 2
    assert manifest["recommended_map"].endswith("empty_map.yaml")
    assert route["waypoints"]
    assert route["waypoints"][0]["name"] == "base_start"
    assert route["revisit_waypoint_count"] > 0
    assert any("dwell_sec" in item for item in route["waypoints"])
    assert any(item["name"] == "north_west_table_bypass_lower" for item in route["waypoints"])


def test_mapping_route_report_preserves_reached_and_skipped_evidence():
    waypoint_results = [
        {
            "index": 0,
            "name": "room_entry",
            "status": "reached",
            "reason": "goal_tolerance",
        },
        {
            "index": 1,
            "name": "west_wall",
            "status": "skipped",
            "reason": "no_progress",
        },
    ]
    report = build_route_report(
        route_file="route.yaml",
        phase="manual_assist_ready",
        route_size=2,
        waypoint_results=waypoint_results,
        robot_pose={"x": 1.0, "y": 2.0, "yaw": 0.0},
        auto_map_path="auto_map",
        manual_map_path="manual_map",
        auto_map_saved=True,
        manual_map_saved=False,
    )

    assert report["reached_waypoints"] == 1
    assert report["skipped_waypoints"] == 1
    assert not report["all_waypoints_reached"]
    assert report["phase"] == "manual_assist_ready"
    assert report["waypoints"][1]["reason"] == "no_progress"


def test_finished_mapping_route_does_not_override_manual_velocity():
    class FakePublisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    class FinishedRoute:
        _route_finished = True
        cmd_pub = FakePublisher()

        def _publish_status(self):
            return None

    route = FinishedRoute()
    MecaMindMappingRouteDriver._tick(route)
    assert route.cmd_pub.messages == []
