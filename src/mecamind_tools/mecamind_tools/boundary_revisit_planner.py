"""边界回访规划器 —— 针对建好的地图生成"补扫路线"的离线命令行工具。

【这个文件是干什么的】
自动建图结束后，地图四周的墙体（尤其是外圈边界）常常扫得不完整：
激光只从远处扫过一次的墙会显得断断续续、甚至整段缺失。本工具
读取已保存的地图（map.yaml + .pgm），调用 map_quality_analyzer
评估四个方向（north/south/east/west）的墙体完整度，找出"薄弱边"，
然后为每条薄弱边生成一串贴边行走的回访航点（waypoint），输出成
可供 waypoint 巡航节点使用的 YAML 路线文件。

【注意】这是一个纯离线工具，不是 ROS 节点：
  - 不订阅/发布任何 topic；
  - 输入：地图 yaml 路径 + Gazebo world 文件路径（用来获取房间的
    真实边界坐标，作为回访航点的参考基准）；
  - 输出：stdout 打印路线（YAML 或 JSON），也可用 --output 写入文件。

【总体思路】
  1. analyze_map 对比"地图上检出的墙"与"world 中的真实墙"，
     给出缺失方向列表 missing_walls 和整体质量评级 verdict；
  2. 对每条缺失边，用 _route_for_side 生成人工设计的贴边航点序列
     （在墙内侧留出安全边距，途中带绕障中转点）；
  3. 可选拼接原有巡航路线（--include-base-route），去重后输出。

【初学者重点阅读】
  - build_revisit_route：整体流程的编排；
  - _route_for_side：航点是怎么根据房间边界几何算出来的；
  - _dedupe_waypoints：相邻重复航点的距离去重。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, Iterable, List

import yaml

from .map_quality_analyzer import analyze_map
from .world_geometry import load_world_geometry


def _route_for_side(side: str, min_x: float, max_x: float, min_y: float, max_y: float) -> List[Dict[str, float | str]]:
    """为指定的一条边（north/south/east/west）生成回访航点序列。

    入参 min_x/max_x/min_y/max_y 是从 world 文件解析出的房间外边界
    （世界坐标，单位米）。航点并非贴着墙走，而是向房间内侧缩进一段
    安全距离，让机器人既能用激光近距离补扫墙面，又不会蹭墙。

    每个航点是一个 dict：name 便于日志排查；x/y 是目标坐标；可选的
    yaw 让机器人到点后转向墙面（如 1.57≈+90° 朝北），配合 dwell_sec
    停留 1 秒，给 SLAM 留出稳定观测这段墙的时间。
    没有 yaw/dwell 的点只是绕开家具的"路过点"（bypass）。
    """
    # inset：常规缩进 0.70m；low_inset：南侧局部只缩 0.42m（那里有
    # 柜子等障碍，需要更贴边才能扫到墙根）；dwell_sec：观测停留时长
    inset = 0.70
    low_inset = 0.42
    dwell_sec = 1.0
    # 由边界坐标推出四条"巡墙线"的位置：
    # 例如 west_x 是贴西墙走时机器人的 x 坐标（西墙 + 缩进）
    west_x = min_x + inset
    east_x = max_x - inset
    north_y = max_y - inset
    south_y = min_y + inset
    south_low_y = min_y + low_inset
    # 以下各分支的航点顺序是针对本项目三居室场景手工设计的：
    # 观测点（带 yaw+dwell）之间穿插了绕开桌子、柱子、柜子的中转点
    if side == "north":
        # 北边路线：从东北角开始沿北墙向西补扫（yaw=1.57 即面朝北墙）。
        # 中段先折返到房间中部/入口大厅，再从西侧绕过桌子上到西北角，
        # 这是为了避开北墙中西部的家具，保证路径可通行。
        return [
            {"name": "revisit_north_east", "x": east_x, "y": north_y, "yaw": 1.57, "dwell_sec": dwell_sec},
            {"name": "revisit_north_mid", "x": 0.0, "y": north_y, "yaw": 1.57, "dwell_sec": dwell_sec},
            {"name": "north_to_center_corridor", "x": 0.0, "y": -1.20},
            {"name": "north_to_entry_hall", "x": -2.80, "y": -1.80},
            {"name": "north_west_table_bypass_lower", "x": west_x, "y": 0.60},
            {"name": "north_west_table_bypass_upper", "x": west_x, "y": 2.20, "yaw": 1.57, "dwell_sec": dwell_sec},
            {"name": "revisit_north_west", "x": -2.10, "y": north_y, "yaw": 1.57, "dwell_sec": dwell_sec},
            {"name": "north_west_exit_bypass_upper", "x": west_x, "y": 2.20},
            {"name": "north_west_exit_bypass_lower", "x": west_x, "y": 0.60},
            {"name": "north_west_exit_entry_hall", "x": -2.80, "y": -1.80},
        ]
    if side == "south":
        # 南边路线：自西向东，yaw=-1.57 面朝南墙。中间的 bypass 点
        # 用于绕过南侧的柱子和柜子（先抬高 y 绕柱，再压低 y 贴柜扫墙根）
        return [
            {"name": "revisit_south_west", "x": west_x, "y": south_y, "yaw": -1.57, "dwell_sec": dwell_sec},
            {"name": "south_pillar_left", "x": 0.0, "y": south_y, "yaw": -1.57, "dwell_sec": dwell_sec},
            {"name": "south_pillar_upper_bypass", "x": 1.10, "y": south_y},
            {"name": "south_cabinet_lower_bypass", "x": 1.10, "y": south_low_y},
            {"name": "revisit_south_east", "x": east_x, "y": south_low_y, "yaw": -1.57, "dwell_sec": dwell_sec},
        ]
    if side == "east":
        # 东边路线：南→中→北三个观测点，yaw=0 面朝东墙（+x 方向）
        return [
            {"name": "revisit_east_south", "x": east_x, "y": south_low_y, "yaw": 0.0, "dwell_sec": dwell_sec},
            {"name": "revisit_east_mid", "x": east_x, "y": 0.0, "yaw": 0.0, "dwell_sec": dwell_sec},
            {"name": "revisit_east_north", "x": east_x, "y": north_y, "yaw": 0.0, "dwell_sec": dwell_sec},
        ]
    if side == "west":
        # 西边路线：先在东南侧取一个进入点拉开距离，再沿西墙
        # 南→中→北扫描，yaw=3.14≈180° 面朝西墙（-x 方向）
        return [
            {"name": "west_south_entry_clearance", "x": east_x, "y": south_low_y, "yaw": 3.14, "dwell_sec": dwell_sec},
            {"name": "revisit_west_south", "x": west_x, "y": south_y, "yaw": 3.14, "dwell_sec": dwell_sec},
            {"name": "revisit_west_mid", "x": west_x, "y": 0.0, "yaw": 3.14, "dwell_sec": dwell_sec},
            {"name": "revisit_west_north", "x": west_x, "y": north_y, "yaw": 3.14, "dwell_sec": dwell_sec},
        ]
    return []


def _load_base_waypoints(route_path: str | Path | None) -> List[Dict[str, Any]]:
    """从既有的巡航路线 YAML 里读出 waypoints 列表。

    用于 --include-base-route：把日常巡航路线拼在回访路线前面，
    这样一次任务既完成巡逻又顺带补扫边界。路径为空时返回空列表。
    """
    if not route_path:
        return []
    with Path(route_path).expanduser().open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    waypoints = config.get("waypoints", [])
    if not isinstance(waypoints, list):
        raise ValueError(f"Base route has no waypoint list: {route_path}")
    return [dict(item) for item in waypoints if isinstance(item, dict)]


def _dedupe_waypoints(waypoints: Iterable[Dict[str, Any]], min_spacing: float) -> List[Dict[str, Any]]:
    """去掉与**前一个**航点距离小于 min_spacing 的点。

    拼接多段路线时接缝处容易出现两个几乎重合的点，机器人会在
    原地"到点—再到点"地抖动。只和上一个保留点比较（而不是全局
    去重）是有意的：路线后段可以合法地重新经过前面走过的位置。
    """
    result: List[Dict[str, Any]] = []
    for item in waypoints:
        x = float(item["x"])
        y = float(item["y"])
        duplicate = False
        if result:
            old = result[-1]
            dx = x - float(old["x"])
            dy = y - float(old["y"])
            # 欧氏距离小于间距阈值即视为重复点
            if (dx * dx + dy * dy) ** 0.5 < min_spacing:
                duplicate = True
        if not duplicate:
            result.append(item)
    return result


def build_revisit_route(
    map_yaml: str | Path,
    world_path: str | Path,
    base_route: str | Path | None = None,
    include_base_route: bool = False,
    include_all_sides: bool = False,
    min_spacing: float = 0.20,
) -> Dict[str, object]:
    """核心入口：分析地图质量并生成完整的回访路线字典。

    流程：
      1. analyze_map 评估地图：返回缺失墙体方向 missing_walls
         和质量结论 verdict；
      2. load_world_geometry 读取 world 文件里房间的真实外边界，
         作为航点坐标的几何基准；
      3. 确定要回访哪些边 → 逐边生成航点 → 可选拼接基础路线 → 去重。

    返回的 dict 除了 waypoints 外还带上来源、质量结论等元信息，
    方便写入 YAML 后追溯这条路线是根据哪张地图生成的。
    """
    quality = analyze_map(map_yaml, world_path)
    world = load_world_geometry(world_path)
    missing = list(quality.get("missing_walls", []))
    # 兜底策略：分析器没有明确指出缺失边、但整体质量又不达标时，
    # 默认回访 north 和 east（经验上最容易扫弱的两条边），
    # 保证工具总能给出一条有意义的路线而不是空路线。
    if not missing and quality.get("verdict") != "commercial_ready":
        missing = ["north", "east"]

    revisit_waypoints: List[Dict[str, float | str]] = []
    # include_all_sides 时四边全走；否则只走缺失边。
    # 用固定顺序 east→north→south→west 重排，使多边回访的
    # 行走路线首尾衔接更顺（少走回头路），且输出结果可复现。
    selected_sides = ("east", "north", "south", "west") if include_all_sides else tuple(missing)
    ordered_missing = [side for side in ("east", "north", "south", "west") if side in selected_sides]
    for side in ordered_missing:
        revisit_waypoints.extend(_route_for_side(side, world.min_x, world.max_x, world.min_y, world.max_y))
    base_waypoints = _load_base_waypoints(base_route) if include_base_route else []
    # 基础巡航路线在前、回访航点在后，拼接后做相邻去重
    waypoints = _dedupe_waypoints([*base_waypoints, *revisit_waypoints], min_spacing=min_spacing)
    return {
        "source_map": str(map_yaml),
        "world": str(world_path),
        "base_route": str(base_route) if base_route else "",
        "quality_verdict": quality["verdict"],
        "missing_walls": missing,
        "include_all_sides": include_all_sides,
        "revisit_waypoint_count": len(revisit_waypoints),
        "waypoints": waypoints,
    }


def main(argv: Iterable[str] | None = None) -> int:
    """命令行入口：解析参数 → 生成路线 → 打印/写文件。

    典型用法：
      python3 -m mecamind_tools.boundary_revisit_planner map.yaml \\
          --world three_room.world --output revisit_route.yaml
    """
    parser = argparse.ArgumentParser(description="Generate a MecaMind boundary revisit route.")
    parser.add_argument("map_yaml")
    parser.add_argument("--world", required=True)
    parser.add_argument("--base-route", default="")
    parser.add_argument("--include-base-route", action="store_true")
    parser.add_argument("--include-all-sides", action="store_true")
    parser.add_argument("--min-spacing", type=float, default=0.20)
    parser.add_argument("--output", default="")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(list(argv) if argv is not None else None)

    route = build_revisit_route(
        args.map_yaml,
        args.world,
        base_route=args.base_route or None,
        include_base_route=args.include_base_route,
        include_all_sides=args.include_all_sides,
        min_spacing=args.min_spacing,
    )
    # --output 写文件与 stdout 打印互不影响；--json 只改变打印格式
    if args.output:
        Path(args.output).write_text(yaml.safe_dump(route, sort_keys=False, allow_unicode=True), encoding="utf-8")
    print(json.dumps(route, ensure_ascii=False, indent=2) if args.json else yaml.safe_dump(route, sort_keys=False, allow_unicode=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
