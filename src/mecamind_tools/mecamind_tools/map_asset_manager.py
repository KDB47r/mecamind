"""地图资产管理器（MecaMind 第二课：给建好的多张地图"建档 + 选优"）。

这个文件是干什么的？
    一次课程练习下来，maps/ 目录里往往会积累多张地图：自动路线存的、
    手动补扫存的、中途的检查点快照……本工具扫描目录里所有的地图 YAML
    文件，逐张调用 map_quality_analyzer.analyze_map() 打分，最后生成一份
    manifest（JSON 清单）：登记每张地图的质量状态，并按统一规则推荐出
    "最好的那张"，供后续导航课（第三课起）直接引用。

    和 map_quality_analyzer 一样，它是纯离线的命令行工具，不是 ROS 节点，
    不涉及任何 topic。

在建图流程中的角色：
    建图（mapping_route_driver）-> 质检（map_quality_analyzer）->
    建档选优（本文件）。它是流水线的最后一环，输出的 manifest 里的
    recommended_map 字段就是"下一课该加载哪张地图"的答案。

初学者应该重点看的函数：
    - _asset_score()        —— 多级排序键的写法：如何用一个 tuple 表达
                                "先比状态、再比缺墙数、再比覆盖率……"。
    - collect_map_assets()  —— 扫描目录、逐张分析、汇总 manifest 的主流程。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, Iterable, List

import yaml

from .map_quality_analyzer import analyze_map


# 质量状态 -> 排序等级。数字越大越好，用于 _asset_score 的第一优先级比较。
# invalid（文件损坏/解析失败）垫底，commercial_ready（可交付）最高。
_STATUS_RANK = {
    "invalid": 0,
    "needs_remap": 1,
    "navigation_ready_best_effort": 2,
    "commercial_ready": 3,
}


def _looks_like_map_yaml(path: Path) -> bool:
    """粗筛：这个 YAML 像不像 nav2 地图元数据文件？

    地图目录里可能混着其他 YAML（参数文件、路点文件等），地图 YAML 的
    特征是顶层有 "image" 字段。读不出来（损坏/权限问题）时返回 True
    而不是 False——故意把可疑文件放进后续流程，让 analyze_map 报出
    具体错误并以 invalid 状态登记，比悄悄跳过更利于排查。
    """
    try:
        with path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
    except Exception:
        return True
    return isinstance(data, dict) and "image" in data


def _is_checkpoint_map(path: Path) -> bool:
    """按命名约定识别建图途中的检查点快照（见 mapping_route_driver）。

    检查点只是中间产物，默认不登记进 manifest，避免推荐到半成品地图。
    """
    return "_checkpoint_" in path.stem


def _asset_score(asset: Dict[str, object]) -> tuple[int, int, float, float]:
    """给一条地图资产算排序键，供 max(..., key=...) 挑选推荐地图。

    Python 的 tuple 比较是逐元素的，所以这个四元组天然实现了多级
    优先级（对应 manifest 里的 recommendation_rule 字段）：
    1. 状态等级越高越好；
    2. 缺失墙面越少越好（取负数使"少"变成"大"）；
    3. 覆盖率越高越好；
    4. 未知区比例越低越好（同样取负）。
    字段缺失时取最悲观的默认值（缺 missing_walls 记 99 面缺墙），
    保证信息不全的资产不会意外胜出。
    """
    status = str(asset.get("status", "invalid"))
    missing = asset.get("missing_walls", [])
    missing_count = len(missing) if isinstance(missing, list) else 99
    coverage = float(asset.get("coverage", 0.0))
    unknown_ratio = float(asset.get("unknown_ratio", 1.0))
    return (_STATUS_RANK.get(status, 0), -missing_count, coverage, -unknown_ratio)


def collect_map_assets(
    map_dir: str | Path,
    world_path: str | Path | None = None,
    include_checkpoints: bool = False,
) -> Dict[str, object]:
    """扫描地图目录，逐张分析质量，返回完整的 manifest dict。

    流程：
    1. 遍历目录下所有 *.yaml（排序保证输出顺序稳定、可复现）；
    2. 过滤掉检查点快照（除非 include_checkpoints）和明显不是地图的 YAML；
    3. 每张地图交给 analyze_map 打分；分析抛异常不会中断整个扫描，
       而是以 status="invalid" + 错误信息登记，坏文件也留档；
    4. 在有效资产里按 _asset_score 选出推荐地图。

    world_path 传给 analyze_map 用于精确的墙体检测（可选）。
    """
    root = Path(map_dir)
    assets: List[Dict[str, object]] = []
    for yaml_path in sorted(root.glob("*.yaml")):
        if not include_checkpoints and _is_checkpoint_map(yaml_path):
            continue
        if not _looks_like_map_yaml(yaml_path):
            continue
        try:
            report = analyze_map(yaml_path, world_path)
        except Exception as exc:  # noqa: BLE001 - report broken map assets
            assets.append({"map": str(yaml_path), "status": "invalid", "error": str(exc)})
            continue
        # 只摘录做推荐决策需要的字段，manifest 保持精简。
        assets.append(
            {
                "map": str(yaml_path),
                "status": report["verdict"],
                "coverage": report["coverage"],
                "unknown_ratio": report["unknown_ratio"],
                "missing_walls": report["missing_walls"],
            }
        )
    valid_assets = [asset for asset in assets if asset.get("status") != "invalid"]
    recommended = max(valid_assets, key=_asset_score) if valid_assets else None
    return {
        "map_dir": str(root),
        "count": len(assets),
        "recommended_map": recommended["map"] if recommended else "",
        "recommended_status": recommended["status"] if recommended else "",
        # 把推荐规则写进 manifest 本身，读清单的人不用翻源码就知道
        # recommended_map 是怎么选出来的。
        "recommendation_rule": "status > fewer_missing_walls > coverage > lower_unknown_ratio",
        "assets": assets,
    }


def main(argv: Iterable[str] | None = None) -> int:
    """命令行入口：生成 manifest 并打印；--output 可同时写入文件。

    argv 参数允许测试代码直接传入参数列表而不依赖 sys.argv。
    """
    parser = argparse.ArgumentParser(description="Build a MecaMind map asset manifest.")
    parser.add_argument("map_dir")
    parser.add_argument("--world", default=None)
    parser.add_argument("--include-checkpoints", action="store_true")
    parser.add_argument("--output", default="")
    args = parser.parse_args(list(argv) if argv is not None else None)

    manifest = collect_map_assets(args.map_dir, args.world, include_checkpoints=args.include_checkpoints)
    text = json.dumps(manifest, ensure_ascii=False, indent=2)
    if args.output:
        Path(args.output).write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
