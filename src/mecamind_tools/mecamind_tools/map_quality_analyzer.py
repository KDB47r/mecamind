"""地图质量分析器（MecaMind 第二课：给建好的地图"打分"）。

这个文件是干什么的？
    读取 map_saver（或本项目的建图路线驱动器）保存的 PGM/YAML 地图文件，
    从四个维度评估这张 SLAM 地图能不能拿来做导航：
    1. 覆盖率（coverage）        —— 已知区域（自由 + 占据）占多大比例；
    2. 未知区比例（unknown）     —— 灰色未探索区域还剩多少；
    3. 自由空间连通性            —— 可通行区域是不是连成一整片（碎成几块
                                     说明门洞没扫通，机器人过不去）；
    4. 四面外墙完整性            —— 北/南/东/西墙有没有在地图上闭合。
    最后给出三档结论：commercial_ready（可交付）/
    navigation_ready_best_effort（勉强可导航）/ needs_remap（重扫）。

    它是一个纯离线的命令行工具，**不是** ROS 节点：不订阅、不发布任何
    topic，直接读文件、算指标、打印报告（或 JSON）。这也是教学要点之一：
    并非所有机器人工具都要写成节点，离线分析用普通 Python 脚本更简单。

在建图流程中的角色：
    mapping_route_driver 存好地图后，用本工具检查质量；map_asset_manager
    也会调用本模块的 analyze_map() 给目录里每张地图打分并挑出推荐地图。

一个必须理解的坑（垂直翻转）：
    map_saver 写 PGM 时约定"图片第 0 行（顶部）对应世界坐标 y 最大处"，
    而 OccupancyGrid 约定"第 0 行对应 origin（y 最小处）"——两者行序相反。
    所以 load_map_yaml() 加载 PGM 后要做一次上下翻转，把行序恢复成
    OccupancyGrid 的约定，world_to_cell() 的坐标换算才是对的。忘了这一步，
    所有"世界坐标 -> 像素"的查询都会上下颠倒，墙体检测全错。

初学者应该重点看的函数：
    - load_map_yaml()                     —— 读 YAML + PGM，注意垂直翻转。
    - _classify_pixel()                   —— 灰度值如何分成自由/占据/未知三类。
    - _largest_free_component_ratio()     —— BFS 连通域分析（洪水填充）。
    - _wall_scores_from_world()           —— 沿墙体采样打分的原理。
    - analyze_map()                       —— 把所有指标汇总成结论的主流程。
"""

from __future__ import annotations

import argparse
import json
import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import yaml

from .world_geometry import AxisAlignedBox, load_world_geometry


@dataclass(frozen=True)
class OccupancyMap:
    """加载到内存里的栅格地图（行序已统一为 OccupancyGrid 约定）。

    pixels 是 row-major（按行展开）的一维灰度值元组，第 0 行对应世界
    坐标 y 最小的一行（即 origin 所在行）。origin_x/origin_y 是第 0 行
    第 0 列格子左下角在世界坐标系中的位置，resolution 是每格边长（米）。
    用 frozen dataclass 是因为地图加载后只读不改，不可变对象更安全。
    """

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    pixels: tuple[int, ...]

    def at(self, row: int, col: int) -> int:
        """取 (row, col) 格子的灰度值。row-major：行号乘宽度加列号。"""
        return self.pixels[row * self.width + col]

    def world_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        """世界坐标 (米) -> 栅格下标 (row, col)；落在地图外返回 None。

        换算就是"减原点、除分辨率、取整"。注意返回顺序是 (row, col)，
        row 对应 y，col 对应 x——初学者很容易把 x/y 和 row/col 弄反。
        """
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        if 0 <= row < self.height and 0 <= col < self.width:
            return row, col
        return None

    def cell_to_world(self, row: int, col: int) -> tuple[float, float]:
        """栅格下标 -> 该格子中心的世界坐标（+0.5 取的是格子中心而非角点）。"""
        return (
            self.origin_x + (col + 0.5) * self.resolution,
            self.origin_y + (row + 0.5) * self.resolution,
        )


def _read_pgm_token(data: bytes, index: int) -> tuple[bytes, int]:
    """从 PGM 文件头部读出下一个 token（魔数/宽/高/最大灰度）。

    PGM 头是 ASCII 文本，token 之间用空白分隔，且允许插入以 '#' 开头、
    到行尾结束的注释行——很多工具导出的 PGM 都带注释，必须跳过。
    返回 (token 字节串, 读完后的新下标)。
    """
    n = len(data)
    # 先跳过 token 前面的空白和注释。
    while index < n:
        value = data[index]
        if value == ord("#"):
            # 注释：一路吃到换行（10 = \n, 13 = \r）。
            while index < n and data[index] not in (10, 13):
                index += 1
        elif value in b" \t\r\n":
            index += 1
        else:
            break

    # 再读 token 本体，直到遇到空白或注释起始符。
    start = index
    while index < n and data[index] not in b" \t\r\n#":
        index += 1
    return data[start:index], index


def _load_pgm(path: Path) -> tuple[int, int, tuple[int, ...]]:
    """解析 PGM 灰度图，返回 (宽, 高, 像素元组)。

    不依赖 PIL/OpenCV，用几十行代码手工解析——PGM 格式足够简单，
    这样教学环境不用多装图像库。支持两种变体：
    - P2：ASCII 格式，像素是文本数字；
    - P5：二进制格式（map_saver 的输出），头之后紧跟原始字节，
      最大灰度超过 255 时每像素占 2 字节（大端序）。
    注意：返回的像素仍是 PGM 自己的行序（第 0 行 = 图片顶部），
    翻转的事交给 load_map_yaml() 做。
    """
    data = path.read_bytes()
    magic, index = _read_pgm_token(data, 0)
    width_raw, index = _read_pgm_token(data, index)
    height_raw, index = _read_pgm_token(data, index)
    max_value_raw, index = _read_pgm_token(data, index)

    width = int(width_raw)
    height = int(height_raw)
    max_value = int(max_value_raw)
    if max_value <= 0 or max_value > 65535:
        raise ValueError(f"Unsupported PGM max value: {max_value}")

    if magic == b"P2":
        # ASCII 变体：剩余内容全是空白分隔的数字，逐个转 int。
        values = []
        for item in data[index:].split():
            if item.startswith(b"#"):
                continue
            values.append(int(item))
        if len(values) < width * height:
            raise ValueError(f"PGM has too few pixels: {path}")
        return width, height, tuple(values[: width * height])

    if magic == b"P5":
        # 二进制变体：头部最后一个 token 之后有且只有一个空白字符，
        # 之后就是像素数据；这里宽容地跳过所有连续空白。
        while index < len(data) and data[index] in b" \t\r\n":
            index += 1
        pixel_count = width * height
        if max_value < 256:
            # 每像素 1 字节，直接按字节取值。
            raw = data[index : index + pixel_count]
            if len(raw) != pixel_count:
                raise ValueError(f"PGM has too few binary pixels: {path}")
            return width, height, tuple(int(v) for v in raw)

        # 每像素 2 字节（16 位灰度），PGM 规定用大端序。
        raw = data[index : index + pixel_count * 2]
        if len(raw) != pixel_count * 2:
            raise ValueError(f"PGM has too few 16-bit pixels: {path}")
        values = [
            int.from_bytes(raw[i : i + 2], byteorder="big")
            for i in range(0, len(raw), 2)
        ]
        return width, height, tuple(values)

    raise ValueError(f"Unsupported PGM magic: {magic!r}")


def load_map_yaml(path: str | Path) -> OccupancyMap:
    """从 nav2 地图 YAML 文件加载整张地图（含关键的垂直翻转）。

    YAML 里记录着图片文件名、分辨率、原点等元数据；图片路径若是相对
    路径，则相对于 YAML 文件所在目录解析（nav2 的约定）。

    重点是下面的翻转：PGM 第 0 行是"世界 y 最大处"（map_saver 约定，
    见模块 docstring），而本模块的 OccupancyMap 采用 OccupancyGrid 约定
    （第 0 行 = origin = y 最小处），所以要把行序整体倒过来。
    """
    yaml_path = Path(path).expanduser().resolve()
    with yaml_path.open("r", encoding="utf-8") as stream:
        meta = yaml.safe_load(stream) or {}
    image_path = Path(str(meta["image"]))
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    width, height, pixels = _load_pgm(image_path)
    # map_saver 写出的 PGM 第 0 行对应 y 最大处；这里翻转为第 0 行对应
    # origin（y 最小处），使 world_to_cell 的行号与世界坐标一致。
    flipped: list[int] = []
    for row in range(height - 1, -1, -1):
        flipped.extend(pixels[row * width : (row + 1) * width])
    pixels = tuple(flipped)
    origin = list(meta.get("origin", [0.0, 0.0, 0.0]))
    return OccupancyMap(
        width=width,
        height=height,
        resolution=float(meta.get("resolution", 0.05)),
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        pixels=pixels,
    )


def _classify_pixel(value: int) -> str:
    """把 PGM 灰度值分成三类：occupied（占据）/ free（自由）/ unknown（未知）。

    对应 map_saver trinary 模式的写出值：0（黑）= 占据，254（近白）= 自由，
    205（灰）= 未知。这里用宽松的区间（<=100 / >=230）而不是精确相等，
    是为了兼容被图像软件重新保存过、灰度略有偏移的地图文件。
    """
    if value <= 100:
        return "occupied"
    if value >= 230:
        return "free"
    return "unknown"


def _counts(grid: OccupancyMap) -> dict[str, int]:
    """统计整张地图上三类格子各有多少个。"""
    counts = {"free": 0, "occupied": 0, "unknown": 0}
    for value in grid.pixels:
        counts[_classify_pixel(value)] += 1
    return counts


def _ratio_from_counts(counts: dict[str, int]) -> tuple[float, float]:
    """由格子计数算出 (覆盖率, 未知区比例)。

    覆盖率 = 已知格子（自由 + 占据）/ 总格子。空计数时返回
    (0 覆盖, 100% 未知)，代表"完全没有信息"这个最保守的结论。
    """
    total = sum(counts.values())
    if not total:
        return 0.0, 1.0
    coverage = (counts["free"] + counts["occupied"]) / total
    unknown_ratio = counts["unknown"] / total
    return coverage, unknown_ratio


def _counts_in_world_roi(
    grid: OccupancyMap,
    world_path: str | Path,
    padding: float = 0.12,
) -> tuple[dict[str, int], dict[str, float]]:
    """统计世界内部 ROI 的覆盖情况。

    ROI 向内收缩一个墙厚（padding），只统计机器人有可能观测到的区域；
    外墙外侧永远不可见，算进去会把覆盖率上限压到 90% 以下。

    补充解释：SLAM 地图的栅格范围通常比房子本身大一圈（外面全是未知），
    如果对整张图算覆盖率，那些"物理上不可能被扫到"的格子会永远拉低
    分数，导致再好的地图也达不到 95% 的验收线。所以拿仿真世界文件里
    的真实房屋边界框出一个 ROI（感兴趣区域），只在 ROI 内统计。
    做法：在 ROI 内按地图分辨率均匀撒采样点，逐点换算成栅格再分类计数；
    采样点落在栅格范围外说明该处根本没被建出来，计为 unknown。
    """
    world = load_world_geometry(world_path)
    min_x = world.min_x + padding
    max_x = world.max_x - padding
    min_y = world.min_y + padding
    max_y = world.max_y - padding
    x_count = max(1, int(math.ceil((max_x - min_x) / grid.resolution)))
    y_count = max(1, int(math.ceil((max_y - min_y) / grid.resolution)))
    counts = {"free": 0, "occupied": 0, "unknown": 0}

    for yi in range(y_count):
        y = min_y + (yi + 0.5) * grid.resolution
        for xi in range(x_count):
            x = min_x + (xi + 0.5) * grid.resolution
            cell = grid.world_to_cell(x, y)
            if cell is None:
                counts["unknown"] += 1
                continue
            counts[_classify_pixel(grid.at(cell[0], cell[1]))] += 1

    return counts, {
        "min_x": round(min_x, 3),
        "max_x": round(max_x, 3),
        "min_y": round(min_y, 3),
        "max_y": round(max_y, 3),
    }


def _neighbors(row: int, col: int) -> Iterable[tuple[int, int]]:
    """产出 4-邻接（上下左右）的邻居坐标，供连通域搜索用。

    用 4-邻接而不是 8-邻接，是因为机器人不能"穿对角线"：两个只在
    对角接触的自由格之间实际是走不通的，按 8-邻接会高估连通性。
    """
    yield row - 1, col
    yield row + 1, col
    yield row, col - 1
    yield row, col + 1


def _largest_free_component_ratio(grid: OccupancyMap) -> float:
    """计算最大自由空间连通域占全部自由格子的比例。

    原理是经典的"洪水填充"（flood fill）连通域分析，用 BFS 实现：
    1. 扫描全图，遇到没访问过的自由格子就以它为种子；
    2. 从种子出发做广度优先搜索，把所有 4-邻接可达的自由格子标记为
       同一个连通域，同时数这个域有多少格；
    3. 记录见过的最大域，最后除以自由格总数。

    这个比值接近 1.0 说明可通行区域是一整片；明显小于 1（比如 0.6）
    说明地图上的自由空间被"假墙"割裂了——常见原因是门洞处激光没扫透、
    留下一条未知/占据的窄带，导航时机器人会被困在其中一块里。
    """
    visited: set[tuple[int, int]] = set()
    total_free = 0
    largest = 0

    for row in range(grid.height):
        for col in range(grid.width):
            if _classify_pixel(grid.at(row, col)) != "free":
                continue
            total_free += 1
            if (row, col) in visited:
                # 已经属于之前找到的某个连通域，不再重复搜索。
                continue
            # 以 (row, col) 为种子开始 BFS 洪水填充。
            size = 0
            queue = deque([(row, col)])
            visited.add((row, col))
            while queue:
                r, c = queue.popleft()
                size += 1
                for nr, nc in _neighbors(r, c):
                    if not (0 <= nr < grid.height and 0 <= nc < grid.width):
                        continue
                    if (nr, nc) in visited:
                        continue
                    if _classify_pixel(grid.at(nr, nc)) != "free":
                        continue
                    visited.add((nr, nc))
                    queue.append((nr, nc))
            largest = max(largest, size)

    return largest / total_free if total_free else 0.0


def _occupied_near(grid: OccupancyMap, row: int, col: int, radius: int = 2) -> bool:
    """判断 (row, col) 周围 radius 格范围内是否存在占据格。

    墙体检测时不要求采样点精确落在黑色像素上——SLAM 建出的墙有一两格
    的定位误差很正常，所以给一个小邻域的容忍窗口。
    """
    for r in range(max(0, row - radius), min(grid.height, row + radius + 1)):
        for c in range(max(0, col - radius), min(grid.width, col + radius + 1)):
            if _classify_pixel(grid.at(r, c)) == "occupied":
                return True
    return False


def _sample_box(box: AxisAlignedBox, step: float) -> Iterable[tuple[float, float]]:
    """在一个轴对齐矩形（墙体）表面均匀撒采样点。

    按 step（取地图分辨率）把矩形的长和宽等分，产出网格状采样点，
    保证整面墙都被检查到而不是只查几个角。max(2, ...) 保证薄墙
    至少也有边界上的采样点。
    """
    x_count = max(2, int(math.ceil(box.width / step)))
    y_count = max(2, int(math.ceil(box.depth / step)))
    for xi in range(x_count + 1):
        x = box.min_x + box.width * xi / x_count
        for yi in range(y_count + 1):
            y = box.min_y + box.depth * yi / y_count
            yield x, y


def _cell_near_boundary(
    grid: OccupancyMap,
    x: float,
    y: float,
    tolerance_cells: int = 2,
) -> tuple[int, int] | None:
    """world_to_cell 的宽松版：允许采样点落在地图边界外不超过 tolerance_cells。

    SLAM 地图通常恰好裁剪在墙体内侧面，墙体外半部分不在栅格范围内，
    这里把边界附近的采样点收敛到最近的边界栅格，避免误判墙体缺失。
    """
    col = int((x - grid.origin_x) / grid.resolution)
    row = int((y - grid.origin_y) / grid.resolution)
    # 偏出容忍范围（说明真的不在地图附近）才返回 None。
    if col < -tolerance_cells or col >= grid.width + tolerance_cells:
        return None
    if row < -tolerance_cells or row >= grid.height + tolerance_cells:
        return None
    # 在容忍范围内则把下标夹回 [0, 尺寸-1]，用最近的边界格代替。
    return (
        min(max(row, 0), grid.height - 1),
        min(max(col, 0), grid.width - 1),
    )


def _wall_scores_from_world(grid: OccupancyMap, world_path: str | Path | None) -> dict[str, float]:
    """按仿真世界的真实墙体位置给四面外墙打分（0.0 ~ 1.0）。

    打分原理（对每面墙）：
    1. 从世界文件里按名字（如 "north_wall"）找到这面墙的矩形；
    2. 在墙面上均匀撒采样点（_sample_box）；
    3. 每个采样点换算到地图栅格（允许贴边落外，见 _cell_near_boundary），
       看它附近有没有占据格（_occupied_near）；
    4. 得分 = 命中的采样点数 / 总采样点数。
    得分接近 1.0 说明这面墙在地图上完整闭合；偏低说明有缺口——通常
    是机器人没在这面墙附近走过、激光没扫到。没有世界文件时返回空
    dict，由调用方退回到 _edge_scores 的启发式方法。
    """
    if not world_path:
        return {}
    world = load_world_geometry(world_path)
    scores: dict[str, float] = {}
    for side in ("north", "south", "east", "west"):
        wall = next((box for box in world.boxes if f"{side}_wall" in box.name), None)
        if wall is None:
            continue
        total = 0
        hits = 0
        for x, y in _sample_box(wall, grid.resolution):
            total += 1
            cell = _cell_near_boundary(grid, x, y)
            if cell is None:
                continue
            if _occupied_near(grid, cell[0], cell[1]):
                hits += 1
        scores[side] = hits / total if total else 0.0
    return scores


def _edge_scores(grid: OccupancyMap) -> dict[str, float]:
    """不知道真实墙体位置时的兜底打分：检查地图四条边附近的占据密度。

    假设 SLAM 地图会大致裁剪到外墙位置，那么地图边缘 0.20 米宽的条带里
    应该有相当比例的占据格。得分 = 条带内占据格数 / 条带总格数。
    这个方法比 _wall_scores_from_world 粗糙（地图边界不一定贴着墙），
    只在拿不到世界文件时使用。
    注意行号方向：行序已翻转为 OccupancyGrid 约定，所以行号大 = y 大 =
    北边，行号小 = 南边。
    """
    # 条带宽度：0.20 米折算成格数，至少 2 格。
    strip = max(2, int(0.20 / max(grid.resolution, 1.0e-6)))
    sides = {
        "north": [(r, c) for r in range(max(0, grid.height - strip), grid.height) for c in range(grid.width)],
        "south": [(r, c) for r in range(0, min(strip, grid.height)) for c in range(grid.width)],
        "east": [(r, c) for r in range(grid.height) for c in range(max(0, grid.width - strip), grid.width)],
        "west": [(r, c) for r in range(grid.height) for c in range(0, min(strip, grid.width))],
    }
    result = {}
    for side, cells in sides.items():
        occupied = sum(1 for row, col in cells if _classify_pixel(grid.at(row, col)) == "occupied")
        result[side] = occupied / len(cells) if cells else 0.0
    return result


def analyze_map(map_yaml: str | Path, world_path: str | Path | None = None) -> dict[str, object]:
    """主入口：加载地图、算全部指标、给出结论和改进建议。

    流程：
    1. 加载地图，先算全图（raw）的覆盖率/未知比例，作为参考基线；
    2. 如果提供了世界文件，改用房屋内部 ROI 重新统计（更公平，
       原因见 _counts_in_world_roi）；
    3. 算自由空间连通性和四面墙得分（墙得分优先用世界文件的精确
       方法，否则退回边缘启发式）；
    4. 按阈值下结论：
       - commercial_ready：覆盖率 >= 95%、未知 <= 5%、四墙齐全；
       - navigation_ready_best_effort：覆盖率 >= 85% 且最大自由连通域
         占比 >= 80%（能导航但质量一般）；
       - needs_remap：以上都不满足，建议重扫。
    5. 根据短板生成对应的改进建议（哪面墙缺、要不要补扫等）。
    返回的 dict 可直接 JSON 序列化，供上层工具（如 map_asset_manager）使用。
    """
    grid = load_map_yaml(map_yaml)
    raw_counts = _counts(grid)
    raw_coverage, raw_unknown_ratio = _ratio_from_counts(raw_counts)
    roi_bounds = {}
    if world_path:
        counts, roi_bounds = _counts_in_world_roi(grid, world_path)
        coverage_scope = "world_roi"
    else:
        counts = raw_counts
        coverage_scope = "full_map"
    coverage, unknown_ratio = _ratio_from_counts(counts)
    free_component_ratio = _largest_free_component_ratio(grid)
    # or 短路：世界文件缺失时 _wall_scores_from_world 返回 {}（假值），
    # 自动落到 _edge_scores 的启发式打分。
    wall_scores = _wall_scores_from_world(grid, world_path) or _edge_scores(grid)
    # 得分低于 0.35 判定为"这面墙缺失"（墙有大段缺口）。
    missing_walls = [side for side, score in wall_scores.items() if score < 0.35]

    if coverage >= 0.95 and unknown_ratio <= 0.05 and not missing_walls:
        verdict = "commercial_ready"
    elif coverage >= 0.85 and free_component_ratio >= 0.80:
        verdict = "navigation_ready_best_effort"
    else:
        verdict = "needs_remap"

    recommendations = []
    if missing_walls:
        recommendations.append(
            "Revisit or manually drive along missing map edges: " + ", ".join(missing_walls)
        )
    if unknown_ratio > 0.05:
        recommendations.append("Run another frontier/gap-fill pass before accepting the map.")
    if free_component_ratio < 0.80:
        recommendations.append("Check door openings and costmap inflation; free space is fragmented.")
    if verdict == "commercial_ready":
        recommendations.append("Map can enter zone/waypoint annotation and saved-map navigation tests.")

    return {
        "map": str(Path(map_yaml)),
        "size": {"width": grid.width, "height": grid.height, "resolution": grid.resolution},
        "origin": [grid.origin_x, grid.origin_y, 0.0],
        "coverage_scope": coverage_scope,
        "roi_bounds": roi_bounds,
        "counts": counts,
        "raw_counts": raw_counts,
        "coverage": round(coverage, 4),
        "unknown_ratio": round(unknown_ratio, 4),
        "raw_coverage": round(raw_coverage, 4),
        "raw_unknown_ratio": round(raw_unknown_ratio, 4),
        "largest_free_component_ratio": round(free_component_ratio, 4),
        "wall_scores": {key: round(value, 4) for key, value in wall_scores.items()},
        "missing_walls": missing_walls,
        "verdict": verdict,
        "recommendations": recommendations,
    }


def format_report(result: dict[str, object]) -> str:
    """把 analyze_map 的结果 dict 排版成人类可读的多行文本报告。"""
    size = result["size"]
    counts = result["counts"]
    return "\n".join(
        [
            "MecaMind map quality report",
            f"map: {result['map']}",
            f"size: {size['width']}x{size['height']} @ {size['resolution']}m",
            f"coverage_scope: {result['coverage_scope']}",
            f"counts: free={counts['free']} occupied={counts['occupied']} unknown={counts['unknown']}",
            f"coverage: {float(result['coverage']) * 100.0:.1f}%",
            f"unknown_ratio: {float(result['unknown_ratio']) * 100.0:.1f}%",
            f"raw_full_map_coverage: {float(result['raw_coverage']) * 100.0:.1f}%",
            f"raw_full_map_unknown_ratio: {float(result['raw_unknown_ratio']) * 100.0:.1f}%",
            f"largest_free_component_ratio: {float(result['largest_free_component_ratio']) * 100.0:.1f}%",
            f"wall_scores: {json.dumps(result['wall_scores'], ensure_ascii=False)}",
            f"missing_walls: {', '.join(result['missing_walls']) if result['missing_walls'] else 'none'}",
            f"verdict: {result['verdict']}",
            "recommendations:",
            *[f"- {item}" for item in result["recommendations"]],
        ]
    )


def main() -> None:
    """命令行入口：解析参数、分析地图、按需求打印文本报告或 JSON。"""
    parser = argparse.ArgumentParser(description="Analyze MecaMind YAML/PGM map quality.")
    parser.add_argument("map_yaml", help="Path to a nav2_map_server YAML file.")
    parser.add_argument("--world", default=None, help="Optional MecaMind world file for wall checks.")
    parser.add_argument("--json", action="store_true", help="Print JSON instead of text.")
    args = parser.parse_args()

    result = analyze_map(args.map_yaml, args.world)
    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        print(format_report(result))


if __name__ == "__main__":
    main()
