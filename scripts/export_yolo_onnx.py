#!/usr/bin/env python3
"""把 Ultralytics YOLO 权重（.pt）导出为 ONNX 模型（第 4 课 10.3.4）。

【为什么要导出 ONNX】
- .pt 权重依赖 torch + ultralytics 整套环境，体积大、启动慢；
- .onnx 只需要 onnxruntime 一个包就能推理，是跨框架、跨语言的
  通用交换格式——部署到无 GPU 的 PC / 嵌入式设备时的常规做法。
- 导出后 detector_node 用 backend:=onnx 即可加载（OnnxBackend）。

【用法】
    python3 scripts/export_yolo_onnx.py                        # 默认 yolov8n.pt -> models/yolov8n.onnx
    python3 scripts/export_yolo_onnx.py --weights yolov8s.pt   # 换权重
    python3 scripts/export_yolo_onnx.py --imgsz 320            # 换输入分辨率（更快，精度略降）

导出完成后会用 onnxruntime 加载一次并跑一帧随机输入做冒烟检查，
确认输出形状是 YOLOv8 约定的 (1, 84, N)。
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent


def export(weights: Path, output: Path, imgsz: int) -> Path:
    from ultralytics import YOLO

    print(f"[export] 加载权重：{weights}")
    model = YOLO(str(weights))
    # opset 12 兼容面广；dynamic=False 固定输入尺寸，与 OnnxBackend 的
    # letterbox 预处理（默认 640）保持一致。
    exported = Path(model.export(format="onnx", imgsz=imgsz, opset=12, dynamic=False))
    output.parent.mkdir(parents=True, exist_ok=True)
    if exported.resolve() != output.resolve():
        shutil.move(str(exported), str(output))
    print(f"[export] 已导出：{output}（{output.stat().st_size / 1e6:.1f} MB）")
    return output


def smoke_check(model_path: Path, imgsz: int) -> None:
    import numpy as np
    import onnxruntime

    sess = onnxruntime.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
    name = sess.get_inputs()[0].name
    blob = np.random.rand(1, 3, imgsz, imgsz).astype(np.float32)
    out = sess.run(None, {name: blob})[0]
    print(f"[check] onnxruntime 推理成功，输出形状：{out.shape}")
    if out.ndim != 3 or out.shape[1] != 84:
        raise SystemExit(f"[check] 输出形状不符合 YOLOv8 约定 (1, 84, N)：{out.shape}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--weights", default=str(PROJECT_ROOT / "yolov8n.pt"))
    parser.add_argument("--output", default=str(PROJECT_ROOT / "models" / "yolov8n.onnx"))
    parser.add_argument("--imgsz", type=int, default=640)
    args = parser.parse_args()

    weights = Path(args.weights)
    if not weights.is_file():
        print(f"[export] 找不到权重文件：{weights}", file=sys.stderr)
        return 1
    output = export(weights, Path(args.output), args.imgsz)
    smoke_check(output, args.imgsz)
    print("[export] 完成。检测节点用法：")
    print(f"  ros2 launch mecamind_bringup perception.launch.py backend:=onnx model_path:={output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
