# 一帧图像的全部检测结果 + 推理元信息。
# header.stamp 使用图像帧的采集时刻（而非发布时刻），
# 下游据此可以计算端到端延迟。

std_msgs/Header header

mecamind_interfaces/Detection2D[] detections

string  backend       # 产生本帧结果的推理后端：ultralytics / onnx / hsv / fake
float32 inference_ms  # 本帧推理耗时（毫秒），用于性能观测
