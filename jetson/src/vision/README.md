# vision 目录说明

本目录提供不保存业务流程状态的视觉函数。正式接口接收 BGR 格式的
`numpy.ndarray` 图像帧，不负责摄像头、图像读取、视频流或通信。

## 接口

- `marker.py`：`detect_numbered_marker(frame_bgr)`，识别带数字的同心圆标记。
- `materials.py`：`detect_colored_materials(frame_bgr, model)`，使用已加载的 YOLO 模型检测物料和空槽。
- `materials.py`：`estimate_disk_center(detections, frame_shape)`，使用检测点估算物料盘中心。
- `qr.py`：`detect_qr(frame_bgr)`，识别二维码并直接返回任务码字符串。
- `yolo.py`：`load_yolo_model(model_path)` 和 `detect_yolo(...)`，负责模型加载和通用推理。

物料视觉统一使用 `assets/models/6color-circle-v3.pt`。模型应由调用方加载一次，
随后在逐帧处理中复用，禁止在检测函数内部重复加载模型。

模型类别为：`0 Red`、`1 Yellow`、`2 Blue`、`3 Green`、`4 Black`、
`5 LightBlue`、`6 EmptySlot`。
