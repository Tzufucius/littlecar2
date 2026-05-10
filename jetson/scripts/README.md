# scripts 目录说明

本目录存放可直接运行的视觉调试脚本。原 notebook 中可复用的 YOLO 推理流程已经整理为脚本，后续调试优先在这里维护。

- `test_qr.py`：读取静态图片测试二维码识别。
- `test_yolo.py`：读取静态图片测试 YOLO 推理。
- `yolo_infer_example.py`：从 notebook 合并而来的 YOLO 推理示例，包含绘制检测框并保存结果图。

这些脚本不接收命令行参数。需要更换图片、模型、设备或阈值时，直接修改脚本顶部常量。
