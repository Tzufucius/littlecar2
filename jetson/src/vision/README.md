# vision 目录说明

本目录提供不保存业务流程状态的视觉算法函数。所有检测函数都接收 BGR 格式的 `numpy.ndarray` 图像帧，不负责摄像头、图片读取、视频流或通信。

## 函数接口

- `marker.py`：`detect_numbered_marker(frame_bgr)`，识别带数字的同心圆标记，返回中心、数字、数字角度、置信度、圈数和缩放因子。
- `materials.py`：`detect_colored_materials(frame_bgr)`，使用 Hough 圆检测和 HSV 分类识别彩色物料，最多返回三个结果。
- `qr.py`：`detect_qr(frame_bgr)`，识别二维码并返回内容、角点、中心和时间戳，不进行变化过滤。
- `yolo.py`：`load_yolo_model(model_path)` 和 `detect_yolo(...)`，保留 YOLO 能力但不自动加载或调用模型。

数字模板在模块内部只初始化一次；除第三方检测器实例外不维护长期状态。传统方法是当前默认视觉方案，YOLO 需要由调用方显式加载。

物料盘旋转中心定位当前不属于正式视觉 API。研究和验证代码仍位于 `scripts/`，本目录不得反向导入这些脚本。
