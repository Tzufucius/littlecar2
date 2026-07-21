# vision 目录说明

本目录存放视觉识别相关代码。

- `camera.py`：OpenCV 摄像头封装
- `circle_dector.py`：同心圆图案检测，使用椭圆拟合、同轴环筛选和单应性透视校正输出亚像素圆心
- `qr_detector.py`：二维码识别和内容变化过滤
- `yolo_detector.py`：Ultralytics YOLO 推理封装
- `vision_service.py`：组合二维码和 YOLO 的视觉服务
- `types.py`：视觉结果数据结构

`CircleDetector` 支持图像路径、摄像头和内存帧输入。默认按六圈名义半径
`[28.0, 30.5, 35.5, 40.2, 44.9, 49.6]` 匹配图案；可通过构造参数或
`update_config()` 覆盖检测区域、名义半径和拟合阈值。推理过程不统计耗时，结果的
`time_ms` 固定为 `None`。
