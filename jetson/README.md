# littlecar2 Jetson 视觉软件包

本目录是 Jetson 侧视觉代码。当前只保留二维码识别、YOLO 推理、摄像头读取和主流程示例。

## 目录

- `src/vision/`：摄像头、二维码、YOLO 和视觉服务。
- `src/app/`：硬编码运行入口和视觉主流程。
- `src/domain/`：视觉流程事件类型。
- `assets/`：测试图片和模型文件。
- `scripts/`：可直接运行的视觉调试脚本。

## 运行方式

在 VS Code 集成终端或已配置 `PYTHONPATH=./src` 的终端中执行：

```powershell
cd F:\Project\littleCar2\zhengdian\4-29\jetson
python -m app.main
```

当前 `src/app/main.py` 不接收命令行参数。图片路径、模型路径、摄像头编号、运行模式和阈值都写在源码顶部常量中，便于上车前快速固定一套流程。

## 常用脚本

```powershell
python scripts/test_qr.py
python scripts/test_yolo.py
python scripts/yolo_infer_example.py
```

## 主流程位置

整体视觉流程主函数写在 `src/app/robot_main.py`。

- `RobotMain.run_once_image(...)`：处理单张图片，适合离线调试。
- `RobotMain.run_camera_loop(...)`：打开摄像头循环运行，适合上车主流程。
- `RobotMain.print_event(...)`：打印二维码和 YOLO 最优目标结果。
