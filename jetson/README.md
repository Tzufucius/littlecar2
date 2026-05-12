# littlecar2 Jetson 视觉与上位机协议

这个仓库承载 Jetson 侧代码，分成两条相对独立的主线：

1. 视觉主流程，负责摄像头、二维码和 YOLO 推理。
2. 上位机协议包，负责 Jetson 与 STM32 的串口通信、命令封装和帧解析。

## 目录

- `src/vision/`：摄像头、二维码、YOLO 和视觉服务。
- `src/app/`：视觉主流程入口和业务编排。
- `src/domain/`：视觉流程事件模型。
- `src/protocol/`：Jetson 上位机通信协议包，提供 `Car` 高层接口。
- `assets/`：测试图片和模型文件。
- `scripts/`：可直接运行的视觉调试脚本。

## 运行方式

在 `jetson/` 目录下执行：

```powershell
python -m app.main
```

当前 `src/app/main.py` 仍然只负责视觉主流程，不主动接入串口协议。协议能力放在 `src/protocol/` 中，后续如果需要联动视觉和底盘，可以在业务层显式引入 `from protocol import Car`。

## 常用脚本

```powershell
python scripts/test_qr.py
python scripts/test_yolo.py
python scripts/yolo_infer_example.py
```

## 视觉主流程位置

整体视觉流程主函数写在 `src/app/robot_main.py`。

- `RobotMain.run_once_image(...)`：处理单张图片，适合离线调试。
- `RobotMain.run_camera_loop(...)`：打开摄像头循环运行，适合上车主流程。
- `RobotMain.print_event(...)`：打印二维码和 YOLO 最优目标结果，便于检查。
