# littlecar2 Jetson 软件包

本目录是 Jetson 侧代码。当前重点保留视觉识别与串口通信骨架，方便后续把二维码、YOLO 推理、任务流程和 STM32 通信组合起来。

## 目录

- `src/vision/`：摄像头、YOLO、二维码和视觉服务。
- `src/comm/`：STM32 通信接口、Mock 客户端、串口客户端和协议工具。
- `src/app/`：命令行入口和任务流程骨架。
- `src/domain/`：领域枚举和事件类型。
- `assets/`：静态测试图片。
- `models/`：YOLO 模型文件。
- `scripts/`：独立调试脚本。
- `notebooks/`：保留的手动测试 notebook。

## 安装

```bash
cd jetson
python -m venv .venv
source .venv/bin/activate
pip install -e .
```

Jetson 上如果使用系统 OpenCV 或 TensorRT 版本，请按实际环境调整依赖安装方式。

## 常用命令

```bash
python scripts/test_qr.py
python scripts/test_yolo.py
python scripts/test_uart.py
littlecar2
```

## 主流程位置

整体流程主函数写在 `src/app/robot_main.py`。

- `RobotMain.run_once_image(...)`：处理单张图片，适合离线调试。
- `RobotMain.run_camera_loop(...)`：打开摄像头循环运行，适合上车主流程。
- `RobotMain.handle_event(...)`：把二维码和 YOLO 识别结果转换为后续动作的位置，后续 STM32 控制命令应从这里继续接入。

`src/app/main.py` 只是命令行入口，不建议在里面堆业务逻辑。

## 参数修改方式

本工程不使用配置文件，也不从命令行传参。需要修改路径、模型、摄像头、串口或运行模式时，直接改对应脚本顶部常量：

- `src/app/main.py`：选择运行单张图片还是摄像头主循环
- `src/app/robot_main.py`：模型、摄像头、串口和主循环间隔
- `scripts/test_qr.py`：二维码测试图片
- `scripts/test_yolo.py`：YOLO 测试图片、模型、设备和置信度
- `scripts/test_uart.py`：mock/真实串口、串口号和波特率
