# littlecar2 Jetson 软件包

本目录是 Jetson 侧代码。当前重点保留视觉识别与串口通信骨架，方便后续把二维码、YOLO 推理、任务流程和 STM32 通信组合起来。

## 目录

- `src/littlecar2/vision/`：摄像头、YOLO、二维码和视觉服务。
- `src/littlecar2/comm/`：STM32 通信接口、Mock 客户端、串口客户端和协议工具。
- `src/littlecar2/app/`：命令行入口和任务流程骨架。
- `src/littlecar2/domain/`：领域枚举和事件类型。
- `configs/default.yaml`：默认模型、摄像头和串口配置。
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
python scripts/test_qr.py --image assets/同色.png
python scripts/test_yolo.py --image assets/sim_train_00025.jpg
python scripts/test_uart.py --mock
littlecar2 --config configs/default.yaml --once assets/同色.png
```
