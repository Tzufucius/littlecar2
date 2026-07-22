# littlecar2 Jetson 上位机

## 目录职责

```text
jetson/
  main.py       唯一的流程编排入口，当前默认为空
  src/vision    可直接调用的视觉算法函数
  src/protocol  下位机通信协议包，本轮不调整
  tests/        视觉与协议测试
  scripts/      算法研究和测试脚本，保持原样
  assets/       模型和测试图像，保持原样
  docs/         项目文档
```

`main.py` 只负责导入正式软件包，并为后续流程组合预留 `main()`。直接运行时不会自动打开摄像头、读取图片、加载模型或发送串口命令。

## 运行方式

```powershell
conda run -n low_numpy pip install -e .
conda run -n low_numpy python main.py
```

安装后也可以使用控制台入口：

```powershell
littlecar2
```

## 当前视觉 API

正式视觉代码只接收 BGR 格式的 `numpy.ndarray` 图像帧：

```python
from vision import (
    detect_colored_materials,
    detect_numbered_marker,
    detect_qr,
    detect_yolo,
    load_yolo_model,
)
```

传统彩色物料检测和带数字同心圆检测是当前主要方案。YOLO 保留为可选能力，只有显式调用 `load_yolo_model()` 时才加载模型。物料盘旋转中心定位暂未接入正式视觉 API。

摄像头、图片读取、视频流、通信和小车控制流程由用户在根目录 `main.py` 中自行组织。`scripts/` 仅用于研究和验证，正式运行代码不得反向导入脚本。

## 测试

```powershell
conda run -n low_numpy python -m unittest discover -s tests -v
```

协议包本轮保持不变；下位机定义校准将在后续任务中单独处理。
