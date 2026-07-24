# littlecar2 Jetson

## 目录

```text
jetson/
  main.py       正式流程入口
  src/vision    正式视觉接口
  src/protocol  下位机通信协议
  tests/        自动化测试
  scripts/      算法研究和验证脚本
  assets/       模型和测试图片
```

## 运行

```powershell
conda run -n low_numpy pip install -e .
conda run -n low_numpy python main.py
```

## 视觉 API

```python
from vision import (
    detect_colored_materials,
    detect_numbered_marker,
    detect_qr,
    estimate_disk_center,
    load_yolo_model,
)

model = load_yolo_model("assets/models/6color-circle-v3.pt")
detections = detect_colored_materials(frame_bgr, model)
center = estimate_disk_center(detections, frame_bgr.shape)
```

物料颜色和物料盘中心定位均使用 YOLO。模型只在启动阶段加载一次，之后逐帧复用。
`EmptySlot` 类别会保留在检测结果中，并可参与中心定位。

二维码接口直接返回任务码字符串；未识别到二维码时返回 `None`。

## 测试

```powershell
conda run -n low_numpy python -m pytest tests -q
```

`main.py` 不会自动打开摄像头、读取图片、加载模型或发送串口命令。
