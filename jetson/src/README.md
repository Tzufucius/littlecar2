# src 目录说明

`src/` 存放 Jetson 侧 Python 源码，当前分为四个顶层模块：

- `app`：视觉主流程入口与编排。
- `vision`：摄像头、二维码和 YOLO 识别封装。
- `domain`：视觉流程事件模型。
- `protocol`：Jetson 上位机协议包，负责串口命令、ACK、DATA 和 STATUS 解析。

运行参数不通过配置文件传入。需要修改模型、摄像头、图片路径、协议串口或运行模式时，直接修改对应模块或 `scripts/` 的顶部常量。

开发时在 `jetson/` 目录下执行，并确保 `src` 在 `PYTHONPATH` 中：

```powershell
python -m app.main
```

如果要使用上位机协议包，可直接从 `protocol` 目录导入：

```python
from protocol import Car
```
