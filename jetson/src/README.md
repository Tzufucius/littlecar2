# src 目录说明

本目录存放 Jetson 侧 Python 源码。当前只保留视觉相关模块，`app`、`vision`、`domain` 直接作为顶层模块。

运行参数不使用配置文件传入。需要改模型、摄像头、图片路径或运行模式时，直接修改 `app/main.py`、`app/robot_main.py` 或 `scripts/` 中的顶部常量。

开发时在 `jetson/` 目录执行，并确保 `src` 在 `PYTHONPATH` 中：

```powershell
python -m app.main
```
