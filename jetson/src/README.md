# src 目录说明

本目录存放 Jetson 侧 Python 源码。当前采用扁平结构，`app`、`vision`、`comm`、`domain` 直接作为顶层模块。

运行参数不使用配置文件传入。需要改模型、摄像头、串口或运行模式时，直接修改 `app/main.py`、`app/robot_main.py` 或 `scripts/` 中的顶部常量。

开发时在 `jetson/` 目录执行：

```bash
pip install -e .
```

安装后可通过 `import app`、`import vision` 等方式调用，也可以使用命令 `littlecar2`。
