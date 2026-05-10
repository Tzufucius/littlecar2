# src 目录说明

本目录存放 Jetson 侧 Python 源码。当前采用扁平结构，`app`、`vision`、`comm`、`domain` 直接作为顶层模块。

开发时在 `jetson/` 目录执行：

```bash
pip install -e .
```

安装后可通过 `import app`、`import vision` 等方式调用，也可以使用命令 `littlecar2`。
