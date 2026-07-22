# src 目录说明

`src/` 存放 Jetson 正式运行包：

- `vision/`：直接调用的视觉算法函数。
- `protocol/`：Jetson 与下位机之间的通信协议实现，本轮不调整。

本轮删除了旧的 `app/` 和 `domain/` 封装。业务流程不再由服务类或工作流类预置，而是由根目录 `main.py` 按实际任务直接组合函数。

开发时可以在 `jetson/` 目录执行入口，也可以安装项目后导入包：

```powershell
conda run -n low_numpy python main.py
conda run -n low_numpy pip install -e .
```

正式运行代码只依赖 `src/` 下的软件包，不反向导入 `scripts/`。协议包的现有下位机定义和接口留待后续单独校准。
