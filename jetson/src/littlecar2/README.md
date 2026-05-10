# littlecar2 包说明

本包是 Jetson 侧主程序代码。

- `app/`：命令行入口和任务流程编排
- `vision/`：摄像头、二维码识别、YOLO 推理和视觉服务
- `comm/`：与 STM32 通信的接口、串口实现、mock 实现和协议工具
- `domain/`：跨模块共享的枚举和事件类型
