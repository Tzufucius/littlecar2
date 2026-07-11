# Core 目录说明

`Core` 存放当前 STM32 工程的核心源码、模块接口和 HAL 入口文件。

## 子目录

- `Inc/`：公共头文件、模块接口声明、HAL 配置头文件。
- `Src/`：主程序入口、外设初始化、中断入口、设备驱动、传感器解析、通信协议和高级控制实现。

## 当前命名规则

- `sensor_*`：传感器相关模块，例如 `sensor_wit`、`sensor_ops`。
- `drive_*`：执行器和运动驱动相关模块，例如 `drive_emm`、`drive_bus_servo`。
- `comm_*`：通信相关模块，例如 `comm_pc`、`comm_jetson`、`comm_protocol`。
- `advance_*`：基于底层驱动或传感器数据封装出的高级动作、坐标系或业务能力，例如 `advance_chassis`、`advance_world`。

## 当前主要模块

- `drive_emm`：张大头 Emm_V5 步进闭环驱动协议，使用 `USART3`。
- `drive_bus_servo`：总线舵机控制模块，使用 `UART4`。
- `sensor_wit`：WIT / HWT905 IMU 解析模块，使用 `USART2`。
- `sensor_ops`：OPS 定位系统解析模块，使用 `UART5`。
- `comm_pc`：PC / Jetson 原始接收调试桥接层，接入协议解析器。
- `comm_jetson`：Jetson 原始接收调试兼容入口。
- `comm_protocol`：上位机二进制协议解析、命令入队、UART 中断发送队列与 ACK 回发。
- `advance_chassis`：基于 `drive_emm` 的麦克纳姆底盘高级运动接口。
- `advance_world`：维护 world 坐标系、全局位姿和 world/base 速度变换。
- `advance_motion`：世界速度与 `GotoPose` 异步状态机；由 `main.c` 周期调用并受协议层控制权仲裁保护。
- `advance_arm`：机械臂的物料盘、旋转、夹爪、丝杆和前后平移动作封装；所有设备 ID 与运动参数由调用方传入。

## 闭环安全边界

- `drive_emm` 负责 USART3 DMA 发送队列、DMA/IDLE 回包解析和四个底盘电机的反馈新鲜度监督。
- `advance_world` 负责 OPS 安装补偿、位置与航向的独立时间戳，以及 WIT 航向失效时的安全失效处理。
- `comm_protocol` 负责让手动速度与 `GotoPose` 控制源互斥，并在急停、心跳超时和普通停止时取消活动目标。
- `main.c` 使用 TIM6 置位任务标志，主循环以 `__WFI()` 等待事件后延后执行任务；不在定时器中断中直接运行底盘业务。
- 详细配置、参数含义与上板验收流程见 `MDK-ARM/docs/下位机闭环与安全修复说明.md`。
