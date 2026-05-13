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
- `comm_protocol`：上位机二进制协议解析、命令入队、ACK 回发。
- `advance_chassis`：基于 `drive_emm` 的麦克纳姆底盘高级运动接口。
- `advance_world`：维护 world 坐标系、全局位姿和 world/base 速度变换。
