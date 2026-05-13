# Inc 目录说明

本目录存放 `Core` 层头文件。

- `main.h`：主程序公共声明。
- `stm32f4xx_hal_conf.h`：HAL 组件配置。
- `stm32f4xx_it.h`：中断入口声明。
- `drive_emm.h`：张大头 Emm_V5 步进闭环驱动接口。
- `drive_bus_servo.h`：总线舵机控制接口。
- `sensor_wit.h`：WIT / HWT905 IMU 数据接口。
- `sensor_ops.h`：OPS 定位系统数据接口。
- `comm_pc.h`：PC / Jetson 原始接收桥接接口。
- `comm_jetson.h`：Jetson 原始接收调试兼容接口。
- `comm_protocol.h`：上位机二进制协议解析接口。
- `advance_chassis.h`：麦克纳姆底盘高级运动接口。
- `advance_world.h`：全局坐标系、world 位姿和坐标变换接口。
