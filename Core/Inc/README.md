# Inc 目录说明

本目录存放 `Core` 层头文件。

- `main.h`：主程序公共声明。
- `stm32f4xx_hal_conf.h`：HAL 组件配置。
- `stm32f4xx_it.h`：中断入口声明。
- `drive_emm.h`：张大头 Emm_V5 步进闭环驱动接口。
- `drive_bus_servo.h`：总线舵机控制与预留位置反馈接口；实际回读协议待接入。
- `sensor_wit.h`：WIT / HWT905 IMU 数据接口。
- `sensor_ops.h`：OPS 定位系统数据接口。
- `sensor_limit.h`：PC0~PC3 光电限位读取接口、限位标识和可配置有效电平定义。
- `comm_common.h`：PC / Jetson 共用的通信来源类型。
- `comm_host.h`：PC / Jetson 原始接收桥接接口。
- `comm_protocol.h`：上位机二进制协议解析、来源回包与控制权租约接口。
- `advance_chassis.h`：麦克纳姆底盘高级运动接口。
- `advance_world.h`：全局坐标系、world 位姿和坐标变换接口。
- `advance_arm.h`：完全开环的阻塞式机械臂接口，提供夹爪、固定抓取/放置和停止控制。
- `advance_test.h`：下位机人工联调接口，仅提供阻塞式测试。

`CMDSET_SERVO (0x04)` 的机械臂扩展命令：

- `0x10 ARM_GRAB`：1 字节 Payload，`0` 打开夹爪，`1` 闭合夹爪；固定阻塞 1000 ms。
- `0x12 ARM_PICK`、`0x13 ARM_PLACE`：零 Payload，同步执行固定 5 秒取放流程。
- `0x14 ARM_ABORT`：停止两步进轴。
- `0x11 ARM_CONFIG`、`0x15 ARM_GET_STATUS`、`0x16 ARM_RESET_ZERO`：保留命令编号，但统一返回 `ACK_UNKNOWN_CMD`。
