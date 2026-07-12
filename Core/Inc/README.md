# Inc 目录说明

本目录存放 `Core` 层头文件。

- `main.h`：主程序公共声明。
- `stm32f4xx_hal_conf.h`：HAL 组件配置。
- `stm32f4xx_it.h`：中断入口声明。
- `drive_emm.h`：张大头 Emm_V5 步进闭环驱动接口。
- `drive_bus_servo.h`：总线舵机控制与预留位置反馈接口；实际回读协议待接入。
- `sensor_wit.h`：WIT / HWT905 IMU 数据接口。
- `sensor_ops.h`：OPS 定位系统数据接口。
- `comm_pc.h`：PC / Jetson 原始接收桥接接口。
- `comm_jetson.h`：Jetson 原始接收调试兼容接口。
- `comm_protocol.h`：上位机二进制协议解析接口。
- `advance_chassis.h`：麦克纳姆底盘高级运动接口。
- `advance_world.h`：全局坐标系、world 位姿和坐标变换接口。
- `advance_arm.h`：机械臂闭环控制器接口，含人工置零后的坐标有效性、非阻塞抓取/放置任务与故障状态。

`CMDSET_SERVO (0x04)` 的机械臂扩展命令：

- `0x11 ARM_CONFIG`：配置升降/伸缩步进电机、夹爪舵机、方向、速度、加速度和位置容差；配置后等待两路新鲜反馈自动确认人工零点。
- `0x12 ARM_PICK`、`0x13 ARM_PLACE`：启动非阻塞抓取或放置状态机。
- `0x14 ARM_ABORT`：停止两步进轴并使坐标失效。
- `0x15 ARM_GET_STATUS`：返回任务状态、坐标有效性、当前/目标脉冲及故障标志。
