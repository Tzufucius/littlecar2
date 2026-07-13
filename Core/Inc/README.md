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

- `0x11 ARM_CONFIG`：废弃兼容命令，仅接受与固定机械臂配置一致的参数并触发人工置零流程。
- `0x12 ARM_PICK`、`0x13 ARM_PLACE`：零 Payload 启动固定非阻塞取放动作；旧 36 字节格式仅兼容一个发布周期。
- `0x16 ARM_RESET_ZERO`：操作者人工归零后建立两轴软件坐标。
- `0x14 ARM_ABORT`：停止两步进轴并使坐标失效。
- `0x15 ARM_GET_STATUS`：返回任务状态、坐标有效性、当前/目标脉冲及故障标志。
