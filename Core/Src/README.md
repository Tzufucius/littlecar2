# Src 目录说明

本目录存放 `Core` 层源文件实现。

- `main.c`：系统初始化、外设初始化、主循环和 HAL 回调分发。
- `stm32f4xx_hal_msp.c`：底层 MSP 初始化。
- `stm32f4xx_it.c`：中断入口。
- `system_stm32f4xx.c`：系统时钟和内核支持。
- `drive_emm.c`：张大头 Emm_V5 步进闭环驱动协议实现，使用 `USART3`，并提供底盘和机械臂电机的反馈监督。
- `drive_bus_servo.c`：总线舵机控制实现，使用 `UART4`；已预留反馈状态读取，等待实物回读协议接入。
- `sensor_wit.c`：WIT / HWT905 IMU DMA + IDLE 接收和数据解析，使用 `USART2`。
- `sensor_ops.c`：OPS 定位系统单字节接收和位姿解析，使用 `UART5`。
- `sensor_limit.c`：光电限位的静态 GPIO 映射与电平读取实现；不包含 GPIO 初始化、EXTI 回调、消抖或电机控制。
- `comm_host.c`：PC / Jetson 原始接收、hex/ascii 打印和协议解析桥接。
- `comm_protocol.c`：上位机二进制协议找帧、CRC 校验、命令分发、ACK 回发和控制权租约。
- `advance_chassis.c`：基于 `drive_emm` 多电机命令实现底盘前进、后退、平移、旋转和差速转向。
- `advance_world.c`：基于 OPS 原始位姿维护工程 world 坐标系，并提供 world/base 坐标变换。
- `advance_arm.c`：完全开环的机械臂阻塞流程；每个原子动作直接发送命令并固定等待 1000 ms，不读取反馈、不检查限位，也不维护软件坐标。
- `advance_test.c`：下位机人工联调实现；仅保留阻塞测试用于基础方向和丝杆电机确认。
