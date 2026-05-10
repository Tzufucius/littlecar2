# Src 目录说明

本目录存放 `Core` 层源文件实现。

- `main.c`：系统初始化和主循环
- `stm32f4xx_hal_msp.c`：底层 MSP 初始化
- `stm32f4xx_it.c`：中断入口
- `system_stm32f4xx.c`：系统时钟和内核支持
- `zdt_stepper.c`：张大头步进电机协议实现，使用 `USART3`
- `bus_servo.c`：总线舵机设备层实现，使用 `UART4`
- `wit_imu.c`：WIT 陀螺仪协议层实现，使用 `USART2`
- `host_rx.c`：统一处理 PC USART1 字节中断接收和 Jetson USART6 DMA+IDLE 原始接收，并通过 `printf` 输出来源、HEX 和 ASCII
- `jetson_debug.c`：Jetson USART6 原始接收调试实现，接收数据通过 `printf` 从 USART1 输出
- `ops_sensor.c`：OPS 定位系统协议层实现，使用 `UART5`
- `chassis_motion.c`：基于 Emm_V5 多电机命令实现底盘前进、后退、平移、原地旋转和差速转向
