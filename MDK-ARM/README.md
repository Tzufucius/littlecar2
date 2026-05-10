# 物流小车 STM32F407ZGT6 下位机工程

## 1. 项目说明
本工程基于 STM32F407ZGT6 和 HAL 库，用于管理底盘、电机、舵机、传感器以及与上位机的通信。

相关设计资料统一放在 `docs/` 目录下。

## 2. 当前外设分配
| 外设 | 模式 | TX | RX | 当前用途 |
| --- | --- | --- | --- | --- |
| USART1 | Asynchronous | PA9 | PA10 | 调试串口 |
| USART2 | Asynchronous | PA2 | PA3 | WIT 陀螺仪 |
| USART3 | Asynchronous | PB10 | PB11 | 张大头步进电机 |
| UART4 | Asynchronous | PA0 | PA1 | 总线舵机 |
| UART5 | Asynchronous | PC12 | PD2 | OPS |
| USART6 | Asynchronous | PC6 | PC7 | Jetson |

## 3. 张大头（zdt）步进电机模块
- 协议实现位于 `Core\Inc\Emm_V5.h` 和 `Core\Inc\Emm_V5.h`（源自于官方例程）
- `Core/Src/zdt_stepper.c`是暂时搁置的手写代码（不使用）
- 当前使用 `USART3`
- 发送采用阻塞式串口发送
- 接收采用 `DMA + UART 空闲中断`
- 底盘上层运动接口位于 `Core/Inc/chassis_motion.h` 和 `Core/Src/chassis_motion.c`，基于 `Emm_V5` 多电机命令实现麦克纳姆轮运动控制
- 四个电机 ID、方向修正、各动作默认 RPM 和平滑加速度预设均在 `chassis_motion.h` 中配置

## 4. 总线舵机模块
- 协议实现位于 `Core/Inc/bus_servo.h` 和 `Core/Src/bus_servo.c`
- 当前使用 `UART4`
- 基线来源于旧项目 `ft_servo.c` 实际使用的发包方式
- 当前只实现设备层发送接口，不包含抓取、摆放、预设动作等流程层逻辑
- 当前不再参考已废弃的 `fashion_star_uart_servo` 方案

## 5. WIT 陀螺仪模块
- 协议实现位于 `Core/Inc/wit_imu.h` 和 `Core/Src/wit_imu.c`
- 当前使用 `USART2`
- 接收采用 `DMA + UART 空闲中断`
- 当前解析 `0x51` 加速度、`0x52` 角速度、`0x53` 姿态角三类标准数据帧
- 当前对外缓存 `accel_g / gyro_dps / angle_deg` 三组三轴数据，每组都带 `valid` 和 `updated_tick`

## 6. OPS 定位系统模块
- 协议实现位于 `Core/Inc/ops_sensor.h` 和 `Core/Src/ops_sensor.c`
- 当前使用 `UART5`
- 接收采用单字节中断方式
- 当前只解析 OPS 上行位姿数据帧，不做任何转发到 Jetson 或上位机的逻辑
- 当前缓存 `zangle/xangle/yangle/pos_x/pos_y/w_z` 以及对应的 `valid` 和更新时间

## 7. 开发约束
- 代码应写在 CubeMX 预留的 `USER CODE` 区域
- 外设驱动和业务流程尽量分层，避免把流程动作直接写入底层协议模块
- 每个目录下的说明文档需要随着模块演进同步更新
- 不要进行编译测试，用户会手动测试运行
- 代码编写简洁，不要额外复杂验证，直观实现用户指定的最小例程
- git 在 F:\Project\littleCar2\zhengdian\4-29\.git
- 不得直接修改硬件配置，若需要修改硬件配置需要告知用户在CubeMX中修改
