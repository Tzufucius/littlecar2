# 物流小车 STM32F407ZGT6 下位机工程

## 1. 项目说明

仓库地址：https://github.com/Tuzfucius/littlecar2
本工程基于 STM32F407ZGT6 和 HAL 库，用于管理底盘电机、总线舵机、WIT IMU、OPS 定位系统，以及 PC / Jetson 上位机通信。
我们使用有4个步进电机来控制底盘的前进后退等移动运动。由三个舵机来控制一些旋转的运动，包括夹爪的开合物料盘的旋转以及机械臂的旋转。

当前 README.md 所在目录是 Keil 工程文件 `4-29.uvprojx` 所在的 `MDK-ARM` 目录。上层目录为 `4-29`，是整个工程目录，主要包含：

| 目录 | 作用 |
| --- | --- |
| `Core/` | STM32 下位机业务代码、HAL 回调分发和模块接口 |
| `Drivers/` | STM32 HAL / CMSIS 驱动 |
| `MDK-ARM/` | Keil 工程文件、EIDE 配置和下位机文档入口 |
| `jetson/` | Jetson Orin Nano 或 Windows PC 侧上位机代码 |

下位机详细设计资料统一放在 `docs/` 目录下。每个源码目录也应保留对应 `README.md`，用于说明该目录下代码的职责边界。

## 2. 命名规范

当前工程采用模块前缀命名，后续新增文件、类型、宏和函数时应优先遵守以下规则：

| 前缀 | 使用范围 | 当前示例 |
| --- | --- | --- |
| `sensor_` | 传感器接入、解析和数据缓存 | `sensor_wit.*`、`sensor_ops.*` |
| `drive_` | 底层执行器、驱动协议和设备控制 | `drive_emm.*`、`drive_bus_servo.*` |
| `advance_` | 高级运动、组合动作、坐标系和业务能力封装 | `advance_chassis.*`、`advance_world.*` |
| `comm_` | PC / Jetson 通信、协议解析、收发桥接 | `comm_pc.*`、`comm_jetson.*`、`comm_protocol.*` |
| `car_` | 车辆自身状态、属性和全局数据视图 | `car_pose.*` |

约束：

- 新增 `.c/.h` 文件应按上述前缀命名，不再使用旧的 `chassis_motion`、`bus_servo`、`host_rx`、`wit_imu`、`ops_sensor` 等文件名。
- 对外 API 可以保留硬件或设备语义清晰的函数名，例如 `Chassis_*`、`WIT_*`、`OPS_*`、`BusServo_*`；文件名和模块归属必须按前缀分类。
- 宏、状态枚举和内部辅助函数应尽量跟随所属模块前缀，避免跨模块出现同名状态或含义不清的全局符号。
- 文档、工程清单和代码引用应同步使用新文件名，避免新旧命名混用。

## 3. 当前外设分配

| 外设 | 模式 | TX | RX | 当前用途 | 对应模块 |
| --- | --- | --- | --- | --- | --- |
| USART1 | Asynchronous | PA9 | PA10 | 调试串口 / PC 原始接收 | `comm_pc.*` |
| USART2 | Asynchronous | PA2 | PA3 | WIT / HWT905 IMU | `sensor_wit.*` |
| USART3 | Asynchronous | PB10 | PB11 | 张大头 Emm_V5 步进闭环电机 | `drive_emm.*` |
| UART4 | Asynchronous | PA0 | PA1 | 总线舵机 | `drive_bus_servo.*` |
| UART5 | Asynchronous | PC12 | PD2 | OPS 定位系统 | `sensor_ops.*` |
| USART6 | Asynchronous | PC6 | PC7 | Jetson 原始接收 / 通信协议 | `comm_pc.*`、`comm_jetson.*`、`comm_protocol.*` |

对于电机和舵机，我们通过id编号进行配置
- 步进电机
    - 1 左前
    - 2 左后
    - 3 右前
    - 4 右后
    - 5 控制丝杆上下运动
    - 6 控制机械臂前后平移
- 舵机
    - 1 物机械臂旋转
    - 2 机械臂抓取
    - 3 料盘旋转

## 4. 模块说明

### 4.1 步进电机驱动

- 文件：`Core/Inc/drive_emm.h`、`Core/Src/drive_emm.c`
- 用途：张大头 Emm_V5 步进闭环电机底层协议。
- 串口：`USART3`
- 发送：使用 `HAL_UART_Transmit_DMA`，工程中已配置 `USART3 TX -> DMA1_Stream3`。
- 典型接口：`drive_emm_Vel_Control()`、`drive_emm_Pos_Control()`、`drive_emm_MMCL_Vel_Control()`、`drive_emm_MMCL_Stop_Now()`。

### 4.2 总线舵机驱动

- 文件：`Core/Inc/drive_bus_servo.h`、`Core/Src/drive_bus_servo.c`
- 用途：总线舵机设备层控制。
- 串口：`UART4`
- 当前边界：提供发送控制接口和统一回调入口，暂不包含抓取、摆放、预设动作等高级流程。
- 典型接口：`BusServo_Init()`、`BusServo_SetPosition()`、`BusServo_SetPositionEx()`、`BusServo_SendGroup()`。

### 4.3 WIT IMU 传感器

- 文件：`Core/Inc/sensor_wit.h`、`Core/Src/sensor_wit.c`
- 用途：WIT / HWT905 IMU 接收、找帧、校验和数据缓存。
- 串口：`USART2`
- 接收：`DMA + UART 空闲中断`
- 当前解析：`0x51` 加速度、`0x52` 角速度、`0x53` 姿态角三类标准帧。
- 对外数据：`accel_g`、`gyro_dps`、`angle_deg`，每组三轴数据带 `valid` 和 `updated_tick`。
- 典型接口：`WIT_Init()`、`WIT_Poll()`、`WIT_OnUartRxEvent()`、`WIT_GetData()`。

### 4.4 OPS 定位传感器

- 文件：`Core/Inc/sensor_ops.h`、`Core/Src/sensor_ops.c`
- 用途：OPS 全方位定位系统上行位姿帧接收和缓存。
- 串口：`UART5`
- 接收：单字节中断。
- 当前边界：只解析 OPS 上行位姿数据帧，不转发到 Jetson，不下发 OPS 配置命令。
- 对外数据：`zangle_deg`、`xangle_deg`、`yangle_deg`、`pos_x_mm`、`pos_y_mm`、`w_z_dps`。
- 典型接口：`OPS_Init()`、`OPS_Poll()`、`OPS_OnByteReceived()`、`OPS_GetPose()`、`OPS_GetPoseRef()`。

### 4.5 底盘高级运动

- 文件：`Core/Inc/advance_chassis.h`、`Core/Src/advance_chassis.c`
- 用途：基于 `drive_emm` 多电机同步命令实现麦克纳姆轮底盘动作。
- 当前能力：前进、后退、左右平移、左右旋转、差速转向、四轮 RPM 直接控制、三轴麦克纳姆速度合成。
- 配置位置：四个电机 ID、方向修正、默认 RPM、默认加速度和预设动作参数集中在 `advance_chassis.h`。
- 典型接口：`Chassis_Enable()`、`Chassis_Stop()`、`Chassis_SetMotorRPMEx()`、`Chassis_MoveMecanumEx()`。

### 4.6 全局坐标系

- 文件：`Core/Inc/advance_world.h`、`Core/Src/advance_world.c`
- 用途：维护工程统一的 world 坐标系，完成 OPS 原始坐标到 world 位姿的转换，并提供 world/base 速度变换。
- 坐标定义：`world +Y` 为小车初始车头方向，`world +X` 为初始右侧，`yaw=0` 朝 `world +Y`，yaw 逆时针为正。
- 角度方向：俯视小车时，逆时针旋转为角度增大。若 OPS 或 WIT 因安装方向导致角度增减相反，在 `advance_world.h` 中将 `ADVANCE_WORLD_OPS_YAW_REVERSED` 或 `ADVANCE_WORLD_WIT_YAW_REVERSED` 改为 `1`。
- 初始化：OPS / WIT 掉电后会保留自身历史状态，软件不假设其上电为零；OPS 静止初始化后调用 `AdvanceWorld_ResetOrigin()`，将当前 OPS 位置、OPS 航向和 WIT yaw 记录为本次 world 坐标系零点。
- 典型接口：`AdvanceWorld_Init()`、`AdvanceWorld_Poll()`、`AdvanceWorld_GetPose()`、`AdvanceWorld_WorldToBodyVelocity()`。

### 4.7 PC / Jetson 通信

- 文件：`Core/Inc/comm_pc.h`、`Core/Src/comm_pc.c`
- 用途：PC 和 Jetson 双路原始接收、日志输出和协议桥接。
- PC：`USART1`，`115200 8N1`，`DMA Circular + UART 空闲中断`。
- Jetson：`USART6`，`115200 8N1`，`DMA Circular + UART 空闲中断`。
- `comm_pc.c` 会把原始字节输入 `comm_protocol.c`，用于后续上位机二进制协议解析。
- 典型接口：`HostRx_InitPc()`、`HostRx_InitJetson()`、`HostRx_Poll()`、`HostRx_OnUartRxEvent()`。

### 4.8 Jetson 调试兼容层

- 文件：`Core/Inc/comm_jetson.h`、`Core/Src/comm_jetson.c`
- 用途：保留 Jetson 原始接收调试兼容入口。
- 当前建议：新通信逻辑优先进入 `comm_pc.*` 和 `comm_protocol.*`，避免再扩展独立调试分支。

### 4.9 上位机协议层

- 文件：`Core/Inc/comm_protocol.h`、`Core/Src/comm_protocol.c`
- 用途：PC / Jetson 共用的二进制协议找帧、CRC 校验、命令队列、ACK 回发和主循环分发。
- 当前边界：UART 回调只搬运原始字节，完整命令在 `HostProtocol_Poll()` 中执行。
- 典型接口：`HostProtocol_RegisterSource()`、`HostProtocol_OnBytes()`、`HostProtocol_Poll()`。

### 4.10 车辆状态视图

- 文件：`Core/Inc/car_pose.h`、`Core/Src/car_pose.c`
- 用途：汇总车辆自身位姿相关数据指针，作为上层读取 IMU 和 OPS 数据的统一入口。
- 当前数据：`carpose_imu` 指向 WIT 数据，`carpose_ops` 指向 OPS 位姿数据。
- 典型接口：`CarPose_Init()`。

## 5. 主循环与回调边界

- `main.c` 负责系统初始化、外设初始化、模块初始化、主循环轮询和 HAL 回调分发。
- `HAL_UARTEx_RxEventCallback()` 中只分发 DMA / IDLE 接收事件，不直接执行业务动作。
- `HAL_UART_RxCpltCallback()` 中分发单字节中断接收，例如 OPS 和总线舵机。
- `HAL_UART_ErrorCallback()` 中按串口来源调用对应模块错误处理函数并重启接收。
- 上位机命令的实际执行应进入 `HostProtocol_Poll()`，再调用 `advance_`、`drive_`、`sensor_` 或 `car_` 相关接口。

## 6. 调试边界

- PC 端调试日志通过 `USART1 printf` 输出，因此 PC 端可能同时看到 STM32 日志和 PC 输入回显日志。
- PC 发送 `hello\r\n` 时，正常输出应包含 `PC RX len=7 hex=68 65 6C 6C 6F 0D 0A ascii=hello\r\n`。
- Jetson 发送 `hello\r\n` 时，正常输出应包含 `JETSON RX len=7 hex=68 65 6C 6C 6F 0D 0A ascii=hello\r\n`。
- 若输出 `PC ERR code=0x...`、`JETSON ERR code=0x...` 或 WIT Frame Error，优先检查波特率、TX/RX 交叉、共地、电平、串口设备名和串口占用。
- 当前 README 只描述下位机工程结构和模块边界，不替代 `docs/` 中的具体协议文档。

## 7. 开发约束（重要）

- **仅允许阅读和修改三个文件夹下的代码内容，`Core`、`jetson`、`MDK-ARM`**
- 代码应写在 CubeMX 预留的 `USER CODE` 区域，避免再次生成代码时丢失。
- 不得直接修改硬件配置；如需调整引脚、DMA、NVIC 或串口参数，应先说明需要用户在 CubeMX 中修改。
- 外设驱动和业务流程必须分层，不要把流程动作直接写入底层协议模块。
- 新增模块必须优先判断归属前缀：传感器用 `sensor_`，控制驱动用 `drive_`，高级方法用 `advance_`，通信用 `comm_`，车辆自身属性用 `car_`。
- 每个目录下的说明文档需要随着模块演进同步更新。
- 本工程通常不在 Codex 环境内编译，下位机运行由用户手动上板测试。
- 代码编写保持简洁，优先实现用户指定的最小例程，不额外加入复杂验证链路。
- Git 仓库根目录位于 `F:\Project\littleCar2\zhengdian\4-29\.git`。

## 8. 世界速度与方向配置

底盘现在保留原有 RPM 调试接口，并新增物理速度接口：

- `Chassis_SetBodyVelocityEx(vx_right_mm_s, vy_forward_mm_s, wz_ccw_deg_s, acc)`
- `AdvanceMotion_SetWorldVelocityEx(vx_world_mm_s, vy_world_mm_s, wz_ccw_deg_s, acc)`

方向约定：

- `vx_right_mm_s > 0`：车体向右。
- `vy_forward_mm_s > 0`：车体向前。
- `wz_ccw_deg_s > 0`：俯视逆时针旋转。

若实车方向与约定相反，优先调整 `Core/Inc/advance_chassis.h` 中的编译期宏：

- `CHASSIS_MOTOR_*_SIGN`：单个电机正反方向。
- `CHASSIS_BODY_X_SIGN`：整体右移方向。
- `CHASSIS_BODY_Y_SIGN`：整体前进方向。
- `CHASSIS_BODY_WZ_SIGN`：整体逆时针旋转方向。
- `ADVANCE_WORLD_OPS_YAW_REVERSED` / `ADVANCE_WORLD_WIT_YAW_REVERSED`：OPS / WIT yaw 读数方向。

实车调试建议先低速确认：单轮 ID、前进、右移、逆时针旋转，再验证 world `+Y` 在不同 yaw 下方向保持一致。

## 9. GotoPose 异步目标点控制

本次新增 `advance_motion` 目标点状态机，但不修改 `main.c` 测试逻辑。模块只提供 `AdvanceMotion_Poll()` 调度入口；实车接入时需要在主循环中周期调用，目标才会持续推进。

新增接口：

- `AdvanceMotion_GotoPose(const WorldGoalPose2D_t *goal)`：按默认加速度接收目标点。
- `AdvanceMotion_GotoPoseEx(const WorldGoalPose2D_t *goal, uint8_t acc)`：接收目标点并指定 Emm 加速度参数。
- `AdvanceMotion_Poll()`：异步状态机轮询，内部按 `ADVANCE_MOTION_CONTROL_PERIOD_MS` 自调度。
- `AdvanceMotion_Cancel()`：取消当前目标并平滑停车。
- `AdvanceMotion_GetStatus()`：读取状态、当前位姿、误差和活动目标摘要。

状态机状态：

| 状态 | 含义 |
| --- | --- |
| `ADVANCE_MOTION_STATE_IDLE` | 空闲 |
| `ADVANCE_MOTION_STATE_RUNNING` | 正在执行目标 |
| `ADVANCE_MOTION_STATE_ARRIVED` | 到达并保持满足阈值 |
| `ADVANCE_MOTION_STATE_TIMEOUT` | 超过目标超时时间 |
| `ADVANCE_MOTION_STATE_NO_POSE` | 位姿无效或超时 |
| `ADVANCE_MOTION_STATE_NO_ORIGIN` | world 原点未建立 |
| `ADVANCE_MOTION_STATE_CANCELED` | 被取消或被直接速度命令打断 |

控制参数集中在 `Core/Inc/advance_motion.h`：

- `ADVANCE_MOTION_CONTROL_PERIOD_MS = 20`
- `ADVANCE_MOTION_KP_POS = 1.0f`
- `ADVANCE_MOTION_KP_YAW = 2.0f`
- `ADVANCE_MOTION_POS_TOLERANCE_MM = 20.0f`
- `ADVANCE_MOTION_YAW_TOLERANCE_DEG = 2.0f`
- `ADVANCE_MOTION_ARRIVE_HOLD_MS = 150`
- `ADVANCE_MOTION_POSE_TIMEOUT_MS = 100`
- `ADVANCE_MOTION_DEFAULT_VMAX_MM_S = 200.0f`
- `ADVANCE_MOTION_DEFAULT_WMAX_DEG_S = 90.0f`

Yaw 控制默认由目标 `goal_flags` 决定。设置 `ADVANCE_MOTION_GOAL_USE_YAW` 时，状态机同时控制目标 yaw；不设置时只控制 x/y，`wz` 输出为 0。

协议新增底盘命令：

| CmdID | 命令 | Payload 长度 | 说明 |
| ---: | --- | ---: | --- |
| `0x07` | `CHASSIS_GOTO_POSE` | 22 | 接收 world 目标点并启动异步状态机 |
| `0x08` | `CHASSIS_CANCEL_GOAL` | 0 | 取消当前目标并平滑停车 |
| `0x0B` | `CHASSIS_GET_MOTION_STATUS` | 0 | ACK 后返回 `MSG_DATA` 状态包 |

`CHASSIS_GOTO_POSE` Payload：

| 偏移 | 字段 | 类型 | 单位 |
| ---: | --- | --- | --- |
| 0 | `x_mm` | `int32_t` | mm |
| 4 | `y_mm` | `int32_t` | mm |
| 8 | `yaw_cdeg` | `int32_t` | 0.01 deg |
| 12 | `vmax_mm_s` | `int16_t` | mm/s |
| 14 | `wmax_cdeg_s` | `int16_t` | 0.01 deg/s |
| 16 | `timeout_ms` | `uint32_t` | ms |
| 20 | `goal_flags` | `uint8_t` | bit0 表示启用 yaw |
| 21 | `acc` | `uint8_t` | Emm 加速度参数 |

状态查询 `0x0B` 会先返回 ACK，再返回同 `Seq/CmdSet/CmdID` 的 `MSG_DATA`，包含状态、active 标志、目标摘要、当前 world 位姿、位置误差、yaw 误差、elapsed、timeout 和更新时间。`0x0A` 预留给后续 world pose 查询。
