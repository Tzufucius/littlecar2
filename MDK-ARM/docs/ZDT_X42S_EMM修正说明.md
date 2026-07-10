# ZDT_X42S Emm 固件协议修正说明

依据文件：`ZDT_X42S第二代闭环步进电机用户手册V1.0.3_251224.pdf`。

> 当前工程实际驱动文件为 `Core/Inc/drive_emm.h` 和 `Core/Src/drive_emm.c`，不再使用本文历史内容中的 `zdt_stepper`、`Emm_V5` 或 `chassis_motion` 文件名。完整的闭环与通信保护说明见 `docs/下位机闭环与安全修复说明.md`。

## 修正依据

- 串口 TTL/RS485 自由协议格式：`Addr + Code + Data + CheckSum`。
- 默认校验方式：固定字节 `0x6B`。
- 电机地址：`1-255`，`0` 为广播地址。
- Emm 固件速度模式命令：`Addr F6 Dir VelH VelL Acc Sync 6B`。
- Emm 固件位置模式命令：`Addr FD Dir VelH VelL Acc Pulse[4] Mode Sync 6B`。
- 立即停止命令：`Addr FE 98 Sync 6B`。
- 多机同步启动命令：`Addr FF 66 6B`，通常使用广播地址 `0x00`。

## 本次代码修正

- 在 `zdt_stepper.h` 中补齐 `ZDT_Dir`、`ZDT_Sync`、`ZDT_PosMode`、`ZDT_AckStatus` 类型。
- 将位置模式从旧的二值 `bool` 修正为手册定义的三态：
  - `ZDT_POS_RELATIVE_LAST_TARGET = 0x00`
  - `ZDT_POS_ABSOLUTE_ZERO = 0x01`
  - `ZDT_POS_RELATIVE_CURRENT = 0x02`
- 保留旧 `Datou_*` 函数作为兼容层，内部转调新的 `ZDT_*` 接口。
- 新增读取版本、读取实时速度、读取实时位置、读取状态标志、修改 ID、应答判断和 Emm 位置/速度解析接口。
- 按手册约束将 Emm 速度输入限制到 `0-3000RPM`，避免发送超范围值导致电机返回 `E2`。

## 当前接口示例

```c
ZDT_Enable(&huart3, 1, true, ZDT_SYNC_DISABLE);
HAL_Delay(100);

ZDT_SetSpeed(&huart3, 1, ZDT_DIR_CW, 100, 10, ZDT_SYNC_DISABLE);
HAL_Delay(2000);

ZDT_Stop(&huart3, 1, ZDT_SYNC_DISABLE);
```

读取实时速度：

```c
ZDT_ReadSpeed(&huart3, 1);
```

若收到 `01 35 01 05 DC 6B`，`ZDT_ParseSpeedEmm()` 会解析为 `-1500RPM`。

## 底盘运动层

当前工程新增 `chassis_motion` 作为 ZDT 电机之上的底盘运动层：

- 头文件：`Core/Inc/chassis_motion.h`
- 源文件：`Core/Src/chassis_motion.c`
- 说明文档：`docs/底盘运动控制说明.md`

该模块基于 `Emm_V5` 多电机命令实现麦克纳姆轮运动控制，提供前进、后退、左右平移、左右原地旋转、差速转向和通用麦克纳姆轮速度合成接口。

电机 ID、方向修正、默认 RPM 和平滑加速度预设均在 `chassis_motion.h` 中配置。

## 注意事项

- 项目当前仍使用 `USART3` 控制 ZDT 电机，波特率应保持 `115200, 8N1`。
- 固件必须处于 Emm 模式；X 固件的速度和位置命令格式不同，不能混用。
- 广播地址适合急停、同步启动等动作命令，不适合修改 ID 或普通读取命令。
