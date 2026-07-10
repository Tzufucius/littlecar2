# 坐标系与 GotoPose 使用说明

## 1. 文档定位

本文说明当前下位机中 world 坐标系、车体坐标系、yaw 正方向、世界速度控制和 `GotoPose` 异步目标点控制的使用方法与运算逻辑。

相关代码：

| 模块 | 文件 | 职责 |
| --- | --- | --- |
| 底盘运动学 | `Core/Inc/advance_chassis.h`、`Core/Src/advance_chassis.c` | 车体速度到四轮 RPM 的麦轮解算 |
| 世界坐标 | `Core/Inc/advance_world.h`、`Core/Src/advance_world.c` | OPS / WIT 数据到 world 位姿的转换 |
| 高级运动 | `Core/Inc/advance_motion.h`、`Core/Src/advance_motion.c` | 世界速度和 GotoPose 状态机 |
| 通信协议 | `Core/Inc/comm_protocol.h`、`Core/Src/comm_protocol.c` | 上位机命令解析、ACK 和状态返回 |

## 2. 坐标系约定

### 2.1 world 坐标系

`world` 是软件建立的全局坐标系，不直接等同于 OPS 传感器上电后的原始坐标。

约定如下：

| 量 | 正方向 |
| --- | --- |
| `world +Y` | 软件原点建立时，小车车头朝向 |
| `world +X` | 软件原点建立时，小车右侧方向 |
| `yaw = 0 deg` | 小车车头朝向 `world +Y` |
| `yaw > 0` | 俯视小车，逆时针旋转 |

这样定义后，小车在 yaw 为 0 时，车体坐标和 world 坐标的方向一致：

- 车体向右 = `world +X`
- 车体向前 = `world +Y`
- 逆时针旋转 = yaw 增大

### 2.2 车体坐标系

车体坐标系随车转动，用于底盘速度控制：

| 参数 | 正方向 |
| --- | --- |
| `vx_right_mm_s` | 车体向右 |
| `vy_forward_mm_s` | 车体向前 |
| `wz_ccw_deg_s` | 俯视逆时针旋转 |

底盘物理速度入口：

```c
Chassis_SetBodyVelocityEx(vx_right_mm_s, vy_forward_mm_s, wz_ccw_deg_s, acc);
```

### 2.3 方向配置宏

实车调试时，如果某个方向与约定相反，应优先调整编译期宏，而不是修改控制算法。

单轮方向：

```c
#define CHASSIS_MOTOR_LF_SIGN (1)
#define CHASSIS_MOTOR_RF_SIGN (1)
#define CHASSIS_MOTOR_LR_SIGN (1)
#define CHASSIS_MOTOR_RR_SIGN (1)
```

整车轴向：

```c
#define CHASSIS_BODY_X_SIGN (1)
#define CHASSIS_BODY_Y_SIGN (1)
#define CHASSIS_BODY_WZ_SIGN (1)
```

传感器 yaw 方向：

```c
#define ADVANCE_WORLD_OPS_YAW_REVERSED (0)
#define ADVANCE_WORLD_WIT_YAW_REVERSED (0)
```

调试顺序建议：

1. 先低速确认四个电机 ID 与单轮方向。
2. 再确认 `vx > 0` 是否右移。
3. 确认 `vy > 0` 是否前进。
4. 确认 `wz > 0` 是否俯视逆时针旋转。
5. 最后确认 yaw 增大方向是否为俯视逆时针。

## 3. world 原点建立

OPS 和 WIT 可能保留自身历史读数，因此软件不假设传感器上电后就是零点。

建立 world 原点的入口：

```c
AdvanceWorld_ResetOrigin();
```

调用该函数时，模块会记录当前 OPS 位置、OPS yaw 和 WIT yaw 作为本次运行的软件原点。之后 `AdvanceWorld_GetPose()` 返回的是相对该软件原点的 world 位姿。

要求：

- 小车应静止。
- OPS 位姿有效。
- WIT yaw 数据有效。
- 建议在实车摆正、传感器稳定后再重置原点。

## 4. OPS / WIT 到 world 位姿

`advance_world` 的核心任务是把传感器原始数据转换为统一 world 位姿。

简化理解：

```text
OPS 原始位置 / OPS 原始 yaw / WIT 原始 yaw
    -> 方向符号修正
    -> 扣除 ResetOrigin 时记录的原点偏移
    -> 得到 world x/y/yaw
```

当前位置输出：

```c
WorldPose2D_t pose;
AdvanceWorld_GetPose(&pose);
```

`WorldPose2D_t` 中关键字段：

| 字段 | 单位 | 含义 |
| --- | --- | --- |
| `x_mm` | mm | 当前 world X |
| `y_mm` | mm | 当前 world Y |
| `yaw_deg` | deg | 当前 world yaw |
| `valid` | 0/1 | 位姿是否有效 |
| `origin_ready` | 0/1 | 软件原点是否建立 |
| `updated_tick` | ms | 位姿更新时间 |

当前实现已提供 OPS 安装补偿配置：`ADVANCE_WORLD_OPS_X_REVERSED`、`ADVANCE_WORLD_OPS_Y_REVERSED`、`ADVANCE_WORLD_OPS_XY_SWAPPED`、`ADVANCE_WORLD_OPS_YAW_OFFSET_DEG`、`ADVANCE_WORLD_OPS_OFFSET_X_MM`、`ADVANCE_WORLD_OPS_OFFSET_Y_MM`。偏移量以底盘旋转中心为基准，软件会先把 OPS 传感器坐标换算为底盘中心坐标，再建立 world 原点。

`WorldPose2D_t` 同时保存位置更新时间 `updated_tick` 与航向更新时间 `yaw_updated_tick`。原点建立时若选用了 WIT yaw，则 WIT 失效会使 world 位姿失效，而不会静默切回 OPS yaw，避免两个零点基准不同导致航向跳变。

## 5. 世界速度到车体速度

上位机发送 world 速度时，希望小车沿固定场地方向运动，而不是沿车头方向运动。

接口：

```c
AdvanceMotion_SetWorldVelocityEx(vx_world_mm_s, vy_world_mm_s, wz_ccw_deg_s, acc);
```

转换逻辑：

1. 读取当前 world yaw。
2. 将 world 平移速度旋转到车体坐标。
3. 调用 `Chassis_SetBodyVelocityEx()` 输出到底盘。

由于本工程约定 `yaw = 0` 时车头朝 `world +Y`，所以速度变换以车体前方轴为 `+Y`。代码中的 `AdvanceWorld_WorldToBodyVelocity()` 封装了这一步。

直观效果：

| 当前 yaw | 发送 world `+Y` | 车体应执行 |
| --- | --- | --- |
| `0 deg` | 场地向前 | 车体前进 |
| `90 deg` | 场地向前 | 车体向右或向左补偿，取决于当前朝向 |
| `-90 deg` | 场地向前 | 车体向相反侧补偿 |

目标是让运动方向固定在 world 坐标，而不是固定在车头坐标。

## 6. 麦轮速度解算

`Chassis_SetBodyVelocityEx()` 会把车体速度转换为四轮 RPM。

输入：

```text
vx = 车体右移速度，mm/s
vy = 车体前进速度，mm/s
wz = 车体逆时针角速度，deg/s
```

处理步骤：

1. 应用整车方向宏：`CHASSIS_BODY_X_SIGN`、`CHASSIS_BODY_Y_SIGN`、`CHASSIS_BODY_WZ_SIGN`。
2. 使用轮半径、半车长、半车宽、减速比计算四轮目标 RPM。
3. 对四轮混合结果做整体等比例缩放，保证最大绝对 RPM 不超过 `CHASSIS_MAX_RPM`。
4. 应用单轮方向宏 `CHASSIS_MOTOR_*_SIGN`。
5. 通过 `drive_emm` 多电机同步命令发送。

整体等比例缩放很重要。它不会改变四轮之间的比例关系，因此不会像单轮独立截断那样破坏运动方向。

## 7. GotoPose 定位

`GotoPose` 是异步目标点控制。上位机发送目标后，STM32 只在 ACK 中表示“已接收”，不表示“已经到达”。目标是否完成需要查询状态。

代码入口：

```c
AdvanceMotion_GotoPoseEx(&goal, acc);
AdvanceMotion_Poll();
AdvanceMotion_GetStatus(&status);
AdvanceMotion_Cancel();
```

`main.c` 已在 `HostRx_Poll()` 后周期调用 `AdvanceMotion_Poll()`。上位机命令在本轮被处理后可立即进入控制状态机；急停与取消命令会先撤销活动目标，阻止下一控制周期重新下发速度。

## 8. GotoPose 目标结构

目标结构为 `WorldGoalPose2D_t`：

| 字段 | 单位 | 说明 |
| --- | --- | --- |
| `x_mm` | mm | world 目标 X |
| `y_mm` | mm | world 目标 Y |
| `yaw_deg` | deg | 目标 yaw，俯视逆时针为正 |
| `vmax_mm_s` | mm/s | 平移速度上限 |
| `wmax_deg_s` | deg/s | 旋转速度上限 |
| `timeout_ms` | ms | 目标超时，0 表示不启用目标超时 |
| `goal_flags` | bit mask | bit0 启用 yaw 控制 |

默认控制参数在 `advance_motion.h`：

| 宏 | 默认值 | 含义 |
| --- | ---: | --- |
| `ADVANCE_MOTION_CONTROL_PERIOD_MS` | 20 | 控制周期 |
| `ADVANCE_MOTION_KP_POS` | 1.0 | 位置 P 控制系数 |
| `ADVANCE_MOTION_KP_YAW` | 2.0 | yaw P 控制系数 |
| `ADVANCE_MOTION_POS_TOLERANCE_MM` | 20.0 | 到达位置阈值 |
| `ADVANCE_MOTION_YAW_TOLERANCE_DEG` | 2.0 | 到达 yaw 阈值 |
| `ADVANCE_MOTION_ARRIVE_HOLD_MS` | 150 | 到达保持时间 |
| `ADVANCE_MOTION_POSE_TIMEOUT_MS` | 100 | 位姿超时阈值 |
| `ADVANCE_MOTION_DEFAULT_VMAX_MM_S` | 200.0 | 默认平移速度上限 |
| `ADVANCE_MOTION_DEFAULT_WMAX_DEG_S` | 90.0 | 默认旋转速度上限 |

如果 `vmax_mm_s <= 0` 或 `wmax_deg_s <= 0`，状态机会使用默认速度上限。

## 9. GotoPose 控制运算逻辑

状态机每 `ADVANCE_MOTION_CONTROL_PERIOD_MS` 执行一次控制计算。

### 9.1 前置检查

每次控制先检查：

1. 当前是否有活动目标。
2. world 原点是否建立。
3. 当前位姿是否有效。
4. 当前位姿是否超过 `ADVANCE_MOTION_POSE_TIMEOUT_MS`。
5. 是否超过目标 `timeout_ms`。

如果前置条件失败，状态机会平滑停车并进入对应状态：

| 条件 | 状态 |
| --- | --- |
| 原点未建立 | `NO_ORIGIN` |
| 位姿无效或超时 | `NO_POSE` |
| 目标超时 | `TIMEOUT` |
| 主动取消 | `CANCELED` |

### 9.2 位置误差

位置误差在 world 坐标下计算：

```text
error_x = goal.x_mm - pose.x_mm
error_y = goal.y_mm - pose.y_mm
position_error = sqrt(error_x^2 + error_y^2)
```

### 9.3 平移速度 P 控制

用 P 控制生成 world 平移速度：

```text
vx_world = error_x * ADVANCE_MOTION_KP_POS
vy_world = error_y * ADVANCE_MOTION_KP_POS
```

然后按二维向量模长限幅：

```text
speed = sqrt(vx_world^2 + vy_world^2)
if speed > vmax:
    scale = vmax / speed
    vx_world *= scale
    vy_world *= scale
```

这种限幅方式会保持速度方向不变，只降低速度大小。

### 9.4 yaw 误差与旋转速度

只有 `goal_flags` 设置 `ADVANCE_MOTION_GOAL_USE_YAW` 时才控制 yaw。

yaw 误差先 wrap 到 `[-180, 180]`：

```text
yaw_error = wrap(goal.yaw_deg - pose.yaw_deg)
```

再用 P 控制生成角速度：

```text
wz_ccw = yaw_error * ADVANCE_MOTION_KP_YAW
```

最后限幅到 `wmax_deg_s`：

```text
if wz_ccw > wmax: wz_ccw = wmax
if wz_ccw < -wmax: wz_ccw = -wmax
```

如果没有启用 yaw 控制：

```text
wz_ccw = 0
```

### 9.5 world 速度输出

状态机得到 `vx_world`、`vy_world`、`wz_ccw` 后调用内部世界速度输出：

```text
world velocity
    -> AdvanceWorld_WorldToBodyVelocity()
    -> Chassis_SetBodyVelocityEx()
    -> 四轮 RPM
    -> drive_emm
```

这保证 `GotoPose` 的目标点始终基于 world 坐标，而不是车头坐标。

### 9.6 到达判定

位置到达条件：

```text
position_error <= ADVANCE_MOTION_POS_TOLERANCE_MM
```

若启用 yaw，还需要：

```text
abs(yaw_error) <= ADVANCE_MOTION_YAW_TOLERANCE_DEG
```

首次满足条件时会立即向底盘下发零速度；随后不会立刻进入 `ARRIVED`，而是需要连续保持：

```text
ADVANCE_MOTION_ARRIVE_HOLD_MS
```

这样可以避免位姿短暂抖动导致误判到达。

## 10. 状态查询协议

底盘命令位于 `CMDSET_CHASSIS = 0x03`。

| CmdID | 命令 | Payload 长度 | 说明 |
| ---: | --- | ---: | --- |
| `0x07` | `CHASSIS_GOTO_POSE` | 22 | 启动异步目标点 |
| `0x08` | `CHASSIS_CANCEL_GOAL` | 0 | 取消目标 |
| `0x09` | `CHASSIS_RESET_WORLD_ORIGIN` | 0 | 重置 world 原点 |
| `0x0A` | `CHASSIS_GET_WORLD_POSE` | 0 | 预留 |
| `0x0B` | `CHASSIS_GET_MOTION_STATUS` | 0 | 查询运动状态 |

`0x07` Payload：

| 偏移 | 字段 | 类型 | 单位 |
| ---: | --- | --- | --- |
| 0 | `x_mm` | `int32_t` | mm |
| 4 | `y_mm` | `int32_t` | mm |
| 8 | `yaw_cdeg` | `int32_t` | 0.01 deg |
| 12 | `vmax_mm_s` | `int16_t` | mm/s |
| 14 | `wmax_cdeg_s` | `int16_t` | 0.01 deg/s |
| 16 | `timeout_ms` | `uint32_t` | ms |
| 20 | `goal_flags` | `uint8_t` | bit0 启用 yaw |
| 21 | `acc` | `uint8_t` | Emm 加速度参数 |

`0x0B` 会先返回 ACK，再返回同 `Seq/CmdSet/CmdID` 的 `MSG_DATA`。状态包字段详见 `docs/上下位机通信协议.md`。

## 11. 上位机使用流程

推荐流程：

1. 上电后等待 OPS / WIT 数据稳定。
2. 小车静止，发送 `CHASSIS_RESET_WORLD_ORIGIN`。
3. 低速发送 `CHASSIS_SET_WORLD_VELOCITY` 验证 world 方向。
4. 发送小距离 `CHASSIS_GOTO_POSE`，例如 `(300, 0)` 或 `(0, 300)`。
5. 周期查询 `CHASSIS_GET_MOTION_STATUS`。
6. 状态进入 `ARRIVED` 后发送下一个目标。
7. 异常时发送 `CHASSIS_CANCEL_GOAL` 或安全停止命令。

## 12. 实车调试建议

建议按以下顺序排查，不要一开始就调整控制算法：

1. 单轮测试，确认电机 ID。
2. 单轮正反方向测试，调整 `CHASSIS_MOTOR_*_SIGN`。
3. 车体右移、前进、逆时针旋转测试，调整 `CHASSIS_BODY_*_SIGN`。
4. 静止重置 world 原点。
5. 旋转小车，确认 yaw 逆时针为正；必要时调整 `ADVANCE_WORLD_*_YAW_REVERSED`。
6. 测试 world `+Y` 在 yaw 为 0、90、-90 度时是否保持场地方向一致。
7. 低速测试 `GotoPose(300, 0)` 和 `GotoPose(0, 300)`。
8. 最后再启用 yaw 控制，测试小角度目标。

如果某项方向相反，优先修改对应方向宏；只有当所有方向宏确认无误后，才考虑物理参数标定或控制参数调整。
