# protocol 目录说明

本目录是 Jetson 上位机到 STM32F407 下位机的通信协议包。它只新增通信能力，不改变现有视觉主流程。

## 协议对应关系

帧格式与 `MDK-ARM/docs/上下位机通信协议.md` 保持一致：

```text
5A A5 + Version + MsgType + CmdSet + CmdID + Seq_L + Seq_H + Length + Payload + CRC16
```

- `Version` 固定为 `0x01`。
- `Seq` 使用 `uint16_t` 小端编码，由上位机递增分配。
- `Length` 是 Payload 长度，范围 `0~255`。
- `CRC16` 使用 `CRC16-Modbus`，计算范围是 `Version` 到 Payload 末尾，即 `frame[2]` 起 `7 + payload_len` 字节。
- 多字节字段统一小端编码。

## 基本用法

Windows 手测可使用实际串口，例如：

```python
from protocol import Car

with Car(port="COM8", baudrate=115200) as car:
    car.system.ping()
    car.system.claim_control()
    car.heartbeat.start()
    car.safety.clear()
    car.chassis.enable(True)
    car.chassis.move_mecanum(forward=80, strafe=0, rotate=0, acc=10)
    car.chassis.stop(mode=1)
```

Jetson 上通常使用实际 UART 设备：

```python
from protocol import Car

car = Car(port="/dev/ttyTHS1", baudrate=115200)
car.system.ping()
```

## 对外接口

- `car.system.ping()`：发送 `SYS_PING`，等待 ACK。
- `car.system.heartbeat()`：发送 `SYS_HEARTBEAT`，Payload 为当前上位机毫秒时间。
- `car.system.claim_control()`：申请控制权租约；成功后才可发送运动和机械臂动作。
- `car.system.release_control()`：立即停车并释放控制权租约。
- `car.safety.estop(source=0)`：发送急停命令。
- `car.safety.safe_stop(mode=0)`：发送安全停止命令，`0` 为平滑停止，`1` 为立即停止。
- `car.safety.clear()`：清除可恢复安全状态。
- `car.chassis.enable(True)`：使能或失能底盘。
- `car.chassis.stop(mode=1)`：停止底盘。
- `car.chassis.set_motor_rpm(lf, rf, lr, rr, acc=10)`：设置四轮 RPM。
- `car.chassis.move_mecanum(forward, strafe, rotate, acc=10)`：发送麦克纳姆轮速度指令。
- `car.chassis.set_body_velocity(vx_mm_s, vy_mm_s, wz_cdeg_s, acc=10)`：发送车体坐标速度。
- `car.chassis.set_world_velocity(vx_mm_s, vy_mm_s, wz_cdeg_s, acc=10)`：发送 world 坐标速度。
- `car.chassis.goto_pose(WorldGoal(...))` / `cancel_goal()`：下发或取消异步目标点。
- `car.chassis.reset_world_origin()`：以当前有效传感器数据建立 world 原点。
- `car.chassis.get_motion_status()`：查询异步目标状态，返回 `MotionStatus`。
- `car.arm.grab()` / `release()`：控制固定标定的夹爪开合。
- `car.arm.pick()` / `place()`：同步执行固定取放动作；调用方应使用大于 4.5 秒的 ACK 超时。
- `car.arm.abort()`：停止两步进轴。阻塞动作执行期间，STM32 主循环不会立即处理该命令。
- `car.arm.reset_zero()` / `get_status()`：当前 STM32 固件不支持，调用会收到 `ACK_UNKNOWN_CMD`。
- `car.heartbeat.start()` / `stop()`：启动或停止默认 100 ms 的后台心跳服务。
- `car.read_status(timeout=0.0)`：读取一帧周期 `MSG_STATUS`，无数据时返回 `None`。

## 异常语义

- ACK 超时或 DATA 超时会抛出 `ProtocolTimeoutError`。
- ACK 返回非 `OK` 会抛出 `CommandRejectedError`，异常对象保留 `cmd_set`、`cmd_id`、`seq`、`result` 和 `detail`。
- 帧头、版本、长度或 CRC 不合法时，底层解析器会丢弃坏帧。

## 当前限制

当前 STM32 侧已实现 `SYSTEM`、`SAFETY`、`CHASSIS` 和固定参数的 `SERVO` 命令。`ARM_GRAB` 固定等待 500 ms，`ARM_PICK` 与 `ARM_PLACE` 均为约 4.5 秒的阻塞调用，ACK 仅在动作结束后发送；`ARM_CONFIG`、`ARM_GET_STATUS` 和 `ARM_RESET_ZERO` 已不再提供功能。`SYS_SET_MODE` 为废弃兼容命令，Jetson 不再公开该接口。`SENSOR_GET_IMU`、`SENSOR_GET_OPS` 和周期 `MSG_STATUS` 尚未由 STM32 `comm_protocol.c` 分发；相关解析函数只作为后续协议扩展的内部预留，不能用于正式联动流程。
