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
    car.system.heartbeat()
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
- `car.system.set_mode(mode)`：发送 `SYS_SET_MODE`。
- `car.safety.estop(source=0)`：发送急停命令。
- `car.safety.safe_stop(mode=0)`：发送安全停止命令，`0` 为平滑停止，`1` 为立即停止。
- `car.safety.clear()`：清除可恢复安全状态。
- `car.chassis.enable(True)`：使能或失能底盘。
- `car.chassis.stop(mode=1)`：停止底盘。
- `car.chassis.set_motor_rpm(lf, rf, lr, rr, acc=10)`：设置四轮 RPM。
- `car.chassis.move_mecanum(forward, strafe, rotate, acc=10)`：发送麦克纳姆轮速度指令。
- `car.wit.get_data()`：发送 `SENSOR_GET_IMU`，收到 `MSG_DATA` 后解析为 `ImuData`。
- `car.ops.get_pose()`：发送 `SENSOR_GET_OPS`，收到 `MSG_DATA` 后解析为 `OpsPose`。
- `car.read_status(timeout=0.0)`：读取一帧周期 `MSG_STATUS`，无数据时返回 `None`。

## 异常语义

- ACK 超时或 DATA 超时会抛出 `ProtocolTimeoutError`。
- ACK 返回非 `OK` 会抛出 `CommandRejectedError`，异常对象保留 `cmd_set`、`cmd_id`、`seq`、`result` 和 `detail`。
- 帧头、版本、长度或 CRC 不合法时，底层解析器会丢弃坏帧。

## 当前限制

当前 STM32 侧已实现 `SYSTEM`、`SAFETY`、`CHASSIS` 的 ACK 类命令。`SENSOR_GET_IMU` 和 `SENSOR_GET_OPS` 的 Python 接口已经按协议预留，但真实数据依赖下位机后续实现 `CMDSET_SENSOR` 查询命令和对应 `MSG_DATA` 返回。在下位机未实现前，这两个接口预期会收到 `UNKNOWN_CMD` 或等待 DATA 超时。
