# littlecar2 Jetson 上位机

本目录是 Jetson / Windows 上位机工程入口，负责视觉识别、任务编排和 STM32 通信。上位机只发送统一控制命令，不发送电机、总线舵机、OPS 或 WIT 的设备原始帧。

## 文件架构

```text
jetson/
  src/
    app/        应用入口、视觉编排和可组合的小车动作函数
    domain/     业务事件模型
    protocol/   串口、协议帧、ACK/DATA 处理和 Car 高层接口
    vision/     摄像头、二维码和 YOLO 识别
  tests/        离线模拟串口测试与手动实车测试代码
  assets/       模型和测试图片
  scripts/      独立视觉调试脚本
```

`app` 只能调用 `protocol.Car` 和 `app.actions`；`vision` 不得直接读写串口；`protocol` 是唯一串口所有者。新增业务动作应由 `app.actions` 中的小函数组合，不能在视觉模块或任务代码中直接拼装协议帧。

## 主函数与运行方式

视觉主入口是 `src/app/main.py`，调用 `RobotMain`：

```powershell
python -m app.main
```

`RobotMain.run_once_image(...)` 处理单张图片，`RobotMain.run_camera_loop(...)` 运行相机循环。当前视觉入口不会自动控制车辆，避免识别程序启动时误动作。

底盘与夹爪手动测试使用独立入口：

```powershell
python -m app.test_workflow --port COM8
# Jetson 示例：python -m app.test_workflow --port /dev/ttyTHS1
```

该入口依次完成链路检查、启动心跳、清除安全锁、使能底盘、前进、后退、左转、右转、夹取、松开和安全停车。它只在显式执行时运行。

## 动作函数编排

`src/app/actions.py` 提供 `drive_for()`、`move_forward()`、`move_backward()`、`turn_left()`、`turn_right()`、`grab()`、`release()`。每个函数均可单独调用；`run_motion_and_grip_test()` 只是按顺序组合这些函数，而非引入通用工作流类。

首次实车测试使用文件顶部集中定义的保守参数：移动 `30 RPM`、转向 `20 RPM`、单次 `0.5 s`。实车标定后只修改这些常量或在函数调用中传入覆盖值。`drive_for()` 始终在 `finally` 中发送立即停车。

## 通信启动顺序

```python
from protocol import Car
from app.actions import move_forward, grab

with Car(port="/dev/ttyTHS1") as car:
    car.system.ping()
    car.heartbeat.start()
    car.safety.clear()
    car.chassis.enable(True)
    try:
        move_forward(car)
        grab(car)
    finally:
        car.safety.safe_stop(mode=1)
        car.chassis.enable(False)
```

STM32 使用 `USART6`、`115200 8N1`。上位机每个命令等待同一 `Seq` 的 ACK；ACK 超时、串口异常或非成功 ACK 后，不得继续下发运动或夹爪命令。`Car.close()` 会停止心跳并关闭串口。

## 当前可调用能力

- `car.system`：`ping()`、`heartbeat()`、`set_mode()`。
- `car.safety`：`estop()`、`safe_stop()`、`clear()`。
- `car.chassis`：使能、停车、RPM、麦克纳姆速度、车体/世界速度、GotoPose 和目标状态查询。
- `car.arm`：`grab()` 与 `release()`；夹爪使用下位机舵机 ID `2`，开合位置由 `Core/Inc/advance_arm.h` 集中配置，默认松开 `0`、夹取 `1000`。

`SENSOR_GET_OPS`、`SENSOR_GET_IMU` 和周期 `MSG_STATUS` 仍未在 STM32 `comm_protocol.c` 中实现分发，不能纳入正式工作流。

## 开发与测试约束

1. 保持基础帧格式、CRC、小端字节序和既有 `CmdSet/CmdID` 不变；新增能力优先增加命令 ID。
2. 新增命令必须同步修改 STM32 分发、上位机常量与封装、离线模拟测试、实车手动测试代码和本文档。
3. 时间控制动作只用于调试，不等同于定位闭环；需要精确位移时使用已具备传感器前提的 `goto_pose()`。
4. 实车前必须确认串口交叉、共地、底盘架空或安全区域；不自动执行硬件测试。
5. 提交前执行离线测试：`conda run -n low_numpy python -m unittest discover -s tests -v`。
