"""High-level car API over the STM32 host protocol."""

from __future__ import annotations

import struct
import time

from .commands import ChassisCmd, CmdSet, SafetyCmd, SensorCmd, ServoCmd, SystemCmd
from .frame import parse_imu_data, parse_motion_status, parse_ops_pose, parse_status_report
from .heartbeat import HeartbeatRunner
from .transport import SerialTransport
from .types import ImuData, MotionStatus, OpsPose, StatusReport, WorldGoal


class Car:
    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        ack_timeout: float = 0.2,
        data_timeout: float = 0.5,
        transport: SerialTransport | None = None,
    ) -> None:
        self.transport = transport or SerialTransport(
            port=port,
            baudrate=baudrate,
            ack_timeout=ack_timeout,
            data_timeout=data_timeout,
        )
        self.system = SystemClient(self.transport)
        self.safety = SafetyClient(self.transport)
        self.chassis = ChassisClient(self.transport)
        self.arm = ArmClient(self.transport)
        self.wit = WitClient(self.transport)
        self.ops = OpsClient(self.transport)
        self.heartbeat = HeartbeatRunner(self.system)

        self.System = self.system
        self.Safety = self.safety
        self.Chassis = self.chassis
        self.Arm = self.arm
        self.Wit = self.wit
        self.Ops = self.ops

    def close(self) -> None:
        self.heartbeat.stop()
        self.transport.close()

    def __enter__(self) -> "Car":
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        self.close()

    def read_status(self, timeout: float = 0.0) -> StatusReport | None:
        frame = self.transport.read_status(timeout=timeout)
        if frame is None:
            return None
        return parse_status_report(frame.payload)


class SystemClient:
    def __init__(self, transport: SerialTransport) -> None:
        self._transport = transport

    def ping(self) -> None:
        self._transport.send_command(CmdSet.SYSTEM, SystemCmd.PING)

    def heartbeat(self, jetson_time_ms: int | None = None) -> None:
        if jetson_time_ms is None:
            jetson_time_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
        payload = struct.pack("<I", jetson_time_ms)
        self._transport.send_command(CmdSet.SYSTEM, SystemCmd.HEARTBEAT, payload)

    def claim_control(self) -> None:
        self._transport.send_command(CmdSet.SYSTEM, SystemCmd.CLAIM_CONTROL)

    def release_control(self) -> None:
        self._transport.send_command(CmdSet.SYSTEM, SystemCmd.RELEASE_CONTROL)

class SafetyClient:
    def __init__(self, transport: SerialTransport) -> None:
        self._transport = transport

    def estop(self, source: int = 0) -> None:
        self._transport.send_command(CmdSet.SAFETY, SafetyCmd.ESTOP, struct.pack("<B", source))

    def safe_stop(self, mode: int = 0) -> None:
        self._transport.send_command(CmdSet.SAFETY, SafetyCmd.SAFE_STOP, struct.pack("<B", mode))

    def clear(self) -> None:
        self._transport.send_command(CmdSet.SAFETY, SafetyCmd.CLEAR)


class ChassisClient:
    def __init__(self, transport: SerialTransport) -> None:
        self._transport = transport

    def enable(self, enabled: bool = True) -> None:
        self._transport.send_command(CmdSet.CHASSIS, ChassisCmd.ENABLE, struct.pack("<B", int(enabled)))

    def stop(self, mode: int = 1) -> None:
        self._transport.send_command(CmdSet.CHASSIS, ChassisCmd.STOP, struct.pack("<B", mode))

    def set_motor_rpm(self, lf: int, rf: int, lr: int, rr: int, acc: int = 10) -> None:
        payload = struct.pack("<hhhhB", lf, rf, lr, rr, acc)
        self._transport.send_command(CmdSet.CHASSIS, ChassisCmd.SET_MOTOR_RPM, payload)

    def move_mecanum(self, forward: int, strafe: int, rotate: int, acc: int = 10) -> None:
        payload = struct.pack("<hhhB", forward, strafe, rotate, acc)
        self._transport.send_command(CmdSet.CHASSIS, ChassisCmd.MOVE_MECANUM, payload)

    def set_body_velocity(self, vx_mm_s: int, vy_mm_s: int, wz_cdeg_s: int, acc: int = 10) -> None:
        """Set body velocity: +X right, +Y forward, positive yaw counter-clockwise."""
        payload = struct.pack("<hhhB", vx_mm_s, vy_mm_s, wz_cdeg_s, acc)
        self._transport.send_command(CmdSet.CHASSIS, ChassisCmd.SET_BODY_VELOCITY, payload)

    def set_world_velocity(self, vx_mm_s: int, vy_mm_s: int, wz_cdeg_s: int, acc: int = 10) -> None:
        """Set velocity in the STM32-maintained world coordinate system."""
        payload = struct.pack("<hhhB", vx_mm_s, vy_mm_s, wz_cdeg_s, acc)
        self._transport.send_command(CmdSet.CHASSIS, ChassisCmd.SET_WORLD_VELOCITY, payload)

    def goto_pose(self, goal: WorldGoal) -> None:
        """Submit an asynchronous world target; use ``get_motion_status`` to track it."""
        flags = 0x01 if goal.use_yaw else 0x00
        payload = struct.pack(
            "<iiihhIBB",
            goal.x_mm,
            goal.y_mm,
            goal.yaw_cdeg,
            goal.vmax_mm_s,
            goal.wmax_cdeg_s,
            goal.timeout_ms,
            flags,
            goal.acc,
        )
        self._transport.send_command(CmdSet.CHASSIS, ChassisCmd.GOTO_POSE, payload)

    def cancel_goal(self) -> None:
        self._transport.send_command(CmdSet.CHASSIS, ChassisCmd.CANCEL_GOAL)

    def reset_world_origin(self) -> None:
        self._transport.send_command(CmdSet.CHASSIS, ChassisCmd.RESET_WORLD_ORIGIN)

    def get_motion_status(self) -> MotionStatus:
        frame = self._transport.send_command(
            CmdSet.CHASSIS,
            ChassisCmd.GET_MOTION_STATUS,
            expect_data=True,
        )
        if frame is None:
            raise RuntimeError("expected motion-status DATA frame")
        return parse_motion_status(frame.payload)


class ArmClient:
    """High-level arm actions currently implemented by the STM32 protocol."""

    def __init__(self, transport: SerialTransport) -> None:
        self._transport = transport

    def grab(self) -> None:
        self._transport.send_command(CmdSet.SERVO, ServoCmd.ARM_GRAB, b"\x01")

    def release(self) -> None:
        self._transport.send_command(CmdSet.SERVO, ServoCmd.ARM_GRAB, b"\x00")

    def pick(self) -> None:
        self._transport.send_command(CmdSet.SERVO, ServoCmd.ARM_PICK)

    def place(self) -> None:
        self._transport.send_command(CmdSet.SERVO, ServoCmd.ARM_PLACE)

    def abort(self) -> None:
        self._transport.send_command(CmdSet.SERVO, ServoCmd.ARM_ABORT)

    def reset_zero(self) -> None:
        self._transport.send_command(CmdSet.SERVO, ServoCmd.ARM_RESET_ZERO)

    def get_status(self) -> bytes:
        frame = self._transport.send_command(
            CmdSet.SERVO,
            ServoCmd.ARM_GET_STATUS,
            expect_data=True,
        )
        if frame is None:
            raise RuntimeError("expected arm-status DATA frame")
        return frame.payload


class WitClient:
    def __init__(self, transport: SerialTransport) -> None:
        self._transport = transport

    def get_data(self) -> ImuData:
        frame = self._transport.send_command(CmdSet.SENSOR, SensorCmd.GET_IMU, expect_data=True)
        if frame is None:
            raise RuntimeError("expected IMU DATA frame")
        return parse_imu_data(frame.payload)


class OpsClient:
    def __init__(self, transport: SerialTransport) -> None:
        self._transport = transport

    def get_pose(self) -> OpsPose:
        frame = self._transport.send_command(CmdSet.SENSOR, SensorCmd.GET_OPS, expect_data=True)
        if frame is None:
            raise RuntimeError("expected OPS DATA frame")
        return parse_ops_pose(frame.payload)
