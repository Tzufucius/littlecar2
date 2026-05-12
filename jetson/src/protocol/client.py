"""High-level car API over the STM32 host protocol."""

from __future__ import annotations

import struct
import time

from .commands import ChassisCmd, CmdSet, SafetyCmd, SensorCmd, SystemCmd
from .frame import parse_imu_data, parse_ops_pose, parse_status_report
from .transport import SerialTransport
from .types import ImuData, OpsPose, StatusReport


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
        self.wit = WitClient(self.transport)
        self.ops = OpsClient(self.transport)

        self.System = self.system
        self.Safety = self.safety
        self.Chassis = self.chassis
        self.Wit = self.wit
        self.Ops = self.ops

    def close(self) -> None:
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

    def set_mode(self, mode: int) -> None:
        self._transport.send_command(CmdSet.SYSTEM, SystemCmd.SET_MODE, struct.pack("<B", mode))


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
