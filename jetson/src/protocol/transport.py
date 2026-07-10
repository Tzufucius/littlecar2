"""Serial transport for request/ACK/DATA protocol exchange."""

from __future__ import annotations

import time
from threading import RLock
from typing import Optional

from .commands import AckCode, MsgType
from .exceptions import CommandRejectedError, ProtocolTimeoutError
from .frame import Frame, FrameParser, pack_frame, parse_ack


class SerialTransport:
    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        ack_timeout: float = 0.2,
        data_timeout: float = 0.5,
        read_size: int = 64,
        serial_instance: object | None = None,
    ) -> None:
        self.ack_timeout = ack_timeout
        self.data_timeout = data_timeout
        self.read_size = read_size
        self._parser = FrameParser()
        self._pending: list[Frame] = []
        self._next_seq = 1
        self._lock = RLock()

        if serial_instance is not None:
            self.serial = serial_instance
        else:
            try:
                import serial
            except ImportError as exc:
                raise RuntimeError("pyserial is required for SerialTransport") from exc
            self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=0)

    def close(self) -> None:
        close = getattr(self.serial, "close", None)
        if close is not None:
            close()

    def __enter__(self) -> "SerialTransport":
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        self.close()

    def next_seq(self) -> int:
        seq = self._next_seq
        self._next_seq = (self._next_seq + 1) & 0xFFFF
        if self._next_seq == 0:
            self._next_seq = 1
        return seq

    def send_command(
        self,
        cmd_set: int,
        cmd_id: int,
        payload: bytes = b"",
        expect_data: bool = False,
    ) -> Frame | None:
        with self._lock:
            seq = self.next_seq()
            self.serial.write(pack_frame(MsgType.CMD, cmd_set, cmd_id, seq, payload))
            ack_frame = self.wait_for_ack(seq, cmd_set, cmd_id, self.ack_timeout)
            ack = parse_ack(ack_frame)
            if ack.ack_seq != seq:
                raise ProtocolTimeoutError(f"ACK seq mismatch: expected {seq}, got {ack.ack_seq}")
            if ack.result != AckCode.OK:
                raise CommandRejectedError(cmd_set, cmd_id, seq, ack.result, ack.detail)
            if expect_data:
                return self.wait_for_data(seq, cmd_set, cmd_id, self.data_timeout)
            return None

    def wait_for_ack(self, seq: int, cmd_set: int, cmd_id: int, timeout: float) -> Frame:
        return self._wait_for_frame(MsgType.ACK, seq, cmd_set, cmd_id, timeout, "ACK")

    def wait_for_data(self, seq: int, cmd_set: int, cmd_id: int, timeout: float) -> Frame:
        return self._wait_for_frame(MsgType.DATA, seq, cmd_set, cmd_id, timeout, "DATA")

    def read_status(self, timeout: float = 0.0) -> Optional[Frame]:
        with self._lock:
            deadline = time.monotonic() + timeout
            while True:
                frame = self._pop_matching(MsgType.STATUS, None, None, None)
                if frame is not None:
                    return frame
                if timeout == 0.0 or time.monotonic() >= deadline:
                    return None
                self._read_once()

    def _wait_for_frame(
        self,
        msg_type: int,
        seq: int,
        cmd_set: int,
        cmd_id: int,
        timeout: float,
        label: str,
    ) -> Frame:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self._pop_matching(msg_type, seq, cmd_set, cmd_id)
            if frame is not None:
                return frame
            self._read_once()
        raise ProtocolTimeoutError(
            f"timeout waiting for {label}: seq={seq}, cmd_set=0x{cmd_set:02X}, cmd_id=0x{cmd_id:02X}"
        )

    def _read_once(self) -> None:
        data = self.serial.read(self.read_size)
        if data:
            self._pending.extend(self._parser.feed(data))
        else:
            time.sleep(0.001)

    def _pop_matching(
        self,
        msg_type: int,
        seq: int | None,
        cmd_set: int | None,
        cmd_id: int | None,
    ) -> Frame | None:
        for index, frame in enumerate(self._pending):
            if frame.msg_type != msg_type:
                continue
            if seq is not None and frame.seq != seq:
                continue
            if cmd_set is not None and frame.cmd_set != cmd_set:
                continue
            if cmd_id is not None and frame.cmd_id != cmd_id:
                continue
            return self._pending.pop(index)
        return None
