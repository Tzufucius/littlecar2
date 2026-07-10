"""Binary frame packing and parsing for the STM32 host protocol."""

from __future__ import annotations

from dataclasses import dataclass
from struct import unpack_from
from typing import List

from .commands import HEADER, MAX_PAYLOAD_LEN, PROTOCOL_VERSION, MsgType
from .exceptions import ProtocolError
from .types import AckResult, ImuData, MotionStatus, OpsPose, StatusReport

BASE_HEADER_LEN = 9
CRC_LEN = 2
MIN_FRAME_LEN = BASE_HEADER_LEN + CRC_LEN


@dataclass(frozen=True)
class Frame:
    msg_type: int
    cmd_set: int
    cmd_id: int
    seq: int
    payload: bytes = b""


def crc16_modbus(data: bytes | bytearray | memoryview) -> int:
    """Return CRC16-Modbus with polynomial 0xA001 and initial value 0xFFFF."""
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


def pack_frame(msg_type: int, cmd_set: int, cmd_id: int, seq: int, payload: bytes = b"") -> bytes:
    if not 0 <= seq <= 0xFFFF:
        raise ValueError("seq must be in uint16 range")
    if len(payload) > MAX_PAYLOAD_LEN:
        raise ValueError("payload length must be <= 255")

    body = bytes(
        [
            PROTOCOL_VERSION,
            int(msg_type) & 0xFF,
            int(cmd_set) & 0xFF,
            int(cmd_id) & 0xFF,
            seq & 0xFF,
            (seq >> 8) & 0xFF,
            len(payload),
        ]
    ) + payload
    crc = crc16_modbus(body)
    return HEADER + body + crc.to_bytes(2, "little")


def unpack_frame(raw: bytes) -> Frame:
    if len(raw) < MIN_FRAME_LEN:
        raise ProtocolError("frame too short")
    if raw[:2] != HEADER:
        raise ProtocolError("bad frame header")
    if raw[2] != PROTOCOL_VERSION:
        raise ProtocolError(f"unsupported protocol version: 0x{raw[2]:02X}")

    payload_len = raw[8]
    total_len = BASE_HEADER_LEN + payload_len + CRC_LEN
    if len(raw) != total_len:
        raise ProtocolError("frame length mismatch")

    crc_recv = int.from_bytes(raw[-2:], "little")
    crc_calc = crc16_modbus(raw[2:-2])
    if crc_recv != crc_calc:
        raise ProtocolError("bad frame crc")

    seq = raw[6] | (raw[7] << 8)
    return Frame(
        msg_type=raw[3],
        cmd_set=raw[4],
        cmd_id=raw[5],
        seq=seq,
        payload=bytes(raw[9:-2]),
    )


class FrameParser:
    """Incremental parser that scans a byte stream for complete valid frames."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes | bytearray | memoryview) -> List[Frame]:
        self._buffer.extend(data)
        frames: List[Frame] = []

        while True:
            start = self._buffer.find(HEADER)
            if start < 0:
                self._buffer.clear()
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < MIN_FRAME_LEN:
                break

            if self._buffer[2] != PROTOCOL_VERSION:
                del self._buffer[0]
                continue

            total_len = BASE_HEADER_LEN + self._buffer[8] + CRC_LEN
            if len(self._buffer) < total_len:
                break

            raw = bytes(self._buffer[:total_len])
            del self._buffer[:total_len]
            try:
                frames.append(unpack_frame(raw))
            except ProtocolError:
                continue

        return frames


def parse_ack(frame: Frame) -> AckResult:
    if frame.msg_type != MsgType.ACK:
        raise ProtocolError("frame is not ACK")
    if len(frame.payload) != 4:
        raise ProtocolError("ACK payload length must be 4")
    ack_seq, result, detail = unpack_from("<HBB", frame.payload)
    return AckResult(ack_seq=ack_seq, result=result, detail=detail)


def parse_imu_data(payload: bytes) -> ImuData:
    if len(payload) < 19:
        raise ProtocolError("IMU DATA payload length must be at least 19")
    values = unpack_from("<hhhhhhhhhB", payload)
    return ImuData(*values)


def parse_ops_pose(payload: bytes) -> OpsPose:
    if len(payload) < 23:
        raise ProtocolError("OPS DATA payload length must be at least 23")
    zangle, xangle, yangle, pos_x, pos_y, wz, valid = unpack_from("<iiiiihB", payload)
    return OpsPose(
        zangle_cdeg=zangle,
        xangle_cdeg=xangle,
        yangle_cdeg=yangle,
        pos_x_mm=pos_x,
        pos_y_mm=pos_y,
        wz_cdeg_s=wz,
        valid=valid != 0,
    )


def parse_status_report(payload: bytes) -> StatusReport:
    if len(payload) < 24:
        raise ProtocolError("STATUS payload length must be at least 24")
    values = unpack_from("<IBBBBHHiii", payload)
    return StatusReport(*values)


def parse_motion_status(payload: bytes) -> MotionStatus:
    """Parse the fixed 56-byte ``CHASSIS_GET_MOTION_STATUS`` response."""
    if len(payload) != 56:
        raise ProtocolError("motion status payload length must be 56")
    values = unpack_from("<BBBBiiiiiiiiiiIII", payload)
    return MotionStatus(
        state=values[0],
        active=values[1] != 0,
        goal_flags=values[2],
        pose_x_mm=values[4],
        pose_y_mm=values[5],
        pose_yaw_cdeg=values[6],
        error_x_mm=values[7],
        error_y_mm=values[8],
        position_error_mm=values[9],
        yaw_error_cdeg=values[10],
        goal_x_mm=values[11],
        goal_y_mm=values[12],
        goal_yaw_cdeg=values[13],
        elapsed_ms=values[14],
        timeout_ms=values[15],
        updated_tick=values[16],
    )
