from dataclasses import dataclass
from typing import ClassVar


HEADER = bytes([0x5A, 0xA5])
VERSION = 0x01


@dataclass(frozen=True)
class Frame:
    msg_type: int
    cmd_set: int
    cmd_id: int
    seq: int
    payload: bytes = b""

    HEAD_LEN: ClassVar[int] = 9

    def to_bytes(self) -> bytes:
        if len(self.payload) > 255:
            raise ValueError("payload 长度不能超过 255")
        body = bytes(
            [
                VERSION,
                self.msg_type & 0xFF,
                self.cmd_set & 0xFF,
                self.cmd_id & 0xFF,
                self.seq & 0xFF,
                (self.seq >> 8) & 0xFF,
                len(self.payload),
            ]
        ) + self.payload
        crc = crc16_modbus(body)
        return HEADER + body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def parse_frame(raw: bytes) -> Frame:
    if len(raw) < Frame.HEAD_LEN + 2:
        raise ValueError("帧长度不足")
    if raw[0:2] != HEADER:
        raise ValueError("帧头错误")
    if raw[2] != VERSION:
        raise ValueError(f"版本不支持: {raw[2]}")

    payload_len = raw[8]
    total_len = Frame.HEAD_LEN + payload_len + 2
    if len(raw) < total_len:
        raise ValueError("帧数据不完整")

    body = raw[2 : 9 + payload_len]
    expected_crc = raw[9 + payload_len] | (raw[10 + payload_len] << 8)
    actual_crc = crc16_modbus(body)
    if expected_crc != actual_crc:
        raise ValueError("CRC 校验失败")

    seq = raw[6] | (raw[7] << 8)
    return Frame(
        msg_type=raw[3],
        cmd_set=raw[4],
        cmd_id=raw[5],
        seq=seq,
        payload=raw[9 : 9 + payload_len],
    )
