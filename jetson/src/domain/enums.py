from enum import IntEnum


class MsgType(IntEnum):
    CMD = 0x01
    ACK = 0x02
    DATA = 0x03
    STATUS = 0x04


class CmdSet(IntEnum):
    SYSTEM = 0x01
    SAFETY = 0x02
    CHASSIS = 0x03
    SERVO = 0x04
    SENSOR = 0x05
    CONFIG = 0x06
    DEBUG = 0x7F


class AckResult(IntEnum):
    OK = 0x00
    BAD_CRC = 0x01
    BAD_LENGTH = 0x02
    BAD_PARAM = 0x03
    BUSY = 0x04
    DENIED = 0x05
    UNKNOWN_CMD = 0x06
    FAULT = 0x07
