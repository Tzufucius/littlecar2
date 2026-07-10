"""Protocol command constants shared by frame and client layers."""

from enum import IntEnum


PROTOCOL_VERSION = 0x01
HEADER = b"\x5a\xa5"
MAX_PAYLOAD_LEN = 255


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


class SystemCmd(IntEnum):
    PING = 0x01
    HEARTBEAT = 0x02
    SET_MODE = 0x03


class SafetyCmd(IntEnum):
    ESTOP = 0x01
    SAFE_STOP = 0x02
    CLEAR = 0x03


class ChassisCmd(IntEnum):
    ENABLE = 0x01
    STOP = 0x02
    SET_MOTOR_RPM = 0x03
    MOVE_MECANUM = 0x04
    SET_BODY_VELOCITY = 0x05
    SET_WORLD_VELOCITY = 0x06
    GOTO_POSE = 0x07
    CANCEL_GOAL = 0x08
    RESET_WORLD_ORIGIN = 0x09
    GET_WORLD_POSE = 0x0A
    GET_MOTION_STATUS = 0x0B


class ServoCmd(IntEnum):
    ARM_GRAB = 0x10


class SensorCmd(IntEnum):
    GET_OPS = 0x01
    GET_IMU = 0x02
    GET_POSE_FUSION = 0x03
    GET_MOTOR_STATE = 0x04
    STATUS = 0x80


class AckCode(IntEnum):
    OK = 0x00
    BAD_CRC = 0x01
    BAD_LENGTH = 0x02
    BAD_PARAM = 0x03
    BUSY = 0x04
    DENIED = 0x05
    UNKNOWN_CMD = 0x06
    FAULT = 0x07


class MotionState(IntEnum):
    IDLE = 0x00
    RUNNING = 0x01
    ARRIVED = 0x02
    TIMEOUT = 0x03
    NO_POSE = 0x04
    NO_ORIGIN = 0x05
    CANCELED = 0x06
