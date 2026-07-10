"""可组合的小车动作函数，不包含视觉决策或串口实现。"""

from __future__ import annotations

import time

from protocol import Car


# 首次实车测试使用保守低速、短时参数；标定后在此集中修改。
DEFAULT_MOVE_RPM = 30
DEFAULT_TURN_RPM = 20
DEFAULT_ACTION_DURATION_S = 0.5
DEFAULT_ACCELERATION = 10
DEFAULT_GRIP_WAIT_S = 0.5


def drive_for(
    car: Car,
    *,
    forward_rpm: int = 0,
    strafe_rpm: int = 0,
    rotate_rpm: int = 0,
    duration_s: float = DEFAULT_ACTION_DURATION_S,
    acc: int = DEFAULT_ACCELERATION,
) -> None:
    """以麦克纳姆轮速度运动指定时间，并保证退出时立即停车。"""
    if duration_s < 0:
        raise ValueError("duration_s must be non-negative")

    car.chassis.move_mecanum(forward_rpm, strafe_rpm, rotate_rpm, acc)
    try:
        time.sleep(duration_s)
    finally:
        car.chassis.stop(mode=1)


def move_forward(car: Car, duration_s: float = DEFAULT_ACTION_DURATION_S, speed_rpm: int = DEFAULT_MOVE_RPM) -> None:
    drive_for(car, forward_rpm=speed_rpm, duration_s=duration_s)


def move_backward(car: Car, duration_s: float = DEFAULT_ACTION_DURATION_S, speed_rpm: int = DEFAULT_MOVE_RPM) -> None:
    drive_for(car, forward_rpm=-speed_rpm, duration_s=duration_s)


def turn_left(car: Car, duration_s: float = DEFAULT_ACTION_DURATION_S, speed_rpm: int = DEFAULT_TURN_RPM) -> None:
    """下位机约定 rotate_rpm < 0 为左转。"""
    drive_for(car, rotate_rpm=-speed_rpm, duration_s=duration_s)


def turn_right(car: Car, duration_s: float = DEFAULT_ACTION_DURATION_S, speed_rpm: int = DEFAULT_TURN_RPM) -> None:
    """下位机约定 rotate_rpm > 0 为右转。"""
    drive_for(car, rotate_rpm=speed_rpm, duration_s=duration_s)


def grab(car: Car, wait_s: float = DEFAULT_GRIP_WAIT_S) -> None:
    car.arm.grab()
    time.sleep(wait_s)


def release(car: Car, wait_s: float = DEFAULT_GRIP_WAIT_S) -> None:
    car.arm.release()
    time.sleep(wait_s)


def run_motion_and_grip_test(car: Car) -> None:
    """按固定顺序组合基础动作，供手动实车联调调用。"""
    move_forward(car)
    move_backward(car)
    turn_left(car)
    turn_right(car)
    grab(car)
    release(car)
