"""Typed payload objects returned by the protocol client."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class AckResult:
    ack_seq: int
    result: int
    detail: int


@dataclass(frozen=True)
class ImuData:
    accel_x_mg: int
    accel_y_mg: int
    accel_z_mg: int
    gyro_x_cdeg_s: int
    gyro_y_cdeg_s: int
    gyro_z_cdeg_s: int
    angle_x_cdeg: int
    angle_y_cdeg: int
    angle_z_cdeg: int
    valid_mask: int


@dataclass(frozen=True)
class OpsPose:
    zangle_cdeg: int
    xangle_cdeg: int
    yangle_cdeg: int
    pos_x_mm: int
    pos_y_mm: int
    wz_cdeg_s: int
    valid: bool


@dataclass(frozen=True)
class StatusReport:
    stm32_time_ms: int
    system_state: int
    fault_code: int
    chassis_state: int
    sensor_valid_mask: int
    active_seq: int
    last_done_seq: int
    pos_x_mm: int
    pos_y_mm: int
    yaw_cdeg: int


@dataclass(frozen=True)
class WorldGoal:
    """World-coordinate target consumed by ``CHASSIS_GOTO_POSE``."""

    x_mm: int
    y_mm: int
    yaw_cdeg: int = 0
    vmax_mm_s: int = 0
    wmax_cdeg_s: int = 0
    timeout_ms: int = 0
    use_yaw: bool = False
    acc: int = 10


@dataclass(frozen=True)
class MotionStatus:
    state: int
    active: bool
    goal_flags: int
    pose_x_mm: int
    pose_y_mm: int
    pose_yaw_cdeg: int
    error_x_mm: int
    error_y_mm: int
    position_error_mm: int
    yaw_error_cdeg: int
    goal_x_mm: int
    goal_y_mm: int
    goal_yaw_cdeg: int
    elapsed_ms: int
    timeout_ms: int
    updated_tick: int
