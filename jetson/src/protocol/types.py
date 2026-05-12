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
