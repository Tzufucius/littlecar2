"""手动实车动作测试入口；不会被视觉主入口自动调用。"""

from __future__ import annotations

import argparse

from protocol import Car

from .actions import run_motion_and_grip_test


def run_manual_test(port: str, baudrate: int = 115200) -> None:
    """完成链路准备、动作组合测试和无条件安全收尾。"""
    with Car(port=port, baudrate=baudrate) as car:
        car.system.ping()
        car.heartbeat.start()
        car.safety.clear()
        car.chassis.enable(True)
        try:
            run_motion_and_grip_test(car)
        finally:
            car.safety.safe_stop(mode=1)
            car.chassis.enable(False)


def main() -> None:
    parser = argparse.ArgumentParser(description="小车底盘与夹爪手动测试工作流")
    parser.add_argument("--port", required=True, help="Windows 例如 COM8；Jetson 例如 /dev/ttyTHS1")
    parser.add_argument("--baudrate", type=int, default=115200)
    args = parser.parse_args()
    run_manual_test(args.port, args.baudrate)


if __name__ == "__main__":
    main()
