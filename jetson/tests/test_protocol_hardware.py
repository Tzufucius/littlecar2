"""Hardware-in-the-loop checks for a flashed STM32 controller.

These tests are intentionally skipped until ``STM32_SERIAL_PORT`` is configured.
Set ``STM32_ENABLE_MOTION_TESTS=1`` only after the chassis is lifted or its motion
area is safe.  Motion tests use zero velocity, but GotoPose can move the vehicle.
"""

import os
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from protocol import Car, WorldGoal


SERIAL_PORT = os.getenv("STM32_SERIAL_PORT")
RUN_MOTION_TESTS = os.getenv("STM32_ENABLE_MOTION_TESTS") == "1"


@unittest.skipUnless(SERIAL_PORT, "set STM32_SERIAL_PORT to run STM32 hardware checks")
class ProtocolHardwareTest(unittest.TestCase):
    def setUp(self) -> None:
        self.car = Car(SERIAL_PORT, ack_timeout=0.5, data_timeout=1.0)

    def tearDown(self) -> None:
        try:
            self.car.safety.safe_stop(mode=1)
        finally:
            self.car.close()

    def test_system_and_safety_link(self):
        self.car.system.ping()
        self.car.system.heartbeat()
        self.car.safety.safe_stop(mode=1)
        self.car.safety.clear()

    @unittest.skipUnless(RUN_MOTION_TESTS, "set STM32_ENABLE_MOTION_TESTS=1 after preparing a safe test area")
    def test_chassis_command_acknowledgements(self):
        chassis = self.car.chassis
        chassis.enable(True)
        chassis.set_motor_rpm(0, 0, 0, 0)
        chassis.move_mecanum(0, 0, 0)
        chassis.set_body_velocity(0, 0, 0)
        chassis.reset_world_origin()
        chassis.set_world_velocity(0, 0, 0)
        chassis.goto_pose(WorldGoal(x_mm=0, y_mm=0, timeout_ms=1000))
        chassis.get_motion_status()
        chassis.cancel_goal()
        chassis.stop(mode=1)
        chassis.enable(False)


if __name__ == "__main__":
    unittest.main()
