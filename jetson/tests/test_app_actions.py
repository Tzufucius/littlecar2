import sys
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from app.actions import drive_for, run_motion_and_grip_test, turn_left, turn_right
from app.test_workflow import run_manual_test


class FakeChassis:
    def __init__(self, calls):
        self.calls = calls

    def move_mecanum(self, forward, strafe, rotate, acc):
        self.calls.append(("move", forward, strafe, rotate, acc))

    def enable(self, enabled):
        self.calls.append(("enable", enabled))

    def stop(self, mode):
        self.calls.append(("stop", mode))


class FakeArm:
    def __init__(self, calls):
        self.calls = calls

    def grab(self):
        self.calls.append(("grab",))

    def release(self):
        self.calls.append(("release",))


class FakeCar:
    def __init__(self):
        self.calls = []
        self.chassis = FakeChassis(self.calls)
        self.arm = FakeArm(self.calls)


class FakeSystem:
    def __init__(self, calls):
        self.calls = calls

    def ping(self):
        self.calls.append(("ping",))


class FakeHeartbeat:
    def __init__(self, calls):
        self.calls = calls

    def start(self):
        self.calls.append(("heartbeat",))


class FakeSafety:
    def __init__(self, calls):
        self.calls = calls

    def clear(self):
        self.calls.append(("clear",))

    def safe_stop(self, mode):
        self.calls.append(("safe_stop", mode))


class FakeManualCar(FakeCar):
    last_instance = None

    def __init__(self, *args, **kwargs):
        super().__init__()
        self.system = FakeSystem(self.calls)
        self.heartbeat = FakeHeartbeat(self.calls)
        self.safety = FakeSafety(self.calls)
        FakeManualCar.last_instance = self

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.calls.append(("close",))


class ActionsTest(unittest.TestCase):
    @patch("app.actions.time.sleep")
    def test_turn_direction_and_stop(self, sleep):
        car = FakeCar()

        turn_left(car, duration_s=0.1, speed_rpm=12)
        turn_right(car, duration_s=0.2, speed_rpm=13)

        self.assertEqual(
            car.calls,
            [("move", 0, 0, -12, 10), ("stop", 1), ("move", 0, 0, 13, 10), ("stop", 1)],
        )
        self.assertEqual(sleep.call_count, 2)

    @patch("app.actions.time.sleep", side_effect=RuntimeError("interrupted"))
    def test_drive_for_stops_when_wait_is_interrupted(self, _sleep):
        car = FakeCar()

        with self.assertRaisesRegex(RuntimeError, "interrupted"):
            drive_for(car, forward_rpm=20)

        self.assertEqual(car.calls, [("move", 20, 0, 0, 10), ("stop", 1)])

    @patch("app.actions.time.sleep")
    def test_combined_workflow_composes_small_functions(self, _sleep):
        car = FakeCar()

        run_motion_and_grip_test(car)

        self.assertEqual(
            car.calls,
            [
                ("move", 30, 0, 0, 10), ("stop", 1),
                ("move", -30, 0, 0, 10), ("stop", 1),
                ("move", 0, 0, -20, 10), ("stop", 1),
                ("move", 0, 0, 20, 10), ("stop", 1),
                ("grab",), ("release",),
            ],
        )

    @patch("app.test_workflow.run_motion_and_grip_test", side_effect=RuntimeError("workflow failed"))
    @patch("app.test_workflow.Car", FakeManualCar)
    def test_manual_workflow_stops_and_disables_after_failure(self, _run_motion):
        with self.assertRaisesRegex(RuntimeError, "workflow failed"):
            run_manual_test("unused")

        self.assertEqual(
            FakeManualCar.last_instance.calls,
            [
                ("ping",), ("heartbeat",), ("clear",),
                ("enable", True), ("safe_stop", 1), ("enable", False), ("close",),
            ],
        )


if __name__ == "__main__":
    unittest.main()
