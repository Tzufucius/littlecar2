import struct
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from protocol import Car, WorldGoal
from protocol.commands import AckCode, ChassisCmd, CmdSet, MsgType
from protocol.frame import pack_frame, unpack_frame
from protocol.transport import SerialTransport


class AckingSerial:
    """In-memory STM32 peer used to verify every currently implemented host API."""

    def __init__(self) -> None:
        self.writes = []
        self._rx = bytearray()
        self.closed = False

    def write(self, raw: bytes) -> int:
        command = unpack_frame(raw)
        self.writes.append(command)
        ack_payload = struct.pack("<HBB", command.seq, AckCode.OK, 0)
        self._rx.extend(pack_frame(MsgType.ACK, command.cmd_set, command.cmd_id, command.seq, ack_payload))
        if command.cmd_set == CmdSet.CHASSIS and command.cmd_id == ChassisCmd.GET_MOTION_STATUS:
            motion_payload = struct.pack(
                "<BBBBiiiiiiiiiiIII",
                1,
                1,
                1,
                0,
                100,
                200,
                300,
                10,
                20,
                22,
                30,
                400,
                500,
                600,
                700,
                800,
                900,
            )
            self._rx.extend(pack_frame(MsgType.DATA, command.cmd_set, command.cmd_id, command.seq, motion_payload))
        return len(raw)

    def read(self, size: int) -> bytes:
        chunk = bytes(self._rx[:size])
        del self._rx[:size]
        return chunk

    def close(self) -> None:
        self.closed = True


class ProtocolClientTest(unittest.TestCase):
    def setUp(self) -> None:
        self.serial = AckingSerial()
        self.transport = SerialTransport("unused", serial_instance=self.serial, ack_timeout=0.02, data_timeout=0.02)
        self.car = Car("unused", transport=self.transport)

    def tearDown(self) -> None:
        self.car.close()

    def test_system_and_safety_methods_send_supported_commands(self):
        self.car.system.ping()
        self.car.system.heartbeat(1234)
        self.car.system.set_mode(2)
        self.car.safety.estop(3)
        self.car.safety.safe_stop(0)
        self.car.safety.clear()

        self.assertEqual(
            [(frame.cmd_set, frame.cmd_id) for frame in self.serial.writes],
            [
                (CmdSet.SYSTEM, 0x01),
                (CmdSet.SYSTEM, 0x02),
                (CmdSet.SYSTEM, 0x03),
                (CmdSet.SAFETY, 0x01),
                (CmdSet.SAFETY, 0x02),
                (CmdSet.SAFETY, 0x03),
            ],
        )
        self.assertEqual(self.serial.writes[1].payload, struct.pack("<I", 1234))

    def test_all_current_chassis_methods_match_stm32_payloads(self):
        chassis = self.car.chassis
        chassis.enable(True)
        chassis.stop(1)
        chassis.set_motor_rpm(10, -20, 30, -40, acc=8)
        chassis.move_mecanum(20, -30, 40, acc=9)
        chassis.set_body_velocity(100, 200, -300, acc=7)
        chassis.set_world_velocity(-100, 200, 300, acc=6)
        chassis.goto_pose(
            WorldGoal(
                x_mm=1000,
                y_mm=-2000,
                yaw_cdeg=9000,
                vmax_mm_s=400,
                wmax_cdeg_s=500,
                timeout_ms=6000,
                use_yaw=True,
                acc=5,
            )
        )
        chassis.cancel_goal()
        chassis.reset_world_origin()
        status = chassis.get_motion_status()

        self.assertEqual(
            [frame.cmd_id for frame in self.serial.writes],
            [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0B],
        )
        self.assertEqual(self.serial.writes[2].payload, struct.pack("<hhhhB", 10, -20, 30, -40, 8))
        self.assertEqual(self.serial.writes[6].payload, struct.pack("<iiihhIBB", 1000, -2000, 9000, 400, 500, 6000, 1, 5))
        self.assertTrue(status.active)
        self.assertEqual(status.pose_x_mm, 100)
        self.assertEqual(status.goal_yaw_cdeg, 600)
        self.assertEqual(status.updated_tick, 900)

    def test_close_closes_transport(self):
        self.car.close()
        self.assertTrue(self.serial.closed)


if __name__ == "__main__":
    unittest.main()
