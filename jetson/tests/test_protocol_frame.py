import struct
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from protocol.commands import AckCode, ChassisCmd, CmdSet, MsgType, SystemCmd
from protocol.frame import FrameParser, crc16_modbus, pack_frame, parse_ack, unpack_frame


class ProtocolFrameTest(unittest.TestCase):
    def test_crc16_modbus_known_value(self):
        self.assertEqual(crc16_modbus(b"123456789"), 0x4B37)

    def test_sys_ping_pack_fields(self):
        raw = pack_frame(MsgType.CMD, CmdSet.SYSTEM, SystemCmd.PING, seq=0x1234)

        self.assertEqual(raw[:2], b"\x5a\xa5")
        self.assertEqual(raw[2], 0x01)
        self.assertEqual(raw[3], MsgType.CMD)
        self.assertEqual(raw[4], CmdSet.SYSTEM)
        self.assertEqual(raw[5], SystemCmd.PING)
        self.assertEqual(raw[6:8], b"\x34\x12")
        self.assertEqual(raw[8], 0)
        self.assertEqual(len(raw), 11)
        self.assertEqual(int.from_bytes(raw[-2:], "little"), crc16_modbus(raw[2:-2]))

    def test_ack_parse(self):
        payload = struct.pack("<HBB", 7, AckCode.OK, 0)
        frame = unpack_frame(pack_frame(MsgType.ACK, CmdSet.SYSTEM, SystemCmd.PING, 7, payload))

        ack = parse_ack(frame)

        self.assertEqual(ack.ack_seq, 7)
        self.assertEqual(ack.result, AckCode.OK)
        self.assertEqual(ack.detail, 0)

    def test_mecanum_payload_little_endian(self):
        payload = struct.pack("<hhhB", 80, -20, 15, 10)
        raw = pack_frame(MsgType.CMD, CmdSet.CHASSIS, ChassisCmd.MOVE_MECANUM, 3, payload)
        frame = unpack_frame(raw)

        self.assertEqual(frame.payload, b"\x50\x00\xec\xff\x0f\x00\x0a")

    def test_parser_handles_bad_crc_half_packet_and_sticky_packets(self):
        first = pack_frame(MsgType.CMD, CmdSet.SYSTEM, SystemCmd.PING, 1)
        second = pack_frame(MsgType.CMD, CmdSet.CHASSIS, ChassisCmd.STOP, 2, b"\x01")
        bad_crc = bytearray(first)
        bad_crc[-1] ^= 0xFF

        parser = FrameParser()
        self.assertEqual(parser.feed(bytes(bad_crc)), [])
        self.assertEqual(parser.feed(second[:5]), [])
        frames = parser.feed(second[5:] + first)

        self.assertEqual([(f.cmd_set, f.cmd_id, f.seq) for f in frames], [
            (CmdSet.CHASSIS, ChassisCmd.STOP, 2),
            (CmdSet.SYSTEM, SystemCmd.PING, 1),
        ])


if __name__ == "__main__":
    unittest.main()
