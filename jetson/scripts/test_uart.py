import argparse

from comm.mock_client import MockStm32Client
from comm.protocol import Frame
from comm.serial_client import SerialConfig, SerialStm32Client
from domain.enums import CmdSet, MsgType


def main() -> None:
    parser = argparse.ArgumentParser(description="测试 STM32 串口发送")
    parser.add_argument("--mock", action="store_true", help="使用 mock 客户端")
    parser.add_argument("--port", default="/dev/ttyTHS1", help="串口设备")
    parser.add_argument("--baudrate", type=int, default=115200, help="波特率")
    args = parser.parse_args()

    frame = Frame(
        msg_type=MsgType.CMD,
        cmd_set=CmdSet.SYSTEM,
        cmd_id=0x01,
        seq=1,
        payload=b"",
    ).to_bytes()

    if args.mock:
        client = MockStm32Client()
    else:
        client = SerialStm32Client(SerialConfig(port=args.port, baudrate=args.baudrate))

    with client:
        client.send(frame)
        print("已发送:", frame.hex(" ").upper())


if __name__ == "__main__":
    main()
