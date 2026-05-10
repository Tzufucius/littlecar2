from comm.mock_client import MockStm32Client
from comm.protocol import Frame
from comm.serial_client import SerialConfig, SerialStm32Client
from domain.enums import CmdSet, MsgType

USE_MOCK_CLIENT = True
SERIAL_PORT = "/dev/ttyTHS1"
SERIAL_BAUDRATE = 115200


def main() -> None:
    frame = Frame(
        msg_type=MsgType.CMD,
        cmd_set=CmdSet.SYSTEM,
        cmd_id=0x01,
        seq=1,
        payload=b"",
    ).to_bytes()

    if USE_MOCK_CLIENT:
        client = MockStm32Client()
    else:
        client = SerialStm32Client(
            SerialConfig(port=SERIAL_PORT, baudrate=SERIAL_BAUDRATE)
        )

    with client:
        client.send(frame)
        print("已发送:", frame.hex(" ").upper())


if __name__ == "__main__":
    main()
