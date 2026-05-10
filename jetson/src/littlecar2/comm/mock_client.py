from typing import List

from littlecar2.comm.interface import Stm32Client


class MockStm32Client(Stm32Client):
    def __init__(self) -> None:
        self.is_open = False
        self.sent_payloads: List[bytes] = []

    def open(self) -> None:
        self.is_open = True

    def close(self) -> None:
        self.is_open = False

    def send(self, payload: bytes) -> None:
        if not self.is_open:
            raise RuntimeError("MockStm32Client 未打开")
        self.sent_payloads.append(bytes(payload))

    def read(self, size: int = 1024) -> bytes:
        if not self.is_open:
            raise RuntimeError("MockStm32Client 未打开")
        return b""
