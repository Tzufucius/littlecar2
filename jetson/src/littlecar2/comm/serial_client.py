from dataclasses import dataclass
from typing import Optional

import serial

from littlecar2.comm.interface import Stm32Client


@dataclass(frozen=True)
class SerialConfig:
    port: str
    baudrate: int = 115200
    timeout: float = 0.1


class SerialStm32Client(Stm32Client):
    def __init__(self, config: SerialConfig) -> None:
        self.config = config
        self.serial_port: Optional[serial.Serial] = None

    def open(self) -> None:
        self.serial_port = serial.Serial(
            port=self.config.port,
            baudrate=self.config.baudrate,
            timeout=self.config.timeout,
        )

    def close(self) -> None:
        if self.serial_port is not None:
            self.serial_port.close()
            self.serial_port = None

    def send(self, payload: bytes) -> None:
        if self.serial_port is None:
            raise RuntimeError("SerialStm32Client 未打开")
        self.serial_port.write(payload)

    def read(self, size: int = 1024) -> bytes:
        if self.serial_port is None:
            raise RuntimeError("SerialStm32Client 未打开")
        return self.serial_port.read(size)
