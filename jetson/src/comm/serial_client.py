from dataclasses import dataclass
from typing import Optional

import serial

from comm.interface import Stm32Client


@dataclass(frozen=True)
class SerialConfig:
    port: str
    baudrate: int = 115200
    timeout: float = 0.1


class SerialStm32Client(Stm32Client):
    def __init__(self, serial_options: SerialConfig) -> None:
        self.serial_options = serial_options
        self.serial_port: Optional[serial.Serial] = None

    def open(self) -> None:
        self.serial_port = serial.Serial(
            port=self.serial_options.port,
            baudrate=self.serial_options.baudrate,
            timeout=self.serial_options.timeout,
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
