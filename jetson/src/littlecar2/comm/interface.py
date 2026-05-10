from abc import ABC, abstractmethod


class Stm32Client(ABC):
    @abstractmethod
    def open(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def close(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def send(self, payload: bytes) -> None:
        raise NotImplementedError

    @abstractmethod
    def read(self, size: int = 1024) -> bytes:
        raise NotImplementedError

    def __enter__(self) -> "Stm32Client":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
