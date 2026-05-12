"""Protocol exception types."""

from __future__ import annotations

from .commands import AckCode


class ProtocolError(Exception):
    """Base exception for protocol failures."""


class ProtocolTimeoutError(ProtocolError):
    """Raised when an expected ACK or DATA frame is not received in time."""


class CommandRejectedError(ProtocolError):
    """Raised when STM32 returns a non-OK ACK result."""

    def __init__(self, cmd_set: int, cmd_id: int, seq: int, result: int, detail: int) -> None:
        self.cmd_set = cmd_set
        self.cmd_id = cmd_id
        self.seq = seq
        self.result = result
        self.detail = detail
        try:
            result_name = AckCode(result).name
        except ValueError:
            result_name = f"0x{result:02X}"
        super().__init__(
            f"command rejected: cmd_set=0x{cmd_set:02X}, cmd_id=0x{cmd_id:02X}, "
            f"seq={seq}, result={result_name}, detail=0x{detail:02X}"
        )
