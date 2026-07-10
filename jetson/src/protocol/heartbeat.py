"""Background heartbeat service for the STM32 300 ms watchdog."""

from __future__ import annotations

from threading import Event, Lock, Thread
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from .client import SystemClient


class HeartbeatRunner:
    """Periodically send ``SYS_HEARTBEAT`` without owning the serial connection."""

    def __init__(self, system: "SystemClient", interval_s: float = 0.1) -> None:
        if interval_s <= 0 or interval_s >= 0.3:
            raise ValueError("interval_s must be in (0, 0.3) to satisfy the STM32 watchdog")
        self._system = system
        self._interval_s = interval_s
        self._stop_event = Event()
        self._lock = Lock()
        self._thread: Optional[Thread] = None
        self.last_error: Optional[Exception] = None

    @property
    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        with self._lock:
            if self.is_running:
                return
            self.last_error = None
            self._stop_event.clear()
            self._thread = Thread(target=self._run, name="stm32-heartbeat", daemon=True)
            self._thread.start()

    def stop(self, timeout: float = 1.0) -> None:
        with self._lock:
            thread = self._thread
            self._stop_event.set()
        if thread is not None:
            thread.join(timeout=timeout)

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._system.heartbeat()
            except Exception as exc:  # Keep the safety owner alive and expose the failed link.
                self.last_error = exc
            self._stop_event.wait(self._interval_s)
