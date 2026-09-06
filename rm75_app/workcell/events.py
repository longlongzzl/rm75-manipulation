from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Any
from .io import append_jsonl, atomic_json


class Cancelled(RuntimeError):
    pass


class EventLog:
    def __init__(self, directory: Path):
        self.directory = Path(directory)
        self._lock = threading.Lock()
        self._sequence = 0
        self.last: dict[str, Any] = {}

    def emit(self, kind: str, **data: Any) -> dict[str, Any]:
        with self._lock:
            self._sequence += 1
            row = dict(data, sequence=self._sequence, time=time.time(), kind=kind)
            append_jsonl(self.directory / 'events.jsonl', row)
            self.last = row
            atomic_json(self.directory / 'progress.json', row)
            return row


class StopToken:
    """Cancellation does not open the gripper or issue an automatic retreat."""
    def __init__(self, path: Path | None = None):
        self.path = path
        self.event = threading.Event()

    def request(self) -> None:
        self.event.set()

    @property
    def requested(self) -> bool:
        return self.event.is_set() or (self.path is not None and self.path.exists())

    def check(self) -> None:
        if self.requested:
            raise Cancelled('Operator requested a stop')

    def wait(self, seconds: float) -> None:
        end = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < end:
            self.check()
            self.event.wait(min(0.02, end - time.monotonic()))
        self.check()
