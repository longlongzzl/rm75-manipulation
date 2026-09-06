"""Cross-process ownership. A killed real worker leaves a latched uncertainty."""
from __future__ import annotations

import fcntl
from pathlib import Path


class ResourceLease:
    def __init__(self, path: Path):
        self.path = Path(path)
        self._file = None

    def __enter__(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open('a+')
        try:
            fcntl.flock(self._file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self._file.close()
            self._file = None
            raise RuntimeError('Another process owns this robot/workcell') from None
        return self

    def __exit__(self, *exc):
        if self._file is not None:
            fcntl.flock(self._file.fileno(), fcntl.LOCK_UN)
            self._file.close()
            self._file = None
