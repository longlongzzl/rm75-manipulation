"""Small, strict persistence helpers used by the web and worker processes."""
from __future__ import annotations

import hashlib
import json
import math
import os
import tempfile
from pathlib import Path
from typing import Any


def json_default(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if hasattr(value, 'tolist'):
        return value.tolist()
    raise TypeError(f'Not JSON serializable: {type(value).__name__}')


def dumps(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, allow_nan=False,
                      sort_keys=True, default=json_default)


def loads(text: str) -> Any:
    def invalid(value: str) -> None:
        raise ValueError(f'Non-finite JSON number: {value}')
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f'Duplicate JSON key: {key}')
            result[key] = value
        return result
    return json.loads(text, parse_constant=invalid, object_pairs_hook=unique)


def read_json(path: Path, *, max_bytes: int = 8_000_000) -> Any:
    with Path(path).open('rb') as stream:
        raw = stream.read(max_bytes + 1)
    if len(raw) > max_bytes:
        raise ValueError(f'JSON exceeds {max_bytes} bytes: {path}')
    return loads(raw.decode('utf-8'))


def atomic_json(path: Path, value: Any) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    text = dumps(value) + '\n'
    fd, temp = tempfile.mkstemp(prefix=f'.{path.name}.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, path)
    finally:
        if os.path.exists(temp):
            os.unlink(temp)


def append_jsonl(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a', encoding='utf-8') as stream:
        stream.write(dumps(value) + '\n')
        stream.flush()


def digest(value: Any) -> str:
    return hashlib.sha256(dumps(value).encode()).hexdigest()


def contained(path: Path, root: Path) -> Path:
    path, root = Path(path).resolve(), Path(root).resolve()
    if not path.is_relative_to(root):
        raise ValueError(f'Path is outside allowed root: {path}')
    return path


def finite(value: Any, name: str, low: float | None = None,
           high: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be a number')
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f'{name} must be finite')
    if low is not None and result < low or high is not None and result > high:
        raise ValueError(f'{name} is outside [{low}, {high}]')
    return result


def integer(value: Any, name: str, low: int, high: int) -> int:
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f'{name} must be an integer in [{low}, {high}]')
    return value
