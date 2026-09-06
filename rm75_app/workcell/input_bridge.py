"""Expose original input() prompts from parent and inherited-stdin Python children.

The bridge never invents an answer. One pending nonce is bound to one actual
blocking input() call. Non-Python GUI dialogs are deliberately not emulated.
"""
from __future__ import annotations
import builtins
import contextlib
import fcntl
import os
import time
import uuid
from pathlib import Path
from .io import atomic_json, read_json


def install(directory):
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    original = builtins.input
    if getattr(original, '_rm75_input_bridge', False):
        return original

    def observed_input(prompt=''):
        with (directory / 'input.lock').open('a') as lease:
            fcntl.flock(lease.fileno(), fcntl.LOCK_EX)
            path = directory / 'pending_input.json'
            nonce = uuid.uuid4().hex
            atomic_json(path, {'nonce': nonce, 'pid': os.getpid(),
                              'prompt': str(prompt), 'created_at': time.time(),
                              'choices': ['', 'r', 'q']})
            try:
                return original(prompt)
            finally:
                with contextlib.suppress(FileNotFoundError, ValueError):
                    if read_json(path).get('nonce') == nonce:
                        path.unlink()
    observed_input._rm75_input_bridge = True
    builtins.input = observed_input
    return original


def install_subprocess_bridge(directory):
    """Preserve Popen IO/env/cwd while wrapping Python .py provider entrypoints.

    Module launches, shell commands and non-Python subprocesses are untouched.
    This is process-local and restored by the worker; it never patches an
    installed interpreter's startup hooks.
    """
    import re
    import subprocess
    original = subprocess.Popen
    if getattr(original, '_rm75_input_bridge', False):
        return original
    bootstrap = Path(__file__).with_name('child_entry.py').resolve()

    class BridgedPopen(original):
        _rm75_input_bridge = True

        def __init__(self, args, *positional, **kwargs):
            command = args
            if isinstance(args, (list, tuple)) and len(args) > 1 and not kwargs.get('shell', False):
                executable = Path(str(args[0])).name
                if re.fullmatch(r'python(?:\d+(?:\.\d+)*)?', executable):
                    index = 1
                    while index < len(args) and str(args[index]) in ('-u', '-B', '-s', '-E'):
                        index += 1
                    if index < len(args) and str(args[index]).endswith('.py') and Path(str(args[index])).resolve() != bootstrap:
                        command = [*args[:index], str(bootstrap), *args[index:]]
                        env = dict(os.environ if kwargs.get('env') is None else kwargs['env'])
                        env['RM75_WORKCELL_INPUT_DIR'] = str(Path(directory).resolve())
                        kwargs['env'] = env
            super().__init__(command, *positional, **kwargs)
    subprocess.Popen = BridgedPopen
    return original
