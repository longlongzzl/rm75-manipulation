"""Regression for the legacy-test collection incident; never contact a device."""
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "tools/run_network_isolated.py"


@pytest.mark.skipif(sys.platform != "linux", reason="Linux seccomp isolation")
def test_network_denial_covers_native_calls_and_exec_children():
    probe = """
import ctypes, errno, socket, subprocess, sys
lib = ctypes.CDLL(None, use_errno=True)
for family in (socket.AF_INET, socket.AF_INET6):
    for kind in (socket.SOCK_STREAM, socket.SOCK_DGRAM):
        assert lib.socket(family, kind, 0) == -1
        assert ctypes.get_errno() == errno.EPERM
a, b = socket.socketpair()
a.sendall(b'local IPC'); assert b.recv(32) == b'local IPC'
a.close(); b.close()
if len(sys.argv) == 1:
    subprocess.run([sys.executable, '-c', sys.argv[0], 'child'], check=True)
"""
    # Pass source through stdin-free argv so the exec child repeats the probe.
    launcher = "import sys; source=sys.argv[1]; sys.argv=[source]; exec(source)"
    result = subprocess.run([sys.executable, str(RUNNER), "--", sys.executable,
                             "-c", launcher, probe], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Network isolation active" in result.stderr


@pytest.mark.parametrize("selection", [[], ["."], ["rm75_app/_vendor/test_hardware.py"]])
def test_legacy_import_is_never_collected(tmp_path, selection):
    for name in ("pytest.ini", "conftest.py"):
        shutil.copy2(ROOT / name, tmp_path / name)
    (tmp_path / "tests").mkdir()
    (tmp_path / "tests/test_safe.py").write_text("def test_safe(): pass\n")
    legacy = tmp_path / "rm75_app/_vendor"
    legacy.mkdir(parents=True)
    sentinel = tmp_path / "legacy_imported"
    (legacy / "test_hardware.py").write_text(
        f"from pathlib import Path\nPath({str(sentinel)!r}).touch()\n"
        "raise RuntimeError('legacy script was imported')\n"
    )
    env = dict(os.environ, PYTEST_DISABLE_PLUGIN_AUTOLOAD="1")
    result = subprocess.run([sys.executable, "-m", "pytest", "--collect-only", "-q", *selection],
                            cwd=tmp_path, env=env, capture_output=True, text=True)
    assert not sentinel.exists(), result.stdout + result.stderr
    if selection and selection[0] != ".":
        assert result.returncode == 4
        assert "restricted to tests/" in result.stderr
    else:
        assert result.returncode == 0, result.stdout + result.stderr
        assert "test_safe.py::test_safe" in result.stdout
