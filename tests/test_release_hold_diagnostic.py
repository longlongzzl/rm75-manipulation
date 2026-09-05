from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


def test_only_selected_open_changes_target_then_restores(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "tools"))
    from diagnose_release_hold import observed_open_control
    calls = []
    class Executor:
        def set_gripper(self, closed):
            calls.append((closed, self._last_commanded_target.copy()))
    executor = Executor()
    planned = np.ones(7)
    executor._last_commanded_target = planned
    executor.control_dt = 0.05
    executor.demo = SimpleNamespace(current_arm_qpos=lambda: np.zeros(7))
    original = Executor.set_gripper
    records = []
    with observed_open_control(Executor, 2, records):
        for closed in [True, False, True, False, False]:
            executor.set_gripper(closed)
            assert executor._last_commanded_target is planned
    assert Executor.set_gripper is original
    for index, (_, target) in enumerate(calls):
        np.testing.assert_array_equal(target, np.zeros(7) if index == 3 else planned)
    assert len(records) == 1


def test_exception_restores_method_and_target(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "tools"))
    from diagnose_release_hold import observed_open_control
    class Executor:
        def set_gripper(self, closed):
            raise RuntimeError("simulation failed")
    executor = Executor()
    planned = np.ones(7)
    executor._last_commanded_target = planned
    executor.control_dt = 0.05
    executor.demo = SimpleNamespace(current_arm_qpos=lambda: np.zeros(7))
    original = Executor.set_gripper
    with pytest.raises(RuntimeError, match="simulation failed"):
        with observed_open_control(Executor, 1, []):
            executor.set_gripper(False)
    assert executor._last_commanded_target is planned
    assert Executor.set_gripper is original


def test_multiple_selected_openings_restore_nested_wrappers(monkeypatch):
    from contextlib import ExitStack
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "tools"))
    from diagnose_release_hold import observed_open_control
    calls = []
    class Executor:
        def set_gripper(self, closed):
            calls.append(self._last_commanded_target.copy())
    executor = Executor()
    planned = np.ones(7)
    executor._last_commanded_target = planned
    executor.control_dt = 0.05
    executor.demo = SimpleNamespace(current_arm_qpos=lambda: np.zeros(7))
    original = Executor.set_gripper
    records = []
    with ExitStack() as stack:
        for index in [1, 2, 3]:
            stack.enter_context(observed_open_control(Executor, index, records))
        for _ in range(3):
            executor.set_gripper(True)
            executor.set_gripper(False)
            assert executor._last_commanded_target is planned
    assert Executor.set_gripper is original
    assert [r["open_index"] for r in records] == [1, 2, 3]
    for index, target in enumerate(calls):
        np.testing.assert_array_equal(target, planned if index % 2 == 0 else np.zeros(7))
