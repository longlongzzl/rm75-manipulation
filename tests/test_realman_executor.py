from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pytest

from rm75_app.execution.realman_executor import (
    RM75_JOINT_NAMES,
    RealManConnectionConfig,
    RealManExecutionConfig,
    RealManHardwareError,
    RealManSDKSession,
    RealManTrajectoryExecutor,
)
from rm75_app.planning.contracts import JointTrajectory


@dataclass
class _Handle:
    id: int = 1


class _RegisterParam:
    def __init__(self, *args):
        self.args = args


class _FakeArm:
    def __init__(self):
        self.joint_deg = np.zeros(7, dtype=np.float64)
        self.follow_commands: list[np.ndarray] = []
        self.hand_commands: list[tuple[list[int], bool]] = []
        self.register_writes: list[tuple[tuple, list[int] | int]] = []
        self.gripper_raw = 0
        self.stopped = False
        self.deleted = False

    def rm_create_robot_arm(self, ip, port):
        assert ip == "192.168.101.20"
        assert port == 8080
        return _Handle()

    def rm_get_joint_degree(self):
        return 0, self.joint_deg.tolist()

    def rm_movej_follow(self, degrees):
        values = np.asarray(degrees, dtype=np.float64)
        self.follow_commands.append(values.copy())
        self.joint_deg = values.copy()
        return 0

    def rm_set_arm_stop(self):
        self.stopped = True
        return 0

    def rm_set_modbus_mode(self, port, baud, timeout):
        assert (port, baud, timeout) == (1, 115200, 2)
        return 0

    def rm_write_single_register(self, param, value):
        self.register_writes.append((param.args, int(value)))
        return 0

    def rm_write_registers(self, param, values):
        values = list(values)
        self.register_writes.append((param.args, values))
        if param.args[:2] == (1, 258):
            self.gripper_raw = int(values[1])
        return 0

    def rm_read_holding_registers(self, param):
        if param.args[:2] == (1, 259):
            return 0, self.gripper_raw
        return 0, 0

    def rm_delete_robot_arm(self):
        self.deleted = True
        return 0


def _session(arm: _FakeArm) -> RealManSDKSession:
    return RealManSDKSession(
        RealManConnectionConfig(),
        arm_factory=lambda: arm,
        register_param_factory=_RegisterParam,
    )


def _executor(arm: _FakeArm) -> RealManTrajectoryExecutor:
    session = _session(arm)
    session.connect()
    return RealManTrajectoryExecutor(
        session,
        RealManExecutionConfig(
            control_period_s=0.02,
            endpoint_timeout_s=0.0,
            terminal_hold_s=0.0,
            gripper_settle_s=0.0,
            pace_commands=False,
        ),
    )


def test_session_connect_reads_physical_joint_state_in_radians():
    arm = _FakeArm()
    arm.joint_deg = np.asarray([90, 0, 0, -90, 0, -90, 60], dtype=np.float64)
    session = _session(arm)
    session.connect()
    state = session.joint_configuration()
    assert state.names == RM75_JOINT_NAMES
    np.testing.assert_allclose(
        state.positions,
        np.deg2rad(arm.joint_deg),
        atol=1e-12,
    )
    session.close()
    assert arm.deleted


def test_preflight_is_ready_but_executor_stays_disarmed():
    arm = _FakeArm()
    executor = _executor(arm)
    report = executor.preflight()
    assert report.ready
    assert all(report.checks.values())
    assert not executor.armed


def test_motion_is_rejected_until_explicitly_armed():
    arm = _FakeArm()
    executor = _executor(arm)
    trajectory = JointTrajectory(
        RM75_JOINT_NAMES,
        np.vstack([np.zeros(7), np.full(7, 0.01)]),
        dt=0.02,
    )
    with pytest.raises(RealManHardwareError, match="disarmed"):
        executor.execute_trajectory("approach", trajectory)
    assert not arm.follow_commands


def test_timed_trajectory_streams_degrees_and_checks_endpoint_feedback():
    arm = _FakeArm()
    executor = _executor(arm)
    executor.arm_execution()
    target = np.linspace(0.002, 0.014, 7)
    trajectory = JointTrajectory(
        RM75_JOINT_NAMES,
        np.vstack([np.zeros(7), target]),
        dt=0.02,
    )
    executor.execute_trajectory("approach", trajectory)
    assert arm.follow_commands
    np.testing.assert_allclose(arm.follow_commands[-1], np.rad2deg(target))
    assert executor.last_stage_metrics["endpoint_error_rad"] <= 1e-12
    assert executor.armed


def test_stage_start_mismatch_is_rejected_without_sending_motion():
    arm = _FakeArm()
    executor = _executor(arm)
    executor.arm_execution()
    trajectory = JointTrajectory(
        RM75_JOINT_NAMES,
        np.vstack([np.full(7, 0.4), np.full(7, 0.41)]),
        dt=0.02,
    )
    with pytest.raises(RealManHardwareError, match="stage start gap"):
        executor.execute_trajectory("bad_start", trajectory)
    assert not arm.follow_commands


def test_gripper_uses_historical_modbus_target_and_readback():
    arm = _FakeArm()
    executor = _executor(arm)
    executor.arm_execution()
    executor.set_gripper(True)
    assert arm.gripper_raw == 9000
    assert executor.last_gripper_metrics["observed_raw"] == 9000
    executor.set_gripper(False)
    assert arm.gripper_raw == 0


def test_stop_disarms_and_calls_controller_stop():
    arm = _FakeArm()
    executor = _executor(arm)
    executor.arm_execution()
    executor.stop()
    assert arm.stopped
    assert not executor.armed
