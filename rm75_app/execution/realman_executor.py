"""Guarded RealMan RM75 hardware execution adapter.

This module intentionally keeps the hardware boundary small.  The planning and
scenario layers exchange :class:`JointTrajectory` / :class:`JointConfiguration`
only; the RealMan SDK is imported lazily when a physical connection is opened.

The SDK calls below are distilled from the historical, physically exercised
RealMan code in ``longlongzzl/lerobot-realman``:

* ``rm_create_robot_arm`` for the RM75 connection;
* ``rm_get_joint_degree`` / ``rm_get_current_arm_state`` for feedback;
* ``rm_movej_follow`` for high-rate joint following;
* Modbus or ``rm_set_hand_follow_pos`` for the gripper.

Connecting never moves the robot.  Motion and gripper commands are rejected until
``arm_execution()`` is called explicitly.  This makes the same adapter suitable
for a future web control panel where connection/preflight and physical arming are
separate user actions.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping

import numpy as np

from rm75_app.planning.contracts import JointConfiguration, JointTrajectory

from .trajectory_executor import sample_timed_joint_path


RM75_JOINT_NAMES = tuple(f"joint_{index}" for index in range(1, 8))


class RealManHardwareError(RuntimeError):
    """Raised when physical RM75 communication or execution is unsafe/invalid."""


@dataclass(frozen=True)
class RealManConnectionConfig:
    """Connection and IO configuration for the physical RM75.

    ``ip`` defaults to the address used by the later PickPlace/LeRobot hardware
    scripts.  Change it explicitly for another robot; opening the connection does
    not issue any motion command.
    """

    ip: str = "192.168.101.20"
    port: int = 8080
    joint_names: tuple[str, ...] = RM75_JOINT_NAMES
    configure_gripper: bool = True
    gripper_backend: str = "modbus"  # ``modbus`` or ``hand_follow``
    gripper_open_raw: int = 0
    gripper_closed_raw: int = 9000
    gripper_speed_register_value: int = 100

    def __post_init__(self) -> None:
        if not self.ip:
            raise ValueError("RealMan ip must not be empty")
        if not 1 <= int(self.port) <= 65535:
            raise ValueError("RealMan port is out of range")
        names = tuple(str(name) for name in self.joint_names)
        if len(names) != 7 or len(set(names)) != 7:
            raise ValueError("RM75 requires exactly seven unique joint names")
        if self.gripper_backend not in {"modbus", "hand_follow"}:
            raise ValueError("gripper_backend must be 'modbus' or 'hand_follow'")
        for value in (self.gripper_open_raw, self.gripper_closed_raw):
            if not 0 <= int(value) <= 9000:
                raise ValueError("gripper raw targets must lie in [0, 9000]")
        object.__setattr__(self, "joint_names", names)


@dataclass(frozen=True)
class RealManExecutionConfig:
    """Physical trajectory guards and streaming timing."""

    control_period_s: float = 0.02
    require_timestamps: bool = True
    max_stage_start_gap_rad: float = 0.12
    max_point_delta_rad: float = 0.20
    endpoint_tolerance_rad: float = 0.08
    endpoint_timeout_s: float = 1.5
    terminal_hold_s: float = 0.10
    gripper_settle_s: float = 0.35
    pace_commands: bool = True

    def __post_init__(self) -> None:
        positive = {
            "control_period_s": self.control_period_s,
            "max_stage_start_gap_rad": self.max_stage_start_gap_rad,
            "max_point_delta_rad": self.max_point_delta_rad,
            "endpoint_tolerance_rad": self.endpoint_tolerance_rad,
        }
        for name, value in positive.items():
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        for name, value in {
            "endpoint_timeout_s": self.endpoint_timeout_s,
            "terminal_hold_s": self.terminal_hold_s,
            "gripper_settle_s": self.gripper_settle_s,
        }.items():
            if not np.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and non-negative")


@dataclass(frozen=True)
class RealManPreflightReport:
    ready: bool
    checks: Mapping[str, bool]
    diagnostics: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "checks", dict(self.checks))
        object.__setattr__(self, "diagnostics", dict(self.diagnostics))


class RealManSDKSession:
    """Minimal RM75 SDK session with no implicit reset/home motion."""

    _STOP_METHODS = (
        "rm_set_arm_stop",
        "rm_set_arm_slow_stop",
        "rm_set_arm_pause",
    )

    def __init__(
        self,
        config: RealManConnectionConfig | None = None,
        *,
        arm_factory: Callable[[], Any] | None = None,
        register_param_factory: Callable[..., Any] | None = None,
    ) -> None:
        self.config = config or RealManConnectionConfig()
        self._arm_factory = arm_factory
        self._register_param_factory = register_param_factory
        self.arm: Any | None = None
        self.handle: Any | None = None
        self.connected = False
        self._sdk_import_error: str | None = None

    def _load_sdk(self) -> tuple[Callable[[], Any], Callable[..., Any] | None]:
        if self._arm_factory is not None:
            return self._arm_factory, self._register_param_factory
        try:
            from Robotic_Arm.rm_robot_interface import (  # type: ignore
                RoboticArm,
                rm_peripheral_read_write_params_t,
                rm_thread_mode_e,
            )
        except Exception as exc:  # pragma: no cover - only on hardware env
            self._sdk_import_error = f"{type(exc).__name__}: {exc}"
            raise RealManHardwareError(
                "RealMan SDK is unavailable; install/import Robotic_Arm.rm_robot_interface"
            ) from exc

        def factory() -> Any:
            return RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)

        return factory, rm_peripheral_read_write_params_t

    @staticmethod
    def _status_ok(status: Any, operation: str) -> None:
        if int(status) != 0:
            raise RealManHardwareError(f"{operation} failed with status={status}")

    def _require_connected(self) -> Any:
        if not self.connected or self.arm is None:
            raise RealManHardwareError("RM75 is not connected")
        return self.arm

    def connect(self) -> None:
        if self.connected:
            return
        factory, register_factory = self._load_sdk()
        arm = factory()
        try:
            handle = arm.rm_create_robot_arm(self.config.ip, int(self.config.port))
            handle_id = getattr(handle, "id", None)
            if handle_id is not None and int(handle_id) <= 0:
                raise RealManHardwareError(
                    f"RM75 connection returned invalid handle id={handle_id}"
                )
            self.arm = arm
            self.handle = handle
            self._register_param_factory = register_factory
            self.connected = True
            # A state read proves that this is not merely a socket creation.
            _ = self.read_joint_radians()
            if self.config.configure_gripper:
                self._configure_gripper()
        except Exception:
            try:
                delete = getattr(arm, "rm_delete_robot_arm", None)
                if callable(delete):
                    delete()
            finally:
                self.arm = None
                self.handle = None
                self.connected = False
            raise

    def _configure_gripper(self) -> None:
        arm = self._require_connected()
        if self.config.gripper_backend == "hand_follow":
            set_plus = getattr(arm, "rm_set_rm_plus_mode", None)
            if callable(set_plus):
                status = set_plus(115200)
                if status is not None:
                    self._status_ok(status, "rm_set_rm_plus_mode")
            set_speed = getattr(arm, "rm_set_hand_speed", None)
            if callable(set_speed):
                status = set_speed(1000)
                if status is not None:
                    self._status_ok(status, "rm_set_hand_speed")
            return
        if self._register_param_factory is None:
            raise RealManHardwareError(
                "Modbus gripper requested but rm_peripheral_read_write_params_t is unavailable"
            )
        set_mode = getattr(arm, "rm_set_modbus_mode", None)
        if callable(set_mode):
            self._status_ok(set_mode(1, 115200, 2), "rm_set_modbus_mode")
        param = self._register_param_factory(1, 260, 1)
        self._status_ok(
            arm.rm_write_single_register(
                param, int(self.config.gripper_speed_register_value)
            ),
            "gripper speed register",
        )

    def read_joint_radians(self) -> np.ndarray:
        arm = self._require_connected()
        getter = getattr(arm, "rm_get_joint_degree", None)
        if callable(getter):
            status, values = getter()
            self._status_ok(status, "rm_get_joint_degree")
            degrees = np.asarray(values, dtype=np.float64).reshape(-1)
        else:
            getter = getattr(arm, "rm_get_current_arm_state", None)
            if not callable(getter):
                raise RealManHardwareError("SDK exposes no supported joint-state method")
            status, state = getter()
            self._status_ok(status, "rm_get_current_arm_state")
            if state is None or "joint" not in state:
                raise RealManHardwareError("current arm state contains no joint vector")
            degrees = np.asarray(state["joint"], dtype=np.float64).reshape(-1)
        if degrees.shape != (7,) or not np.all(np.isfinite(degrees)):
            raise RealManHardwareError(
                f"expected seven finite RM75 joint degrees, got shape={degrees.shape}"
            )
        return np.deg2rad(degrees)

    def joint_configuration(self) -> JointConfiguration:
        return JointConfiguration(self.config.joint_names, self.read_joint_radians())

    def send_joint_follow(self, radians: np.ndarray) -> None:
        arm = self._require_connected()
        target = np.asarray(radians, dtype=np.float64).reshape(-1)
        if target.shape != (7,) or not np.all(np.isfinite(target)):
            raise RealManHardwareError("RM75 target must contain seven finite joint radians")
        degrees = np.rad2deg(target).tolist()
        follower = getattr(arm, "rm_movej_follow", None)
        if not callable(follower):
            raise RealManHardwareError("SDK does not expose rm_movej_follow")
        self._status_ok(follower(degrees), "rm_movej_follow")

    def set_gripper_raw(self, raw: int) -> None:
        arm = self._require_connected()
        raw = int(np.clip(int(raw), 0, 9000))
        if self.config.gripper_backend == "hand_follow":
            command = getattr(arm, "rm_set_hand_follow_pos", None)
            if not callable(command):
                raise RealManHardwareError("SDK does not expose rm_set_hand_follow_pos")
            status = command([raw], False)
            if status is not None:
                self._status_ok(status, "rm_set_hand_follow_pos")
            return
        if self._register_param_factory is None:
            raise RealManHardwareError("Modbus register factory unavailable")
        param = self._register_param_factory(1, 258, 1, 2)
        self._status_ok(
            arm.rm_write_registers(param, [0, raw, 0, 0]),
            "gripper target registers",
        )
        param = self._register_param_factory(1, 264, 1)
        self._status_ok(
            arm.rm_write_single_register(param, 1),
            "gripper execute register",
        )

    def set_gripper(self, closed: bool) -> None:
        self.set_gripper_raw(
            self.config.gripper_closed_raw if closed else self.config.gripper_open_raw
        )

    def read_gripper_raw(self) -> int | None:
        if self.config.gripper_backend != "modbus":
            return None
        arm = self._require_connected()
        if self._register_param_factory is None:
            return None
        reader = getattr(arm, "rm_read_holding_registers", None)
        if not callable(reader):
            return None
        param = self._register_param_factory(1, 259, 1)
        status, value = reader(param)
        self._status_ok(status, "gripper position register")
        return int(value)

    @property
    def stop_available(self) -> bool:
        if self.arm is None:
            return False
        return any(callable(getattr(self.arm, name, None)) for name in self._STOP_METHODS)

    def stop(self) -> bool:
        """Request a controller-side stop using the first SDK method available."""
        arm = self._require_connected()
        for name in self._STOP_METHODS:
            method = getattr(arm, name, None)
            if not callable(method):
                continue
            status = method()
            if status is not None:
                self._status_ok(status, name)
            return True
        return False

    def close(self) -> None:
        if self.arm is None:
            self.connected = False
            return
        arm = self.arm
        try:
            delete = getattr(arm, "rm_delete_robot_arm", None)
            if callable(delete):
                delete()
            else:
                destroy = getattr(arm, "rm_destroy", None)
                if callable(destroy):
                    destroy()
        finally:
            self.arm = None
            self.handle = None
            self.connected = False

    def __enter__(self) -> "RealManSDKSession":
        self.connect()
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.close()


class RealManTrajectoryExecutor:
    """Trajectory sink used by sorting, magnetic assembly, and Push-T.

    A single instance can be passed as both:

    * ``trajectory_sink`` (``execute_trajectory`` / ``set_gripper``), and
    * ``joint_state_provider`` (``joint_configuration``).

    It also exposes ``stop`` for :class:`PickPlaceProgramExecutor` and future web
    controls.
    """

    def __init__(
        self,
        session: RealManSDKSession,
        config: RealManExecutionConfig | None = None,
        *,
        sleep_fn: Callable[[float], None] = time.sleep,
        clock_fn: Callable[[], float] = time.monotonic,
    ) -> None:
        self.session = session
        self.config = config or RealManExecutionConfig()
        self._sleep = sleep_fn
        self._clock = clock_fn
        self._armed = False
        self.last_stage_metrics: dict[str, Any] = {}
        self.last_gripper_metrics: dict[str, Any] = {}

    @property
    def armed(self) -> bool:
        return self._armed

    def arm_execution(self) -> RealManPreflightReport:
        report = self.preflight()
        if not report.ready:
            failed = [name for name, ok in report.checks.items() if not ok]
            raise RealManHardwareError(
                "RM75 physical execution preflight failed: " + ", ".join(failed)
            )
        self._armed = True
        return report

    def disarm_execution(self) -> None:
        self._armed = False

    def _require_armed(self) -> None:
        if not self._armed:
            raise RealManHardwareError(
                "RM75 physical execution is disarmed; call arm_execution() after preflight"
            )

    def preflight(self) -> RealManPreflightReport:
        checks: dict[str, bool] = {
            "connected": bool(self.session.connected),
            "joint_feedback": False,
            "joint_names_match_rm75": tuple(self.session.config.joint_names) == RM75_JOINT_NAMES,
            "movej_follow_available": False,
            "stop_available": False,
            "gripper_io_available": False,
        }
        diagnostics: dict[str, Any] = {
            "ip": self.session.config.ip,
            "port": int(self.session.config.port),
            "armed": bool(self._armed),
        }
        if not self.session.connected or self.session.arm is None:
            return RealManPreflightReport(False, checks, diagnostics)
        arm = self.session.arm
        checks["movej_follow_available"] = callable(getattr(arm, "rm_movej_follow", None))
        checks["stop_available"] = self.session.stop_available
        if self.session.config.gripper_backend == "hand_follow":
            checks["gripper_io_available"] = callable(
                getattr(arm, "rm_set_hand_follow_pos", None)
            )
        else:
            checks["gripper_io_available"] = (
                self.session._register_param_factory is not None
                and callable(getattr(arm, "rm_write_registers", None))
                and callable(getattr(arm, "rm_write_single_register", None))
            )
        try:
            q = self.session.read_joint_radians()
            checks["joint_feedback"] = q.shape == (7,) and bool(np.all(np.isfinite(q)))
            diagnostics["joint_radians"] = q.tolist()
        except Exception as exc:
            diagnostics["joint_feedback_error"] = f"{type(exc).__name__}: {exc}"
        return RealManPreflightReport(all(checks.values()), checks, diagnostics)

    def joint_configuration(self) -> JointConfiguration:
        return self.session.joint_configuration()

    def stop(self) -> None:
        # Disarm first so a caller cannot continue a suffix after a stop request.
        self._armed = False
        if not self.session.stop():
            raise RealManHardwareError("no controller-side stop method is available")

    def _validate_trajectory(
        self,
        trajectory: JointTrajectory,
    ) -> tuple[np.ndarray, float]:
        if tuple(trajectory.joint_names) != tuple(self.session.config.joint_names):
            raise RealManHardwareError(
                "planned joint names do not match the configured physical RM75"
            )
        current = self.session.read_joint_radians()
        start = np.asarray(trajectory.positions[0], dtype=np.float64)
        start_gap = float(np.max(np.abs(current - start)))
        if start_gap > self.config.max_stage_start_gap_rad:
            raise RealManHardwareError(
                f"stage start gap {start_gap:.6f} rad exceeds "
                f"{self.config.max_stage_start_gap_rad:.6f} rad"
            )
        if trajectory.dt is None:
            if self.config.require_timestamps:
                raise RealManHardwareError(
                    "physical RM75 replay requires explicit trajectory dt"
                )
            samples = np.asarray(trajectory.positions, dtype=np.float64)
        else:
            samples = sample_timed_joint_path(
                trajectory,
                self.config.control_period_s,
            )
        if samples.ndim != 2 or samples.shape[1] != 7 or len(samples) == 0:
            raise RealManHardwareError("physical replay produced an invalid sample array")
        chain = np.vstack([current, samples])
        largest_delta = float(np.max(np.abs(np.diff(chain, axis=0))))
        if largest_delta > self.config.max_point_delta_rad:
            raise RealManHardwareError(
                f"trajectory point jump {largest_delta:.6f} rad exceeds "
                f"{self.config.max_point_delta_rad:.6f} rad"
            )
        return samples, start_gap

    def _stream_samples(self, samples: np.ndarray) -> None:
        next_deadline = self._clock()
        for sample in samples:
            self._require_armed()
            self.session.send_joint_follow(sample)
            if not self.config.pace_commands:
                continue
            next_deadline += self.config.control_period_s
            remaining = next_deadline - self._clock()
            if remaining > 0:
                self._sleep(remaining)

        if self.config.terminal_hold_s <= 0:
            return
        hold_count = max(
            1,
            int(math.ceil(self.config.terminal_hold_s / self.config.control_period_s)),
        )
        final = samples[-1]
        for _ in range(hold_count):
            self._require_armed()
            self.session.send_joint_follow(final)
            if self.config.pace_commands:
                self._sleep(self.config.control_period_s)

    def _wait_for_endpoint(self, target: np.ndarray) -> float:
        deadline = self._clock() + self.config.endpoint_timeout_s
        error = float("inf")
        while True:
            observed = self.session.read_joint_radians()
            error = float(np.max(np.abs(observed - target)))
            if error <= self.config.endpoint_tolerance_rad:
                return error
            if self._clock() >= deadline:
                raise RealManHardwareError(
                    f"RM75 endpoint tracking error {error:.6f} rad exceeds "
                    f"{self.config.endpoint_tolerance_rad:.6f} rad"
                )
            self._sleep(min(self.config.control_period_s, 0.05))

    def execute_trajectory(self, stage: str, trajectory: JointTrajectory) -> None:
        self._require_armed()
        samples, start_gap = self._validate_trajectory(trajectory)
        started = self._clock()
        try:
            self._stream_samples(samples)
            endpoint_error = self._wait_for_endpoint(samples[-1])
        except Exception:
            self._armed = False
            # Best effort only; do not hide the original execution failure.
            try:
                if self.session.stop_available:
                    self.session.stop()
            except Exception:
                pass
            raise
        self.last_stage_metrics = {
            "stage": str(stage),
            "sample_count": int(len(samples)),
            "start_gap_rad": start_gap,
            "endpoint_error_rad": endpoint_error,
            "elapsed_s": float(self._clock() - started),
            "control_period_s": self.config.control_period_s,
        }

    def set_gripper(self, closed: bool) -> None:
        self._require_armed()
        started = self._clock()
        try:
            self.session.set_gripper(bool(closed))
            if self.config.gripper_settle_s > 0:
                self._sleep(self.config.gripper_settle_s)
            raw = self.session.read_gripper_raw()
        except Exception:
            self._armed = False
            raise
        self.last_gripper_metrics = {
            "closed": bool(closed),
            "observed_raw": raw,
            "elapsed_s": float(self._clock() - started),
        }
