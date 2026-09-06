"""Flask lifecycle API for the shared physical RM75 executor.

This API intentionally exposes no arbitrary joint/cartesian motion endpoint.
Scenario frontends obtain a guarded executor through ``RealManHardwareManager``;
the browser may only connect, inspect preflight/state, explicitly arm/disarm, stop,
and disconnect the physical robot.
"""

from __future__ import annotations

import threading
from dataclasses import asdict
from typing import Any, Callable

from flask import Blueprint, jsonify, request

from rm75_app.execution.realman_executor import (
    RealManConnectionConfig,
    RealManExecutionConfig,
    RealManHardwareError,
    RealManSDKSession,
    RealManTrajectoryExecutor,
)


ARM_CONFIRMATION = "ARM REAL RM75"


class RealManHardwareManager:
    """Own exactly one RM75 SDK connection for a web/process lifetime."""

    def __init__(
        self,
        *,
        session_factory: Callable[[RealManConnectionConfig], RealManSDKSession] | None = None,
    ) -> None:
        self._session_factory = session_factory or (lambda cfg: RealManSDKSession(cfg))
        self._lock = threading.RLock()
        self._session: RealManSDKSession | None = None
        self._executor: RealManTrajectoryExecutor | None = None

    @property
    def executor(self) -> RealManTrajectoryExecutor:
        with self._lock:
            if self._executor is None:
                raise RealManHardwareError("RM75 web session is not connected")
            return self._executor

    def connect(
        self,
        config: RealManConnectionConfig,
        execution_config: RealManExecutionConfig | None = None,
    ) -> dict[str, Any]:
        with self._lock:
            if self._session is not None and self._session.connected:
                return self.status()
            session = self._session_factory(config)
            session.connect()
            executor = RealManTrajectoryExecutor(session, execution_config)
            self._session = session
            self._executor = executor
            return self.status()

    def disconnect(self) -> dict[str, Any]:
        with self._lock:
            if self._executor is not None:
                self._executor.disarm_execution()
            if self._session is not None:
                self._session.close()
            self._session = None
            self._executor = None
            return self.status()

    def arm(self, confirmation: str) -> dict[str, Any]:
        if confirmation != ARM_CONFIRMATION:
            raise RealManHardwareError(
                f"physical arm confirmation must equal {ARM_CONFIRMATION!r}"
            )
        with self._lock:
            executor = self.executor
            executor.arm_execution()
            return self.status()

    def disarm(self) -> dict[str, Any]:
        with self._lock:
            if self._executor is not None:
                self._executor.disarm_execution()
            return self.status()

    def stop(self) -> dict[str, Any]:
        with self._lock:
            executor = self.executor
            executor.stop()
            return self.status()

    def status(self) -> dict[str, Any]:
        with self._lock:
            connected = bool(self._session is not None and self._session.connected)
            payload: dict[str, Any] = {
                "connected": connected,
                "armed": bool(self._executor is not None and self._executor.armed),
                "ready": False,
                "checks": {},
            }
            if not connected or self._session is None or self._executor is None:
                return payload
            payload["ip"] = self._session.config.ip
            payload["port"] = int(self._session.config.port)
            payload["gripper_backend"] = self._session.config.gripper_backend
            try:
                report = self._executor.preflight()
                payload["ready"] = bool(report.ready)
                payload["checks"] = dict(report.checks)
                payload["diagnostics"] = dict(report.diagnostics)
            except Exception as exc:
                payload["preflight_error"] = f"{type(exc).__name__}: {exc}"
            return payload


def _json_error(exc: Exception, status: int = 400):
    return jsonify({"ok": False, "error": f"{type(exc).__name__}: {exc}"}), status


def create_realman_hardware_blueprint(
    manager: RealManHardwareManager | None = None,
) -> Blueprint:
    manager = manager or RealManHardwareManager()
    blueprint = Blueprint("realman_hardware", __name__)
    # Expose the manager for same-process scenario wiring without a second SDK connection.
    blueprint.realman_manager = manager  # type: ignore[attr-defined]

    @blueprint.get("/api/robot/status")
    def robot_status():
        return jsonify({"ok": True, **manager.status()})

    @blueprint.post("/api/robot/connect")
    def robot_connect():
        try:
            payload = request.get_json(silent=True) or {}
            config = RealManConnectionConfig(
                ip=str(payload.get("ip") or "192.168.101.20"),
                port=int(payload.get("port") or 8080),
                gripper_backend=str(payload.get("gripper_backend") or "modbus"),
                configure_gripper=bool(payload.get("configure_gripper", True)),
            )
            return jsonify({"ok": True, **manager.connect(config)})
        except Exception as exc:
            return _json_error(exc)

    @blueprint.post("/api/robot/arm")
    def robot_arm():
        try:
            payload = request.get_json(silent=True) or {}
            return jsonify(
                {"ok": True, **manager.arm(str(payload.get("confirmation") or ""))}
            )
        except Exception as exc:
            return _json_error(exc)

    @blueprint.post("/api/robot/disarm")
    def robot_disarm():
        try:
            return jsonify({"ok": True, **manager.disarm()})
        except Exception as exc:
            return _json_error(exc)

    @blueprint.post("/api/robot/stop")
    def robot_stop():
        try:
            return jsonify({"ok": True, **manager.stop()})
        except Exception as exc:
            return _json_error(exc)

    @blueprint.post("/api/robot/disconnect")
    def robot_disconnect():
        try:
            return jsonify({"ok": True, **manager.disconnect()})
        except Exception as exc:
            return _json_error(exc)

    return blueprint
