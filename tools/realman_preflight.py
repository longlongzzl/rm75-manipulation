#!/usr/bin/env python3
"""Connect to RM75 and inspect hardware readiness without issuing motion.

This tool deliberately never calls ``arm_execution()``, ``execute_trajectory`` or
``set_gripper``.  It is intended to be the first command run on the robot PC after
bringing the restored hardware adapter back into the standalone repository.
"""

from __future__ import annotations

import argparse
import json

from rm75_app.execution.realman_executor import (
    RealManConnectionConfig,
    RealManExecutionConfig,
    RealManSDKSession,
    RealManTrajectoryExecutor,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", default="192.168.101.20")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument(
        "--gripper-backend",
        choices=("modbus", "hand_follow"),
        default="modbus",
    )
    parser.add_argument(
        "--no-configure-gripper",
        action="store_true",
        help="Skip gripper mode/speed configuration during connection.",
    )
    parser.add_argument(
        "--require-ready",
        action="store_true",
        help="Exit nonzero unless all physical-execution preflight checks pass.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    connection = RealManConnectionConfig(
        ip=args.ip,
        port=args.port,
        gripper_backend=args.gripper_backend,
        configure_gripper=not args.no_configure_gripper,
    )
    session = RealManSDKSession(connection)
    try:
        session.connect()
        executor = RealManTrajectoryExecutor(session, RealManExecutionConfig())
        report = executor.preflight()
        payload = {
            "ready": report.ready,
            "checks": dict(report.checks),
            "diagnostics": dict(report.diagnostics),
            "motion_submitted": False,
            "execution_armed": executor.armed,
        }
        print(json.dumps(payload, indent=2, ensure_ascii=False))
        return 0 if report.ready or not args.require_ready else 2
    finally:
        session.close()


if __name__ == "__main__":
    raise SystemExit(main())
