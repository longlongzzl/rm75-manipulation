"""Shared coordinate-frame conventions for the RM75 tabletop workspace."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import numpy as np


DEFAULT_ROBOT_BASE_WORLD_XYZ_M = (-0.615, 0.0, 0.0)

# Exact kinematic table collision built by ManiSkill's TableSceneBuilder.
# Its visual mesh is rotated, but the collision box is axis-aligned in world.
MANISKILL_TABLE_COLLISION_NAME = "__maniskill_workspace_table__"
MANISKILL_TABLE_WORLD_CENTER_M = (-0.12, 0.0, -0.9196429 / 2.0)
MANISKILL_TABLE_DIMENSIONS_M = (1.209, 2.418, 0.9196429)


def robot_base_world_pose(
    scene_payload: Mapping[str, Any] | None = None,
) -> tuple[np.ndarray, str]:
    """Resolve ``T_world_robot_base`` from a scene, with the RM75 fallback."""

    payload = dict(scene_payload or {})
    for key in ("T_world_robot_base", "robot_base_world_transform"):
        value = payload.get(key)
        if value is None:
            continue
        try:
            transform = np.asarray(value, dtype=np.float64).reshape(4, 4)
            if np.all(np.isfinite(transform)):
                return transform.copy(), key
        except (TypeError, ValueError):
            pass

    for key in (
        "robot_base_world_xyz_m",
        "robot_base_world_p",
        "robot_base_world_position",
        "robot_base_position",
    ):
        value = payload.get(key)
        if value is None:
            continue
        try:
            xyz = np.asarray(value, dtype=np.float64).reshape(-1)
            if xyz.size >= 3 and np.all(np.isfinite(xyz[:3])):
                transform = np.eye(4, dtype=np.float64)
                transform[:3, 3] = xyz[:3]
                return transform, key
        except (TypeError, ValueError):
            pass

    pose = payload.get("robot_base_world_pose")
    if isinstance(pose, Mapping):
        for key in ("position", "p", "xyz"):
            try:
                xyz = np.asarray(pose.get(key), dtype=np.float64).reshape(-1)
                if xyz.size >= 3 and np.all(np.isfinite(xyz[:3])):
                    transform = np.eye(4, dtype=np.float64)
                    transform[:3, 3] = xyz[:3]
                    return transform, f"robot_base_world_pose.{key}"
            except (TypeError, ValueError):
                pass

    transform = np.eye(4, dtype=np.float64)
    transform[:3, 3] = DEFAULT_ROBOT_BASE_WORLD_XYZ_M
    return transform, "rm75_default"


def explicit_robot_base_transform(*, world_xyz=None, world_transform=None) -> np.ndarray:
    """Resolve one explicit rigid base calibration, without silent fallbacks.

    Unlike imported scene hints, a trusted execution calibration must never
    drop rotation, accept scale/reflection, or combine two representations.
    """
    if world_xyz is not None and world_transform is not None:
        raise ValueError("Specify only one robot-base calibration representation")
    matrix = np.eye(4, dtype=np.float64)
    if world_transform is not None:
        matrix = np.asarray(world_transform, dtype=np.float64).copy()
    elif world_xyz is not None:
        xyz = np.asarray(world_xyz, dtype=np.float64)
        if xyz.shape != (3,):
            raise ValueError("Robot-base translation must contain three values")
        matrix[:3, 3] = xyz
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0., 0., 0., 1.], atol=1e-6, rtol=0)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1., atol=1e-6, rtol=0)):
        raise ValueError("Robot-base calibration must be a finite proper SE(3) transform")
    return matrix
