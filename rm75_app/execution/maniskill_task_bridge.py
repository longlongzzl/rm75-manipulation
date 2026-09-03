"""Thin ManiSkill hooks for the shared multi-object task executor."""

from __future__ import annotations

from collections.abc import Callable, Mapping
from functools import lru_cache
from typing import Any

import numpy as np

from rm75_app.orchestration.multi_object_executor import (
    AtomExecution,
    AtomValidation,
    TaskSceneState,
    validate_target_pose,
)
from rm75_app.tasks.manipulation_plan import ManipulationAtom


@lru_cache(maxsize=64)
def _scaled_asset_geometry(asset_name: str) -> tuple[np.ndarray, np.ndarray]:
    """Return simulation-mesh bounds and vertices in the local metric frame."""

    import trimesh

    from rm75_app.assets.object_specs import get_object_spec, resolve_object_spec_scales

    spec = get_object_spec(asset_name)
    if spec is None:
        raise KeyError(f"no object spec for asset {asset_name!r}")
    asset_file = spec.sim_asset_file or spec.mesh_file
    _, scale = resolve_object_spec_scales(spec)
    mesh = trimesh.load(asset_file, force="scene", process=False)
    geometry = mesh.to_geometry() if hasattr(mesh, "to_geometry") else mesh
    vertices = np.asarray(geometry.vertices, dtype=np.float64) * float(scale)
    bounds = np.asarray(mesh.bounds, dtype=np.float64) * float(scale)
    if bounds.shape != (2, 3) or not np.all(np.isfinite(bounds)):
        raise ValueError(f"asset {asset_name!r} has invalid simulation bounds")
    if vertices.ndim != 2 or vertices.shape[1] != 3 or not np.all(np.isfinite(vertices)):
        raise ValueError(f"asset {asset_name!r} has invalid simulation vertices")
    return bounds, vertices


def _scaled_asset_bounds(asset_name: str) -> np.ndarray:
    return _scaled_asset_geometry(asset_name)[0]


def validate_inside_relation(
    atom: ManipulationAtom,
    observed_object_pose: np.ndarray,
    support_pose: np.ndarray,
    support_asset: str,
) -> AtomValidation:
    """Validate containment instead of matching the nominal release pose."""

    _, object_vertices = _scaled_asset_geometry(atom.object_asset)
    support_bounds = _scaled_asset_bounds(support_asset)
    support_from_object = np.linalg.inv(support_pose) @ observed_object_pose

    vertices_support = (
        support_from_object[:3, :3] @ object_vertices.T
    ).T + support_from_object[:3, 3]
    object_min = vertices_support.min(axis=0)
    object_max = vertices_support.max(axis=0)

    # Asset-local Z is not necessarily vertical (the holder uses local Y).
    vertical_axis = int(
        np.argmax(
            np.abs(support_pose[:3, :3].T @ np.asarray([0.0, 0.0, 1.0]))
        )
    )
    horizontal_axes = tuple(axis for axis in range(3) if axis != vertical_axis)
    horizontal_excess = max(
        0.0,
        *(float(support_bounds[0, axis] - object_min[axis]) for axis in horizontal_axes),
        *(float(object_max[axis] - support_bounds[1, axis]) for axis in horizontal_axes),
    )
    vertical_overlap = max(
        0.0,
        float(
            min(object_max[vertical_axis], support_bounds[1, vertical_axis])
            - max(object_min[vertical_axis], support_bounds[0, vertical_axis])
        ),
    )

    # Keep containment strict even when the target-pose tolerance is broad.
    geometry_slack_m = 0.003
    lower_excess = max(
        0.0, float(support_bounds[0, vertical_axis] - object_min[vertical_axis])
    )
    inside = (
        horizontal_excess <= geometry_slack_m
        and lower_excess <= geometry_slack_m
        and vertical_overlap >= 0.002
    )
    target_distance = float(
        np.linalg.norm(
            observed_object_pose[:3, 3] - np.asarray(atom.target_pose)[:3, 3]
        )
    )
    violation = max(
        0.0,
        horizontal_excess - geometry_slack_m,
        lower_excess - geometry_slack_m,
        0.002 - vertical_overlap,
    )
    return AtomValidation(
        inside,
        atom.success.relation,
        position_error_m=violation,
        orientation_error_deg=None,
        message=(
            "object is inside support container"
            if inside
            else "object is outside support container bounds"
        ),
        diagnostics={
            "validation_mode": "geometric_containment",
            "support_object_id": atom.support_object_id,
            "support_asset": support_asset,
            "vertical_local_axis": vertical_axis,
            "object_center_in_support": support_from_object[:3, 3].tolist(),
            "horizontal_excess_m": horizontal_excess,
            "lower_excess_m": lower_excess,
            "vertical_overlap_m": vertical_overlap,
            "geometry_slack_m": geometry_slack_m,
            "release_pose_distance_m": target_distance,
        },
        observed_object_pose=observed_object_pose,
    )


def _pose_matrix(actor: Any) -> np.ndarray:
    pose = getattr(actor, "pose", None)
    matrix = getattr(pose, "to_transformation_matrix", None)
    if not callable(matrix):
        raise TypeError("ManiSkill actor pose has no transformation matrix")
    value = matrix()
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    value = np.asarray(value, dtype=np.float64)
    if value.ndim == 3:
        value = value[0]
    return value.reshape(4, 4)


class ManiSkillTaskBridge:
    """Observe actors and settle physics without coupling the scheduler to SAPIEN."""

    def __init__(
        self,
        env: Any,
        actors: Mapping[str, Any] | Callable[[str], Any],
        *,
        settle_steps: int | None = None,
        hold_action: Callable[[], np.ndarray] | None = None,
    ):
        self.env = env
        self.actors = actors
        self.settle_steps = settle_steps
        self.hold_action = hold_action

    def actor(self, object_id: str) -> Any:
        actor = self.actors(object_id) if callable(self.actors) else self.actors.get(object_id)
        if actor is None:
            raise KeyError(f"ManiSkill scene has no actor for {object_id!r}")
        return actor

    def observe_object_pose(self, object_id: str) -> np.ndarray:
        return _pose_matrix(self.actor(object_id))

    def validate_atom(
        self,
        atom: ManipulationAtom,
        execution: AtomExecution,
        scene: TaskSceneState,
    ) -> AtomValidation:
        steps = atom.success.settle_steps if self.settle_steps is None else int(self.settle_steps)
        if steps > 0:
            self._settle(steps)
        observed = AtomExecution(
            True,
            final_object_pose=self.observe_object_pose(atom.object_id),
            joint_names=execution.joint_names,
            joint_positions=execution.joint_positions,
            artifacts=execution.artifacts,
        )
        if atom.success.relation.strip().lower() == "inside":
            if not atom.support_object_id:
                return AtomValidation(
                    False,
                    atom.success.relation,
                    message="inside relation requires support_object_id",
                    observed_object_pose=observed.final_object_pose,
                )
            support_state = scene.objects.get(atom.support_object_id)
            if support_state is None:
                return AtomValidation(
                    False,
                    atom.success.relation,
                    message=f"support object {atom.support_object_id!r} is absent from task scene",
                    observed_object_pose=observed.final_object_pose,
                )
            try:
                return validate_inside_relation(
                    atom,
                    np.asarray(observed.final_object_pose, dtype=np.float64),
                    self.observe_object_pose(atom.support_object_id),
                    support_state.asset_name,
                )
            except (KeyError, OSError, TypeError, ValueError) as exc:
                return AtomValidation(
                    False,
                    atom.success.relation,
                    message=f"inside geometry validation unavailable: {exc}",
                    diagnostics={"validation_mode": "geometric_containment_error"},
                    observed_object_pose=observed.final_object_pose,
                )
        return validate_target_pose(atom, observed)

    def synchronize_scene(self, scene: TaskSceneState) -> None:
        # ManiSkill is authoritative after physics execution. This hook checks
        # that every manipulated actor remains addressable; planner-world sync
        # is supplied separately by the task builder/backend.
        for object_id in scene.objects:
            self.actor(object_id)

    def _settle(self, steps: int) -> None:
        if self.hold_action is None:
            raise RuntimeError(
                "ManiSkillTaskBridge needs a controller-specific hold_action callback for settling"
            )
        for _ in range(int(steps)):
            self.env.step(np.asarray(self.hold_action(), dtype=np.float32))
