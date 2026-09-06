"""Post-resolution geometric checks for magnetic-panel assemblies."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

import numpy as np

from .contracts import (
    MagneticAssemblySpec,
    MagneticJointType,
    MagneticPanelSpec,
    PanelEdge,
    PanelPoseClass,
    ResolvedMagneticPlacement,
)
from .planner import MagneticAssemblyPlanner, _support_top


def _normalized(value: np.ndarray) -> np.ndarray:
    vector = np.asarray(value, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(vector))
    if norm < 1.0e-9:
        raise ValueError("cannot normalize a zero-length vector")
    return vector / norm


def _edge_local(
    panel: MagneticPanelSpec,
    edge: PanelEdge,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    sx, sy, _sz = panel.size_xyz_m
    if edge is PanelEdge.POS_X:
        return (
            np.asarray([0.5 * sx, 0.0, 0.0]),
            np.asarray([1.0, 0.0, 0.0]),
            np.asarray([0.0, 1.0, 0.0]),
        )
    if edge is PanelEdge.NEG_X:
        return (
            np.asarray([-0.5 * sx, 0.0, 0.0]),
            np.asarray([-1.0, 0.0, 0.0]),
            np.asarray([0.0, 1.0, 0.0]),
        )
    if edge is PanelEdge.POS_Y:
        return (
            np.asarray([0.0, 0.5 * sy, 0.0]),
            np.asarray([0.0, 1.0, 0.0]),
            np.asarray([1.0, 0.0, 0.0]),
        )
    return (
        np.asarray([0.0, -0.5 * sy, 0.0]),
        np.asarray([0.0, -1.0, 0.0]),
        np.asarray([1.0, 0.0, 0.0]),
    )


def _edge_world(
    pose: np.ndarray,
    panel: MagneticPanelSpec,
    edge: PanelEdge,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    point, outward, tangent = _edge_local(panel, edge)
    rotation = pose[:3, :3]
    return (
        pose[:3, 3] + rotation @ point,
        _normalized(rotation @ outward),
        _normalized(rotation @ tangent),
        _normalized(rotation[:, 2]),
    )


@dataclass(frozen=True)
class MagneticGeometryViolation:
    code: str
    message: str
    connection_id: str | None = None
    piece_ids: tuple[str, ...] = ()
    details: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "piece_ids", tuple(self.piece_ids))
        object.__setattr__(self, "details", dict(self.details))


@dataclass(frozen=True)
class MagneticGeometryReport:
    valid: bool
    violations: tuple[MagneticGeometryViolation, ...]
    diagnostics: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "violations", tuple(self.violations))
        object.__setattr__(self, "diagnostics", dict(self.diagnostics))


class MagneticGeometryValidator:
    """Check that resolved poses satisfy the intended connection geometry."""

    def __init__(
        self,
        *,
        maximum_support_height_mismatch_m: float = 0.006,
        maximum_slot_width_error_m: float = 0.006,
        maximum_edge_line_error_m: float = 0.006,
        minimum_facing_cosine: float = 0.80,
        minimum_parallel_cosine: float = 0.94,
        maximum_right_angle_cosine: float = 0.20,
        bridge_span_margin_m: float = 0.015,
    ) -> None:
        self.maximum_support_height_mismatch_m = float(
            maximum_support_height_mismatch_m
        )
        self.maximum_slot_width_error_m = float(maximum_slot_width_error_m)
        self.maximum_edge_line_error_m = float(maximum_edge_line_error_m)
        self.minimum_facing_cosine = float(minimum_facing_cosine)
        self.minimum_parallel_cosine = float(minimum_parallel_cosine)
        self.maximum_right_angle_cosine = float(maximum_right_angle_cosine)
        self.bridge_span_margin_m = float(bridge_span_margin_m)

    def validate(
        self,
        spec: MagneticAssemblySpec,
        placements: Sequence[ResolvedMagneticPlacement],
        catalog: Mapping[str, MagneticPanelSpec],
    ) -> MagneticGeometryReport:
        placements = tuple(placements)
        pose_by_piece = {
            item.piece.piece_id: np.asarray(
                item.target_pose,
                dtype=np.float64,
            )
            for item in placements
        }
        piece_by_id = {item.piece_id: item for item in spec.pieces}
        violations: list[MagneticGeometryViolation] = []
        diagnostics: dict[str, Any] = {}

        def add(
            connection_id: str,
            code: str,
            message: str,
            piece_ids: Sequence[str],
            **details: Any,
        ) -> None:
            violations.append(
                MagneticGeometryViolation(
                    code,
                    message,
                    connection_id,
                    tuple(piece_ids),
                    details,
                )
            )

        for connection in spec.connections:
            child = piece_by_id[connection.child_id]
            child_pose = pose_by_piece[connection.child_id]
            child_panel = catalog[child.asset_name]
            parent_pieces = [piece_by_id[item] for item in connection.parent_ids]
            parent_poses = [pose_by_piece[item] for item in connection.parent_ids]
            parent_panels = [catalog[item.asset_name] for item in parent_pieces]
            connection_diagnostics: dict[str, Any] = {}

            if connection.joint_type is MagneticJointType.VERTICAL_SLOT:
                edges = [
                    _edge_world(pose, panel, edge)
                    for pose, panel, edge in zip(
                        parent_poses,
                        parent_panels,
                        connection.parent_edges,
                    )
                ]
                points = [
                    point + normal * (0.5 * panel.size_xyz_m[2])
                    for (point, _outward, _tangent, normal), panel in zip(
                        edges,
                        parent_panels,
                    )
                ]
                delta = points[1] - points[0]
                # Width is signed distance across the facing inner edges;
                # Euclidean center distance confounds slot width and shear.
                separation = float(np.dot(delta, edges[0][1]))
                shear = abs(float(np.dot(delta, edges[0][2])))
                expected = (
                    child_panel.size_xyz_m[2] + 2.0 * connection.gap_m
                )
                slot_error = abs(separation - expected)
                facing = float(np.dot(edges[0][1], edges[1][1]))
                tangent_parallel = abs(float(np.dot(edges[0][2], edges[1][2])))
                support_normal_parallel = float(np.dot(edges[0][3], edges[1][3]))
                normal_sum = edges[0][3] + edges[1][3]
                up = _normalized(normal_sum if np.linalg.norm(normal_sum) > 1e-9 else edges[0][3])
                height_mismatch = abs(float(np.dot(points[1] - points[0], up)))
                if shear > self.maximum_edge_line_error_m:
                    add(connection.connection_id, "slot_edges_sheared",
                        "support edges are shifted along the slot", connection.parent_ids,
                        shear_m=shear)
                expected_bottom = 0.5 * (points[0] + points[1]) + up * connection.gap_m
                actual_bottom = child_pose[:3, 3] - child_pose[:3, 1] * (0.5 * child_panel.size_xyz_m[1])
                placement_error = float(np.linalg.norm(actual_bottom - expected_bottom))
                if placement_error > self.maximum_edge_line_error_m:
                    add(connection.connection_id, "slot_child_not_centered",
                        "vertical child is not seated at the declared slot", (connection.child_id,),
                        bottom_error_m=placement_error)
                thickness_alignment = abs(float(np.dot(child_pose[:3, 2], edges[0][1])))
                if thickness_alignment < self.minimum_parallel_cosine:
                    add(connection.connection_id, "slot_child_thickness_axis_misaligned",
                        "child thickness axis is not across the slot", (connection.child_id,))
                child_up = _normalized(child_pose[:3, 1])
                child_vertical = float(np.dot(child_up, up))
                connection_diagnostics.update(
                    {
                        "support_edge_separation_m": separation,
                        "expected_slot_width_m": expected,
                        "slot_width_error_m": slot_error,
                        "support_outward_dot": facing,
                        "support_tangent_parallel_cosine": tangent_parallel,
                        "support_normal_dot": support_normal_parallel,
                        "support_height_mismatch_m": height_mismatch,
                        "child_vertical_cosine": child_vertical,
                    }
                )
                if facing > -self.minimum_facing_cosine:
                    add(
                        connection.connection_id,
                        "slot_edges_do_not_face",
                        "the two selected support edges do not face each other",
                        (connection.child_id, *connection.parent_ids),
                        outward_dot=facing,
                    )
                if tangent_parallel < self.minimum_parallel_cosine:
                    add(
                        connection.connection_id,
                        "slot_edges_not_parallel",
                        "the two support edges are not parallel enough for a slot",
                        (connection.child_id, *connection.parent_ids),
                        tangent_parallel_cosine=tangent_parallel,
                    )
                if support_normal_parallel < self.minimum_parallel_cosine:
                    add(
                        connection.connection_id,
                        "slot_support_planes_misaligned",
                        "the two lying supports are not coplanar",
                        (connection.child_id, *connection.parent_ids),
                        support_normal_dot=support_normal_parallel,
                    )
                if height_mismatch > self.maximum_support_height_mismatch_m:
                    add(
                        connection.connection_id,
                        "slot_support_height_mismatch",
                        "the two support edges are at different heights",
                        (connection.child_id, *connection.parent_ids),
                        height_mismatch_m=height_mismatch,
                    )
                if slot_error > self.maximum_slot_width_error_m:
                    add(
                        connection.connection_id,
                        "slot_width_mismatch",
                        "the two-base slot width does not match the vertical "
                        "panel thickness plus the requested gaps",
                        (connection.child_id, *connection.parent_ids),
                        separation_m=separation,
                        expected_m=expected,
                        error_m=slot_error,
                    )
                if child_vertical < self.minimum_parallel_cosine:
                    add(
                        connection.connection_id,
                        "slot_child_not_vertical_in_world",
                        "resolved vertical panel is not aligned with support up",
                        (connection.child_id,),
                        vertical_cosine=child_vertical,
                    )

            elif connection.joint_type is MagneticJointType.RIGHT_ANGLE_EDGE:
                parent_edge = _edge_world(
                    parent_poses[0],
                    parent_panels[0],
                    connection.parent_edges[0],
                )
                child_edge = _edge_world(
                    child_pose,
                    child_panel,
                    connection.child_edge,
                )
                edge_error = float(
                    np.linalg.norm(parent_edge[0] - child_edge[0])
                )
                tangent_parallel = abs(
                    float(np.dot(parent_edge[2], child_edge[2]))
                )
                normal_cosine = abs(
                    float(np.dot(parent_edge[3], child_edge[3]))
                )
                connection_diagnostics.update(
                    {
                        "edge_line_error_m": edge_error,
                        "edge_tangent_parallel_cosine": tangent_parallel,
                        "panel_normal_absolute_cosine": normal_cosine,
                    }
                )
                allowed_error = (
                    connection.gap_m + self.maximum_edge_line_error_m
                )
                if edge_error > allowed_error:
                    add(
                        connection.connection_id,
                        "right_angle_edge_not_mated",
                        "resolved 90-degree panels do not share the requested "
                        "mating edge",
                        (connection.child_id, *connection.parent_ids),
                        edge_error_m=edge_error,
                        allowed_error_m=allowed_error,
                    )
                if tangent_parallel < self.minimum_parallel_cosine:
                    add(
                        connection.connection_id,
                        "right_angle_edges_not_parallel",
                        "mating edge directions are inconsistent",
                        (connection.child_id, *connection.parent_ids),
                        tangent_parallel_cosine=tangent_parallel,
                    )
                if normal_cosine > self.maximum_right_angle_cosine:
                    add(
                        connection.connection_id,
                        "right_angle_geometry_not_orthogonal",
                        "resolved panel planes are not approximately 90 degrees",
                        (connection.child_id, *connection.parent_ids),
                        absolute_normal_cosine=normal_cosine,
                    )

            elif connection.joint_type is MagneticJointType.FLAT_STACK:
                parent_normal = _normalized(parent_poses[0][:3, 2])
                child_normal = _normalized(child_pose[:3, 2])
                normal_parallel = abs(float(np.dot(parent_normal, child_normal)))
                connection_diagnostics["normal_parallel_cosine"] = normal_parallel
                if normal_parallel < self.minimum_parallel_cosine:
                    add(
                        connection.connection_id,
                        "flat_stack_planes_misaligned",
                        "flat-stack panels are not parallel",
                        (connection.child_id, *connection.parent_ids),
                        normal_parallel_cosine=normal_parallel,
                    )

            elif connection.joint_type is MagneticJointType.BRIDGE:
                top_points = []
                for pose, panel, piece in zip(
                    parent_poses,
                    parent_panels,
                    parent_pieces,
                ):
                    top_points.append(_support_top(pose, panel))
                    if float(np.max(np.abs(pose[2, :3]))) < self.minimum_parallel_cosine:
                        add(connection.connection_id, "bridge_support_face_not_level",
                            "bridge support has no approximately horizontal face",
                            (piece.piece_id,))
                span = max(
                    float(np.linalg.norm(left - right))
                    for left in top_points
                    for right in top_points
                )
                heights = [float(item[2]) for item in top_points]
                height_range = max(heights) - min(heights)
                # Use the resolved footprint, not max(width, length): a long
                # side perpendicular to the supports cannot bridge the gap.
                available_span = child_panel.size_xyz_m[0]
                local_supports = [child_pose[:3, :3].T @ (point - child_pose[:3, 3]) for point in top_points]
                for local_point, parent in zip(local_supports, parent_pieces):
                    outside = bool(np.any(np.abs(local_point[:2]) > 0.5 * np.asarray(child_panel.size_xyz_m[:2]) + self.maximum_edge_line_error_m))
                    height_error = abs(float(local_point[2] + 0.5 * child_panel.size_xyz_m[2] + connection.gap_m))
                    if outside or height_error > self.maximum_support_height_mismatch_m:
                        add(connection.connection_id, "bridge_support_outside_footprint",
                            "bridge does not cover the resolved support at its bottom surface",
                            (connection.child_id, parent.piece_id), local_support=local_point.tolist())
                connection_diagnostics.update(
                    {
                        "support_span_m": span,
                        "child_available_span_m": available_span,
                        "support_height_range_m": height_range,
                    }
                )
                if span > available_span + self.bridge_span_margin_m:
                    add(
                        connection.connection_id,
                        "bridge_too_short_for_support_span",
                        "bridge panel is too short for the selected supports",
                        (connection.child_id, *connection.parent_ids),
                        support_span_m=span,
                        available_span_m=available_span,
                    )
                if height_range > self.maximum_support_height_mismatch_m:
                    add(
                        connection.connection_id,
                        "bridge_support_height_mismatch",
                        "bridge supports are not level",
                        (connection.child_id, *connection.parent_ids),
                        support_height_range_m=height_range,
                    )

            diagnostics[connection.connection_id] = connection_diagnostics

        return MagneticGeometryReport(
            not violations,
            tuple(violations),
            diagnostics,
        )


class StrictMagneticAssemblyPlanner(MagneticAssemblyPlanner):
    """Magnetic planner that rejects geometrically inconsistent resolutions."""

    def __init__(
        self,
        catalog: Mapping[str, MagneticPanelSpec],
        *,
        geometry_validator: MagneticGeometryValidator | None = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(catalog, **kwargs)
        self.geometry_validator = geometry_validator or MagneticGeometryValidator()
        self.last_geometry_report: MagneticGeometryReport | None = None

    def resolve(
        self,
        spec: MagneticAssemblySpec,
        inventory: Sequence[Any],
    ) -> tuple[ResolvedMagneticPlacement, ...]:
        placements = super().resolve(spec, inventory)
        report = self.geometry_validator.validate(
            spec,
            placements,
            self.catalog,
        )
        self.last_geometry_report = report
        if not report.valid:
            raise ValueError(
                "invalid resolved magnetic geometry: "
                + "; ".join(
                    f"{item.code}: {item.message}"
                    for item in report.violations
                )
            )
        return placements
