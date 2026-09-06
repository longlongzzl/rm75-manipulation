"""Deterministic geometry and task compilation for magnetic assemblies."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np

from rm75_app.orchestration.multi_object_executor import TaskSceneState
from rm75_app.tasks.manipulation_plan import (
    ManipulationAtom,
    ManipulationPlan,
    ManipulationPrimitive,
    PlacementMode,
    SuccessCriteria,
)

from ..pickplace_program import (
    PickPlaceCompilationResult,
    PickPlaceExecutionReport,
    PickPlaceProgramCompiler,
    PickPlaceProgramExecutor,
)
from .contracts import (
    MagneticAssemblySpec,
    MagneticConnection,
    MagneticInventoryItem,
    MagneticJointType,
    MagneticPanelSpec,
    PanelEdge,
    PanelPoseClass,
    ResolvedMagneticPlacement,
)
from .rules import MagneticAssemblyRules, MagneticValidationReport


def _normalized(value: np.ndarray) -> np.ndarray:
    vector = np.asarray(value, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(vector))
    if norm < 1.0e-9:
        raise ValueError("cannot normalize a zero-length vector")
    return vector / norm


def _translation(value: Sequence[float]) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, 3] = np.asarray(value, dtype=np.float64).reshape(3)
    return transform


def _rotation_z(angle_deg: float) -> np.ndarray:
    angle = np.deg2rad(float(angle_deg))
    c, s = float(np.cos(angle)), float(np.sin(angle))
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(
        [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    return transform


def _edge_local_geometry(
    panel: MagneticPanelSpec,
    edge: PanelEdge,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    sx, sy, _sz = panel.size_xyz_m
    if edge is PanelEdge.POS_X:
        point = np.asarray([0.5 * sx, 0.0, 0.0])
        outward = np.asarray([1.0, 0.0, 0.0])
        tangent = np.asarray([0.0, 1.0, 0.0])
    elif edge is PanelEdge.NEG_X:
        point = np.asarray([-0.5 * sx, 0.0, 0.0])
        outward = np.asarray([-1.0, 0.0, 0.0])
        tangent = np.asarray([0.0, 1.0, 0.0])
    elif edge is PanelEdge.POS_Y:
        point = np.asarray([0.0, 0.5 * sy, 0.0])
        outward = np.asarray([0.0, 1.0, 0.0])
        tangent = np.asarray([1.0, 0.0, 0.0])
    else:
        point = np.asarray([0.0, -0.5 * sy, 0.0])
        outward = np.asarray([0.0, -1.0, 0.0])
        tangent = np.asarray([1.0, 0.0, 0.0])
    return point, outward, tangent


def _edge_world_geometry(
    pose: np.ndarray,
    panel: MagneticPanelSpec,
    edge: PanelEdge,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    point, outward, tangent = _edge_local_geometry(panel, edge)
    rotation = pose[:3, :3]
    return (
        pose[:3, 3] + rotation @ point,
        _normalized(rotation @ outward),
        _normalized(rotation @ tangent),
        _normalized(rotation[:, 2]),
    )


def _pose_from_axes(
    x_axis: np.ndarray,
    y_axis: np.ndarray,
    z_axis: np.ndarray,
    center: np.ndarray,
) -> np.ndarray:
    requested_z = _normalized(z_axis)
    x_axis = _normalized(x_axis)
    y_axis = _normalized(y_axis - x_axis * float(np.dot(x_axis, y_axis)))
    resolved_z = _normalized(np.cross(x_axis, y_axis))
    if float(np.dot(resolved_z, requested_z)) < 0.0:
        y_axis = -y_axis
        resolved_z = -resolved_z
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.stack((x_axis, y_axis, resolved_z), axis=1)
    transform[:3, 3] = np.asarray(center, dtype=np.float64).reshape(3)
    return transform


def _support_top(pose: np.ndarray, panel: MagneticPanelSpec) -> np.ndarray:
    """Center of the highest box face, not an assumed local +Y endpoint.

    Intended for the axis-aligned flat/upright panel domain. The strict
    validator separately rejects support faces that are not approximately level.
    """
    local_axis = int(np.argmax(np.abs(pose[2, :3])))
    sign = 1.0 if pose[2, local_axis] >= 0.0 else -1.0
    return pose[:3, 3] + sign * pose[:3, local_axis] * (0.5 * panel.size_xyz_m[local_axis])


class MagneticAssemblyPlanner:
    """Resolve a validated symbolic structure into world-space placements."""

    def __init__(
        self,
        catalog: Mapping[str, MagneticPanelSpec],
        *,
        rules: MagneticAssemblyRules | None = None,
    ) -> None:
        self.catalog = {str(key): value for key, value in catalog.items()}
        self.rules = rules or MagneticAssemblyRules()

    def validate(
        self,
        spec: MagneticAssemblySpec,
        inventory: Sequence[MagneticInventoryItem],
    ) -> MagneticValidationReport:
        return self.rules.validate(spec, self.catalog, inventory)

    def resolve(
        self,
        spec: MagneticAssemblySpec,
        inventory: Sequence[MagneticInventoryItem],
    ) -> tuple[ResolvedMagneticPlacement, ...]:
        report = self.validate(spec, inventory)
        if not report.valid:
            messages = "; ".join(
                f"{item.code}: {item.message}" for item in report.violations
            )
            raise ValueError(f"invalid magnetic assembly: {messages}")
        piece_by_id = {item.piece_id: item for item in spec.pieces}
        poses: dict[str, np.ndarray] = {}
        output: list[ResolvedMagneticPlacement] = []
        for piece_id in report.topological_piece_order:
            piece = piece_by_id[piece_id]
            connection = spec.connection_for_child(piece_id)
            if connection is None:
                pose = (
                    np.asarray(spec.anchor_pose, dtype=np.float64)
                    @ _translation(piece.anchor_offset_m or (0.0, 0.0, 0.0))
                    @ _rotation_z(piece.anchor_yaw_deg)
                )
                placement = ResolvedMagneticPlacement(
                    piece,
                    pose,
                    (),
                    None,
                    self.catalog[piece.asset_name].engagement_clearance_m,
                    (),
                    diagnostics={"placement_rule": "anchor"},
                )
            else:
                placement = self._resolve_connection(
                    piece,
                    connection,
                    poses,
                    piece_by_id,
                )
            poses[piece_id] = placement.target_pose
            output.append(placement)
        return tuple(output)

    def _resolve_connection(
        self,
        piece: object,
        connection: MagneticConnection,
        poses: Mapping[str, np.ndarray],
        piece_by_id: Mapping[str, object],
    ) -> ResolvedMagneticPlacement:
        child_spec = self.catalog[piece.asset_name]
        parent_specs = [
            self.catalog[piece_by_id[item].asset_name]
            for item in connection.parent_ids
        ]
        parent_poses = [poses[item] for item in connection.parent_ids]
        if connection.joint_type is MagneticJointType.FLAT_STACK:
            parent_pose = parent_poses[0]
            parent_spec = parent_specs[0]
            pose = parent_pose.copy()
            pose[:3, 3] += parent_pose[:3, 2] * (
                0.5 * parent_spec.size_xyz_m[2]
                + 0.5 * child_spec.size_xyz_m[2]
                + connection.gap_m
            )
            diagnostics = {"placement_rule": "flat_stack"}
        elif connection.joint_type is MagneticJointType.VERTICAL_SLOT:
            pose, diagnostics = self._vertical_slot_pose(
                connection,
                child_spec,
                parent_poses,
                parent_specs,
            )
        elif connection.joint_type is MagneticJointType.RIGHT_ANGLE_EDGE:
            pose, diagnostics = self._right_angle_pose(
                connection,
                child_spec,
                parent_poses[0],
                parent_specs[0],
            )
        elif connection.joint_type is MagneticJointType.BRIDGE:
            pose, diagnostics = self._bridge_pose(
                connection,
                child_spec,
                parent_poses,
                parent_specs,
                [
                    piece_by_id[item].pose_class
                    for item in connection.parent_ids
                ],
            )
        else:
            raise ValueError(
                f"unsupported placement connection {connection.joint_type.value!r}"
            )
        return ResolvedMagneticPlacement(
            piece,
            pose,
            connection.parent_ids,
            connection.connection_id,
            connection.clearance_m,
            connection.parent_ids,
            diagnostics={
                **diagnostics,
                "joint_type": connection.joint_type.value,
                "overlap_fraction": connection.overlap_fraction,
                "gap_m": connection.gap_m,
                "clearance_m": connection.clearance_m,
            },
        )

    def _vertical_slot_pose(
        self,
        connection: MagneticConnection,
        child_spec: MagneticPanelSpec,
        parent_poses: list[np.ndarray],
        parent_specs: list[MagneticPanelSpec],
    ) -> tuple[np.ndarray, dict[str, object]]:
        geometries = [
            _edge_world_geometry(pose, panel, edge)
            for pose, panel, edge in zip(
                parent_poses,
                parent_specs,
                connection.parent_edges,
            )
        ]
        top_points = [
            point + normal * (0.5 * panel.size_xyz_m[2])
            for (point, _outward, _tangent, normal), panel in zip(
                geometries,
                parent_specs,
            )
        ]
        up = _normalized(sum(item[3] for item in geometries))
        tangent = geometries[0][2]
        if float(np.dot(tangent, geometries[1][2])) < 0.0:
            tangent = -tangent
        tangent = _normalized(tangent)
        slot_axis = _normalized(np.cross(tangent, up))
        if connection.flip:
            slot_axis = -slot_axis
        center_line = 0.5 * (top_points[0] + top_points[1])
        child_center = center_line + up * (
            0.5 * child_spec.size_xyz_m[1] + connection.gap_m
        )
        pose = _pose_from_axes(tangent, up, slot_axis, child_center)
        edge_separation = float(np.linalg.norm(top_points[0] - top_points[1]))
        return pose, {
            "placement_rule": "vertical_slot_two_supports",
            "support_edge_separation_m": edge_separation,
            "child_thickness_m": child_spec.size_xyz_m[2],
            "slot_center": center_line.tolist(),
        }

    def _right_angle_pose(
        self,
        connection: MagneticConnection,
        child_spec: MagneticPanelSpec,
        parent_pose: np.ndarray,
        parent_spec: MagneticPanelSpec,
    ) -> tuple[np.ndarray, dict[str, object]]:
        parent_edge = connection.parent_edges[0]
        point, outward, tangent, normal = _edge_world_geometry(
            parent_pose,
            parent_spec,
            parent_edge,
        )
        child_z = outward if not connection.flip else -outward
        child_local_point, _unused, child_tangent = _edge_local_geometry(
            child_spec,
            connection.child_edge,
        )
        # Map the SELECTED child's edge tangent, not always its local Y.
        # The old construction only worked for +/-X mating edges.
        local_z = np.asarray([0.0, 0.0, 1.0])
        local_basis = np.stack((child_tangent, np.cross(local_z, child_tangent), local_z), axis=1)
        world_basis = np.stack((tangent, np.cross(child_z, tangent), child_z), axis=1)
        rotation = world_basis @ local_basis.T
        half_overlap_offset = (
            0.5 - float(connection.overlap_fraction)
        ) * child_spec.size_xyz_m[2]
        desired_edge = (
            point
            + normal * half_overlap_offset
            + child_z * connection.gap_m
        )
        center = desired_edge - rotation @ child_local_point
        pose = np.eye(4, dtype=np.float64)
        pose[:3, :3] = rotation
        pose[:3, 3] = center
        return pose, {
            "placement_rule": "right_angle_half_edge_overlap",
            "parent_edge_point": point.tolist(),
            "half_overlap_offset_m": half_overlap_offset,
        }

    def _bridge_pose(
        self,
        connection: MagneticConnection,
        child_spec: MagneticPanelSpec,
        parent_poses: list[np.ndarray],
        parent_specs: list[MagneticPanelSpec],
        parent_pose_classes: list[PanelPoseClass],
    ) -> tuple[np.ndarray, dict[str, object]]:
        top_points: list[np.ndarray] = []
        for pose, panel, pose_class in zip(
            parent_poses,
            parent_specs,
            parent_pose_classes,
        ):
            top_points.append(_support_top(pose, panel))
        center_support = np.mean(np.stack(top_points), axis=0)
        direction = _normalized(top_points[-1] - top_points[0])
        up = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
        if abs(float(np.dot(direction, up))) > 0.95:
            direction = np.asarray([1.0, 0.0, 0.0])
        x_axis = _normalized(direction - up * float(np.dot(direction, up)))
        y_axis = _normalized(np.cross(up, x_axis))
        center = center_support + up * (
            0.5 * child_spec.size_xyz_m[2] + connection.gap_m
        )
        pose = np.eye(4, dtype=np.float64)
        pose[:3, :3] = np.stack((x_axis, y_axis, up), axis=1)
        pose[:3, 3] = center
        return pose, {
            "placement_rule": "multi_support_bridge",
            "support_top_points": [item.tolist() for item in top_points],
        }

    def compile_manipulation_plan(
        self,
        spec: MagneticAssemblySpec,
        inventory: Sequence[MagneticInventoryItem],
        *,
        scene_file: str,
        plan_id: str | None = None,
        position_tolerance_m: float = 0.006,
        orientation_tolerance_deg: float = 8.0,
    ) -> ManipulationPlan:
        placements = self.resolve(spec, inventory)
        atom_by_piece: dict[str, str] = {}
        atoms: list[ManipulationAtom] = []
        for index, placement in enumerate(placements, start=1):
            atom_id = f"magnetic_{index:02d}_{placement.piece.piece_id}"
            dependency_atoms = tuple(
                atom_by_piece[item]
                for item in placement.depends_on_piece_ids
            )
            pose_class = placement.piece.pose_class
            connection = spec.connection_for_child(placement.piece.piece_id)
            atom = ManipulationAtom(
                atom_id=atom_id,
                primitive=ManipulationPrimitive.SNAP_PLACE,
                object_id=placement.piece.object_id,
                object_asset=placement.piece.asset_name,
                target_pose=placement.target_pose,
                support_object_id=(
                    None
                    if not placement.support_ids
                    else spec.piece(placement.support_ids[0]).object_id
                ),
                placement_mode=(
                    PlacementMode.VERTICAL
                    if pose_class is PanelPoseClass.VERTICAL
                    else PlacementMode.SURFACE
                ),
                semantic_operator=(
                    "magnetic_anchor"
                    if connection is None
                    else connection.joint_type.value
                ),
                success=SuccessCriteria(
                    "magnetic_snap",
                    position_tolerance_m,
                    orientation_tolerance_deg,
                    35,
                ),
                depends_on=dependency_atoms,
                metadata={
                    "scenario": "magnetic_assembly",
                    "piece_id": placement.piece.piece_id,
                    "panel_role": placement.piece.role.value,
                    "panel_pose_class": pose_class.value,
                    "magnetic_connection_id": placement.connection_id,
                    "magnetic_support_piece_ids": list(
                        placement.support_ids
                    ),
                    "magnetic_support_object_ids": [
                        spec.piece(item).object_id
                        for item in placement.support_ids
                    ],
                    "place_clearance_m": placement.clearance_m,
                    "magnetic_engagement_clearance_m": placement.clearance_m,
                    **dict(placement.diagnostics),
                },
            )
            atoms.append(atom)
            atom_by_piece[placement.piece.piece_id] = atom_id
        return ManipulationPlan(
            plan_id=plan_id or f"magnetic_{spec.structure_name}",
            scene_file=scene_file,
            atoms=tuple(atoms),
            user_command=spec.description,
            metadata={
                "scenario": "magnetic_assembly",
                "structure_name": spec.structure_name,
                "piece_count": len(spec.pieces),
                "max_piece_count": spec.max_pieces,
                "rules": {
                    "right_angle_overlap_fraction": (
                        self.rules.target_overlap_fraction
                    ),
                    "vertical_requires_two_flat_supports": True,
                },
                **dict(spec.metadata),
            },
        )


@dataclass(frozen=True)
class PreparedMagneticProgram:
    structure: MagneticAssemblySpec
    validation: MagneticValidationReport
    manipulation_plan: ManipulationPlan | None
    compilation: PickPlaceCompilationResult | None


class MagneticAssemblySystem:
    """Facade joining symbolic rules, cuRobo planning, and continuous replay."""

    def __init__(
        self,
        assembly_planner: MagneticAssemblyPlanner,
        motion_compiler: PickPlaceProgramCompiler,
        program_executor: PickPlaceProgramExecutor | None = None,
    ) -> None:
        self.assembly_planner = assembly_planner
        self.motion_compiler = motion_compiler
        self.program_executor = program_executor

    def prepare(
        self,
        structure: MagneticAssemblySpec,
        inventory: Sequence[MagneticInventoryItem],
        scene: TaskSceneState,
        *,
        scene_file: str,
    ) -> PreparedMagneticProgram:
        validation = self.assembly_planner.validate(
            structure,
            inventory,
        )
        if not validation.valid:
            return PreparedMagneticProgram(
                structure,
                validation,
                None,
                None,
            )
        plan = self.assembly_planner.compile_manipulation_plan(
            structure,
            inventory,
            scene_file=scene_file,
        )
        return PreparedMagneticProgram(
            structure,
            validation,
            plan,
            self.motion_compiler.compile(plan, scene),
        )

    def execute(
        self,
        prepared: PreparedMagneticProgram,
    ) -> PickPlaceExecutionReport:
        if not prepared.validation.valid:
            return PickPlaceExecutionReport(
                False,
                (),
                failed_stage="magnetic_rule_validation",
                message="; ".join(
                    f"{item.code}: {item.message}"
                    for item in prepared.validation.violations
                ),
            )
        if prepared.compilation is None or not prepared.compilation.success:
            compilation = prepared.compilation
            return PickPlaceExecutionReport(
                False,
                (),
                failed_atom_id=(
                    None if compilation is None else compilation.failed_atom_id
                ),
                failed_stage=(
                    "motion_compilation"
                    if compilation is None
                    else compilation.failure_stage
                ),
                message=(
                    "motion compilation unavailable"
                    if compilation is None
                    else compilation.message
                ),
                diagnostics=(
                    {}
                    if compilation is None
                    else compilation.diagnostics
                ),
            )
        if self.program_executor is None:
            raise RuntimeError(
                "magnetic assembly system has no physical/simulation executor"
            )
        assert prepared.compilation.program is not None
        return self.program_executor.execute(prepared.compilation.program)
