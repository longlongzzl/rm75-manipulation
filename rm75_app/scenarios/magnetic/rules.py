"""Physical and symbolic validity checks for magnetic-panel structures."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

from .contracts import (
    MagneticAssemblySpec,
    MagneticConnection,
    MagneticInventoryItem,
    MagneticJointType,
    MagneticPanelSpec,
    PanelPoseClass,
)


@dataclass(frozen=True)
class RuleViolation:
    code: str
    message: str
    piece_ids: tuple[str, ...] = ()
    connection_id: str | None = None
    details: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class MagneticValidationReport:
    valid: bool
    violations: tuple[RuleViolation, ...]
    topological_piece_order: tuple[str, ...] = ()
    stable_piece_ids: tuple[str, ...] = ()


class MagneticAssemblyRules:
    """Validate assembly semantics before pose or motion planning occurs."""

    def __init__(
        self,
        *,
        target_overlap_fraction: float = 0.5,
        overlap_tolerance: float = 0.08,
        minimum_gap_m: float = 0.0005,
        maximum_gap_m: float = 0.015,
        minimum_clearance_m: float = 0.004,
        maximum_clearance_m: float = 0.04,
    ) -> None:
        self.target_overlap_fraction = float(target_overlap_fraction)
        self.overlap_tolerance = float(overlap_tolerance)
        self.minimum_gap_m = float(minimum_gap_m)
        self.maximum_gap_m = float(maximum_gap_m)
        self.minimum_clearance_m = float(minimum_clearance_m)
        self.maximum_clearance_m = float(maximum_clearance_m)

    @staticmethod
    def _topological_order(
        spec: MagneticAssemblySpec,
    ) -> tuple[tuple[str, ...], tuple[str, ...]]:
        dependencies = {
            piece.piece_id: set(
                spec.connection_for_child(piece.piece_id).parent_ids
                if spec.connection_for_child(piece.piece_id) is not None
                else ()
            )
            for piece in spec.pieces
        }
        remaining = set(dependencies)
        done: set[str] = set()
        order: list[str] = []
        while remaining:
            ready = sorted(
                item for item in remaining if dependencies[item] <= done
            )
            if not ready:
                return tuple(order), tuple(sorted(remaining))
            order.extend(ready)
            done.update(ready)
            remaining.difference_update(ready)
        return tuple(order), ()

    def validate(
        self,
        spec: MagneticAssemblySpec,
        catalog: Mapping[str, MagneticPanelSpec],
        inventory: Sequence[MagneticInventoryItem],
    ) -> MagneticValidationReport:
        violations: list[RuleViolation] = []
        catalog = {str(key): value for key, value in catalog.items()}
        inventory_tuple = tuple(inventory)
        inventory_by_id = {item.object_id: item for item in inventory_tuple}
        if len(inventory_by_id) != len(inventory_tuple):
            violations.append(
                RuleViolation(
                    "duplicate_inventory_object",
                    "physical inventory object ids must be unique",
                )
            )

        connection_by_child: dict[str, list[MagneticConnection]] = {}
        for connection in spec.connections:
            connection_by_child.setdefault(connection.child_id, []).append(connection)
        for child_id, connections in connection_by_child.items():
            if len(connections) > 1:
                violations.append(
                    RuleViolation(
                        "multiple_placement_connections",
                        f"piece {child_id!r} has multiple placement connections",
                        (child_id,),
                    )
                )

        piece_by_id = {item.piece_id: item for item in spec.pieces}
        for piece in spec.pieces:
            panel = catalog.get(piece.asset_name)
            if panel is None:
                violations.append(
                    RuleViolation(
                        "unknown_panel_asset",
                        f"no magnetic-panel geometry is registered for "
                        f"{piece.asset_name!r}",
                        (piece.piece_id,),
                    )
                )
                continue
            if piece.role not in panel.allowed_roles:
                violations.append(
                    RuleViolation(
                        "role_not_supported",
                        f"asset {piece.asset_name!r} cannot be used as "
                        f"{piece.role.value!r}",
                        (piece.piece_id,),
                    )
                )
            inventory_item = inventory_by_id.get(piece.object_id)
            if inventory_item is None:
                violations.append(
                    RuleViolation(
                        "inventory_object_missing",
                        f"physical piece {piece.object_id!r} is not in the tray inventory",
                        (piece.piece_id,),
                    )
                )
            elif inventory_item.asset_name != piece.asset_name:
                violations.append(
                    RuleViolation(
                        "inventory_asset_mismatch",
                        f"piece {piece.piece_id!r} requests {piece.asset_name!r}, "
                        f"but inventory object {piece.object_id!r} is "
                        f"{inventory_item.asset_name!r}",
                        (piece.piece_id,),
                    )
                )

            matches = connection_by_child.get(piece.piece_id, [])
            connection = matches[0] if len(matches) == 1 else None
            anchored = piece.anchor_offset_m is not None
            if anchored == (connection is not None):
                violations.append(
                    RuleViolation(
                        "ambiguous_or_missing_placement",
                        f"piece {piece.piece_id!r} must have exactly one of an "
                        "anchor offset or a placement connection",
                        (piece.piece_id,),
                    )
                )
            if (
                anchored
                and piece.pose_class is PanelPoseClass.VERTICAL
                and not bool(piece.metadata.get("externally_supported", False))
            ):
                violations.append(
                    RuleViolation(
                        "unsupported_vertical_anchor",
                        "a vertical piece cannot be declared free-standing; "
                        "use a two-base vertical_slot connection",
                        (piece.piece_id,),
                    )
                )

        for connection in spec.connections:
            child = piece_by_id[connection.child_id]
            parents = [piece_by_id[item] for item in connection.parent_ids]
            self._validate_connection(connection, child, parents, violations)

        order, cycle = self._topological_order(spec)
        if cycle:
            violations.append(
                RuleViolation(
                    "cyclic_assembly_dependencies",
                    f"assembly dependencies contain a cycle: {list(cycle)}",
                    cycle,
                )
            )

        stable: set[str] = set()
        for piece_id in order:
            piece = piece_by_id[piece_id]
            connection = spec.connection_for_child(piece_id)
            if connection is None:
                if (
                    piece.anchor_offset_m is not None
                    and (
                        piece.pose_class is PanelPoseClass.FLAT
                        or bool(piece.metadata.get("externally_supported", False))
                    )
                ):
                    stable.add(piece_id)
                continue
            if not set(connection.parent_ids) <= stable:
                continue
            if connection.joint_type in {
                MagneticJointType.VERTICAL_SLOT,
                MagneticJointType.BRIDGE,
                MagneticJointType.FLAT_STACK,
            }:
                stable.add(piece_id)
            elif (
                connection.joint_type is MagneticJointType.RIGHT_ANGLE_EDGE
                and all(
                    piece_by_id[parent_id].pose_class is PanelPoseClass.VERTICAL
                    for parent_id in connection.parent_ids
                )
            ):
                stable.add(piece_id)
        for piece in spec.pieces:
            if piece.piece_id not in stable:
                violations.append(
                    RuleViolation(
                        "structurally_unsupported_piece",
                        f"piece {piece.piece_id!r} is not supported by a stable "
                        "assembly prefix",
                        (piece.piece_id,),
                    )
                )

        return MagneticValidationReport(
            not violations,
            tuple(violations),
            order,
            tuple(sorted(stable)),
        )

    def _validate_connection(
        self,
        connection: MagneticConnection,
        child: Any,
        parents: list[Any],
        violations: list[RuleViolation],
    ) -> None:
        def add(code: str, message: str) -> None:
            violations.append(
                RuleViolation(
                    code,
                    message,
                    tuple([child.piece_id, *connection.parent_ids]),
                    connection.connection_id,
                )
            )

        if not self.minimum_gap_m <= connection.gap_m <= self.maximum_gap_m:
            add(
                "unsafe_connection_gap",
                f"connection gap {connection.gap_m:.4f} m is outside "
                f"[{self.minimum_gap_m:.4f}, {self.maximum_gap_m:.4f}]",
            )
        if not (
            self.minimum_clearance_m
            <= connection.clearance_m
            <= self.maximum_clearance_m
        ):
            add(
                "unsafe_approach_clearance",
                f"connection clearance {connection.clearance_m:.4f} m is outside "
                f"[{self.minimum_clearance_m:.4f}, "
                f"{self.maximum_clearance_m:.4f}]",
            )

        if connection.joint_type is MagneticJointType.ANCHOR:
            if connection.parent_ids:
                add("anchor_has_parent", "an anchor connection cannot have parents")
            return

        if connection.joint_type is MagneticJointType.FLAT_STACK:
            if len(parents) != 1:
                add("flat_stack_parent_count", "flat_stack requires one parent")
            if child.pose_class is not PanelPoseClass.FLAT:
                add("flat_stack_child_not_flat", "flat_stack child must be flat")
            return

        if connection.joint_type is MagneticJointType.RIGHT_ANGLE_EDGE:
            if len(parents) != 1 or len(connection.parent_edges) != 1:
                add(
                    "right_angle_parent_count",
                    "right_angle_edge requires one parent and one parent edge",
                )
            if (
                abs(
                    connection.overlap_fraction
                    - self.target_overlap_fraction
                )
                > self.overlap_tolerance
            ):
                add(
                    "right_angle_overlap",
                    "90-degree magnetic panels must overlap approximately half "
                    "of the mating edge",
                )
            if (
                parents
                and parents[0].pose_class is PanelPoseClass.FLAT
                and child.pose_class is PanelPoseClass.VERTICAL
            ):
                add(
                    "single_flat_support_for_vertical",
                    "a vertical panel cannot stand on one lying panel; use "
                    "vertical_slot with two lying supports",
                )
            if (
                parents
                and parents[0].pose_class is child.pose_class
                and child.pose_class is PanelPoseClass.FLAT
            ):
                add(
                    "right_angle_flat_flat",
                    "right_angle_edge cannot connect two flat panels",
                )
            return

        if connection.joint_type is MagneticJointType.VERTICAL_SLOT:
            if len(parents) != 2 or len(connection.parent_edges) != 2:
                add(
                    "vertical_slot_support_count",
                    "vertical_slot requires two supports and two inner edges",
                )
            if child.pose_class is not PanelPoseClass.VERTICAL:
                add(
                    "vertical_slot_child_not_vertical",
                    "vertical_slot child must be vertical",
                )
            if any(
                parent.pose_class is not PanelPoseClass.FLAT
                for parent in parents
            ):
                add(
                    "vertical_slot_support_not_flat",
                    "both vertical_slot supports must be lying flat",
                )
            if len(set(connection.parent_ids)) != len(connection.parent_ids):
                add(
                    "duplicate_vertical_slot_support",
                    "vertical_slot supports must be two different pieces",
                )
            return

        if connection.joint_type is MagneticJointType.BRIDGE:
            if len(parents) < 2:
                add("bridge_support_count", "bridge requires at least two supports")
            if child.pose_class is not PanelPoseClass.FLAT:
                add("bridge_child_not_flat", "bridge/roof piece must be flat")
