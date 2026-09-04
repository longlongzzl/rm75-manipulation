"""Symbolic contracts for physically valid magnetic-panel assemblies."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping, Sequence

import numpy as np


class PanelRole(str, Enum):
    BASE = "base"
    WALL = "wall"
    ROOF = "roof"
    BRIDGE = "bridge"
    DECORATION = "decoration"
    GENERIC = "generic"


class PanelPoseClass(str, Enum):
    FLAT = "flat"
    VERTICAL = "vertical"


class PanelEdge(str, Enum):
    POS_X = "+x"
    NEG_X = "-x"
    POS_Y = "+y"
    NEG_Y = "-y"


class MagneticJointType(str, Enum):
    ANCHOR = "anchor"
    FLAT_STACK = "flat_stack"
    RIGHT_ANGLE_EDGE = "right_angle_edge"
    VERTICAL_SLOT = "vertical_slot"
    BRIDGE = "bridge"


def pose_matrix(value: Sequence[Sequence[float]]) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError("pose must be a finite 4x4 matrix")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1.0e-6):
        raise ValueError("pose is not homogeneous")
    return matrix.copy()


@dataclass(frozen=True)
class MagneticPanelSpec:
    """Geometry convention: local X/Y span the panel, local Z is thickness."""

    asset_name: str
    size_xyz_m: tuple[float, float, float]
    allowed_roles: tuple[PanelRole, ...] = (
        PanelRole.BASE,
        PanelRole.WALL,
        PanelRole.ROOF,
        PanelRole.BRIDGE,
        PanelRole.GENERIC,
    )
    grasp_clearance_m: float = 0.08
    engagement_clearance_m: float = 0.008
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.asset_name:
            raise ValueError("asset_name must not be empty")
        size = tuple(float(item) for item in self.size_xyz_m)
        if len(size) != 3 or any(item <= 0.0 for item in size):
            raise ValueError("panel dimensions must contain three positive values")
        object.__setattr__(self, "size_xyz_m", size)
        object.__setattr__(
            self,
            "allowed_roles",
            tuple(PanelRole(item) for item in self.allowed_roles),
        )
        object.__setattr__(self, "metadata", dict(self.metadata))


@dataclass(frozen=True)
class MagneticInventoryItem:
    object_id: str
    asset_name: str
    tray_slot: str | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.object_id or not self.asset_name:
            raise ValueError("inventory object_id and asset_name must not be empty")
        object.__setattr__(self, "metadata", dict(self.metadata))


@dataclass(frozen=True)
class MagneticPiece:
    piece_id: str
    object_id: str
    asset_name: str
    role: PanelRole
    pose_class: PanelPoseClass
    anchor_offset_m: tuple[float, float, float] | None = None
    anchor_yaw_deg: float = 0.0
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.piece_id or not self.object_id or not self.asset_name:
            raise ValueError("piece_id, object_id, and asset_name must not be empty")
        object.__setattr__(self, "role", PanelRole(self.role))
        object.__setattr__(self, "pose_class", PanelPoseClass(self.pose_class))
        if self.anchor_offset_m is not None:
            offset = tuple(float(item) for item in self.anchor_offset_m)
            if len(offset) != 3:
                raise ValueError("anchor_offset_m must have three values")
            object.__setattr__(self, "anchor_offset_m", offset)
        object.__setattr__(self, "metadata", dict(self.metadata))


@dataclass(frozen=True)
class MagneticConnection:
    connection_id: str
    joint_type: MagneticJointType
    child_id: str
    parent_ids: tuple[str, ...] = ()
    parent_edges: tuple[PanelEdge, ...] = ()
    child_edge: PanelEdge = PanelEdge.NEG_Y
    overlap_fraction: float = 0.5
    gap_m: float = 0.002
    clearance_m: float = 0.008
    flip: bool = False
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.connection_id or not self.child_id:
            raise ValueError("connection_id and child_id must not be empty")
        object.__setattr__(self, "joint_type", MagneticJointType(self.joint_type))
        object.__setattr__(
            self,
            "parent_ids",
            tuple(str(item) for item in self.parent_ids),
        )
        object.__setattr__(
            self,
            "parent_edges",
            tuple(PanelEdge(item) for item in self.parent_edges),
        )
        object.__setattr__(self, "child_edge", PanelEdge(self.child_edge))
        if not 0.0 <= float(self.overlap_fraction) <= 1.0:
            raise ValueError("overlap_fraction must be in [0, 1]")
        if self.gap_m < 0.0 or self.clearance_m < 0.0:
            raise ValueError("gap and clearance must not be negative")
        object.__setattr__(self, "metadata", dict(self.metadata))


@dataclass(frozen=True)
class MagneticAssemblySpec:
    structure_name: str
    pieces: tuple[MagneticPiece, ...]
    connections: tuple[MagneticConnection, ...]
    anchor_pose: Sequence[Sequence[float]]
    description: str = ""
    max_pieces: int = 12
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.structure_name:
            raise ValueError("structure_name must not be empty")
        pieces = tuple(self.pieces)
        connections = tuple(self.connections)
        if not pieces:
            raise ValueError("assembly must contain at least one piece")
        if len(pieces) > int(self.max_pieces):
            raise ValueError(
                f"assembly uses {len(pieces)} pieces, limit is {self.max_pieces}"
            )
        piece_ids = [item.piece_id for item in pieces]
        object_ids = [item.object_id for item in pieces]
        if len(set(piece_ids)) != len(piece_ids):
            raise ValueError("magnetic piece ids must be unique")
        if len(set(object_ids)) != len(object_ids):
            raise ValueError("one physical inventory object cannot fill two pieces")
        connection_ids = [item.connection_id for item in connections]
        if len(set(connection_ids)) != len(connection_ids):
            raise ValueError("magnetic connection ids must be unique")
        known = set(piece_ids)
        for connection in connections:
            unknown = {connection.child_id, *connection.parent_ids} - known
            if unknown:
                raise ValueError(
                    f"connection {connection.connection_id!r} references "
                    f"unknown pieces: {sorted(unknown)}"
                )
        object.__setattr__(self, "pieces", pieces)
        object.__setattr__(self, "connections", connections)
        object.__setattr__(self, "anchor_pose", pose_matrix(self.anchor_pose))
        object.__setattr__(self, "metadata", dict(self.metadata))

    def piece(self, piece_id: str) -> MagneticPiece:
        return next(item for item in self.pieces if item.piece_id == piece_id)

    def connection_for_child(
        self,
        piece_id: str,
    ) -> MagneticConnection | None:
        matches = [item for item in self.connections if item.child_id == piece_id]
        if len(matches) > 1:
            raise ValueError(f"piece {piece_id!r} has multiple placement connections")
        return None if not matches else matches[0]


@dataclass(frozen=True)
class ResolvedMagneticPlacement:
    piece: MagneticPiece
    target_pose: np.ndarray
    support_ids: tuple[str, ...]
    connection_id: str | None
    clearance_m: float
    depends_on_piece_ids: tuple[str, ...]
    diagnostics: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "target_pose", pose_matrix(self.target_pose))
        object.__setattr__(
            self,
            "support_ids",
            tuple(str(item) for item in self.support_ids),
        )
        object.__setattr__(
            self,
            "depends_on_piece_ids",
            tuple(str(item) for item in self.depends_on_piece_ids),
        )
        object.__setattr__(self, "diagnostics", dict(self.diagnostics))
