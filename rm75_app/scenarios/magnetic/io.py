"""JSON serialization helpers for magnetic assembly frontends and tools."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

from .contracts import (
    MagneticAssemblySpec,
    MagneticConnection,
    MagneticInventoryItem,
    MagneticJointType,
    MagneticPiece,
    PanelEdge,
    PanelPoseClass,
    PanelRole,
)


def magnetic_inventory_from_dict(
    value: Sequence[Mapping[str, Any]],
) -> tuple[MagneticInventoryItem, ...]:
    return tuple(
        MagneticInventoryItem(
            object_id=str(item["object_id"]),
            asset_name=str(item["asset_name"]),
            tray_slot=(
                None
                if item.get("tray_slot") is None
                else str(item["tray_slot"])
            ),
            metadata=dict(item.get("metadata") or {}),
        )
        for item in value
    )


def load_magnetic_inventory(
    path: str | Path,
) -> tuple[MagneticInventoryItem, ...]:
    raw = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    items = raw.get("items", raw) if isinstance(raw, Mapping) else raw
    if not isinstance(items, Sequence) or isinstance(items, (str, bytes)):
        raise ValueError("magnetic inventory JSON must contain an item list")
    return magnetic_inventory_from_dict(items)


def magnetic_inventory_as_dict(
    inventory: Sequence[MagneticInventoryItem],
) -> dict[str, Any]:
    return {
        "schema": "rm75.magnetic_inventory/v1",
        "items": [
            {
                "object_id": item.object_id,
                "asset_name": item.asset_name,
                "tray_slot": item.tray_slot,
                "metadata": dict(item.metadata),
            }
            for item in inventory
        ],
    }


def magnetic_assembly_from_dict(
    value: Mapping[str, Any],
    *,
    default_anchor_pose: Sequence[Sequence[float]] | None = None,
    default_description: str = "",
    max_pieces: int = 12,
) -> MagneticAssemblySpec:
    anchor_pose = value.get("anchor_pose", default_anchor_pose)
    if anchor_pose is None:
        anchor_pose = np.eye(4, dtype=np.float64)
    pieces = tuple(
        MagneticPiece(
            piece_id=str(item["piece_id"]),
            object_id=str(item["object_id"]),
            asset_name=str(item["asset_name"]),
            role=PanelRole(str(item.get("role", "generic"))),
            pose_class=PanelPoseClass(
                str(item.get("pose_class", "flat"))
            ),
            anchor_offset_m=(
                None
                if item.get("anchor_offset_m") is None
                else tuple(float(entry) for entry in item["anchor_offset_m"])
            ),
            anchor_yaw_deg=float(item.get("anchor_yaw_deg", 0.0)),
            metadata=dict(item.get("metadata") or {}),
        )
        for item in value.get("pieces") or ()
    )
    connections = tuple(
        MagneticConnection(
            connection_id=str(item["connection_id"]),
            joint_type=MagneticJointType(str(item["joint_type"])),
            child_id=str(item["child_id"]),
            parent_ids=tuple(
                str(entry) for entry in item.get("parent_ids") or ()
            ),
            parent_edges=tuple(
                PanelEdge(str(entry))
                for entry in item.get("parent_edges") or ()
            ),
            child_edge=PanelEdge(str(item.get("child_edge", "-y"))),
            overlap_fraction=float(item.get("overlap_fraction", 0.5)),
            gap_m=float(item.get("gap_m", 0.002)),
            clearance_m=float(item.get("clearance_m", 0.008)),
            flip=bool(item.get("flip", False)),
            metadata=dict(item.get("metadata") or {}),
        )
        for item in value.get("connections") or ()
    )
    return MagneticAssemblySpec(
        structure_name=str(
            value.get("structure_name") or "magnetic_structure"
        ),
        pieces=pieces,
        connections=connections,
        anchor_pose=anchor_pose,
        description=str(value.get("description") or default_description),
        max_pieces=int(value.get("max_pieces", max_pieces)),
        metadata=dict(value.get("metadata") or {}),
    )


def magnetic_assembly_as_dict(
    spec: MagneticAssemblySpec,
) -> dict[str, Any]:
    return {
        "schema": "rm75.magnetic_assembly/v1",
        "structure_name": spec.structure_name,
        "description": spec.description,
        "max_pieces": spec.max_pieces,
        "anchor_pose": np.asarray(spec.anchor_pose).tolist(),
        "pieces": [
            {
                "piece_id": item.piece_id,
                "object_id": item.object_id,
                "asset_name": item.asset_name,
                "role": item.role.value,
                "pose_class": item.pose_class.value,
                "anchor_offset_m": (
                    None
                    if item.anchor_offset_m is None
                    else list(item.anchor_offset_m)
                ),
                "anchor_yaw_deg": item.anchor_yaw_deg,
                "metadata": dict(item.metadata),
            }
            for item in spec.pieces
        ],
        "connections": [
            {
                "connection_id": item.connection_id,
                "joint_type": item.joint_type.value,
                "child_id": item.child_id,
                "parent_ids": list(item.parent_ids),
                "parent_edges": [edge.value for edge in item.parent_edges],
                "child_edge": item.child_edge.value,
                "overlap_fraction": item.overlap_fraction,
                "gap_m": item.gap_m,
                "clearance_m": item.clearance_m,
                "flip": item.flip,
                "metadata": dict(item.metadata),
            }
            for item in spec.connections
        ],
        "metadata": dict(spec.metadata),
    }


def load_magnetic_assembly(path: str | Path) -> MagneticAssemblySpec:
    raw = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    if not isinstance(raw, Mapping):
        raise ValueError("magnetic assembly JSON must be an object")
    return magnetic_assembly_from_dict(raw)


def save_magnetic_assembly(
    spec: MagneticAssemblySpec,
    path: str | Path,
) -> Path:
    target = Path(path).expanduser().resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_text(
        json.dumps(
            magnetic_assembly_as_dict(spec),
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    temporary.replace(target)
    return target
