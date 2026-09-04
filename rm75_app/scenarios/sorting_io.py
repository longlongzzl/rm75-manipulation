"""JSON helpers for tabletop sorting requests."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from rm75_app.tasks.manipulation_plan import PlacementMode

from .sorting import SortingAssignment, SortingRequest, SortingTarget


def sorting_request_from_dict(value: Mapping[str, Any]) -> SortingRequest:
    targets = tuple(
        SortingTarget(
            target_id=str(item["target_id"]),
            pose=item["pose"],
            support_object_id=(
                None
                if item.get("support_object_id") is None
                else str(item["support_object_id"])
            ),
            capacity=int(item.get("capacity", 1)),
            slot_spacing_m=float(item.get("slot_spacing_m", 0.07)),
            placement_mode=PlacementMode(
                str(item.get("placement_mode", "surface_place"))
            ),
            metadata=dict(item.get("metadata") or {}),
        )
        for item in value.get("targets") or ()
    )
    assignments = tuple(
        SortingAssignment(
            object_id=str(item["object_id"]),
            target_id=str(item["target_id"]),
            priority=int(item.get("priority", 0)),
            metadata=dict(item.get("metadata") or {}),
        )
        for item in value.get("assignments") or ()
    )
    return SortingRequest(
        request_id=str(value.get("request_id") or "sorting_request"),
        scene_file=str(value["scene_file"]),
        assignments=assignments,
        targets=targets,
        user_command=str(value.get("user_command") or ""),
        buffer_target_id=(
            None
            if value.get("buffer_target_id") is None
            else str(value["buffer_target_id"])
        ),
        occupancy_radius_m=float(value.get("occupancy_radius_m", 0.045)),
        position_tolerance_m=float(
            value.get("position_tolerance_m", 0.02)
        ),
        orientation_tolerance_deg=float(
            value.get("orientation_tolerance_deg", 20.0)
        ),
        metadata=dict(value.get("metadata") or {}),
    )


def sorting_request_as_dict(request: SortingRequest) -> dict[str, Any]:
    return {
        "schema": "rm75.sorting_request/v1",
        "request_id": request.request_id,
        "scene_file": request.scene_file,
        "user_command": request.user_command,
        "buffer_target_id": request.buffer_target_id,
        "occupancy_radius_m": request.occupancy_radius_m,
        "position_tolerance_m": request.position_tolerance_m,
        "orientation_tolerance_deg": request.orientation_tolerance_deg,
        "assignments": [
            {
                "object_id": item.object_id,
                "target_id": item.target_id,
                "priority": item.priority,
                "metadata": dict(item.metadata),
            }
            for item in request.assignments
        ],
        "targets": [
            {
                "target_id": item.target_id,
                "pose": np.asarray(item.pose).tolist(),
                "support_object_id": item.support_object_id,
                "capacity": item.capacity,
                "slot_spacing_m": item.slot_spacing_m,
                "placement_mode": item.placement_mode.value,
                "metadata": dict(item.metadata),
            }
            for item in request.targets
        ],
        "metadata": dict(request.metadata),
    }


def load_sorting_request(path: str | Path) -> SortingRequest:
    value = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    if not isinstance(value, Mapping):
        raise ValueError("sorting request JSON must be an object")
    return sorting_request_from_dict(value)


def save_sorting_request(
    request: SortingRequest,
    path: str | Path,
) -> Path:
    target = Path(path).expanduser().resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_text(
        json.dumps(
            sorting_request_as_dict(request),
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    temporary.replace(target)
    return target
