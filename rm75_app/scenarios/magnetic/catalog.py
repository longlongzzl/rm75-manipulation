"""Catalog and tray-inventory adapters for magnetic assembly."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Mapping, Sequence

from rm75_app.orchestration.multi_object_executor import (
    ObjectLifecycle,
    TaskSceneState,
)

from .contracts import (
    MagneticInventoryItem,
    MagneticPanelSpec,
    PanelRole,
)


def load_magnetic_catalog(
    path: str | Path,
) -> dict[str, MagneticPanelSpec]:
    raw = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    assets = raw.get("assets", raw)
    if not isinstance(assets, Mapping):
        raise ValueError("magnetic catalog must contain an object mapping")
    output: dict[str, MagneticPanelSpec] = {}
    for asset_name, value in assets.items():
        if not isinstance(value, Mapping):
            raise ValueError(f"catalog entry {asset_name!r} must be an object")
        spec = MagneticPanelSpec(
            asset_name=str(asset_name),
            size_xyz_m=tuple(float(item) for item in value["size_xyz_m"]),
            allowed_roles=tuple(
                PanelRole(str(item))
                for item in value.get(
                    "allowed_roles",
                    (
                        "base",
                        "wall",
                        "roof",
                        "bridge",
                        "generic",
                    ),
                )
            ),
            grasp_clearance_m=float(
                value.get("grasp_clearance_m", 0.08)
            ),
            engagement_clearance_m=float(
                value.get("engagement_clearance_m", 0.008)
            ),
            metadata=dict(value.get("metadata") or {}),
        )
        output[spec.asset_name] = spec
    return output


def save_magnetic_catalog(
    catalog: Mapping[str, MagneticPanelSpec],
    path: str | Path,
) -> Path:
    target = Path(path).expanduser().resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema": "rm75.magnetic_panel_catalog/v1",
        "assets": {
            name: {
                "size_xyz_m": list(spec.size_xyz_m),
                "allowed_roles": [
                    item.value for item in spec.allowed_roles
                ],
                "grasp_clearance_m": spec.grasp_clearance_m,
                "engagement_clearance_m": spec.engagement_clearance_m,
                "metadata": dict(spec.metadata),
            }
            for name, spec in sorted(catalog.items())
        },
    }
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    temporary.replace(target)
    return target


def inventory_from_scene(
    scene: TaskSceneState,
    catalog: Mapping[str, MagneticPanelSpec],
    *,
    include_placed: bool = False,
    max_items: int = 12,
) -> tuple[MagneticInventoryItem, ...]:
    """Read the existing tray/scene state without inventing object instances."""

    allowed_lifecycle = {ObjectLifecycle.AVAILABLE}
    if include_placed:
        allowed_lifecycle.add(ObjectLifecycle.PLACED)
    items = [
        MagneticInventoryItem(
            object_id=state.object_id,
            asset_name=state.asset_name,
            tray_slot=(
                None
                if state.metadata.get("tray_slot") is None
                else str(state.metadata["tray_slot"])
            ),
            metadata={
                **dict(state.metadata),
                "scene_revision": scene.revision,
                "lifecycle": state.lifecycle.value,
            },
        )
        for state in scene.objects.values()
        if state.asset_name in catalog
        and state.movable
        and state.lifecycle in allowed_lifecycle
    ]
    items.sort(
        key=lambda item: (
            item.tray_slot is None,
            "" if item.tray_slot is None else item.tray_slot,
            item.object_id,
        )
    )
    if len(items) > int(max_items):
        items = items[: int(max_items)]
    return tuple(items)


def catalog_from_scene_metadata(
    scene: TaskSceneState,
    *,
    default_roles: Sequence[PanelRole] = (
        PanelRole.BASE,
        PanelRole.WALL,
        PanelRole.ROOF,
        PanelRole.BRIDGE,
        PanelRole.GENERIC,
    ),
) -> dict[str, MagneticPanelSpec]:
    """Build a catalog when calibrated dimensions live in scene metadata.

    Every matching object must expose either ``size_xyz_m`` or
    ``dimensions_m``. This supports the existing tray importer while the final
    persistent catalog is still being calibrated.
    """

    output: dict[str, MagneticPanelSpec] = {}
    for state in scene.objects.values():
        if state.asset_name in output:
            continue
        dimensions = state.metadata.get(
            "size_xyz_m",
            state.metadata.get("dimensions_m"),
        )
        if dimensions is None:
            continue
        roles = state.metadata.get("magnetic_allowed_roles")
        output[state.asset_name] = MagneticPanelSpec(
            state.asset_name,
            tuple(float(item) for item in dimensions),
            allowed_roles=(
                tuple(PanelRole(str(item)) for item in roles)
                if roles is not None
                else tuple(default_roles)
            ),
            grasp_clearance_m=float(
                state.metadata.get("grasp_clearance_m", 0.08)
            ),
            engagement_clearance_m=float(
                state.metadata.get("engagement_clearance_m", 0.008)
            ),
            metadata={"source": "task_scene_metadata"},
        )
    return output
