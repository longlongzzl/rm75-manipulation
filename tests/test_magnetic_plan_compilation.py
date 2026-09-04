from __future__ import annotations

import numpy as np

from rm75_app.scenarios.magnetic import (
    MagneticAssemblySpec,
    MagneticConnection,
    MagneticInventoryItem,
    MagneticJointType,
    MagneticPanelSpec,
    MagneticPiece,
    PanelEdge,
    PanelPoseClass,
    PanelRole,
    StrictMagneticAssemblyPlanner,
)
from rm75_app.tasks.manipulation_plan import (
    ManipulationPrimitive,
    PlacementMode,
)


def test_valid_vertical_slot_compiles_to_shared_pickplace_atoms() -> None:
    catalog = {
        "panel": MagneticPanelSpec(
            "panel",
            (0.10, 0.06, 0.008),
            allowed_roles=(
                PanelRole.BASE,
                PanelRole.WALL,
                PanelRole.GENERIC,
            ),
            engagement_clearance_m=0.009,
        )
    }
    inventory = tuple(
        MagneticInventoryItem(f"panel_{index}", "panel", f"A{index + 1}")
        for index in range(3)
    )
    separation = 0.06 + 0.008 + 2.0 * 0.002
    structure = MagneticAssemblySpec(
        "wall",
        (
            MagneticPiece(
                "left",
                "panel_0",
                "panel",
                PanelRole.BASE,
                PanelPoseClass.FLAT,
                (0.0, -0.5 * separation, 0.0),
            ),
            MagneticPiece(
                "right",
                "panel_1",
                "panel",
                PanelRole.BASE,
                PanelPoseClass.FLAT,
                (0.0, 0.5 * separation, 0.0),
            ),
            MagneticPiece(
                "wall",
                "panel_2",
                "panel",
                PanelRole.WALL,
                PanelPoseClass.VERTICAL,
            ),
        ),
        (
            MagneticConnection(
                "slot",
                MagneticJointType.VERTICAL_SLOT,
                "wall",
                ("left", "right"),
                (PanelEdge.POS_Y, PanelEdge.NEG_Y),
                gap_m=0.002,
                clearance_m=0.009,
            ),
        ),
        np.eye(4),
    )
    planner = StrictMagneticAssemblyPlanner(catalog)

    plan = planner.compile_manipulation_plan(
        structure,
        inventory,
        scene_file="scene.json",
    )

    assert len(plan.atoms) == 3
    left, right, wall = plan.atoms
    assert all(
        atom.primitive is ManipulationPrimitive.SNAP_PLACE
        for atom in plan.atoms
    )
    assert left.placement_mode is PlacementMode.SURFACE
    assert right.placement_mode is PlacementMode.SURFACE
    assert wall.placement_mode is PlacementMode.VERTICAL
    assert wall.depends_on == (left.atom_id, right.atom_id)
    assert wall.metadata["magnetic_support_piece_ids"] == ["left", "right"]
    assert wall.metadata["magnetic_support_object_ids"] == [
        "panel_0",
        "panel_1",
    ]
    assert wall.metadata["magnetic_engagement_clearance_m"] == 0.009
    assert wall.metadata["place_clearance_m"] == 0.009
    assert wall.support_object_id == "panel_0"
    assert plan.metadata["piece_count"] == 3
    assert plan.metadata["max_piece_count"] == 12
