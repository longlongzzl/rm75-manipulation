from __future__ import annotations

import json

import numpy as np

from rm75_app.scenarios.magnetic import (
    MagneticAssemblyFrontend,
    MagneticAssemblyRules,
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


def _catalog() -> dict[str, MagneticPanelSpec]:
    roles = (
        PanelRole.BASE,
        PanelRole.WALL,
        PanelRole.BRIDGE,
        PanelRole.ROOF,
        PanelRole.GENERIC,
    )
    return {
        "panel": MagneticPanelSpec(
            "panel",
            (0.10, 0.06, 0.008),
            allowed_roles=roles,
            engagement_clearance_m=0.008,
        )
    }


def _inventory(count: int = 8) -> tuple[MagneticInventoryItem, ...]:
    return tuple(
        MagneticInventoryItem(f"panel_{index}", "panel", f"slot_{index}")
        for index in range(count)
    )


def _wall_spec(*, separation: float = 0.072) -> MagneticAssemblySpec:
    return MagneticAssemblySpec(
        "wall",
        (
            MagneticPiece(
                "base_left",
                "panel_0",
                "panel",
                PanelRole.BASE,
                PanelPoseClass.FLAT,
                (0.0, -0.5 * separation, 0.0),
            ),
            MagneticPiece(
                "base_right",
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
                "wall_slot",
                MagneticJointType.VERTICAL_SLOT,
                "wall",
                ("base_left", "base_right"),
                (PanelEdge.POS_Y, PanelEdge.NEG_Y),
                gap_m=0.002,
                clearance_m=0.008,
            ),
        ),
        np.eye(4),
        max_pieces=12,
    )


def test_vertical_panel_on_one_flat_panel_is_rejected() -> None:
    spec = MagneticAssemblySpec(
        "bad_wall",
        (
            MagneticPiece(
                "base",
                "panel_0",
                "panel",
                PanelRole.BASE,
                PanelPoseClass.FLAT,
                (0.0, 0.0, 0.0),
            ),
            MagneticPiece(
                "wall",
                "panel_1",
                "panel",
                PanelRole.WALL,
                PanelPoseClass.VERTICAL,
            ),
        ),
        (
            MagneticConnection(
                "bad_single_support",
                MagneticJointType.RIGHT_ANGLE_EDGE,
                "wall",
                ("base",),
                (PanelEdge.POS_Y,),
                overlap_fraction=0.5,
            ),
        ),
        np.eye(4),
    )

    report = MagneticAssemblyRules().validate(
        spec,
        _catalog(),
        _inventory(),
    )

    codes = {item.code for item in report.violations}
    assert not report.valid
    assert "right_angle_parent_not_vertical" in codes
    assert "structurally_unsupported_piece" in codes


def test_two_flat_supports_resolve_a_stable_vertical_slot() -> None:
    planner = StrictMagneticAssemblyPlanner(_catalog())

    placements = planner.resolve(_wall_spec(), _inventory())

    assert [item.piece.piece_id for item in placements] == [
        "base_left",
        "base_right",
        "wall",
    ]
    wall = placements[-1]
    assert wall.support_ids == ("base_left", "base_right")
    assert np.allclose(wall.target_pose[:3, 1], [0.0, 0.0, 1.0])
    assert planner.last_geometry_report is not None
    assert planner.last_geometry_report.valid
    slot = planner.last_geometry_report.diagnostics["wall_slot"]
    assert np.isclose(slot["support_edge_separation_m"], 0.012)
    assert np.isclose(slot["expected_slot_width_m"], 0.012)


def test_vertical_slot_with_wrong_physical_width_is_rejected_after_resolution() -> None:
    planner = StrictMagneticAssemblyPlanner(_catalog())

    try:
        planner.resolve(_wall_spec(separation=0.10), _inventory())
    except ValueError as exc:
        assert "slot_width_mismatch" in str(exc)
    else:
        raise AssertionError("bad slot geometry was not rejected")


def test_right_angle_corner_requires_half_edge_overlap() -> None:
    wall = _wall_spec()
    pieces = wall.pieces + (
        MagneticPiece(
            "corner",
            "panel_3",
            "panel",
            PanelRole.WALL,
            PanelPoseClass.VERTICAL,
        ),
    )
    connections = wall.connections + (
        MagneticConnection(
            "corner_joint",
            MagneticJointType.RIGHT_ANGLE_EDGE,
            "corner",
            ("wall",),
            (PanelEdge.POS_X,),
            child_edge=PanelEdge.NEG_X,
            overlap_fraction=0.25,
        ),
    )
    spec = MagneticAssemblySpec(
        "bad_corner",
        pieces,
        connections,
        np.eye(4),
    )

    report = MagneticAssemblyRules().validate(
        spec,
        _catalog(),
        _inventory(),
    )

    assert "right_angle_overlap" in {
        item.code for item in report.violations
    }


def test_valid_wall_corner_passes_symbolic_and_geometry_checks() -> None:
    wall = _wall_spec()
    spec = MagneticAssemblySpec(
        "corner",
        wall.pieces
        + (
            MagneticPiece(
                "corner",
                "panel_3",
                "panel",
                PanelRole.WALL,
                PanelPoseClass.VERTICAL,
            ),
        ),
        wall.connections
        + (
            MagneticConnection(
                "corner_joint",
                MagneticJointType.RIGHT_ANGLE_EDGE,
                "corner",
                ("wall",),
                (PanelEdge.POS_X,),
                child_edge=PanelEdge.NEG_X,
                overlap_fraction=0.5,
                gap_m=0.002,
                clearance_m=0.008,
            ),
        ),
        np.eye(4),
    )
    planner = StrictMagneticAssemblyPlanner(_catalog())

    placements = planner.resolve(spec, _inventory())

    assert len(placements) == 4
    assert planner.last_geometry_report is not None
    assert planner.last_geometry_report.valid
    diagnostics = planner.last_geometry_report.diagnostics["corner_joint"]
    assert diagnostics["panel_normal_absolute_cosine"] < 1.0e-6


def test_duplicate_child_connections_are_reported_without_validator_crash() -> None:
    wall = _wall_spec()
    duplicate = MagneticConnection(
        "wall_slot_duplicate",
        MagneticJointType.VERTICAL_SLOT,
        "wall",
        ("base_left", "base_right"),
        (PanelEdge.POS_Y, PanelEdge.NEG_Y),
    )
    spec = MagneticAssemblySpec(
        "duplicate",
        wall.pieces,
        wall.connections + (duplicate,),
        np.eye(4),
    )

    report = MagneticAssemblyRules().validate(
        spec,
        _catalog(),
        _inventory(),
    )

    assert "multiple_placement_connections" in {
        item.code for item in report.violations
    }


def test_frontend_repairs_invalid_llm_graph_before_returning() -> None:
    valid = {
        "structure_name": "wall",
        "pieces": [
            {
                "piece_id": "base_left",
                "object_id": "panel_0",
                "asset_name": "panel",
                "role": "base",
                "pose_class": "flat",
                "anchor_offset_m": [0.0, -0.036, 0.0],
            },
            {
                "piece_id": "base_right",
                "object_id": "panel_1",
                "asset_name": "panel",
                "role": "base",
                "pose_class": "flat",
                "anchor_offset_m": [0.0, 0.036, 0.0],
            },
            {
                "piece_id": "wall",
                "object_id": "panel_2",
                "asset_name": "panel",
                "role": "wall",
                "pose_class": "vertical",
                "anchor_offset_m": None,
            },
        ],
        "connections": [
            {
                "connection_id": "slot",
                "joint_type": "vertical_slot",
                "child_id": "wall",
                "parent_ids": ["base_left", "base_right"],
                "parent_edges": ["+y", "-y"],
                "child_edge": "-y",
                "overlap_fraction": 0.5,
                "gap_m": 0.002,
                "clearance_m": 0.008,
            }
        ],
    }

    class FakeLLM:
        def __init__(self) -> None:
            self.calls = 0

        def complete(self, prompt: str) -> str:
            self.calls += 1
            if self.calls == 1:
                return json.dumps(
                    {
                        "structure_name": "invalid",
                        "pieces": [
                            {
                                "piece_id": "wall",
                                "object_id": "panel_0",
                                "asset_name": "panel",
                                "role": "wall",
                                "pose_class": "vertical",
                                "anchor_offset_m": [0.0, 0.0, 0.0],
                            }
                        ],
                        "connections": [],
                    }
                )
            assert "previous response was invalid" in prompt.lower()
            return json.dumps(valid)

    llm = FakeLLM()
    frontend = MagneticAssemblyFrontend(
        _catalog(),
        _inventory(),
        max_pieces=12,
    )

    structure = frontend.generate(
        "搭一面墙",
        anchor_pose=np.eye(4),
        llm=llm,
        allow_template_fallback=False,
    )

    assert llm.calls == 2
    assert structure.structure_name == "wall"
    assert MagneticAssemblyRules().validate(
        structure,
        _catalog(),
        _inventory(),
    ).valid
