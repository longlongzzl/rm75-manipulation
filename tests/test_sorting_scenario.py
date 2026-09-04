from __future__ import annotations

import numpy as np

from rm75_app.orchestration.multi_object_executor import (
    SceneObjectState,
    TaskSceneState,
)
from rm75_app.scenarios.sorting import (
    SortingAssignment,
    SortingPlanCompiler,
    SortingRequest,
    SortingTarget,
)


def _pose(x: float, y: float, z: float = 0.0) -> np.ndarray:
    value = np.eye(4, dtype=np.float64)
    value[:3, 3] = [x, y, z]
    return value


def _swap_scene() -> TaskSceneState:
    return TaskSceneState(
        {
            "a": SceneObjectState("a", "redcube", _pose(0.0, 0.0)),
            "b": SceneObjectState("b", "lvmukuai", _pose(0.10, 0.0)),
        },
        joint_names=("j1", "j2"),
        joint_positions=np.asarray([0.0, 0.0]),
    )


def _swap_request(*, with_buffer: bool) -> SortingRequest:
    targets = [
        SortingTarget("target_for_a", _pose(0.10, 0.0)),
        SortingTarget("target_for_b", _pose(0.0, 0.0)),
    ]
    if with_buffer:
        targets.append(SortingTarget("buffer", _pose(0.25, 0.12)))
    return SortingRequest(
        "swap",
        "scene.json",
        (
            SortingAssignment("a", "target_for_a"),
            SortingAssignment("b", "target_for_b"),
        ),
        tuple(targets),
        buffer_target_id="buffer" if with_buffer else None,
        occupancy_radius_m=0.03,
    )


def test_sorting_swap_requires_buffer_target() -> None:
    compiler = SortingPlanCompiler()

    try:
        compiler.compile(_swap_request(with_buffer=False), _swap_scene())
    except ValueError as exc:
        assert "cycle" in str(exc)
    else:
        raise AssertionError("occupied-target cycle was not rejected")


def test_sorting_swap_expands_to_buffer_then_two_final_moves() -> None:
    plan = SortingPlanCompiler().compile(
        _swap_request(with_buffer=True),
        _swap_scene(),
    )

    assert len(plan.atoms) == 3
    first, second, third = plan.atoms
    assert first.metadata["temporary_buffer_move"] is True
    assert first.object_id == "b"
    assert second.object_id == "a"
    assert first.atom_id in second.depends_on
    assert third.object_id == "b"
    assert first.atom_id in third.depends_on
    assert second.atom_id in third.depends_on
    assert np.allclose(third.target_pose, _pose(0.0, 0.0))


def test_sorting_target_capacity_is_checked_before_planning() -> None:
    try:
        SortingRequest(
            "overfull",
            "scene.json",
            (
                SortingAssignment("a", "one_slot"),
                SortingAssignment("b", "one_slot"),
            ),
            (SortingTarget("one_slot", _pose(0.2, 0.0), capacity=1),),
        )
    except ValueError as exc:
        assert "capacity" in str(exc)
    else:
        raise AssertionError("overfull sorting target was not rejected")


def test_sorting_multi_slot_target_generates_distinct_poses() -> None:
    scene = _swap_scene()
    request = SortingRequest(
        "group",
        "scene.json",
        (
            SortingAssignment("a", "tray"),
            SortingAssignment("b", "tray"),
        ),
        (
            SortingTarget(
                "tray",
                _pose(0.30, 0.10),
                capacity=2,
                slot_spacing_m=0.06,
            ),
        ),
        occupancy_radius_m=0.02,
    )

    plan = SortingPlanCompiler().compile(request, scene)

    assert len(plan.atoms) == 2
    first_xy = np.asarray(plan.atoms[0].target_pose)[:2, 3]
    second_xy = np.asarray(plan.atoms[1].target_pose)[:2, 3]
    assert not np.allclose(first_xy, second_xy)
    assert np.isclose(np.linalg.norm(first_xy - second_xy), 0.06)
