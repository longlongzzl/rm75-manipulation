from __future__ import annotations

from pathlib import Path

import pytest

from rm75_app.llm.orchestrator import (
    SceneState,
    _desk_slot_support_z,
    _precise_world_z_bounds,
    _scene_object_surface_z,
    mock_llm_parse,
    resolve_plan,
)


SCENE_FILE = Path("assets/test_scenes/gluestick_desk_regression.json")


def test_carrot_on_named_desk_uses_physical_desk_top() -> None:
    scene = SceneState.load(SCENE_FILE)
    llm_plan = mock_llm_parse("把胡萝卜放到桌子上", scene)

    assert llm_plan["operator"] == "on"
    resolved = resolve_plan(scene, llm_plan)

    assert len(resolved) == 1
    step = resolved[0]
    assert step.source_id == "carriot"
    assert step.target_object_id == "desk"
    carrot_bottom, _ = _precise_world_z_bounds(step.target_pose, "carriot")
    desk_top = _desk_slot_support_z(scene.get("desk"))
    worktable_top = _scene_object_surface_z(scene)

    assert carrot_bottom == pytest.approx(desk_top + 0.002, abs=1.0e-5)
    assert carrot_bottom > worktable_top + 0.02


def test_explicit_pose_goal_on_desk_uses_same_support_height() -> None:
    scene = SceneState.load(SCENE_FILE)
    resolved = resolve_plan(
        scene,
        {
            "kind": "single_step",
            "operator": "pose_goal",
            "source_ref": "carriot",
            "pose_goal": {
                "position": {"type": "on_top", "target": "desk"},
            },
        },
    )

    carrot_bottom, _ = _precise_world_z_bounds(resolved[0].target_pose, "carriot")
    desk_top = _desk_slot_support_z(scene.get("desk"))
    assert carrot_bottom == pytest.approx(desk_top + 0.002, abs=1.0e-5)


def test_object_center_of_desk_infers_small_desk_surface() -> None:
    scene = SceneState.load(SCENE_FILE)
    resolved = resolve_plan(
        scene,
        {
            "kind": "single_step",
            "operator": "pose_goal",
            "source_ref": "shuazi",
            "pose_goal": {
                "position": {"type": "object_center", "object": "desk"},
                "orientation": {"type": "keep_current"},
            },
        },
    )

    brush_bottom, _ = _precise_world_z_bounds(resolved[0].target_pose, "shuazi")
    desk_top = _desk_slot_support_z(scene.get("desk"))
    assert brush_bottom == pytest.approx(desk_top + 0.002, abs=1.0e-5)
    assert "inferred small_desk surface" in " ".join(resolved[0].warnings)


def test_unstable_on_to_side_fallback_requires_explicit_confirmation() -> None:
    scene = SceneState.load(SCENE_FILE)
    resolved = resolve_plan(
        scene,
        {
            "kind": "single_step",
            "operator": "on_or_side_fallback",
            "source_ref": "tennis",
            "target_ref": "hongshupian",
            "fallback_direction": "right",
            "fallback_distance_m": 0.02,
        },
    )

    step = resolved[0]
    assert step.description == "tennis -> side of hongshupian after stability fallback"
    assert step.requires_confirmation is True
    assert step.confirmation_kind == "on_to_side_stability_fallback"
    assert "是否接受改为放在旁边" in str(step.confirmation_message)


def test_stable_on_to_side_plan_does_not_require_confirmation() -> None:
    scene = SceneState.load(SCENE_FILE)
    resolved = resolve_plan(
        scene,
        {
            "kind": "single_step",
            "operator": "on_or_side_fallback",
            "source_ref": "gluestick",
            "target_ref": "desk",
            "fallback_direction": "right",
            "fallback_distance_m": 0.02,
        },
    )

    step = resolved[0]
    assert step.description == "gluestick -> on desk"
    assert step.requires_confirmation is False

