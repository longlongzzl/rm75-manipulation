from __future__ import annotations

import numpy as np
import pytest
import trimesh

from rm75_app.assets.object_specs import (
    discrete_orientation_symmetry_rotations,
    get_object_spec,
    resolve_object_spec_scales,
)
from rm75_app.core.frames import (
    MANISKILL_TABLE_COLLISION_NAME,
    MANISKILL_TABLE_DIMENSIONS_M,
    MANISKILL_TABLE_WORLD_CENTER_M,
)
from rm75_app.orchestration.multi_object_executor import SceneObjectState, TaskSceneState
from rm75_app.pickplace.atom_task_builder import (
    AtomTaskBuilderConfig,
    FixedSceneAtomTaskBuilder,
    _axis_angle_rotation,
    _pose_to_matrix,
)
from rm75_app.tasks.manipulation_plan import ManipulationAtom, ManipulationPrimitive


def test_builder_preserves_object_goal_for_every_grasp_pair() -> None:
    source = np.eye(4)
    source[:3, 3] = [0.35, 0.05, 0.03]
    target = np.eye(4)
    target[:3, 3] = [0.20, -0.10, 0.04]
    scene = TaskSceneState(
        {"carrot_1": SceneObjectState("carrot_1", "carriot", source)},
        joint_names=("j1", "j2"),
        joint_positions=[0.0, 0.0],
    )
    atom = ManipulationAtom(
        "atom_01",
        ManipulationPrimitive.PICK_PLACE,
        "carrot_1",
        "carriot",
        target,
    )
    builder = FixedSceneAtomTaskBuilder(
        config=AtomTaskBuilderConfig(
            grasp_yaw_offsets_deg=(0.0, 90.0),
            grasp_tilt_toward_base_deg=(0.0,),
        )
    )
    task = builder(atom, scene)
    # Two requested yaws x two closing-axis signs x seven contact shifts.
    assert len(task.grasp_candidates) == 28
    assert set(task.place_candidates_by_grasp) == {
        candidate.candidate_id for candidate in task.grasp_candidates
    }
    for grasp in task.grasp_candidates:
        place = task.places_for_grasp(grasp.candidate_id)[0]
        T_world_tcp_grasp = _pose_to_matrix(grasp.pose)
        T_tcp_object = np.linalg.inv(T_world_tcp_grasp) @ source
        T_world_tcp_place = _pose_to_matrix(place.pose)
        reconstructed_target = T_world_tcp_place @ T_tcp_object
        release_target = target.copy()
        release_target[2, 3] += get_object_spec("carriot").tabletop_release_clearance_m
        np.testing.assert_allclose(reconstructed_target, release_target, atol=1e-6)
        planned_target = np.asarray(place.metadata["target_object_pose"])
        distance = float(np.linalg.norm(planned_target[:3, 3] - target[:3, 3]))
        if "target_region_distance_m" in place.metadata:
            assert distance == pytest.approx(0.015)
            np.testing.assert_allclose(place.metadata["semantic_target_pose"], target)
        else:
            np.testing.assert_allclose(planned_target, target)


def test_cubic_symmetry_contains_24_unique_proper_rotations() -> None:
    rotations = discrete_orientation_symmetry_rotations("cubic")

    assert len(rotations) == 24
    assert len({tuple(item.reshape(-1)) for item in rotations}) == 24
    for rotation in rotations:
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-8)
        assert np.linalg.det(rotation) == pytest.approx(1.0)


def test_builder_converts_world_scene_to_robot_base_planning_frame() -> None:
    source = np.eye(4)
    source[:3, 3] = [-0.05, 0.12, 0.04]
    target = np.eye(4)
    target[:3, 3] = [-0.25, -0.28, 0.10]
    scene = TaskSceneState(
        {"ball": SceneObjectState("ball", "tennis", source)},
        joint_names=("j1", "j2"),
        joint_positions=[0.0, 0.0],
    )
    atom = ManipulationAtom(
        "atom_01", ManipulationPrimitive.PICK_PLACE, "ball", "tennis", target
    )
    world_task = FixedSceneAtomTaskBuilder(
        config=AtomTaskBuilderConfig(
            grasp_yaw_offsets_deg=(0.0,),
            grasp_tilt_toward_base_deg=(0.0,),
            sphere_grasp_yaw_offsets_deg=(0.0,),
        )
    )(atom, scene)
    base_task = FixedSceneAtomTaskBuilder(
        config=AtomTaskBuilderConfig(
            grasp_yaw_offsets_deg=(0.0,),
            grasp_tilt_toward_base_deg=(0.0,),
            sphere_grasp_yaw_offsets_deg=(0.0,),
            robot_base_world_xyz_m=(-0.615, 0.0, 0.0),
        )
    )(atom, scene)

    expected_delta = np.asarray([0.615, 0.0, 0.0])
    np.testing.assert_allclose(
        base_task.grasp_candidates[0].pose.position
        - world_task.grasp_candidates[0].pose.position,
        expected_delta,
    )
    np.testing.assert_allclose(
        base_task.place_candidates[0].pose.position
        - world_task.place_candidates[0].pose.position,
        expected_delta,
    )
    np.testing.assert_allclose(
        base_task.scene.objects[0].pose.position
        - world_task.scene.objects[0].pose.position,
        expected_delta,
    )


def test_builder_adds_exact_maniskill_table_in_robot_base_frame() -> None:
    pose = np.eye(4)
    scene = TaskSceneState(
        {"ball": SceneObjectState("ball", "tennis", pose)},
        joint_names=("j1", "j2"),
        joint_positions=[0.0, 0.0],
    )
    atom = ManipulationAtom(
        "atom_01", ManipulationPrimitive.PICK_PLACE, "ball", "tennis", pose
    )
    task = FixedSceneAtomTaskBuilder(
        config=AtomTaskBuilderConfig(
            robot_base_world_xyz_m=(-0.615, 0.0, 0.0),
            include_maniskill_workspace_table=True,
        )
    )(atom, scene)

    table = next(
        item
        for item in task.scene.objects
        if item.name == MANISKILL_TABLE_COLLISION_NAME
    )
    assert table.kind == "cuboid"
    np.testing.assert_allclose(table.dimensions, MANISKILL_TABLE_DIMENSIONS_M)
    np.testing.assert_allclose(
        table.pose.position,
        np.asarray(MANISKILL_TABLE_WORLD_CENTER_M) - [-0.615, 0.0, 0.0],
    )


def test_builder_uses_sphere_proxy_and_automatic_attachment_for_tennis() -> None:
    pose = np.eye(4)
    pose[:3, 3] = [0.2, -0.2, 0.04]
    scene = TaskSceneState(
        {"ball": SceneObjectState("ball", "tennis", pose)},
        joint_names=("j1", "j2"),
        joint_positions=[0.0, 0.0],
    )
    atom = ManipulationAtom(
        "atom_01",
        ManipulationPrimitive.PICK_PLACE,
        "ball",
        "tennis",
        pose,
    )

    task = FixedSceneAtomTaskBuilder()(atom, scene)

    collision = next(item for item in task.scene.objects if item.name == "ball")
    assert collision.kind == "sphere"
    assert collision.radius is not None and collision.radius > 0.0
    assert collision.metadata["collision_proxy"] == "sphere"
    assert collision.metadata["attachment_num_spheres"] is None
    assert collision.metadata["visual_mesh_path"].endswith("tennis.glb")


def test_builder_uses_oriented_cuboid_proxy_for_non_spherical_asset() -> None:
    pose = np.eye(4)
    pose[:3, 3] = [0.2, -0.2, 0.04]
    scene = TaskSceneState(
        {"carrot": SceneObjectState("carrot", "carriot", pose)},
        joint_names=("j1", "j2"),
        joint_positions=[0.0, 0.0],
    )
    atom = ManipulationAtom(
        "atom_01",
        ManipulationPrimitive.PICK_PLACE,
        "carrot",
        "carriot",
        pose,
    )

    collision = FixedSceneAtomTaskBuilder()(atom, scene).scene.objects[0]

    assert collision.kind == "cuboid"
    assert collision.metadata["collision_proxy"] == "cuboid"
    assert collision.dimensions is not None
    assert np.max(collision.dimensions) > 4.0 * np.min(collision.dimensions)


def test_symmetric_tennis_grasp_yaw_ignores_object_rotation() -> None:
    builder = FixedSceneAtomTaskBuilder(
        config=AtomTaskBuilderConfig(
            grasp_yaw_offsets_deg=(0.0,),
            sphere_grasp_yaw_offsets_deg=(0.0,),
        )
    )
    first = np.eye(4)
    first[:3, 3] = [0.1, -0.2, 0.04]
    second = first.copy()
    second[:3, :3] = np.asarray(
        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    )

    first_grasp = builder._grasp_candidates("tennis", first)[0]
    second_grasp = builder._grasp_candidates("tennis", second)[0]

    np.testing.assert_allclose(
        first_grasp.pose.quaternion_wxyz,
        second_grasp.pose.quaternion_wxyz,
        atol=1e-8,
    )


def test_default_grasp_relation_sets_use_continuous_pen_freedom() -> None:
    transform = np.eye(4)
    transform[:3, 3] = [0.25, 0.10, 0.04]
    builder = FixedSceneAtomTaskBuilder()

    tennis = builder._grasp_candidates("tennis", transform)
    carrot = builder._grasp_candidates("carriot", transform)
    pen = builder._grasp_candidates("bi", transform)

    assert len(tennis) == 64
    assert len(carrot) == 70
    coarse_pen = [
        item for item in pen if item.metadata.get("refinement_parent_id") is None
    ]
    refined_pen = [
        item for item in pen if item.metadata.get("refinement_parent_id") is not None
    ]
    pen_capability = get_object_spec("bi").grasp_capability
    tennis_capability = get_object_spec("tennis").grasp_capability
    carrot_capability = get_object_spec("carriot").grasp_capability
    assert len(coarse_pen) == 70
    assert not refined_pen
    assert {
        item.metadata["yaw_offset_deg"] for item in tennis
    } == set(tennis_capability.closing_axis_yaw_offsets_deg)
    assert {
        item.metadata["tilt_toward_base_deg"] for item in tennis
    } == set(tennis_capability.approach_tilt_samples_deg)
    assert {
        round(float(item.metadata["axis_shift_m"]), 3) for item in coarse_pen
    } == {0.0, -0.048, -0.04, -0.02, 0.02, 0.04, 0.048}
    assert {
        item.metadata["tilt_toward_base_deg"] for item in coarse_pen
    } == set(pen_capability.free_rotation_seed_angles_deg)
    assert {item.metadata["free_rotation_axis_local"] for item in coarse_pen} == {
        "y"
    }
    assert {item.metadata["yaw_offset_deg"] for item in coarse_pen} == {0.0, 180.0}
    assert {item.metadata["closing_axis_flipped"] for item in coarse_pen} == {
        False,
        True,
    }
    assert {item.metadata["grasp_capability"] for item in coarse_pen} == {
        "pen_table"
    }
    assert {
        item.metadata["tilt_toward_base_deg"] for item in carrot
    } == set(carrot_capability.free_rotation_seed_angles_deg)
    assert {item.metadata["free_rotation_axis_local"] for item in carrot} == {
        "y"
    }


def test_box_and_brush_keep_full_discrete_tilt_sets_without_grasp_level_filter() -> None:
    transform = np.eye(4)
    transform[:3, 3] = [0.25, 0.10, 0.04]
    builder = FixedSceneAtomTaskBuilder(
        config=AtomTaskBuilderConfig(use_continuous_grasp_freedom=False)
    )

    cube = builder._grasp_candidates("lvmukuai", transform)
    brush = builder._grasp_candidates("shuazi", transform)

    assert len(cube) == 16
    assert {item.metadata["tilt_toward_base_deg"] for item in cube} == {
        0.0,
        15.0,
        30.0,
        45.0,
    }
    assert {item.metadata["tilt_toward_base_deg"] for item in brush} == {
        0.0,
        20.0,
        30.0,
        45.0,
    }
    assert all("level_jaws_required" not in item.metadata for item in cube + brush)
    assert all("max_closing_axis_world_z" not in item.metadata for item in cube + brush)


@pytest.mark.parametrize("asset_name", ["tennis", "bi", "carriot"])
def test_all_grasp_yaw_tilt_candidates_use_world_z_frame(asset_name: str) -> None:
    transform = np.eye(4)
    transform[:3, 3] = [0.61, 0.14, 0.037]
    builder = FixedSceneAtomTaskBuilder()

    candidates = builder._grasp_candidates(asset_name, transform)

    for candidate in candidates:
        rotation = _pose_to_matrix(candidate.pose)[:3, :3]
        tilt_deg = float(candidate.metadata["tilt_toward_base_deg"])
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-7)
        assert rotation[2, 1] == pytest.approx(0.0, abs=1e-7)
        assert rotation[2, 2] == pytest.approx(
            -np.cos(np.deg2rad(tilt_deg)), abs=1e-7
        )
        assert candidate.metadata["orientation_frame"] == "world_z"


def test_pen_grasp_world_yaw_tracks_long_axis_but_keeps_world_z_tilt() -> None:
    builder = FixedSceneAtomTaskBuilder()
    first = np.eye(4)
    first[:3, 3] = [0.2, -0.1, 0.02]
    second = first.copy()
    second[:3, :3] = _axis_angle_rotation(
        np.asarray([1.0, 1.0, 0.0]), np.deg2rad(57.0)
    )

    first_candidates = builder._grasp_candidates("bi", first)
    second_candidates = builder._grasp_candidates("bi", second)

    assert not np.allclose(
        first_candidates[0].pose.quaternion_wxyz,
        second_candidates[0].pose.quaternion_wxyz,
    )
    for candidates in (first_candidates, second_candidates):
        assert {item.metadata["yaw_offset_deg"] for item in candidates} == {
            0.0,
            180.0,
        }
        for candidate in candidates:
            rotation = _pose_to_matrix(candidate.pose)[:3, :3]
            assert rotation[2, 1] == pytest.approx(0.0, abs=1e-7)
            assert candidate.metadata["orientation_frame"] == "world_z"
            assert candidate.metadata["world_yaw_basis"] == (
                "world_object_long_axis_perpendicular"
            )


def test_pen_signed_tilt_explores_both_sides_without_rotating_closing_axis() -> None:
    transform = np.eye(4)
    transform[:3, 3] = [0.61, 0.14, 0.037]
    candidates = FixedSceneAtomTaskBuilder(
        config=AtomTaskBuilderConfig(use_continuous_grasp_freedom=False)
    )._grasp_candidates("bi", transform)
    coarse = [
        item for item in candidates
        if item.metadata.get("refinement_parent_id") is None
        and item.metadata["axis_shift_m"] == 0.0
    ]
    positive = next(
        item for item in coarse if item.metadata["tilt_toward_base_deg"] == 30.0
    )
    negative = next(
        item for item in coarse if item.metadata["tilt_toward_base_deg"] == -30.0
    )
    positive_rotation = _pose_to_matrix(positive.pose)[:3, :3]
    negative_rotation = _pose_to_matrix(negative.pose)[:3, :3]

    np.testing.assert_allclose(
        positive_rotation[:, 1], negative_rotation[:, 1], atol=1e-8
    )
    np.testing.assert_allclose(
        positive_rotation[:2, 2], -negative_rotation[:2, 2], atol=1e-8
    )
    assert positive_rotation[2, 2] == pytest.approx(negative_rotation[2, 2])


def test_discrete_pen_fallback_preserves_saved_candidate_lattice() -> None:
    transform = np.eye(4)
    candidates = FixedSceneAtomTaskBuilder(
        config=AtomTaskBuilderConfig(use_continuous_grasp_freedom=False)
    )._grasp_candidates("bi", transform)
    coarse = [
        item for item in candidates
        if item.metadata.get("refinement_parent_id") is None
    ]
    refined = [
        item for item in candidates
        if item.metadata.get("refinement_parent_id") is not None
    ]

    assert len(coarse) == 154
    assert len(refined) == 264
    assert {item.metadata["free_rotation_axis_local"] for item in candidates} == {
        None
    }


def test_axial_objects_expand_equivalent_place_spin_without_changing_axis() -> None:
    source = np.eye(4)
    source[:3, 3] = [0.25, 0.10, 0.04]
    target = np.eye(4)
    target[:3, 3] = [0.10, -0.20, 0.12]
    scene = TaskSceneState(
        {"pen": SceneObjectState("pen", "bi", source)},
        joint_names=("j1", "j2"),
        joint_positions=[0.0, 0.0],
    )
    atom = ManipulationAtom(
        "atom_01", ManipulationPrimitive.PICK_PLACE, "pen", "bi", target
    )

    task = FixedSceneAtomTaskBuilder()(atom, scene)
    places = task.places_for_grasp(task.grasp_candidates[0].candidate_id)

    # Tabletop release keeps the parallel jaws level; axial spins that would
    # put one finger above the other are physically invalid.
    assert len(places) == 56
    assert {item.metadata["axial_spin_deg"] for item in places} == {
        -167.5,
        -12.5,
        -10.0,
        -5.0,
        0.0,
        5.0,
        10.0,
        12.5,
        167.5,
        170.0,
        175.0,
        180.0,
        185.0,
        190.0,
    }
    assert {item.metadata["axis_flipped"] for item in places} == {False, True}
    assert {
        item.metadata.get("additional_release_height_m", 0.0) for item in places
    } == {0.0, 0.003}
    assert {item.metadata["total_release_height_m"] for item in places} == {0.002, 0.005}
    assert next(
        item for item in places
        if not item.metadata["axis_flipped"]
        and item.metadata["axial_spin_deg"] == 0.0
    ).metadata["search_tier"] == 0
    assert all(
        item.metadata["search_tier"] == 3
        for item in places
        if item.metadata["axis_flipped"]
    )
    assert all(item.metadata["axis_direction_equivalent"] for item in places)
    assert all(item.metadata["level_jaws_required"] for item in places)
    assert max(
        float(item.metadata["estimated_jaw_height_difference_m"])
        for item in places
    ) <= 0.021


def test_gluestick_tabletop_release_totals_are_two_and_five_millimeters() -> None:
    source = np.eye(4)
    source[:3, 3] = [0.25, 0.10, 0.04]
    target = np.eye(4)
    target[:3, 3] = [0.10, -0.20, 0.06]
    scene = TaskSceneState(
        {"glue": SceneObjectState("glue", "gluestick", source)},
        joint_names=("j1", "j2"),
        joint_positions=[0.0, 0.0],
    )
    atom = ManipulationAtom(
        "atom_glue", ManipulationPrimitive.PICK_PLACE, "glue", "gluestick", target
    )

    task = FixedSceneAtomTaskBuilder()(atom, scene)
    places = task.places_for_grasp(task.grasp_candidates[0].candidate_id)
    total_release_heights = {
        round(float(item.metadata["total_release_height_m"]), 6)
        for item in places
    }

    assert total_release_heights == {0.002, 0.005}
    assert any("__release_total_5mm" in item.candidate_id for item in places)
    assert all(
        abs(float(item.metadata["closing_axis_world_z"]))
        <= float(item.metadata["max_closing_axis_world_z"]) + 1.0e-8
        for item in places
    )
    expected_axis = target[:3, :3] @ np.asarray([0.0, 1.0, 0.0])
    for place in places:
        equivalent = np.asarray(place.metadata["target_object_pose"])
        actual_axis = equivalent[:3, :3] @ np.asarray([0.0, 1.0, 0.0])
        expected_sign = -1.0 if place.metadata["axis_flipped"] else 1.0
        np.testing.assert_allclose(
            actual_axis, expected_sign * expected_axis, atol=1e-8
        )


def test_vertical_axial_object_exposes_rotation_as_world_yaw_without_duplicates() -> None:
    source = np.eye(4)
    source[:3, 3] = [0.25, 0.10, 0.075]
    target = np.eye(4)
    # Local +Y (the can's long axis) points along world +Z.
    target[:3, :3] = np.asarray(
        [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]
    )
    target[:3, 3] = [0.10, -0.20, 0.075]
    scene = TaskSceneState(
        {"chips": SceneObjectState("chips", "hongshupian", source)},
        joint_names=("j1", "j2"),
        joint_positions=[0.0, 0.0],
    )
    atom = ManipulationAtom(
        "atom_01", ManipulationPrimitive.PICK_PLACE, "chips", "hongshupian", target
    )

    task = FixedSceneAtomTaskBuilder()(atom, scene)
    places = task.places_for_grasp(task.grasp_candidates[0].candidate_id)

    assert len(places) == 8
    assert {item.metadata["axial_rotation_semantics"] for item in places} == {
        "world_yaw"
    }
    assert {item.metadata["axis_flipped"] for item in places} == {False}
    rotations = {
        tuple(np.round(_pose_to_matrix(item.pose)[:3, :3], 8).reshape(-1))
        for item in places
    }
    assert len(rotations) == len(places)


def test_grasp_position_uses_world_mesh_center_not_asset_origin() -> None:
    builder = FixedSceneAtomTaskBuilder(
        config=AtomTaskBuilderConfig(
            grasp_yaw_offsets_deg=(0.0,),
            grasp_tilt_toward_base_deg=(0.0,),
            grasp_z_offset_m=0.005,
        )
    )
    transform = np.eye(4)
    transform[:3, :3] = np.asarray(
        [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
    )
    transform[:3, 3] = [-0.2, 0.1, 0.03]

    grasp = builder._grasp_candidates("carriot", transform)[0]

    spec = get_object_spec("carriot")
    assert spec is not None
    mesh_scale, _ = resolve_object_spec_scales(spec)
    mesh = trimesh.load(spec.mesh_file, force="scene", process=False)
    scale = np.eye(4)
    scale[:3, :3] *= mesh_scale
    mesh.apply_transform(transform @ scale)
    expected = np.mean(np.asarray(mesh.bounds), axis=0)
    expected[2] += 0.005

    np.testing.assert_allclose(grasp.pose.position, expected, atol=1e-8)


def test_sphere_release_expands_base_tilt_candidates_and_preserves_target_pose() -> None:
    source = np.eye(4)
    source[:3, 3] = [0.0, 0.14, 0.037]
    target = np.eye(4)
    target[:3, 3] = [-0.30, -0.28, 0.103]
    scene = TaskSceneState(
        {"ball": SceneObjectState("ball", "tennis", source)},
        joint_names=("j1", "j2"),
        joint_positions=[0.0, 0.0],
    )
    atom = ManipulationAtom(
        "atom_01",
        ManipulationPrimitive.PICK_PLACE,
        "ball",
        "tennis",
        target,
    )
    builder = FixedSceneAtomTaskBuilder(
        config=AtomTaskBuilderConfig(
            grasp_yaw_offsets_deg=(0.0,),
            sphere_grasp_yaw_offsets_deg=(0.0,),
        )
    )

    task = builder(atom, scene)
    places = task.places_for_grasp("grasp_00_+0deg")

    assert len(places) == 16
    assert {item.metadata["release_tilt_toward_base_deg"] for item in places} == {
        0.0,
        15.0,
        30.0,
        45.0,
    }
    for place in places:
        assert place.metadata["sphere_release"] is True
        assert place.metadata["sphere_release_lift_m"] == pytest.approx(0.03)
        planned_target = np.asarray(place.metadata["target_object_pose"])
        distance = float(np.linalg.norm(planned_target[:3, 3] - target[:3, 3]))
        if "target_region_distance_m" in place.metadata:
            assert distance == pytest.approx(0.015)
            np.testing.assert_allclose(place.metadata["semantic_target_pose"], target)
        else:
            np.testing.assert_allclose(planned_target, target)
    tilted = next(
        item
        for item in places
        if item.metadata["release_tilt_toward_base_deg"] == 15.0
        and item.metadata["release_world_yaw_offset_deg"] == 0.0
    )
    tilted_rotation = _pose_to_matrix(tilted.pose)[:3, :3]
    assert tilted_rotation[2, 2] > -1.0
    assert tilted_rotation[2, 2] == pytest.approx(-np.cos(np.deg2rad(15.0)))
