from __future__ import annotations

from pathlib import Path

import numpy as np
import torch
import yaml

from rm75_app.planning.backends.curobo2 import Curobo2Backend
from rm75_app.planning.contracts import Pose, PoseCandidate
from rm75_app.planning.gripper_collision import (
    DYNAMIC_GRIPPER_LINKS,
    DynamicGripperSphereController,
    gripper_link_transforms,
)


APP_ROOT = Path(__file__).parents[1]
URDF = APP_ROOT / "assets/robot_models/RM75_gripper/RM75-B/urdf/RM75-B.urdf"
SPHERES = APP_ROOT / "assets/curobo_rm75_config/spheres/rm75_rough.yml"


class FakeKinematicsConfig:
    def __init__(self) -> None:
        source = yaml.safe_load(SPHERES.read_text(encoding="utf-8"))["collision_spheres"]
        rows = []
        self.indices = {}
        for link_name in DYNAMIC_GRIPPER_LINKS:
            start = len(rows)
            rows.extend(
                [*item["center"], item["radius"]]
                for item in source[link_name]
            )
            self.indices[link_name] = list(range(start, len(rows)))
        self.link_spheres = torch.tensor([rows], dtype=torch.float32)

    def get_sphere_index_from_link_name(self, link_name: str):
        return self.indices[link_name]


def test_dynamic_gripper_uses_one_sphere_set_and_tracks_open_closed_pose() -> None:
    source = yaml.safe_load(SPHERES.read_text(encoding="utf-8"))["collision_spheres"]
    assert len(source["gripper_base_link"]) == 2
    assert sum(len(source[name]) for name in DYNAMIC_GRIPPER_LINKS) == 36

    params = FakeKinematicsConfig()
    controller = DynamicGripperSphereController(
        URDF,
        reference_joint_position=0.6,
        open_joint_position=0.0,
        closed_joint_position=0.6,
    )
    reference = params.link_spheres.clone()
    controller.apply(params, "open")
    opened = params.link_spheres.clone()
    assert not torch.allclose(opened, reference)
    controller.apply(params, "closed")
    torch.testing.assert_close(params.link_spheres, reference)

    link = "left_pad"
    index = params.indices[link][0]
    transforms_open = gripper_link_transforms(URDF, 0.0)
    transforms_reference = gripper_link_transforms(URDF, 0.6)
    local_open = opened[0, index, :3].numpy()
    local_reference = reference[0, index, :3].numpy()
    world_from_remap = transforms_reference[link] @ np.r_[local_open, 1.0]
    expected_world = transforms_open[link] @ np.r_[local_reference, 1.0]
    np.testing.assert_allclose(world_from_remap, expected_world, atol=2.0e-7)


def test_pose_ik_cache_isolated_by_gripper_state() -> None:
    backend = Curobo2Backend()
    candidate = PoseCandidate(
        "same", Pose([0.3, 0.0, 0.2], [1.0, 0.0, 0.0, 0.0])
    )
    open_key = backend._pose_cache_key(candidate)
    backend._gripper_collision_state = "closed"
    closed_key = backend._pose_cache_key(candidate)
    assert open_key != closed_key
