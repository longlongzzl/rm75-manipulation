from pathlib import Path

import numpy as np
import pytest
import torch

from rm75_app.planning.gripper_collision import (
    DYNAMIC_GRIPPER_LINKS,
    DynamicGripperSphereController,
    gripper_link_transforms,
    remap_link_sphere_centers,
)


URDF = Path(__file__).resolve().parents[2] / "assets/robot_models/RM75_gripper/RM75-B/urdf/RM75-B.urdf"


class SphereStorage:
    def __init__(self):
        self.indices = {name: [i] for i, name in enumerate(DYNAMIC_GRIPPER_LINKS)}
        self.link_spheres = torch.tensor(
            [[[0.01, 0.02, 0.03, 0.004] for _ in DYNAMIC_GRIPPER_LINKS]],
            dtype=torch.float32,
        )

    def get_sphere_index_from_link_name(self, name):
        return self.indices[name]


def measured_positions():
    return {
        f"gripper_{side}_{part}_Joint": value
        for side, values in (("Left", (0.01, -0.02, 0.03)), ("Right", (0.04, -0.05, 0.06)))
        for part, value in zip(("1", "2", "Support"), values)
    }


def test_measured_spheres_use_all_six_joints_and_restore_reference():
    storage = SphereStorage()
    controller = DynamicGripperSphereController(urdf_path=URDF)
    original = storage.link_spheres.clone()
    positions = measured_positions()
    controller.apply(storage, "open")
    acknowledgement = controller.apply_measured(storage, positions)
    reference = gripper_link_transforms(URDF, 0.6)
    measured = gripper_link_transforms(URDF, positions)
    for name, indices in storage.indices.items():
        expected = remap_link_sphere_centers(
            original[0, indices, :3].numpy(), reference[name], measured[name]
        )
        np.testing.assert_allclose(storage.link_spheres[0, indices, :3].numpy(), expected, atol=1e-6)
    assert acknowledgement["sphere_count"] == len(DYNAMIC_GRIPPER_LINKS)
    assert all(row["radii_unchanged"] for row in acknowledgement["links"])
    assert torch.equal(storage.link_spheres[:, :, 3], original[:, :, 3])
    controller.apply(storage, "closed")
    torch.testing.assert_close(storage.link_spheres, original)


@pytest.mark.parametrize("invalid", ["missing", "extra", "nonfinite"])
def test_invalid_measured_configuration_does_not_mutate_spheres(invalid):
    storage = SphereStorage()
    original = storage.link_spheres.clone()
    positions = measured_positions()
    key = next(iter(positions))
    if invalid == "missing":
        positions.pop(key)
    elif invalid == "extra":
        positions["joint_1"] = 0.0
    else:
        positions[key] = float("nan")
    controller = DynamicGripperSphereController(urdf_path=URDF)
    with pytest.raises((ValueError, KeyError)):
        controller.apply_measured(storage, positions)
    assert torch.equal(storage.link_spheres, original)
