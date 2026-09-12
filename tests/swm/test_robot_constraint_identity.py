from types import SimpleNamespace

import numpy as np
import pytest

from rm75_app.swm.native_robot_mirror import constraint_state
from rm75_app.swm.native_body_mirror import compare_native_state
from rm75_app.swm.scene import SceneInvalid


def constraint_fixture():
    parent = SimpleNamespace(entity=object())
    child = SimpleNamespace(entity=object())
    pose = SimpleNamespace(to_transformation_matrix=lambda: np.eye(4))
    drive = SimpleNamespace(parent=parent, entity=child.entity,
        pose_in_parent=pose, pose_in_child=pose, get_drive_target=lambda: pose,
        get_drive_velocity_target=lambda: (np.zeros(3), np.zeros(3)))
    for axis in ('x', 'y', 'z', 'twist', 'cone', 'pyramid'):
        setattr(drive, 'get_limit_' + axis, lambda: (0., 0., 0., 0.))
    for axis in ('x', 'y', 'z', 'twist', 'swing', 'slerp'):
        setattr(drive, 'get_drive_property_' + axis, lambda: (1., 2., 3., 'acceleration'))
    return drive, {'parent': parent, 'child': child}


def test_constraint_reads_native_properties_not_saved_constructor_values():
    drive, links = constraint_fixture()
    expected = constraint_state(drive, links)
    drive.get_limit_twist = lambda: (-1., 1., 0., 0.)
    with pytest.raises(SceneInvalid, match='twist'):
        compare_native_state(expected, constraint_state(drive, links))


@pytest.mark.parametrize('endpoint', ['parent', 'entity'])
def test_constraint_rejects_foreign_endpoint(endpoint):
    drive, links = constraint_fixture()
    setattr(drive, endpoint, object())
    with pytest.raises(SceneInvalid, match='outside the owned'):
        constraint_state(drive, links)
