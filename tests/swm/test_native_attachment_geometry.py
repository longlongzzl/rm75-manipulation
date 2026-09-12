import numpy as np
import pytest

from rm75_app.swm.native_attachment import compare_attachment_spheres
from rm75_app.swm.scene import SceneInvalid


def test_original_sphere_geometry_and_disabled_padding_are_retained():
    expected = np.array([[.1, .2, .3, .02]])
    actual = np.vstack([expected, [0., 0., 0., -100.]])
    assert compare_attachment_spheres(expected, actual) == 0


@pytest.mark.parametrize('column,delta', [(0, .0001), (1, .0001), (2, .0001), (3, -.001)])
def test_changed_attachment_pose_or_radius_is_rejected(column, delta):
    expected = np.array([[.1, .2, .3, .02]])
    actual = expected.copy()
    actual[0, column] += delta
    with pytest.raises(SceneInvalid, match='geometry or measured transform'):
        compare_attachment_spheres(expected, actual)


def test_extra_active_payload_is_not_hidden_in_padding():
    expected = np.array([[.1, .2, .3, .02]])
    with pytest.raises(SceneInvalid, match='slot coverage'):
        compare_attachment_spheres(expected, np.vstack([expected, expected]))
