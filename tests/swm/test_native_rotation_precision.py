"""Rigid float32 sensor roundoff must not invent motion or mask real motion."""
import numpy as np
import pytest
from rm75_app.swm.scene import pose_error
from .conftest import pose


@pytest.mark.parametrize('yaw', [.2, .7, 1.3, 2.4])
def test_float32_pose_is_identical_to_itself_without_rewriting(yaw):
    measured = np.asarray(pose(.2,.3,.4,yaw=yaw), dtype=np.float32).astype(float)
    measured[:3,:3] *= 1. - 6e-8
    saved = measured.copy()
    position, rotation = pose_error(measured, measured)
    assert position == 0
    assert rotation < 1e-12
    np.testing.assert_array_equal(measured, saved)


@pytest.mark.parametrize('angle', [1e-7, 1e-5, .001, .1, np.pi-1e-7, np.pi])
def test_real_rotation_survives_sensor_roundoff(angle):
    initial = np.asarray(pose(yaw=.7))
    initial[:3,:3] *= 1.-6e-8
    final = np.asarray(pose(yaw=.7+angle))
    final[:3,:3] *= 1.+4e-8
    assert pose_error(initial,final)[1] == pytest.approx(angle, abs=1e-12)
    assert pose_error(final,initial)[1] == pytest.approx(angle, abs=1e-12)


@pytest.mark.parametrize('fault', ['scale','reflection','shear'])
def test_nonrigid_input_still_rejected(fault):
    matrix=np.eye(4)
    if fault=='scale': matrix[0,0]=.99
    elif fault=='reflection': matrix[0,0]=-1
    else: matrix[0,1]=.01
    with pytest.raises(ValueError, match='proper and orthonormal'):
        pose_error(matrix,np.eye(4))
