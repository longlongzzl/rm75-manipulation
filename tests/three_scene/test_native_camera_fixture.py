import numpy as np
import pytest
from tools.audit_native_camera_fixture import inverse_native_affine_pose


@pytest.mark.parametrize('direct',[False,True])
@pytest.mark.parametrize('map_base',[False,True])
def test_inverse_preserves_pose_for_both_original_calibration_conventions(direct,map_base):
    calibration=np.eye(4);calibration[:3,3]=[.2,-.3,.4]
    base=np.eye(4);base[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]];base[:3,3]=[-.3,0,0]
    local=np.eye(4);local[:3,:3]=[[1,0,0],[0,0,-1],[0,1,0]]
    offset=np.array([.01,.02,.03]);world=base.copy();world[:3,3]=[-.2,.1,.05]
    before=world.copy()
    camera=inverse_native_affine_pose(world,calibration,base,offset,local,
        direct_calibration=direct,map_robot_base=map_base)
    mapped=(base if map_base else np.eye(4))@(calibration if direct else np.linalg.inv(calibration))@camera@local
    mapped[:3,3]+=offset
    np.testing.assert_allclose(mapped,world,atol=1.e-12)
    np.testing.assert_array_equal(world,before)


@pytest.mark.parametrize('offset',[[0,0],[0,0,np.nan]])
def test_invalid_offsets_fail_before_fixture_generation(offset):
    with pytest.raises(ValueError):inverse_native_affine_pose(np.eye(4),np.eye(4),np.eye(4),offset,np.eye(4))
