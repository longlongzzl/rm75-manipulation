from dataclasses import asdict
import numpy as np
import pytest
from rm75_app.planning.contracts import Pose
from rm75_app.pusht.model import Config,choose_push
from tools.diagnose_pusht_tool_envelope import local_spheres,place_spheres,fixture_observation


def test_tool_geometry_roundtrip_preserves_radii_rotation_and_original_arrays():
    spheres=np.array([[1.,2.,3.,.01],[-1.,1.,2.,.02]])
    pose=Pose([.5,.7,.9],[.5,.5,.5,.5]);before=spheres.copy()
    local=local_spheres(spheres,pose)
    np.testing.assert_allclose(place_spheres(local,pose),spheres,atol=1e-12)
    np.testing.assert_array_equal(spheres,before)
    np.testing.assert_array_equal(local[:,3],spheres[:,3])
    shifted=place_spheres(local,Pose(pose.position+[.1,0,0],pose.quaternion_wxyz))
    np.testing.assert_allclose(shifted[:,:3]-spheres[:,:3],np.tile([.1,0,0],(2,1)),atol=1e-12)


@pytest.mark.parametrize('value',[[],[[1,2,3]],[[1,2,float('nan'),.01]]])
def test_malformed_native_geometry_rejected(value):
    with pytest.raises(ValueError):local_spheres(value,Pose([0,0,0],[1,0,0,0]))


def fixture():
    cfg=Config();push,_=choose_push((.35,-.18,0),(.38,-.18,0),cfg)
    return dict(scenario='authored_simulation_fixture',hardware_connected=False,execute_real=False,
                case='baseline',fixture='low_table',config=asdict(cfg),push=push.as_dict())


def test_saved_fixture_pose_is_reconstructed_and_selected_push_must_match():
    data=fixture();_,push,obs=fixture_observation(data)
    assert obs.source=='simulation' and obs.pose==(.35,-.18,0.) and push.as_dict()==data['push']
    data['push']['length_m']+=.001
    with pytest.raises(ValueError,match='does not match'):fixture_observation(data)


@pytest.mark.parametrize('key,value',[('hardware_connected',True),('execute_real',True),
    ('scenario','live'),('case','unknown')])
def test_live_unknown_or_mutated_fixtures_never_enter_gpu_diagnostic(key,value):
    data=fixture();data[key]=value
    with pytest.raises(ValueError):fixture_observation(data)
