from types import SimpleNamespace as NS
import numpy as np
import pytest

from rm75_app.planning.contracts import BatchPlanningRequest,JointConfiguration,Pose,PoseCandidate
from rm75_app.pusht.cartesian_ik import plan_cartesian_line,PushPathRejected


def fixture(mode=None):
    names=('x','y','z','redundant');goals=[];checks=[]
    request=BatchPlanningRequest(JointConfiguration(names,[0,0,0,0]),
        (PoseCandidate('line',Pose([.012,.008,.004],[1,0,0,0])),))
    def solve(req):
        xyz=req.candidates[0].pose.position;goals.append(xyz.copy())
        if mode=='no_ik': return ()
        if mode=='native_error': raise RuntimeError('GPU failed')
        return (JointConfiguration(names,[*xyz,.001]),JointConfiguration(names,[*xyz,-.02]))
    backend=NS(tool_pose_for_configuration=lambda q,frame:Pose(q.positions[:3],[1,0,0,0]),
               solve_pose_ik_variants=solve)
    def validate(edge):
        checks.append(edge.copy())
        if mode=='all_rejected' or (mode=='retry' and edge[-1,-1]>0):
            raise PushPathRejected('collision at an interpolated point')
    return backend,request,validate,goals,checks


def test_slanted_line_keeps_all_requested_components_and_every_interpolated_point():
    b,r,v,goals,checks=fixture();path,records=plan_cartesian_line(b,r,v)
    expected=np.linspace([0,0,0],r.candidates[0].pose.position,4)[1:]
    np.testing.assert_allclose(goals,expected)
    np.testing.assert_allclose(path[-1,:3],r.candidates[0].pose.position)
    assert len(records)==len(checks)==3
    assert all(np.max(abs(np.diff(edge,axis=0)))<=.02+1e-12 for edge in checks)


def test_other_successful_ik_variants_remain_available_after_collision_rejection():
    b,r,v,goals,checks=fixture('retry');path,records=plan_cartesian_line(b,r,v)
    assert records[0]['selected_rank']==1
    assert records[0]['successful_ik_variants']==2
    assert records[0]['rejections'][0]['rank']==0
    assert path[-1,-1]<0


@pytest.mark.parametrize('mode',['no_ik','all_rejected'])
def test_no_partial_line_is_returned_when_any_waypoint_fails(mode):
    b,r,v,goals,checks=fixture(mode)
    with pytest.raises(PushPathRejected,match='waypoint=1/3'):
        plan_cartesian_line(b,r,v)


def test_native_errors_are_not_swallowed_as_candidate_rejections():
    b,r,v,goals,checks=fixture('native_error')
    with pytest.raises(RuntimeError,match='GPU failed'):
        plan_cartesian_line(b,r,v)
    assert not checks
