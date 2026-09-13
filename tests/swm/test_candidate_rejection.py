"""Expected candidate rejection must not swallow corrupt evidence or cancellation."""
from contextlib import contextmanager
from types import SimpleNamespace
import sys
import numpy as np
import pytest
from rm75_app.swm.scene import SceneInvalid,digest
from rm75_app.swm.skills import SkillRequest,PlannedSkill
from rm75_app.swm.native_skills import NativePrimitive
from rm75_app.swm.native_push_hypotheses import NativePushHypothesisFactory
from rm75_app.swm.candidate_rejection import CandidateInfeasible
from rm75_app.swm.planning import ParallelHypothesisPlanner
from rm75_app.pusht.motion import PushCollisionRejected
from rm75_app.pusht.retreat_contact import RetreatContactRejected,validate_contact_escape
from rm75_app.pusht.cartesian_ik import PushPathRejected


@pytest.mark.parametrize('failure',[CandidateInfeasible('collision',stage='push',sample=3),
    SceneInvalid('wrong snapshot'),InterruptedError('cancel'),TimeoutError('budget'),ValueError('nonfinite')])
def test_native_context_only_recovers_explicit_infeasibility(tmp_path,monkeypatch,failure):
    from rm75_app.swm import future_push_audit
    urdf=tmp_path/'fixture.urdf';urdf.write_text('not loaded')
    owner=NativePushHypothesisFactory({}, {},urdf,python=sys.executable,directory=tmp_path)
    request=SkillRequest('push','a',target=np.eye(4));snapshot={'snapshot_id':'fixture'}
    primitive=NativePrimitive('push','a',())
    plan=PlannedSkill(digest(request.as_dict()),'fixture',primitive.fingerprint(),primitive,np.eye(4),'fixture')
    owner.request=request;owner.goal=[0,0,0];owner.candidates=[dict(primitive=primitive,motion={})]
    monkeypatch.setattr(owner,'_prepare',lambda *args:None)
    monkeypatch.setattr(owner,'_predict',lambda *args:{})
    exited=[]
    @contextmanager
    def executor(*args):
        try:yield SimpleNamespace()
        finally:exited.append(True)
    monkeypatch.setattr(owner,'_executor',executor)
    def reject(*args):raise failure
    monkeypatch.setattr(future_push_audit,'audit_future_push',reject)
    try:
        if isinstance(failure,CandidateInfeasible):
            result=owner().evaluate(plan,snapshot,{})
            assert result['feasible'] is False and result['rejection']['sample']==3
            assert not owner.closed and len(owner.candidates)==1
        else:
            with pytest.raises(type(failure)):owner().evaluate(plan,snapshot,{})
            assert owner.closed and not owner.directory.exists()
        assert exited==[True]
    finally:owner.close()


@pytest.mark.parametrize('all_reject',[False,True])
def test_one_bad_hypothesis_eliminates_candidate_not_search(all_reject):
    request=SkillRequest('push','a',target=np.eye(4));snapshot={'valid':True,'snapshot_id':'fixture'}
    plans=[PlannedSkill(digest(request.as_dict()),'fixture',name,object(),np.eye(4),'fixture') for name in ('A','B')]
    hypotheses=[dict(id=str(i),parameters=dict(static_friction=.3,dynamic_friction=.3,density_kg_m3=1000+i)) for i in range(2)]
    closed=[]
    class Solver:
        owns_real_executor=False
        def solve_candidates(self,*args):return plans
        def evaluate(self,plan,snap,parameters):
            rejected=all_reject or (plan.payload_digest=='A' and parameters['density_kg_m3']==1001)
            return dict(snapshot_id=snap['snapshot_id'],payload_digest=plan.payload_digest,
                feasible=not rejected,cost=None if rejected else (0 if plan.payload_digest=='A' else 1))
        def close(self):closed.append(True)
    planner=ParallelHypothesisPlanner(Solver,max_candidates=2)
    if all_reject:
        with pytest.raises(RuntimeError,match='No candidate survives'):planner.solve(request,snapshot,hypotheses)
    else:
        winner,evidence=planner.solve(request,snapshot,hypotheses)
        assert winner.payload_digest=='B' and evidence['results'][0]['feasible'] is False
    assert len(closed)==6


def test_corrupt_contact_depth_is_not_ordinary_infeasibility():
    row=dict(collision_type='world',robot_link='finger',world_object='table',penetration_m=float('nan'))
    with pytest.raises(ValueError):PushCollisionRejected([row])
    with pytest.raises(PushPathRejected) as error:validate_contact_escape([[row],[]],{'finger'})
    assert not isinstance(error.value,RetreatContactRejected)
    row['penetration_m']=.001
    with pytest.raises(RetreatContactRejected):validate_contact_escape([[row],[]],{'finger'})
