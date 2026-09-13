import copy
import pytest
from .conftest import pose
from .test_identification_program import transition,Replay,hypotheses
from rm75_app.swm.identification import AdaptivePhysicsManager,IsolatedReplayPool,infer_posterior
from rm75_app.swm.planning import ParallelHypothesisPlanner
from rm75_app.swm.native_skills import PushNativePhase
from rm75_app.swm.skills import SkillRequest,PlannedSkill
from rm75_app.swm.scene import digest,SceneInvalid


def build_managed_fixture(rig):
    data=transition(rig)
    _,rows=IsolatedReplayPool(lambda req:Replay(req,[])).run(data,hypotheses())
    belief=infer_posterior(data,hypotheses(),rows)
    rig.world.update_physics('a',belief,expected_physics_revision=0)
    events=[]
    manager=AdaptivePhysicsManager(rig.world,None,count=2,seed=12,emit=lambda **row:events.append(row))
    return manager,rig.world.snapshot(),events


def test_original_push_binding_passes_current_bank_into_solver_and_audits(rig):
    manager,snapshot,events=build_managed_fixture(rig);seen=[]
    class Solver:
        owns_real_executor=False
        def solve(self,request,snap,parameters):
            seen.append(('solve',copy.deepcopy(parameters)))
            return PlannedSkill(digest(request.as_dict()),snap['snapshot_id'],'candidate',{},pose(.35,z=.03),'fixture')
        def evaluate(self,plan,snap,parameters):
            seen.append(('evaluate',copy.deepcopy(parameters)))
            return dict(snapshot_id=snap['snapshot_id'],payload_digest=plan.payload_digest,feasible=True,cost=1.)
        def close(self):pass
    def unused_execution_boundary(*args):
        raise AssertionError('Planning must not audit for execution or execute a motion')
    phase=PushNativePhase(None,None,None,unused_execution_boundary,unused_execution_boundary,physics_manager=manager,
                          hypothesis_planner=ParallelHypothesisPlanner(Solver))
    plan=phase.binding().plan(SkillRequest('push','a',pose(.35,z=.03)),snapshot)
    bank=manager.planning_hypotheses('a',snapshot)['hypotheses']
    assert [p for kind,p in seen if kind=='solve']==[h['parameters'] for h in bank]
    assert [p for kind,p in seen if kind=='evaluate']==[h['parameters'] for h in bank]
    assert plan.payload_digest=='candidate' and events[-1]['physics_revision']==1
    assert events[-1]['posterior_transition_digest']==snapshot['physics']['a']['transition_digest']


def test_native_push_does_not_silently_ignore_physical_belief(rig):
    _,snapshot,_=build_managed_fixture(rig)
    with pytest.raises(SceneInvalid,match='SWM_PHYSICS_PLANNER_ADAPTER_REQUIRED'):
        PushNativePhase(None,None,None,None,None).plan(SkillRequest('push','a',pose(.35,z=.03)),snapshot)


def test_planning_bank_rejects_stale_checkpoint(rig):
    manager,snapshot,_=build_managed_fixture(rig)
    rig.world.invalidate('changed')
    with pytest.raises(SceneInvalid,match='latest valid checkpoint'):
        manager.planning_hypotheses('a',snapshot)
