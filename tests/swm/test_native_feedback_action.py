"""Shared actual action identity lifecycle, no hardware or native simulation."""
from types import SimpleNamespace
import numpy as np
import pytest
from rm75_app.planning.contracts import JointTrajectory
from rm75_app.swm.native_skills import SharedPrimitiveExecutor, NativePrimitive, NativeStage
from rm75_app.swm.skills import PlannedSkill, PlanAudit, REQUIRED_AUDITS


@pytest.mark.parametrize('failure', [False, True])
def test_actual_action_binds_before_steps_and_releases_on_every_exit(failure):
    events = []
    class Sink:
        active = None
        def begin_feedback_action(self, aid):
            self.active = aid
            events.append(('begin',aid))
        def execute_trajectory(self, stage, path):
            assert self.active
            events.append(('step',self.active))
            if failure: raise RuntimeError('unknown execution')
        def end_feedback_action(self, aid):
            assert self.active == aid
            events.append(('end',aid))
            self.active = None
    sink = Sink()
    names = tuple(f'joint_{i}' for i in range(1,8))
    path = JointTrajectory(names,np.array([[0.]*7,[.01]*7]),dt=.1)
    payload = NativePrimitive('grasp','bi',(NativeStage('lift',path),))
    plan = PlannedSkill('request','snapshot',payload.fingerprint(),payload,np.eye(4),'fixture')
    runner = SharedPrimitiveExecutor(sink, lambda: dict(source='measured_feedback',idle=True,
        captured_at=1.,joint_names=names,positions=[.01]*7),clock=lambda:1.,
        stop=SimpleNamespace(check=lambda:None))
    if failure:
        with pytest.raises(RuntimeError,match='unknown execution'):
            runner(plan,PlanAudit(plan.payload_digest,'snapshot',REQUIRED_AUDITS))
    else:
        receipt = runner(plan,PlanAudit(plan.payload_digest,'snapshot',REQUIRED_AUDITS))
        assert receipt.actual_action_id == events[0][1]
    assert [row[0] for row in events] == ['begin','step','end']
    assert len({row[1] for row in events}) == 1 and sink.active is None
