import ast
from pathlib import Path
from types import SimpleNamespace as NS
import pytest

from rm75_app.pusht.controller import PushTController
from rm75_app.pusht.observation import Observation
from rm75_app.pusht.model import Config
from rm75_app.workcell.events import StopToken
from rm75_app.workcell.spec import validate_spec


@pytest.mark.parametrize('link,stage,forbidden',[
    ('finger','contact',False),('finger','push',False),('finger','retreat',False),
    ('finger','descend',True),('finger','approach',True),('link_5','push',True),
    ('gripper_base_link','push',True)])
def test_physical_target_contact_checks_whole_robot_not_only_tool(link,stage,forbidden):
    from rm75_app.pusht.physics_replay import forbidden_target_contact
    assert forbidden_target_contact(link,stage,{'finger'}) is forbidden


def test_physics_confirmation_advances_physics_clock_not_wall_sleep():
    class Physics:
        t=1.;n=0;waits=[]
        def now(self):return self.t
        def advance(self,seconds):self.waits.append(seconds);self.t+=seconds
        def observe(self,after=0):
            self.t+=1/30;self.n+=1
            return Observation('physics',self.n,self.t,(.35,-.18,0),'simulation')
        def stable(self):return True
        def execute_push(self,*a):pytest.fail('Already at target')
    physics=Physics();result=PushTController(physics,physics,Config(),StopToken(),NS(emit=lambda *a,**k:None),
        clock=physics.now,wait=physics.advance,verification='physics_pose').run([.35,-.18,0])
    assert result['verification']=='physics_pose' and result['success_observations']==3
    assert sum(physics.waits)==pytest.approx(.3) and physics.n==3


def test_physics_verification_cannot_be_used_for_real_mode():
    with pytest.raises(ValueError):PushTController(None,None,Config(),StopToken(),None,real=True,verification='physics_pose')


@pytest.mark.parametrize('backend',['tool_only_physics','full_arm_physics'])
def test_physics_backend_is_server_configured_and_never_a_real_parameter(profile,backend):
    spec=dict(task='pusht',mode='sim',parameters=dict(initial_pose=[.35,0,0],goal_pose=[.38,0,0],simulation_backend=backend))
    with pytest.raises(ValueError,match='not configured'):validate_spec(spec,profile)
    profile['pusht']['physics']={'enabled_backends':[backend]}
    assert validate_spec(spec,profile)['parameters']['simulation_backend']==backend
    spec['mode']='real'
    with pytest.raises(ValueError,match='never accepted'):validate_spec(spec,profile)


def test_full_arm_only_sets_qpos_at_episode_initialization_and_has_no_duplicate_tool():
    path=Path(__file__).resolve().parents[2]/'rm75_app/simulation/pusht_loop_env.py'
    tree=ast.parse(path.read_text());calls=[]
    for node in ast.walk(tree):
        if isinstance(node,ast.FunctionDef):
            for call in ast.walk(node):
                if isinstance(call,ast.Call) and ast.unparse(call.func).endswith('.set_qpos'):calls.append(node.name)
    assert calls==['_initialize_episode']
    assert 'loader.disable_self_collisions=False' in path.read_text()
    assert 'if not self.full_arm:return super()._load_tool' in path.read_text()
    assert 'set_drive_target' in path.read_text()


def test_physics_executor_uses_fresh_plan_not_saved_replay_or_hardware_execute():
    source=(Path(__file__).resolve().parents[2]/'rm75_app/pusht/physics.py').read_text()
    assert "op='plan'" in source and 'q=self.base.read_q()' in source
    assert "result['source_observation']!=obs.as_dict()" in source
    assert 'RealManArm' not in source and 'self.pose=predict' not in source


def test_interactive_extra_observe_measures_start_q_drift_without_widening_the_gate():
    """The interactive path observes once more before executing a cached plan.

    That observation advances the physics one control step, so the plan start q
    and the simulated q can differ. Both gates still compare against exactly
    1e-5 rad and additionally record the measured drift, so a real state change
    keeps failing the run instead of being absorbed by a wider tolerance.
    """
    source=(Path(__file__).resolve().parents[2]/'rm75_app/pusht/physics.py').read_text()
    tree=ast.parse(source)
    for name,field in (('prepare_push_candidates','last_planning_start_q_drift_rad'),
                       ('_execute_push','last_pre_execution_start_q_drift_rad')):
        fn=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name==name)
        compares=[n for n in ast.walk(fn) if isinstance(n,ast.Compare)
                  and 'drift' in ast.unparse(n.left) and any(isinstance(o,ast.Gt) for o in n.ops)]
        assert len(compares)==1, name
        assert ast.literal_eval(compares[0].comparators[0])==1e-5, name
        recorded=f"self.report['{field}'] = drift"
        assert recorded in ast.unparse(fn), name


def test_observation_evidence_streams_to_disk_and_rebuilds_the_same_array(tmp_path):
    import json
    from rm75_app.workcell.io import dumps
    from rm75_app.pusht.physics import write_observation_array
    rows = [dict(time_s=1.5, stage='push', pose=[.35, -.18, 0.], z_m=.02),
            dict(time_s=1.6, stage='push', pose=[.36, -.18, .01], z_m=.02)]
    stream = tmp_path / 'observations.jsonl'
    stream.write_text(''.join(dumps(row) + '\n' for row in rows))
    out = tmp_path / 'observations.json'
    write_observation_array(out, stream)
    # Byte-identical to the original one-shot dumps(rows) artifact.
    assert out.read_text() == dumps(rows) + '\n'
    assert json.loads(out.read_text()) == rows


def test_long_session_keeps_a_bounded_cache_and_streams_every_sample(tmp_path):
    import json
    from collections import deque
    from rm75_app.pusht import physics as module
    source = (Path(__file__).resolve().parents[2] / 'rm75_app/pusht/physics.py').read_text()
    assert 'deque(maxlen=OBSERVATION_CACHE)' in source
    log = (tmp_path / 'observations.jsonl').open('w', encoding='utf-8')
    session = NS(observations=deque(maxlen=4), observations_recorded=0, observation_log=log)
    for index in range(100):
        module.PhysicsSession.record_observation(session, dict(index=index))
    log.flush(); log.close()
    assert session.observations_recorded == 100
    assert [row['index'] for row in session.observations] == [96, 97, 98, 99]
    rows = [json.loads(line) for line in (tmp_path / 'observations.jsonl').read_text().splitlines()]
    assert [row['index'] for row in rows] == list(range(100))


def test_start_q_drift_still_raises_at_the_original_tolerance_with_measured_value():
    import numpy as np
    from types import SimpleNamespace
    from rm75_app.pusht.physics import PhysicsSession
    obs=Observation('simulation',1,1.,(.35,-.18,0),'simulation')

    class Base:
        def read_q(self):return np.full(7,2e-5)

    session=PhysicsSession.__new__(PhysicsSession)
    session.stop=StopToken();session.report={};session.full_arm=False;session.base=Base()
    session.clock=lambda:1.;session.config=Config()
    session._prepared_selection=(object(),obs.as_dict(),SimpleNamespace(initial=np.zeros(7)))
    with pytest.raises(ValueError,match='2.000e-05 rad > 1e-5 rad'):
        PhysicsSession.execute_push(session,session._prepared_selection[0],obs)
    assert session.report['last_pre_execution_start_q_drift_rad']==pytest.approx(2e-5)

@pytest.mark.parametrize('error',[RuntimeError('execution failed'),KeyboardInterrupt()])
def test_execution_wrapper_cancels_recording_and_preserves_failure(error):
    from rm75_app.pusht.physics import PhysicsSession
    session=PhysicsSession.__new__(PhysicsSession);calls=[]
    def fail(*args):raise error
    session._execute_push=fail
    session.action_recording=NS(cancel=lambda:calls.append('cancel'))
    with pytest.raises(type(error)) as raised:session.execute_push(None,None)
    assert raised.value is error and calls==['cancel']

def test_execution_wrapper_does_not_cancel_successful_recording():
    from rm75_app.pusht.physics import PhysicsSession
    session=PhysicsSession.__new__(PhysicsSession);calls=[];result=object()
    session._execute_push=lambda *args:result
    session.action_recording=NS(cancel=lambda:calls.append('cancel'))
    assert session.execute_push(None,None) is result
    assert calls==[]
