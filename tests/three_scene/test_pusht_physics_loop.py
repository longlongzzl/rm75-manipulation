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
