from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest
from rm75_app.pusht.model import Config,Push,predict
from rm75_app.pusht.observation import Observation
from rm75_app.pusht.controller import PushTController
from rm75_app.pusht.session_control import SessionControl,SessionPolicy,response_is_plausible
from rm75_app.workcell.events import StopToken,Cancelled
from rm75_app.workcell.io import atomic_json,read_json


class Log:
    def __init__(self,directory=None):self.rows=[];self.directory=directory
    def emit(self,kind,**data):self.rows.append(dict(kind=kind,**data))


class Env:
    def __init__(self,target=(.38,0,0)):
        self.pose=(.35,0,0);self.target=target;self.t=1.;self.seq=0;self.moves=0;self.prepares=0
        self._prepared_selection=None
    def observe(self,after=0):
        self.t=max(self.t,after)+.02;self.seq+=1
        return Observation('same',self.seq,self.t,self.pose,'simulation')
    def stable(self):return True
    def wait(self,dt):self.t+=dt
    def clock(self):return self.t
    def prepare_push_candidates(self,proposals,obs,model=None):
        self.prepares+=1;self._prepared_selection='cached';return 0
    def execute_push(self,push,obs):
        self.moves+=1;self.t+=.1;self.pose=self.target


def fixed_rank(monkeypatch):
    import rm75_app.pusht.controller as m
    def rank(pose,target,cfg,**kwargs):
        from rm75_app.pusht.model import candidates
        p=next(candidates(pose,cfg))
        return [(p,{'predicted_cost':.01,'evaluated_friction_rollouts':1})]
    monkeypatch.setattr(m,'rank_pushes',rank)


def disable_fit(monkeypatch):
    from rm75_app.pusht.response import ResponseEstimator
    calls=[]
    monkeypatch.setattr(ResponseEstimator,'update',lambda self,*args:calls.append(args) or {'mock_fit':True})
    return calls


def test_goal_confirmed_from_fresh_observations(monkeypatch):
    fixed_rank(monkeypatch);disable_fit(monkeypatch);env=Env();log=Log();cfg=Config(max_steps=1)
    result=PushTController(env,env,cfg,StopToken(),log,clock=env.clock,wait=env.wait,verification='physics_pose').run(env.target)
    assert result['task_success'] and result['steps']==1 and result['verification']=='physics_pose'
    assert len([r for r in log.rows if r['kind']=='goal_confirmation'])==2


def test_keep_running_beyond_old_step_limit(monkeypatch):
    fixed_rank(monkeypatch);disable_fit(monkeypatch);env=Env();log=Log()
    def execute(push,obs):
        env.moves+=1;env.t+=.1;env.pose=(.35+.01*env.moves,0,0)
    env.execute_push=execute
    result=PushTController(env,env,Config(max_steps=1),StopToken(),log,clock=env.clock,wait=env.wait,
        verification='physics_pose',control_policy={'run_until_goal':True}).run(env.target)
    assert result['task_success'] and result['steps']==3


def test_mid_planning_movement_invalidates_without_execution(monkeypatch):
    fixed_rank(monkeypatch);disable_fit(monkeypatch);env=Env();log=Log()
    def prepare(proposals,obs,model=None):
        env.prepares+=1;env._prepared_selection='cached'
        if env.prepares==1:env.pose=(.32,.01,0)
        return 0
    env.prepare_push_candidates=prepare
    result=PushTController(env,env,Config(),StopToken(),log,clock=env.clock,wait=env.wait,
        verification='physics_pose',control_policy={'run_until_goal':True}).run(env.target)
    assert result['task_success'] and env.prepares==2 and env.moves==1
    assert len([r for r in log.rows if r['kind']=='push_plan_invalidated'])==1


def test_explicit_pause_resume_discard_fit_and_reobserve(tmp_path,monkeypatch):
    fixed_rank(monkeypatch);fits=disable_fit(monkeypatch);env=Env();log=Log(tmp_path)
    atomic_json(tmp_path/'session_commands/00000001.json',{'sequence':1,'action':'pause'})
    original_wait=env.wait;once=[]
    def wait(dt):
        original_wait(dt)
        if not once:
            assert read_json(tmp_path/'session_status.json')['safe_to_adjust']
            env.pose=(.31,0,0)
            atomic_json(tmp_path/'session_commands/00000002.json',{'sequence':2,'action':'resume'});once.append(1)
    env.wait=wait
    result=PushTController(env,env,Config(),StopToken(),log,clock=env.clock,wait=env.wait,
        verification='physics_pose',control_policy={'run_until_goal':True}).run(env.target)
    assert result['task_success'] and env.moves==1 and once
    assert any(r['kind']=='push_response_discarded' for r in log.rows)


def test_unknown_execution_error_never_auto_retried(monkeypatch):
    fixed_rank(monkeypatch);env=Env();log=Log()
    def execute(*args):env.moves+=1;raise RuntimeError('collision/SDK unknown fault')
    env.execute_push=execute
    with pytest.raises(RuntimeError,match='unknown fault'):
        PushTController(env,env,Config(),StopToken(),log,clock=env.clock,wait=env.wait,
            verification='physics_pose',control_policy={'run_until_goal':True}).run(env.target)
    assert env.moves==1


def test_stagnation_waits_without_repeated_planning_and_cancel_works(monkeypatch):
    fixed_rank(monkeypatch);disable_fit(monkeypatch);env=Env();log=Log();stop=StopToken()
    def execute(*args):env.moves+=1;env.t+=.1
    env.execute_push=execute
    original=env.wait
    def wait(dt):
        original(dt)
        if any(r.get('phase')=='waiting_for_scene_change' for r in log.rows):stop.request()
    with pytest.raises(Cancelled):
        PushTController(env,env,Config(stagnation_steps=2),stop,log,clock=env.clock,wait=wait,
            verification='physics_pose',control_policy={'run_until_goal':True}).run(env.target)
    assert env.moves==2 and env.prepares==2


def test_old_max_steps_still_fails_without_interactive(monkeypatch):
    fixed_rank(monkeypatch);disable_fit(monkeypatch);env=Env();env.execute_push=lambda *args:None
    with pytest.raises(RuntimeError,match='maximum_push_count'):
        PushTController(env,env,Config(max_steps=1),StopToken(),Log(),clock=env.clock,wait=env.wait,verification='physics_pose').run(env.target)


def test_session_sequence_and_state_cache(tmp_path):
    log=Log(tmp_path);c=SessionControl(log,{});executor=SimpleNamespace(_prepared_selection='cache')
    atomic_json(tmp_path/'session_commands/00000001.json',dict(sequence=1,action='pause'))
    assert c.poll(executor,verification='physics_pose')
    assert executor._prepared_selection is None and c.paused and c.epoch==1
    c.state('paused');count=len(log.rows);c.state('paused');assert len(log.rows)==count
    atomic_json(tmp_path/'session_commands/00000003.json',dict(sequence=3,action='resume'))
    with pytest.raises(ValueError,match='sequence gap'):c.commands()


@pytest.mark.parametrize('external,after,expected', [(True,(.36,0,0),False),(False,(.36,0,0),True),(False,(.55,0,0),False),(False,(.35,0,2),False)])
def test_response_filter_does_not_learn_large_intervention(external,after,expected):
    assert response_is_plausible((.35,0,0),Push((.34,0),(1,0),.012,.015),after,intervention=external)==expected


@pytest.mark.parametrize('value',[{'run_until_goal':'true'},{'max_wall_s':-1},{'position_replan_m':.01},{'yaw_replan_rad':.2},{'poll_s':0}])
def test_policy_limits(value):
    with pytest.raises(ValueError):SessionPolicy.from_dict(value)
