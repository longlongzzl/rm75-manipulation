"""Control-flow regression on the merged controller; no GPU or physics claims."""
from types import SimpleNamespace, ModuleType
import sys
import pytest
from rm75_app.pusht.controller import PushTController
from rm75_app.pusht.model import Config, Push
from rm75_app.pusht.observation import Observation


class Log:
    def __init__(self): self.rows=[]; self.directory=None
    def emit(self,kind,**data): self.rows.append(dict(kind=kind,**data))


class Environment:
    def __init__(self):
        self.pose=(.35,0.,0.); self.target=(.40,0.,0.); self.t=1.; self.seq=0
        self.executions=0; self.plans=[]; self._prepared_selection='sentinel'
        self.poses_after_push=None
    def observe(self,after=0.):
        self.t=max(self.t,after)+.01; self.seq+=1
        return Observation('capture',self.seq,self.t,self.pose,'simulation')
    def stable(self): return True
    def wait(self,seconds): self.t+=seconds
    def prepare_push_candidates(self,proposals,obs,model=None):
        assert obs.pose==self.pose, 'planner was given a pre-intervention observation'
        self.plans.append(obs)
        return 0
    def execute_push(self,push,obs):
        self.executions+=1; self.t+=.02
        self.pose=self.poses_after_push[self.executions-1] if self.poses_after_push else self.target


@pytest.fixture
def make(monkeypatch):
    import rm75_app.pusht.controller as module
    monkeypatch.setattr(module,'rank_pushes',lambda *a,**k:[(Push((.3,0.),(1.,0.),.02,.015),
        dict(predicted_cost=0.,evaluated_friction_rollouts=1))])
    fits=[];stub=ModuleType('rm75_app.pusht.response')
    class Estimator:
        def __init__(self,config): self.base=config
        def update(self,*args): fits.append(args);return {'test_response_fit':True}
        def config(self): return self.base
    stub.ResponseEstimator=Estimator
    monkeypatch.setitem(sys.modules,'rm75_app.pusht.response',stub)
    def build(**options):
        env=Environment(); log=Log();stop=SimpleNamespace(check=lambda:None,wait=lambda s:None)
        cfg=options.pop('config',Config(max_steps=8))
        controller=PushTController(env,env,cfg,stop,log,clock=lambda:env.t,wait=env.wait,
                                  verification='physics_pose',**options)
        return controller,env,log,fits
    return build


def test_goal_confirmation_intervention_restarts_before_any_plan(make,monkeypatch):
    c,e,log,_=make(control_policy={'run_until_goal':True});e.pose=e.target
    fired=[]
    def boundary():
        if not fired and e.t>1.1:
            fired.append(True);e.pose=(.32,.01,0.);c.control.epoch+=1;return True
        return False
    monkeypatch.setattr(c,'_pause_boundary',boundary)
    result=c.run(e.target)
    assert result['task_success'] and e.executions==1
    assert e.plans[0].pose==(.32,.01,0.)
    assert len([r for r in log.rows if r['kind']=='goal_confirmation'])==2


def test_stability_window_restarts_even_when_intervention_returns_same_pose(make,monkeypatch):
    c,e,_,_=make();initial=e.observe();calls=[]
    def boundary():
        calls.append(True)
        return len(calls)==3
    monkeypatch.setattr(c,'_pause_boundary',boundary)
    c._stable_observation(initial)
    assert len(calls)==5, 'three stable samples may not span an intervention'


def test_scripted_disturbance_resets_old_stagnation_and_epoch(make):
    fired=[]; box={}
    def disturb(step):
        if step==1 and not fired:
            fired.append(True);box['env'].pose=(.30,0.,0.);return True
        return False
    c,e,log,fits=make(control_policy={'run_until_goal':True},config=Config(stagnation_steps=2),disturb=disturb)
    box['env']=e;e.poses_after_push=[(.351,0.,0.),(.31,0.,0.),e.target]
    # Prevent an old controller waiting forever once its stale best score wins.
    def wait_fault(obs): raise AssertionError('stale progress history caused a false stagnation wait')
    c._wait_for_change=wait_fault
    result=c.run(e.target)
    assert result['task_success'] and result['steps']==3 and result['external_scene_epochs']>=1
    assert any(r['kind']=='response_fit_skipped' for r in log.rows)
    assert e.plans[1].pose==(.30,0.,0.)
    assert any(r.get('phase')=='waiting_for_stability' for r in log.rows)


def test_scripted_hook_is_not_a_real_mode_api():
    with pytest.raises(PermissionError,match='simulation-only'):
        PushTController(None,None,Config(),SimpleNamespace(wait=lambda s:None),Log(),real=True,disturb=lambda step:True)


def test_scripted_first_frame_intervention_clears_pending_plan_without_an_action(make):
    fired=[]
    c,e,log,_=make(disturb=lambda step:not fired and not fired.append(1))
    e.pose=e.target
    result=c.run(e.target)
    assert result['steps']==0 and c.control.epoch==1 and e._prepared_selection is None
    assert any(r.get('phase')=='waiting_for_stability' for r in log.rows)


def test_no_disturbance_does_not_reset_epoch(make):
    c,e,_,_=make();e.target=(.37,0.,0.);result=c.run(e.target)
    assert result['steps']==1 and result['external_scene_epochs']==0


def test_execution_fault_is_not_swallowed(make):
    c,e,_,_=make(control_policy={'run_until_goal':True})
    def fail(*a):e.executions+=1;raise RuntimeError('execution fault')
    e.execute_push=fail
    with pytest.raises(RuntimeError,match='execution fault'):c.run(e.target)
    assert e.executions==1


def test_normal_bounded_mode_still_stops(make):
    c,e,_,_=make(config=Config(max_steps=1));e.poses_after_push=[e.pose]
    with pytest.raises(RuntimeError,match='maximum_push_count'):c.run(e.target)
    assert e.executions==1


def test_replayed_observation_remains_a_failure(make):
    c,e,_,_=make();sample=e.observe()
    e.observe=lambda after=0.:sample
    with pytest.raises(ValueError):c.run(e.target)
