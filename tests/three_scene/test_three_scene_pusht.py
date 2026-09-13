import time
import numpy as np
import pytest
from rm75_app.pusht.model import Config,valid_pose,choose_push,predict,error,wrap,candidates
from rm75_app.pusht.observation import Observation,JsonObserver
from rm75_app.pusht.controller import PushTController
from rm75_app.workcell.events import StopToken,EventLog,Cancelled
from rm75_app.workcell.io import atomic_json
from rm75_app.workcell.worker import run_pusht
@pytest.mark.parametrize('params',[{'horizon':0},{'max_steps':True},{'speed_mps':.5},{'workspace':[1,0,0,1]},{'stem_width_m':.4},{'friction_scales':[]},{'obstacles':[[0,0,-1]]},{'bogus':1}])
def test_config_rejects_unsafe_or_invalid(params):
    with pytest.raises(ValueError):Config.from_dict(params)
def test_t_candidates_have_bounded_futures():
    cfg=Config();pose=[.35,0,0];target=[.38,.02,.15];assert len(list(candidates(pose,cfg)))==360;push,report=choose_push(pose,target,cfg);assert push.length_m<=cfg.maximum_push_length_m and valid_pose(predict(pose,push,cfg),cfg) and report['prediction_is_observation'] is False and error(predict(pose,push,cfg),target,cfg)<error(pose,target,cfg)
def test_target_boundary_rejected():
    with pytest.raises(ValueError):choose_push([.35,0,0],[.65,0,0],Config())
def test_json_observer_cannot_use_file_mtime(tmp_path):
    path=tmp_path/'pose.json';stop=StopToken();atomic_json(path,Observation('s',1,time.time()-30,(.35,0,0),'live_tracker').as_dict())
    with pytest.raises(TimeoutError):JsonObserver(path,stop,timeout_s=.05).observe(after=time.time()-1)
def test_stagnation_is_not_success(tmp_path):
    class Frozen:
        n=0;now=1000.;executed=0
        def observe(self,after=0):
            self.n+=1;self.now=max(self.now,after)+.01
            return Observation('s',self.n,self.now,(.35,0,0),'live_tracker')
        def execute_push(self,*args):self.executed+=1
    env=Frozen()
    with pytest.raises(RuntimeError,match='stagnation'):PushTController(env,env,Config(stagnation_steps=2),StopToken(),EventLog(tmp_path),real=True,clock=lambda:env.now).run([.4,0,0])
    assert env.executed==2

@pytest.mark.parametrize('delay_point',['search','before_execution'])
def test_slow_planning_still_rejects_expired_observation(tmp_path,monkeypatch,delay_point):
    from rm75_app.pusht import controller
    now=[1000.];executed=[]
    class Frozen:
        def observe(self,after=0):
            now[0]=max(now[0],after)+.01
            return Observation('s',1,now[0],(.35,0,0),'live_tracker')
        def execute_push(self,*args):executed.append(True)
    original_rank=controller.rank_pushes
    def slow_rank(*args,**kwargs):
        result=original_rank(*args,**kwargs)
        if delay_point=='search':now[0]+=2.
        return result
    monkeypatch.setattr(controller,'rank_pushes',slow_rank)
    events=EventLog(tmp_path);original_emit=events.emit
    def emit(event,**values):
        result=original_emit(event,**values)
        if delay_point=='before_execution' and event=='push_planned':now[0]+=2.
        return result
    monkeypatch.setattr(events,'emit',emit)
    env=Frozen()
    with pytest.raises(ValueError,match='stale_or_future_observation'):
        PushTController(env,env,Config(),StopToken(),events,real=True,clock=lambda:now[0]).run([.4,0,0])
    assert not executed
@pytest.mark.parametrize('goal',[[.35,0,0],[.38,0,0],[.36,.02,.15]])
def test_surrogate_complete_controller(profile,tmp_path,goal):
    spec={'task':'pusht','mode':'sim','parameters':{'initial_pose':[.35,0,0],'goal_pose':goal,'speed_mps':.015,'max_steps':60}};result=run_pusht(spec,profile,StopToken(),EventLog(tmp_path));assert result['task_success'] is True and result['verification']=='surrogate_pose' and result['model_validated_on_robot'] is False
