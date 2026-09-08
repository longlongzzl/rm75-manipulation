"""Motion bridge contract tests with a Cartesian fake, NOT GPU/physics evidence."""
from dataclasses import replace
from types import SimpleNamespace as NS
import time
import numpy as np
import pytest
import torch
from rm75_app.pusht.motion import CuroboPushExecutor
from rm75_app.pusht.model import Config,Push
from rm75_app.pusht.observation import Observation
from rm75_app.planning.contracts import Pose,JointConfiguration


def fixture(failure=None):
    trace=[]
    q0=np.array([.25,0,.18,0,0,0,0.])
    class Arm:
        hz=10;start_gap=.05
        def read_joints(self): return q0.copy()
        def execute(self,path,ts,stage): trace.append(('execute',stage))
    class Backend:
        def __init__(self):
            self.enabled={'pusht_target_0':True,'pusht_target_1':True}
            self.stage=None
            self.observed_contact_switches=[]
        def set_gripper_collision_state(self,closed):
            assert closed is True
            trace.append(('closed_model',))
        def closed_gripper_tool_geometry(self,q):
            from rm75_app.pusht.closed_gripper import ToolGeometry
            # Cartesian test double only, NOT the native RM75 geometry.
            return ToolGeometry(np.array([[0.,0.,0.,.005]]),('pusher',))
        def update_scene(self,scene): self.scene=scene
        def _ensure_planner(self):
            return NS(joint_names=[f'joint_{i}' for i in range(1,8)],device_cfg=NS(device='cpu',dtype=torch.float32))
        def _import_modules(self):
            return {'torch':torch,'JointState':NS(from_position=lambda p,**kw:p)}
        def _obstacle_enabled(self,name): return self.enabled[name]
        def _set_obstacle_enabled(self,name,value): self.enabled[name]=value
        def _collision_diagnostics_for_states(self,p,states):
            rows=[]
            if self.stage=='push':
                rows=[{'collision_type':'world','robot_link':'pusher','world_object':'pusht_target_0'}]
                if failure=='collision': rows.append({'collision_type':'world','robot_link':'arm','world_object':'table'})
                if failure=='self': rows.append({'collision_type':'self','robot_link':'pusher','other_robot_link':'arm'})
                if failure=='neighbor': rows.append({'collision_type':'world','robot_link':'pusher','world_object':'neighbor'})
            trace.append(('audit',self.stage,len(states['path'])))
            return rows
        def tool_pose_for_configuration(self,q,tool): return Pose(q.positions[:3],[1,0,0,0])
        def plan_candidates(self,request):
            self.stage=request.candidates[0].candidate_id.split(':')[1]
            self.observed_contact_switches.append((self.stage,self.enabled.copy()))
            trace.append(('plan',self.stage))
            if failure==self.stage: raise RuntimeError('solver failed')
            end=np.array(request.current.positions);end[:3]=request.candidates[0].pose.position
            path=np.linspace(request.current.positions,end,3)
            if failure=='corridor' and self.stage=='push': path[1,1]+=.02
            return NS(best=lambda cs:NS(trajectory=NS(positions=path)))
        def plan_linear_candidates(self,request,**kwargs):
            trace.append(('linear',request.candidates[0].candidate_id,kwargs))
            return self.plan_candidates(request)
        def solve_pose_ik_variants(self,request):
            trace.append(('ik',request.candidates[0].candidate_id,request.candidates[0].pose.position.copy()))
            result=self.plan_candidates(request).best(request.candidates)
            q=result.trajectory.positions[-1].copy()
            if failure=='corridor' and self.stage=='push': q[1]+=.02
            return (JointConfiguration(request.current.names,q),)
    class Observer:
        def observe(self,after):
            trace.append(('observe',))
            if failure=='stale': return obs
            return replace(obs,sequence=2,captured_at=obs.captured_at-.1 if failure=='time_reverse' else time.time(),
                pose=(.4,0,0) if failure=='drift' else obs.pose)
    backend=Backend();arm=Arm()
    motion={'tool_frame':'gripper_tcp','push_tcp_z_m':.1,'hover_clearance_m':.08,
        'tool_quaternion_wxyz':[1,0,0,0],'pusher_contact_links':['pusher'],
        'tool_collision_geometry_verified':True,'closed_gripper_verified':True,
        'object_centroid_z_m':.1,'object_height_m':.02,
        'static_collision_objects':[{'name':'table','kind':'cuboid','position':[.35,0,0],
            'quaternion_wxyz':[1,0,0,0],'dimensions':[.8,.8,.02]}]}
    stop=NS(check=lambda:None);events=NS(emit=lambda *a,**kw:trace.append(('event',a[0])))
    executor=CuroboPushExecutor(backend,arm,Config(),motion,stop,events,Observer())
    # Put the actual exposed left crossbar contact at y=0 and x=.30.
    obs=Observation('test',1,time.time(),(.35,-.02058823529411765,0),'live_tracker')
    push=Push((.295,0),(1,0),.012,.015)
    return executor,push,obs,trace,backend


def test_plan_only_has_complete_chain_and_never_observes_or_executes():
    e,p,o,t,b=fixture();prepared=e.plan_push(p,o)
    assert [s for s,_,_ in prepared.stages]==['approach','descend','contact','push','retreat']
    assert not any(x[0] in ('execute','observe') for x in t)
    assert all(b.enabled.values())
    for stage,states in b.observed_contact_switches:
        assert all(states.values()) is (stage in ('approach','descend'))


def test_real_source_requires_actual_closed_state_review_before_model_or_motion():
    executor,push,obs,trace,_=fixture()
    executor.profile.pop('closed_gripper_verified')
    with pytest.raises(PermissionError,match='Actual closed'):
        executor.plan_push(push,obs)
    assert not any(row[0] in ('execute','closed_model') for row in trace)


def test_simulation_label_cannot_bypass_actual_closed_state_execution_gate():
    executor,push,obs,trace,_=fixture()
    executor.profile.pop('closed_gripper_verified')
    with pytest.raises(PermissionError,match='before execution'):
        executor.execute_push(push,replace(obs,source='simulation'))
    assert not any(row[0] in ('execute','plan','closed_model') for row in trace)


def test_execution_waits_for_entire_audited_chain_and_new_observation():
    e,p,o,t,b=fixture();e.execute_push(p,o)
    first=next(i for i,r in enumerate(t) if r[0]=='execute')
    assert list(dict.fromkeys(r[1] for r in t[:first] if r[0]=='plan'))==['approach','descend','contact','push','retreat']
    assert ('observe',) in t[:first]
    assert [r[1] for r in t if r[0]=='execute']==['approach','descend','contact','push','retreat']


@pytest.mark.parametrize('failure',['approach','descend','contact','push','retreat','collision','self','neighbor','corridor','stale','drift','time_reverse'])
def test_any_chain_or_freshness_failure_prevents_all_motion_and_restores_world(failure):
    e,p,o,t,b=fixture(failure)
    with pytest.raises((RuntimeError,ValueError)): e.execute_push(p,o)
    assert not any(r[0]=='execute' for r in t)
    assert all(b.enabled.values())


def test_speed_changes_prepared_time_and_cartesian_bound():
    e,p,o,_,_=fixture();fast=e.plan_push(p,o)
    e,p,o,_,_=fixture();slow=e.plan_push(replace(p,speed_mps=.005),o)
    assert sum(ts[-1] for _,_,ts in slow.stages)>sum(ts[-1] for _,_,ts in fast.stages)
    for _,path,ts in slow.stages:
        assert np.max(np.linalg.norm(np.diff(path[:,:3],axis=0),axis=1)/np.diff(ts))<=.005+1e-9


def test_contact_moves_use_endpoint_ik_without_changing_requested_line():
    e,p,o,t,b=fixture();e.plan_push(p,o)
    rows=[r for r in t if r[0]=='ik']
    assert list(dict.fromkeys(r[1].split(':')[1] for r in rows))==['descend','contact','push','retreat']
    assert not any(r[0]=='linear' for r in t)
    assert all(abs(r[2][1])<1e-12 for r in rows)


def test_time_resampled_path_also_requires_cartesian_corridor(monkeypatch):
    import rm75_app.pusht.motion as motion
    original=motion.time_parameterize
    e,p,o,t,b=fixture()
    def bend(*args,**kwargs):
        path,ts=original(*args,**kwargs)
        if b.stage=='descend': path[len(path)//2,1]+=.02
        return path,ts
    monkeypatch.setattr(motion,'time_parameterize',bend)
    with pytest.raises(RuntimeError,match='non_cartesian_contact_path:descend'):
        e.execute_push(p,o)
    assert not any(r[0]=='execute' for r in t)
