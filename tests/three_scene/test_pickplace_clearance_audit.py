from types import SimpleNamespace as NS
import numpy as np
from rm75_app.workcell.pickplace_clearance_audit import sphere_box_contacts
import pytest
import threading


def test_box_contact_reports_geometry_without_modification_or_filtering():
    spheres=np.array([[0,0,0,.01],[2,0,0,.01],[0,0,0,-100.]])
    original=spheres.copy()
    box=NS(name='table',dims=[1,1,1],pose=[0,0,0,1,0,0,0])
    contacts=sphere_box_contacts(spheres,['finger','arm','disabled'],[box],['table'])
    assert len(contacts)==1 and contacts[0]['link']=='finger'
    assert contacts[0]['overlap_m']==.51 and not contacts[0]['enabled']
    np.testing.assert_array_equal(spheres,original)


@pytest.mark.parametrize('fails',[False,True])
def test_diagnostic_restores_each_ablation_and_exception(fails):
    from rm75_app.workcell.pickplace_curobo_only import preserve_diagnostic_world_state
    class Planner:
        def __init__(self):self._disabled_world_obstacles={'target_cache'}
        def set_world_obstacles_enabled(self,names,*,enabled):
            if enabled:self._disabled_world_obstacles.difference_update(names)
            else:self._disabled_world_obstacles.update(names)
            return list(names)
        def diagnose_start_state_world_collision(self,q):
            self.set_world_obstacles_enabled(['target_cache'],enabled=False)
            self.set_world_obstacles_enabled(['target_cache'],enabled=True)
            assert self._disabled_world_obstacles=={'target_cache'}
            self.set_world_obstacles_enabled(['table'],enabled=False)
            if fails:raise ValueError('interrupted ablation')
            self.set_world_obstacles_enabled(['table'],enabled=True)
            return {'valid':False}
    preserve_diagnostic_world_state(Planner,threading.RLock())
    planner=Planner()
    if fails:
        with pytest.raises(ValueError):planner.diagnose_start_state_world_collision([])
    else:assert planner.diagnose_start_state_world_collision([])=={'valid':False}
    assert planner._disabled_world_obstacles=={'target_cache'}
    assert 'set_world_obstacles_enabled' not in vars(planner)


@pytest.mark.parametrize('problem',[None,'middle_collision','disabled_neighbor'])
def test_clearance_audit_checks_every_waypoint_without_world_exemptions(problem):
    from rm75_app.workcell.pickplace_clearance_audit import validate_clearance_path
    from rm75_app.workcell.pickplace_curobo_only import CuroboOnlyUnsupported
    checked=[]
    def check(q):
        checked.append(q)
        return not (problem=='middle_collision' and q==1),'WORLD_COLLISION'
    planner=NS(collision_enabled=True,config=NS(self_collision_check=True),
        attached_object_active=False,_disabled_collision_links=set(),
        _disabled_world_obstacles={'active_target_object'},check_start_state=check)
    if problem=='disabled_neighbor':planner._disabled_world_obstacles.add('neighbor')
    if problem:
        with pytest.raises(CuroboOnlyUnsupported):validate_clearance_path(planner,[0,1,2])
        assert checked==([0,1] if problem=='middle_collision' else [])
    else:
        assert validate_clearance_path(planner,[0,1,2])['samples']==3
        assert checked==[0,1,2]


@pytest.mark.parametrize('kind',['pose','joint'])
@pytest.mark.parametrize('collides',[False,True])
def test_actual_pose_and_joint_execution_boundaries_fail_before_forwarding(kind,collides):
    from rm75_app.workcell.pickplace_clearance_audit import install_execution_guards
    from rm75_app.workcell.pickplace_curobo_only import CuroboOnlyUnsupported
    calls=[]
    def pose(demo,bridge_mod,real_exec,label,pose,q_path,gripper_pos,args,**kwargs):
        calls.append(q_path);return True,q_path[-1]
    def joint(demo,bridge_mod,real_exec,label,q_path,gripper_pos,args,**kwargs):
        calls.append(q_path);return True,q_path[-1]
    base=NS(execute_pose_path_stage=pose,execute_joint_path_stage=joint)
    planner=NS(collision_enabled=True,config=NS(self_collision_check=True),
        attached_object_active=False,_disabled_collision_links=set(),
        _disabled_world_obstacles={'active_target_object'},
        check_start_state=lambda q:(not(collides and q==1),'WORLD_COLLISION'))
    demo=NS(planner=NS(native=planner));records=[]
    install_execution_guards(base,threading.RLock(),records)
    install_execution_guards(base,threading.RLock(),records)
    kwargs=dict(demo=demo,bridge_mod=None,real_exec=None,label='post_place_clearance',
                q_path=[0,1,2],gripper_pos=0,args=NS())
    if kind=='pose':kwargs['pose']=object()
    function=getattr(base,f'execute_{kind}_path_stage')
    if collides:
        with pytest.raises(CuroboOnlyUnsupported):function(**kwargs)
        assert not calls and not records
    else:
        assert function(**kwargs)==(True,2)
        assert calls==[[0,1,2]] and len(records)==1 and records[0]['samples']==3
