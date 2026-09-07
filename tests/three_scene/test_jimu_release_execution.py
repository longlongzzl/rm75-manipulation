import ast
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import threading
from types import SimpleNamespace as NS

import numpy as np
import pytest
import torch

from rm75_app.workcell.jimu_release_execution import NativePathParts,install_release_execution_guard
from rm75_app.workcell.pickplace_curobo_only import CuroboOnlyUnsupported
from rm75_app.workcell.jimu_execution_stages import guarded_stage,stage_kind


def native_concat():
    # Exercise the exact hash-verified native pure concatenation function.
    source=Path(__file__).resolve().parents[2]/'rm75_app/_vendor/working_snapshot/pick_jiaobang/rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place.py'
    node=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.FunctionDef)
              and n.name=='_concat_joint_paths')
    scope={'np':np};exec(compile(ast.Module(body=[node],type_ignores=[]),str(source),'exec'),scope)
    return scope['_concat_joint_paths']


def points(*values):return [np.full(7,value,dtype=np.float32) for value in values]


def test_native_concat_identity_values_duplicate_boundary_and_join_are_preserved():
    original=native_concat();returned=[]
    def concat(*paths):
        result=original(*paths);returned.append(result);return result
    direct=NS(_concat_joint_paths=concat);registry=NativePathParts(direct)
    left=points(0,.01,.01,.02);right=points(.02,.02,.03,.04)
    expected=original(left,right);actual=direct._concat_joint_paths(left,right)
    assert actual is returned[0] and len(returned)==1
    np.testing.assert_array_equal(actual,expected)
    clearance,returning=registry.split(actual)
    np.testing.assert_array_equal(clearance,points(0,.01,.02))
    np.testing.assert_array_equal(returning,points(.02,.03,.04))
    with pytest.raises(CuroboOnlyUnsupported,match='missing'):registry.split(actual)


@pytest.mark.parametrize('change',['copy','mutation','thread','evicted'])
def test_native_boundary_cannot_match_copies_mutations_other_threads_or_expired_records(change):
    direct=NS(_concat_joint_paths=native_concat());registry=NativePathParts(direct)
    actual=direct._concat_joint_paths(points(0,.02),points(.02,.04))
    if change=='copy':actual=[q.copy() for q in actual]
    if change=='mutation':actual[1][0]+=.001
    if change=='evicted':
        for _ in range(16):direct._concat_joint_paths(points(0,.02),points(.02,.04))
    with pytest.raises(CuroboOnlyUnsupported):
        if change=='thread':
            with ThreadPoolExecutor(1) as pool:pool.submit(registry.split,actual).result()
        else:registry.split(actual)


def fixture(mode='finger_target'):
    names={'arm':0,'gripper_Left_2_Link':1,'gripper_Right_2_Link':2,'gripper_base_link':3}
    config=NS(link_name_to_idx_map=names,link_sphere_idx_map=torch.tensor([0,1,2,3]))
    world=NS(enabled=True,forward=lambda spheres:spheres.clone())
    self_check=NS(enabled=True,forward=lambda spheres:spheres.clone())
    rollout=NS(kinematics=NS(kinematics_config=config),primitive_collision_cost=world,
               primitive_collision_constraint=world,robot_self_collision_constraint=self_check)
    owner=NS(use_cuda_graph=False,get_all_rollout_instances=lambda:[rollout],rollout_fn=rollout)
    planner=NS(motion_gen=owner,ik_solver=owner,_cuda_graph_batch_ik_solvers={},
        robot_cfg_dict={'kinematics':{'lock_joints':{'finger':.719}}},
        collision_enabled=True,config=NS(self_collision_check=True),attached_object_active=False,
        _disabled_collision_links=set(),_disabled_world_obstacles={'active_target_object'},
        _world=NS(objects=[NS(name=name) for name in ('virtual_table_plane','scene_obstacle_piece','neighbor')]))
    calls=[];spheres=torch.ones((1,1,4,4));original=spheres.clone()
    def check(q):
        filtered=world.forward(spheres)
        assert torch.equal(self_check.forward(spheres),original) and torch.equal(spheres,original)
        calls.append((float(q[0]),set(planner._disabled_world_obstacles),filtered[0,0,:,3].tolist()))
        contact=float(q[0])<.015
        if mode=='return_target':contact=float(q[0])>=.03
        if contact:
            if mode=='self':return False,'SELF_COLLISION'
            obj='virtual_table_plane' if mode=='table' else 'neighbor' if mode=='neighbor' else 'scene_obstacle_piece'
            link=0 if mode=='arm_target' else 3 if mode=='palm_target' else 1
            if obj not in planner._disabled_world_obstacles and filtered[0,0,link,3]>0:
                return False,'WORLD_COLLISION'
        return True,'VALID'
    def toggle(names,*,enabled):
        for name in names:
            if enabled:planner._disabled_world_obstacles.discard(name)
            else:planner._disabled_world_obstacles.add(name)
        return list(names)
    planner.check_start_state=check;planner.set_world_obstacles_enabled=toggle
    direct=NS(_concat_joint_paths=native_concat(),_CUROBO_GPU_LOCK=threading.RLock(),
              _current_source_object_name=lambda args:'piece')
    demo=NS(planner=NS(native=planner),active_joint_names=['finger'],robot=NS(get_qpos=lambda:np.array([.719])))
    demo.current_arm_qpos=lambda:np.zeros(7,dtype=np.float32)
    executed=[];sentinel=object()
    def execute(demo,bridge_mod,real_exec,label,pose,q_path,gripper_pos,args,*,use_attach=False):
        executed.append((q_path,gripper_pos,args));return sentinel
    portable=NS(direct=direct,_jimu_execute_pose_path_stage_base=execute)
    rows=install_release_execution_guard(portable,lambda row:None)
    path=direct._concat_joint_paths(points(0,.02),points(.02,.04))
    options=NS(_episode_place_released=True)
    def run(*,real_exec=None,label='post_place_clearance_return_to_cycle_start'):
        return portable._jimu_execute_pose_path_stage_base(demo,None,real_exec,label,None,path,.719,options)
    return planner,portable,demo,options,path,rows,executed,calls,sentinel,run


def test_release_pair_intersection_preserves_table_self_and_original_execution():
    planner,_,_,options,path,rows,executed,calls,sentinel,run=fixture()
    forward=planner.motion_gen.rollout_fn.primitive_collision_constraint.forward
    assert run() is sentinel and len(executed)==1 and executed[0][0] is path
    assert executed[0][1:]==(.719,options)
    row=rows[0]
    assert row['passed'] and row['state_unchanged'] and row['execution_guard']
    assert row['release_contact_samples']>0 and row['clearance_samples']>=3 and row['return_samples']>=3
    assert row['return_world_exempt_links']==[] and row['world_filter_calls']>0
    assert not row['self_collision_input_modified']
    assert planner._disabled_world_obstacles=={'active_target_object'}
    assert planner.motion_gen.rollout_fn.primitive_collision_constraint.forward is forward
    assert all('virtual_table_plane' not in disabled for _,disabled,_ in calls)
    assert all(radii==[1.,1.,1.,1.] for q,_,radii in calls if q>=.02)


@pytest.mark.parametrize('mode',['table','neighbor','self','arm_target','palm_target','return_target'])
def test_forbidden_contacts_never_execute_and_restore_collision_state(mode):
    planner,_,_,_,_,rows,executed,_,_,run=fixture(mode)
    forward=planner.motion_gen.rollout_fn.primitive_collision_constraint.forward
    with pytest.raises(CuroboOnlyUnsupported):run()
    assert executed==[] and rows[0]['passed'] is False and rows[0]['state_unchanged']
    assert planner._disabled_world_obstacles=={'active_target_object'}
    assert planner.motion_gen.rollout_fn.primitive_collision_constraint.forward is forward


@pytest.mark.parametrize('missing',['binding','released','gripper_sync','table','source','self_check','boundary'])
def test_missing_qualification_fails_closed(missing):
    planner,_,demo,options,path,rows,executed,_,_,run=fixture()
    if missing=='binding':demo.planner.native=None
    if missing=='released':options._episode_place_released=False
    if missing=='gripper_sync':demo.robot.get_qpos=lambda:np.array([.6])
    if missing=='table':planner._world.objects=[obj for obj in planner._world.objects if obj.name!='virtual_table_plane']
    if missing=='source':planner._world.objects=[obj for obj in planner._world.objects if obj.name!='scene_obstacle_piece']
    if missing=='self_check':planner.motion_gen.rollout_fn.robot_self_collision_constraint.enabled=False
    if missing=='boundary':path[1][0]+=.001
    with pytest.raises(CuroboOnlyUnsupported):run()
    assert executed==[] and not rows[0]['passed']


@pytest.mark.parametrize('mode',['real_flag','real_executor','prefetch','transport'])
def test_real_prefetch_and_unrelated_stage_do_not_inherit_release_exception(mode):
    _,_,_,options,_,rows,executed,_,sentinel,run=fixture()
    options.execute_real=mode=='real_flag';options._planning_prefetch_capture_only=mode=='prefetch'
    assert run(real_exec=object() if mode=='real_executor' else None,
               label='transport' if mode=='transport' else 'post_place_clearance_return_to_cycle_start') is sentinel
    assert rows==[] and len(executed)==1


@pytest.mark.parametrize('label',['return_to_cycle_start','post_place_direct_return_to_cycle_start'])
def test_independent_return_never_inherits_finger_released_target_exception(label):
    _,_,_,_,_,rows,executed,_,_,run=fixture('finger_target')
    with pytest.raises(CuroboOnlyUnsupported,match='return full-world collision'):
        run(label=label)
    assert not executed and not rows[0]['passed'] and rows[0]['state_unchanged']
    assert rows[0]['stage_kind']=='return_only'


def test_independent_return_audits_actual_entry_connector_without_rewriting_native_path():
    planner,p,demo,options,_,rows,executed,calls,sentinel,_=fixture()
    path=points(.02,.04);demo.current_arm_qpos=lambda:np.full(7,.018,dtype=np.float32)
    assert p._jimu_execute_pose_path_stage_base(demo,None,None,'return_to_cycle_start',None,path,.719,options) is sentinel
    row=rows[0]
    assert row['passed'] and row['entry_connector_audited']
    assert row['clearance_samples']==row['release_contact_samples']==row['world_filter_calls']==0
    assert row['permitted_links']==row['return_world_exempt_links']==[]
    assert row['permitted_contact_target'] is None
    assert executed[0][0] is path and calls[0][0]==pytest.approx(.018)
    assert all(disabled=={'active_target_object'} and radii==[1.]*4 for _,disabled,radii in calls)


def test_independent_return_rejects_collision_on_actual_entry_before_clear_path():
    _,p,demo,options,_,rows,executed,_,_,_=fixture()
    with pytest.raises(CuroboOnlyUnsupported,match='return full-world collision at 0'):
        p._jimu_execute_pose_path_stage_base(demo,None,None,'return_to_cycle_start',None,points(.02,.04),.719,options)
    assert not executed and not rows[0]['passed']


def test_original_return_prelift_retains_release_only_pair_policy():
    _,_,_,_,_,rows,executed,_,sentinel,run=fixture()
    assert run(label='return_to_cycle_start_prelift') is sentinel
    assert rows[0]['stage_kind']=='release_only' and rows[0]['release_contact_samples']>0
    assert rows[0]['return_samples']==0 and len(executed)==1


@pytest.mark.parametrize('label',['return_to_start_unknown','post_place_clearance_unknown','x_return_to_cycle_start'])
def test_unknown_return_labels_are_not_silent_bypasses(label):
    _,_,_,_,_,rows,executed,_,_,run=fixture()
    assert guarded_stage(label) and stage_kind(label) is None
    with pytest.raises(CuroboOnlyUnsupported,match='unknown Jimu'):
        run(label=label)
    assert not executed and not rows[0]['passed']


def native_pose_functions():
    source=Path(__file__).resolve().parents[2]/'rm75_app/_vendor/working_snapshot/Beta_demo-codex-v0.9/rm75_jimu_four_wall_portable.py'
    names={'_jimu_execute_pose_path_stage_base','execute_pose_path_stage_jimu'}
    nodes=[node for node in ast.parse(source.read_text()).body if isinstance(node,ast.FunctionDef) and node.name in names]
    assert len(nodes)==2
    return source,ast.Module(body=nodes,type_ignores=[])


def test_reviewed_joint_space_return_planner_really_dispatches_to_pose_executor():
    source=Path(__file__).resolve().parents[2]/'rm75_app/_vendor/working_snapshot/pick_jiaobang/rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place.py'
    tree=ast.parse(source.read_text())
    node=next(n for n in tree.body if isinstance(n,ast.FunctionDef)
              and n.name=='_plan_and_execute_return_to_cycle_start')
    assert any(isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='label' for t in n.targets)
               and isinstance(n.value,ast.Constant) and n.value.value=='return_to_cycle_start' for n in node.body)
    calls=[n for n in ast.walk(node) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)
           and n.func.attr in ('execute_pose_path_stage','execute_joint_path_stage')]
    assert len(calls)==2 and all(n.func.attr=='execute_pose_path_stage' for n in calls)
    assert any(isinstance(call.args[3],ast.Name) and call.args[3].id=='label' for call in calls)
    assert any(isinstance(call.args[3],ast.JoinedStr) and any(isinstance(value,ast.Constant)
               and value.value=='_prelift' for value in call.args[3].values) for call in calls)


@pytest.mark.parametrize('branch',['rendered','window','original'])
@pytest.mark.parametrize('blocked',[False,True])
def test_exact_native_pose_return_wrappers_sync_then_gate_before_each_sim_outlet(monkeypatch,branch,blocked):
    from rm75_app.workcell.jimu_return_diagnostics import install_release_execution_observer
    from rm75_app.workcell import pickplace_gripper_state
    planner,p,demo,options,_,_,_,_,_,_=fixture('return_target' if blocked else 'finger_target')
    demo.current_arm_qpos=lambda:np.full(7,.02,dtype=np.float32)
    gripper=[.6];demo.robot.get_qpos=lambda:np.array(gripper)
    planner.robot_cfg_dict['kinematics']['lock_joints']['finger']=.6
    order=[];outlets=[]
    def set_visual(actual,value,args,*,label):
        assert actual is demo
        gripper[0]=value;order.append('native_partial')
    def sync(native,actual,args):
        assert actual is demo and gripper[0]==.719
        planner.robot_cfg_dict['kinematics']['lock_joints']['finger']=gripper[0]
        order.append('model_sync')
    monkeypatch.setattr(pickplace_gripper_state,'update_from_demo',sync)
    def original(*args,**kwargs):
        outlets.append('original');return True,np.full(7,.04)
    def rendered(*args,**kwargs):outlets.append('rendered')
    def window(*args,**kwargs):outlets.append('window')
    scope={'np':np,'_ORIGINAL_EXECUTE_POSE_PATH_STAGE':original,
        '_jimu_should_use_partial_open_for_grasp':lambda *args:False,
        '_jimu_should_keep_partial_open_after_release':lambda *args:True,
        '_jimu_partial_release_gripper_value':lambda args:.719,
        '_jimu_partial_release_sim_gripper_value':lambda args:.719,
        '_jimu_skip_physics_gripper_sync':lambda args:True,
        '_jimu_set_sim_gripper_visual':set_visual,
        '_jimu_render_dry_run_motion_enabled':lambda *args:branch=='rendered',
        '_jimu_dry_run_motion_window_enabled':lambda *args:branch=='window',
        '_jimu_play_rendered_dry_run_motion':rendered,'_jimu_play_dry_run_motion_window':window,
        '_jimu_record_path_segment':lambda *args,**kwargs:None,'direct':p.direct}
    p.direct._record_profile=lambda *args,**kwargs:None
    p.direct.targeted=NS(base=NS(confirm_planned_motion_or_skip=lambda *args,**kwargs:True))
    path,tree=native_pose_functions();exec(compile(tree,str(path),'exec'),scope)
    p._jimu_execute_pose_path_stage_base=scope['_jimu_execute_pose_path_stage_base']
    rows=install_release_execution_guard(p,lambda row:order.append('gate'))
    install_release_execution_observer(p,lambda row:order.append('observer'),synchronize=True)
    scope['_jimu_execute_pose_path_stage_base']=p._jimu_execute_pose_path_stage_base
    def run():return scope['execute_pose_path_stage_jimu'](demo,None,None,'return_to_cycle_start',None,
        points(.02,.04),0.,options)
    if blocked:
        with pytest.raises(CuroboOnlyUnsupported):run()
        assert not outlets and not rows[0]['passed']
    else:
        ok,q=run();assert ok and outlets==[branch] and rows[0]['passed']
    assert order==['native_partial','model_sync','observer','gate']


def test_return_probe_uses_only_non_moving_sink_and_retains_original_path_and_world():
    from rm75_app.workcell.jimu_return_gate_probe import probe_return
    from rm75_app.workcell.jimu_return_diagnostics import _state
    planner,p,demo,options,_,_,executed,_,_,_=fixture()
    planner.robot_cfg=NS(kinematics=NS(kinematics_config=NS(joint_limits=NS(position=np.array([[-3.]*7,[3.]*7])))))
    path=np.array(points(.02,.04));copy=path.copy();before=_state(planner)
    row=probe_return(p.direct,demo,options,path)
    assert len(row['cases'])==3 and all(item['passed'] for item in row['cases'])
    assert [item['non_motion_sink_calls'] for item in row['cases']]==[1,1,0]
    assert row['cases'][-1]['rejected'] and row['actual_execute_calls']==0 and not executed
    assert _state(planner)==before
    np.testing.assert_array_equal(path,copy)


def test_return_probe_retains_partial_trials_if_no_gpu_collision_negative_found():
    from rm75_app.workcell.jimu_return_gate_probe import probe_return
    planner,p,demo,options,_,_,executed,_,_,_=fixture()
    planner.robot_cfg=NS(kinematics=NS(kinematics_config=NS(joint_limits=NS(position=np.array([[-3.]*7,[3.]*7])))))
    planner.check_start_state=lambda q:(True,'VALID')
    evidence={}
    with pytest.raises(CuroboOnlyUnsupported,match='no qualified GPU collision'):
        probe_return(p.direct,demo,options,np.array(points(.02,.04)),evidence=evidence)
    assert len(evidence['cases'])==2 and evidence['negative_state_queries']==12
    assert not executed and evidence['negative_native_status'] is None
