from copy import deepcopy
from pathlib import Path
import ast

import numpy as np
import pytest

from rm75_app.pusht.physics_replay import TcpFK, TimedProgram, STAGES, audit_endpoints, measured_outcome
from rm75_app.pusht.model import Config, Push


def package():
    return dict(complete_chain=True, validation_success=True, execute_real=False,
        hardware_connected=False, hardware_profile_qualified=False,
        stages=[dict(stage=name, positions=[[i]*7, [i+1]*7], times=[0., 1.]) for i, name in enumerate(STAGES)])


def test_all_original_stages_and_timing_are_preserved_without_teleports():
    data=package(); before=deepcopy(data); program=TimedProgram(data, lambda q:q)
    assert program.duration==5 and data==before
    stage,q=program.sample(2.5)
    assert stage=='contact'; np.testing.assert_allclose(q,[2.5]*7)
    np.testing.assert_allclose(program.sample(-1)[1],[0]*7)
    np.testing.assert_allclose(program.sample(99)[1],[5]*7)


@pytest.mark.parametrize('field',['complete_chain','validation_success','execute_real','hardware_connected','hardware_profile_qualified'])
def test_missing_or_hardware_source_never_runs_physics(field):
    data=package();data[field]=not data[field]
    with pytest.raises(ValueError,match='no-hardware'):TimedProgram(data,lambda q:q)


@pytest.mark.parametrize('error',['order','nan','times','jump'])
def test_original_trajectory_corruption_is_rejected(error):
    data=package()
    if error=='order':data['stages'].reverse()
    if error=='nan':data['stages'][0]['positions'][0][0]=float('nan')
    if error=='times':data['stages'][0]['times']=[0,0]
    if error=='jump':data['stages'][1]['positions'][0][0]+=.01
    with pytest.raises(ValueError):TimedProgram(data,lambda q:q)


def test_tcp_fk_composes_original_joint_origins_in_correct_order(tmp_path):
    parts=[];parent='base_link'
    for i in range(1,8):
        child=f'link_{i}'
        parts.append(f'<joint name="joint_{i}" type="revolute"><parent link="{parent}"/><child link="{child}"/><origin xyz="1 0 0"/><axis xyz="0 0 1"/></joint>')
        parent=child
    parts.append('<joint name="tcp" type="fixed"><parent link="link_7"/><child link="gripper_tcp"/></joint>')
    path=tmp_path/'unit.urdf';path.write_text('<robot>'+''.join(parts)+'</robot>')
    fk=TcpFK(path);np.testing.assert_allclose(fk(np.zeros(7))[:3,3],[7,0,0])
    q=np.zeros(7);q[0]=np.pi/2
    np.testing.assert_allclose(fk(q)[:3,3],[1,6,0],atol=1e-12)
    with pytest.raises(ValueError):fk([0]*6)


def test_short_forward_displacement_does_not_claim_three_cm_goal_reached():
    result=measured_outcome([.35,-.18,0],[.359,-.18,.01],Push((.3,-.18),(1.,0),.012,.015),(.38,-.18,0),Config())
    assert result['forward_displacement_m']==pytest.approx(.009)
    assert not result['goal_reached'] and result['final_goal_error_m']<result['initial_goal_error_m']
    assert result['final_goal_position_error_m']==pytest.approx(.021)
    assert result['final_goal_yaw_error_rad']==pytest.approx(.01)


@pytest.mark.parametrize('corruption', ['position', 'orientation', 'order'])
def test_endpoint_audit_rejects_changed_pose_or_order(corruption):
    transform=np.eye(4)
    program=TimedProgram(package(),lambda q:transform)
    points=[(name,np.zeros(3),False,False) for name in STAGES]
    assert len(audit_endpoints(program,points,np.eye(3),.003,.05))==5
    if corruption=='position':transform[0,3]=.004
    if corruption=='orientation':transform[:3,:3]=np.diag([-1.,-1.,1.])
    if corruption=='order':points.reverse()
    with pytest.raises(ValueError):audit_endpoints(program,points,np.eye(3),.003,.05)


def test_short_push_cannot_pass_goal_with_good_position_but_bad_yaw():
    result=measured_outcome([.35,-.18,0],[.38,-.18,.15],Push((.3,-.18),(1.,0),.012,.015),(.38,-.18,0),Config())
    assert result['final_goal_position_error_m']==0
    assert not result['goal_reached']


def test_no_robot_state_override_preserves_physical_actors_without_controller():
    source=(Path(__file__).resolve().parents[2]/'rm75_app/simulation/pusht_contact_env.py').read_text()
    tree=ast.parse(source)
    node=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='get_state_dict')
    method=ast.unparse(node)
    assert 'self.scene.get_sim_state()' in method and 'agent' not in method


def test_target_is_only_initialized_never_teleported_during_physics_steps():
    path=Path(__file__).resolve().parents[2]/'rm75_app/simulation/pusht_contact_env.py'
    tree=ast.parse(path.read_text())
    calls=[]
    for node in ast.walk(tree):
        if isinstance(node,ast.FunctionDef):
            for call in ast.walk(node):
                if isinstance(call,ast.Call) and ast.unparse(call.func)=='self.target.set_pose':calls.append(node.name)
    assert calls==['_initialize_episode']
    assert 'set_kinematic_target' in path.read_text()
    assert 'set_collision_group' not in path.read_text()
