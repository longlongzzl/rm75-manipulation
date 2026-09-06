import copy,math,time
import numpy as np
import pytest
from rm75_app.workcell.io import loads,dumps,atomic_json,read_json,contained
from rm75_app.workcell.transforms import rigid,quaternion_matrix,matrix_quaternion
from rm75_app.magnetic.design import validate_design,world_targets,piece_matrix
from rm75_app.pusht.observation import Observation
from rm75_app.workcell.spec import validate_spec

@pytest.mark.parametrize('text',['{"x":NaN}','{"x":Infinity}','{"x":1,"x":2}'])
def test_strict_json(text):
    with pytest.raises(ValueError):loads(text)

def test_atomic_persistence_and_no_traversal(tmp_path):
    atomic_json(tmp_path/'p.json',{'中文':[1,2]})
    assert read_json(tmp_path/'p.json')=={'中文':[1,2]}
    with pytest.raises(ValueError):contained(tmp_path/'../elsewhere',tmp_path)

def test_existing_magnetic_format_roundtrip(design):
    original=copy.deepcopy(design)
    result=validate_design(design)
    assert result.payload==original and design==original
    assert result.ordered_roles==('right_wall',)
    assert np.allclose(result.targets_builder['right_wall'][:3,:3],np.column_stack([design['pieces'][1][k] for k in ('u','n','v')]))

def test_floor_half_thickness_not_added_twice(design):
    d=validate_design(design);targets=world_targets(d,np.eye(4))
    assert np.isclose(targets['floor'][1,3],0)
    assert np.isclose(targets['right_wall'][1,3],.0435-.00325)

@pytest.mark.parametrize('fault',['reflection','zero_axis','cycle','missing_parent','coincident','nan','relative'])
def test_magnetic_invalid_design_rejected(design,fault):
    p=design['pieces'][1]
    if fault=='reflection':p['v']=[0,-1,0]
    if fault=='zero_axis':p['u']=[0,0,0]
    if fault=='cycle':design['pieces'][0]['parentRole']='right_wall'
    if fault=='missing_parent':p['parentRole']='no_such_parent'
    if fault=='coincident':p['center']=design['pieces'][0]['center'][:]
    if fault=='nan':p['center'][0]=float('nan')
    if fault=='relative':p['parentRelativeTransform']=np.eye(4).tolist()
    with pytest.raises(ValueError):validate_design(design)

def test_locked_pieces_do_not_consume_12_movable_limit(design):
    for i in range(1,12):
        p=copy.deepcopy(design['pieces'][1]);p['role']=p['id']=f'wall_{i}';p['center'][1]+=.08*i
        design['pieces'].append(p)
    assert len(validate_design(design).ordered_roles)==12
    p=copy.deepcopy(design['pieces'][-1]);p['role']=p['id']='excess';p['center'][1]+=.08
    design['pieces'].append(p)
    with pytest.raises(ValueError):validate_design(design)

@pytest.mark.parametrize('angle',[0,.1,math.pi/2,math.pi,math.pi-.00001])
def test_quaternion_matrix_stable(angle):
    r=quaternion_matrix([math.cos(angle/2),0,math.sin(angle/2),0])
    assert np.allclose(quaternion_matrix(matrix_quaternion(r)),r,atol=1e-8)

def observation(**kw):
    args=dict(session_id='session',sequence=1,captured_at=100,pose=(.35,0,0),source='live_tracker')
    args.update(kw);return Observation(**args)

@pytest.mark.parametrize('kwargs',[{'captured_at':98},{'captured_at':101},{'source':'surrogate'},
                                  {'confidence':.7},{'frame':'camera'},{'sequence':0}])
def test_real_observation_rejects_stale_wrong_source_or_replay(kwargs):
    with pytest.raises(ValueError):observation(**kwargs).validate(now=100.1,after=99,previous=observation(sequence=0,captured_at=99),real=True)

def test_live_observation_success_and_session_change():
    observation().validate(now=100.1,after=99,previous=observation(sequence=0),real=True)
    with pytest.raises(ValueError):observation(session_id='new').validate(now=100.1,previous=observation(sequence=0),real=True)

def test_task_contract_rejects_freeform_commands(profile):
    with pytest.raises(ValueError):validate_spec({'task':'pickplace','mode':'real','parameters':{'object_name':'bi','argv':['--execute-real']}},profile)
    with pytest.raises(ValueError):validate_spec({'task':'pusht','mode':'real','parameters':{'goal_pose':[.4,0,0],'speed_mps':.04}},profile)

@pytest.mark.parametrize('task',['pickplace','magnetic','pusht'])
def test_three_preview_specs(profile,design,task):
    params={'pickplace':{'object_name':'carriot'},'magnetic':{'design':design},'pusht':{'goal_pose':[.38,0,0]}}[task]
    assert validate_spec({'task':task,'mode':'preview','parameters':params},profile)['task']==task
