import copy
import json
from types import SimpleNamespace
import pytest
from rm75_app.workcell.spec import validate_spec
from rm75_app.workcell.iteration_workflows import prepare_request,completion_summary,ordered_sources
from rm75_app.workcell.worker import preview
from rm75_app.magnetic.generation import compile_selection
from rm75_app.workcell.io import digest


def ps(**params):return dict(task='pusht',mode='sim',parameters=dict(goal_pose=[.4,0,0],**params))


def test_sequence_to_native_order_preserves_profile(profile):
    before=copy.deepcopy(profile)
    spec=validate_spec(dict(task='pickplace',mode='sim',parameters=dict(object_names=['hongshupian','bi','tennis','shuazi'],automatic_order=True)),profile)
    effective,changed=prepare_request(spec,profile)
    assert effective['parameters']=={'object_name':'shuazi'}
    assert changed['pickplace']['frozen_source_order']==['shuazi','bi','tennis','hongshupian']
    assert profile==before
    assert preview(spec,profile)['source_order']==changed['pickplace']['frozen_source_order']


@pytest.mark.parametrize('names',[[],['bi','bi'],['invalid'],['bi','../../escape'],['bi',1],'bi'])
def test_sequence_validation(profile,names):
    with pytest.raises((ValueError,PermissionError)):
        validate_spec(dict(task='pickplace',mode='sim',parameters=dict(object_names=names)),profile)


@pytest.mark.parametrize('params',[{'object_name':'bi','object_names':['bi']},{'object_names':['bi'],'automatic_order':'true'},{'object_name':'bi','native_args':[]}])
def test_ambiguous_sequence_refused(profile,params):
    with pytest.raises(ValueError):validate_spec(dict(task='pickplace',mode='sim',parameters=params),profile)


def test_sequence_real_not_silently_enabled(profile):
    with pytest.raises(PermissionError):validate_spec(dict(task='pickplace',mode='real',parameters={'object_names':['bi']}),profile)


def test_original_single_unchanged(profile):
    spec={'task':'pickplace','mode':'sim','parameters':{'object_name':'bi'}}
    assert validate_spec(spec,profile)==spec


def test_partial_completion_is_not_green():
    result={'command_success':False,'native_full_world_validation':{'source_outcomes':[
        dict(source='bi',success=True,foreground=True),dict(source='tennis',success=True,foreground=False,prefetch_capture_only=True)]}}
    report=completion_summary(result,['bi','tennis'])
    assert report['completed_count']==1 and report['pending']==['tennis'] and not report['all_requested_verified']


def test_generation_bound_to_original_recipe(profile,library,proposal):
    r=compile_selection(library,proposal,board_id='grid_3x3')
    spec=validate_spec(dict(task='magnetic',mode='sim',parameters={'design':r['design'],'generation_proof':r['proof']}),profile)
    out,_=prepare_request(spec,profile)
    assert out['parameters']=={'design':r['design']}
    spec['parameters']['design']['pieces'][0]['center'][0]+=.01
    with pytest.raises(ValueError):validate_spec(spec,profile)


def test_push_new_contract_persists_typed_values(profile):
    raw=ps(geometry_id='wide',maximum_push_length_m=.05,run_until_goal=True,simulation_backend='full_arm_physics')
    value=validate_spec(raw,profile)
    assert value['parameters']['run_until_goal'] is True
    assert value['parameters']['geometry_id']=='wide'
    assert value['parameters']['simulation_backend']=='full_arm_physics'


@pytest.mark.parametrize('params',[{'run_until_goal':'yes'},{'run_until_goal':1},{'maximum_push_length_m':.1},{'geometry_id':'../../x'},{'geometry_id':'not_configured'},{'simulation_backend':'fake'},{'collision_enabled':False}])
def test_invalid_push_contract(profile,params):
    with pytest.raises((ValueError,PermissionError)):validate_spec(ps(**params),profile)


def test_real_interactive_refused(profile):
    spec=ps(run_until_goal=True);spec['mode']='real'
    with pytest.raises(PermissionError):validate_spec(spec,profile)


def test_no_new_flags_in_old_request(profile):
    result=validate_spec(ps(),profile)['parameters']
    assert set(result)=={'initial_pose','goal_pose','speed_mps','max_steps'}


def test_original_manifest_preserved_per_job_not_guessed(tmp_path,profile,library,proposal):
    from rm75_app.workcell.io import atomic_json,read_json
    t=library['boards']['grid_3x3']['templates'][0]
    fixed=tmp_path/'old_fixed.json';atomic_json(fixed,{'results':[{'fixture':True}]})
    import hashlib
    manifest={'schema':'jimu_task_manifest_v1','builder_scene_json':'old_builder.json',
        'builder':{'role_target_offsets_builder_m':{'wall_0_0':[.001,0,0]}},
        'tray':{'triangle_slot_indices':[9,10,11,12]},'custom_original_value':17}
    t['native_recipe']={'native_args':[],'task_manifest':manifest,'fixed_scene':str(fixed),
                        'fixed_scene_sha256':hashlib.sha256(fixed.read_bytes()).hexdigest()}
    atomic_json(profile['magnetic']['design_library'],library)
    profile['magnetic']['native_args']=['--joint-search-start-collision-lift-m','.103','--jimu-task-dir','old',
                                       '--jimu-triangle-tray-slot-indices','1','2']
    compiled=compile_selection(library,proposal,board_id='grid_3x3')
    spec={'task':'magnetic','mode':'sim','parameters':{'design':compiled['design'],'generation_proof':compiled['proof']}}
    _,p=prepare_request(spec,profile,tmp_path/'job')
    saved=read_json(tmp_path/'job/original_base_manifest.json')
    assert saved['builder']==manifest['builder'] and saved['tray']==manifest['tray']
    assert saved['custom_original_value']==17 and saved['builder_scene_json']=='builder_scene.json'
    assert '--jimu-triangle-tray-slot-indices' not in p['magnetic']['native_args']
    assert p['magnetic']['native_args'][:2]==['--joint-search-start-collision-lift-m','.103']
    assert p['magnetic']['native_args'][-2]=='--jimu-task-dir'
    assert profile['magnetic']['native_args'][-1]=='2'
