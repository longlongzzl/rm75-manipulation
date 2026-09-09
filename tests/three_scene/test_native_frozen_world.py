import copy
import json
import threading
from types import SimpleNamespace as NS

import pytest

from rm75_app.workcell.native_frozen_world import FrozenWorldValidation, read_contract
from rm75_app.workcell.pickplace_world_coverage import coverage_row, FrozenWorldIncomplete


POSE=[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]


def scene_file(tmp_path, data=None):
    path=tmp_path/'world.json'
    data=data if data is not None else {'objects':{n:{'T_world_obj':POSE} for n in ('source','desk','other')}}
    path.write_text(json.dumps(data));return path


def validator(tmp_path):
    v=FrozenWorldValidation(read_contract(scene_file(tmp_path),'source'),tmp_path,NS(emit=lambda *a,**k:None))
    p=NS(_world=NS(objects=[NS(name='scene_obstacle_'+n) for n in ('desk','other')]),
         _disabled_world_obstacles=set(),_disabled_collision_links=set(),
         collision_enabled=True,config=NS(self_collision_check=True))
    v.worlds=[coverage_row(p,scene_names=v.contract['names'],source='source',label='transport')]
    v.outcomes=[dict(source='source',success=True,foreground=True,prefetch_capture_only=False)]
    return v


def result(v, outcome=None, clearance=None):
    return v.result(outcome if outcome is not None else {'native_full_chain_passed':True},
        clearance if clearance is not None else [dict(all_valid=True,samples=5)])


def test_contract_preserves_file_bytes_and_all_object_names(tmp_path):
    path=scene_file(tmp_path);before=path.read_bytes();contract=read_contract(path,'source')
    assert path.read_bytes()==before and contract['names']==('desk','other','source')
    assert contract['source']=='source' and contract['path']==path


@pytest.mark.parametrize('order',[[],['source'],['other','source'],['source','missing'],['source','source']])
def test_frozen_sequence_rejects_wrong_identity_or_incomplete_order(tmp_path,order):
    with pytest.raises(ValueError):read_contract(scene_file(tmp_path),'source',order)


def test_frozen_sequence_requires_every_requested_source_not_one_success(tmp_path):
    v=validator(tmp_path)
    v.contract['source_order']=('source','other')
    assert not result(v)['command_success']
    v.outcomes.append(dict(source='other',success=True,foreground=True,prefetch_capture_only=False))
    v.worlds.append(dict(v.worlds[0],source='other'))
    assert result(v,clearance=[dict(all_valid=True,samples=5)]*2)['command_success']


@pytest.mark.parametrize('objects',[{},[],{'other':{'T_world_obj':POSE}},
    {'source':None},{'source':{'T_world_obj':[[1,2,3,4]]}},
    {'source':{'T_world_obj':[[True,0,0,0]]*4}},
    {'source':{'T_world_obj':[[float('inf'),0,0,0]]*4}},
    {'source':{'T_world_obj':POSE},'--execute-real':{'T_world_obj':POSE}},
    {'source':{'T_world_obj':POSE},'bad name':{'T_world_obj':POSE}}])
def test_invalid_or_missing_world_input_fails_closed(tmp_path,objects):
    with pytest.raises(ValueError):read_contract(scene_file(tmp_path,{'objects':objects}),'source')


def test_duplicate_json_keys_are_not_silently_replaced(tmp_path):
    path=tmp_path/'world.json';path.write_text('{"objects":{},"objects":{}}')
    with pytest.raises(ValueError,match='Duplicate'):read_contract(path,'source')


def test_positive_is_sim_evidence_never_physical_success(tmp_path):
    v=validator(tmp_path);r=result(v)
    assert r['command_success'] and r['task_success'] is None
    assert r['verification']=='frozen_world_sim_only'
    assert not r['frozen_world_validation']['physical_geometry_qualified']


@pytest.mark.parametrize('problem',['empty_world','missing_object','other_source','prefetch_only',
                                    'no_clearance','invalid_clearance','zero_samples','native_failed'])
def test_each_gate_is_required_even_when_native_reports_final_success(tmp_path,problem):
    v=validator(tmp_path);outcome={'native_full_chain_passed':True};clearance=[dict(all_valid=True,samples=5)]
    if problem=='empty_world':v.worlds=[]
    if problem=='missing_object':v.worlds[0].update(complete=False,missing_names=['other'])
    if problem=='other_source':v.outcomes[0]['source']='other'
    if problem=='prefetch_only':v.outcomes[0].update(foreground=False,prefetch_capture_only=True)
    if problem=='no_clearance':clearance=[]
    if problem=='invalid_clearance':clearance[0]['all_valid']=False
    if problem=='zero_samples':clearance[0]['samples']=0
    if problem=='native_failed':outcome['native_full_chain_passed']=False
    r=result(v,outcome,clearance)
    assert not r['command_success'] and r['task_success'] is None


@pytest.mark.parametrize('change',['modify','delete'])
def test_changed_input_cannot_qualify(tmp_path,change):
    v=validator(tmp_path)
    if change=='modify':v.contract['path'].write_text('{}')
    else:v.contract['path'].unlink()
    r=result(v)
    assert not r['command_success'] and not r['frozen_world_validation']['input_unchanged']


def test_result_does_not_mutate_native_outcome_or_audits(tmp_path):
    v=validator(tmp_path);native={'native_full_chain_passed':True};audits=[dict(all_valid=True,samples=5)]
    before=copy.deepcopy((native,audits,v.worlds,v.outcomes));result(v,native,audits)
    assert (native,audits,v.worlds,v.outcomes)==before


def test_installed_service_observer_persists_actual_world_and_keeps_native_return(tmp_path):
    v=validator(tmp_path);v.worlds=[];v.outcomes=[]
    direct=NS(_refresh_curobo_world=lambda *a,**k:'native_refresh',
        _CUROBO_GPU_LOCK=threading.RLock(),_current_source_object_name=lambda a:'source',
        run_targeted_place_episode_curobo_direct=lambda *a:True)
    p=NS(_world=NS(objects=[NS(name='scene_obstacle_'+n) for n in ('desk','other')]),
         _disabled_world_obstacles=set(),_disabled_collision_links=set(),
         collision_enabled=True,config=NS(self_collision_check=True))
    v.install(direct)
    assert direct._refresh_curobo_world(p,None,NS(),label='transport')=='native_refresh'
    assert direct.run_targeted_place_episode_curobo_direct(None,None,None,NS()) is True
    assert result(v)['command_success']
    p._world.objects.pop()
    with pytest.raises(FrozenWorldIncomplete):
        direct._refresh_curobo_world(p,None,NS(),label='transport')
    persisted=[json.loads(line) for line in (tmp_path/'frozen_world_coverage.jsonl').read_text().splitlines()]
    assert persisted==v.worlds and len(persisted)==2
    assert persisted[-1]['missing_names']==['other'] and not result(v)['command_success']
