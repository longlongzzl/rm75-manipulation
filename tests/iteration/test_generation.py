import copy
import json
from pathlib import Path
from collections import Counter
import pytest
from rm75_app.magnetic.generation import (validate_library,compile_selection,validate_generated_request,
                                         generate,catalog_summary,dependencies,load_library)
from rm75_app.workcell.io import digest,atomic_json,read_json
from tools.import_jimu_design_library import main as import_main


def test_preserves_exact_original_geometry_and_base(library,proposal):
    before=copy.deepcopy(library)
    result=compile_selection(validate_library(library),proposal,board_id='grid_3x3')
    source=library['boards']['grid_3x3']['templates'][0]['design']
    selected=result['design']['pieces'];lookup={p['role']:p for p in source['pieces']}
    assert len(selected)==12 and result['proof']['movable_count']==3
    assert all(p==lookup[p['role']] for p in selected)
    assert result['design']['unknown_original_field']==source['unknown_original_field']
    assert result['proof']['native_planning_verified'] is False
    assert result['proof']['physical_buildability_verified'] is False
    assert library==before
    assert validate_generated_request(library,result['design'],result['proof'])=={'native_args':[]}


@pytest.mark.parametrize('roles', [['wall_0_1'],['wall_0_2'],['base_0'],['fake'],[],['wall_0_0','wall_0_0'],['wall_0_0']*13])
def test_invalid_role_selection_rejected(library,proposal,roles):
    proposal['selected_roles']=roles
    with pytest.raises(ValueError):compile_selection(library,proposal,board_id='grid_3x3')


@pytest.mark.parametrize('extra', ['coordinates','robot_command','native_args','api_key','url','pose'])
def test_llm_cannot_generate_executable_fields(library,proposal,extra):
    proposal[extra]='untrusted'
    with pytest.raises(ValueError):compile_selection(library,proposal,board_id='grid_3x3')


@pytest.mark.parametrize('field,value', [('board_id','arc'),('template_id','unknown'),('title',''),('explanation','')])
def test_identity_text_constraints(library,proposal,field,value):
    proposal[field]=value
    with pytest.raises(ValueError):compile_selection(library,proposal,board_id='grid_3x3')


@pytest.mark.parametrize('field', ['design_digest','locked_digest','inventory'])
def test_library_tamper(library,field):
    library['boards']['arc']['templates'][0][field]={} if field=='inventory' else 'bad'
    with pytest.raises(ValueError):validate_library(library)


def test_missing_grid_plate_not_fabricated(library):
    t=library['boards']['grid_3x3']['templates'][0]
    t['design']['pieces'].pop(8)
    t['design_digest']=digest(t['design']);t['locked_digest']=digest(t['design']['pieces'][:8])
    with pytest.raises(ValueError):validate_library(library)


def test_extra_support_closure(library,proposal):
    t=library['boards']['grid_3x3']['templates'][0]
    t['support_dependencies']={'wall_0_1':['wall_1_0']}
    with pytest.raises(ValueError):compile_selection(library,proposal,board_id='grid_3x3')
    proposal['selected_roles'].append('wall_1_0')
    assert compile_selection(library,proposal,board_id='grid_3x3')['proof']['movable_count']==4


@pytest.mark.parametrize('value', ['wall_0_1',None,123])
def test_dependency_list_required(library,value):
    t=library['boards']['grid_3x3']['templates'][0];t['support_dependencies']={'wall_0_0':value}
    with pytest.raises(ValueError):dependencies(t)


def test_cycle_rejected(library):
    t=library['boards']['grid_3x3']['templates'][0]
    t['support_dependencies']={'wall_0_0':['wall_0_2']}
    with pytest.raises(ValueError):dependencies(t)


def test_generated_request_proof_rechecked(library,proposal):
    r=compile_selection(library,proposal,board_id='grid_3x3')
    r['design']['pieces'][-1]['center'][0]+=.01
    with pytest.raises(ValueError):validate_generated_request(library,r['design'],r['proof'])


def test_model_retry_is_bounded_not_silent(library,proposal):
    calls=[]
    def complete(messages):
        calls.append(copy.deepcopy(messages))
        return 'not json' if len(calls)==1 else json.dumps(proposal)
    result=generate(library,dict(board_id='grid_3x3',prompt='a facade'),complete)
    assert result['model_calls']==2 and len(result['validation_errors'])==1
    assert len(calls[1])==4
    count=[]
    with pytest.raises(ValueError):
        generate(library,dict(board_id='grid_3x3',prompt='bad'),lambda messages:count.append(1) or '{}')
    assert len(count)==2


def test_piece_budget_applies(library,proposal):
    with pytest.raises(ValueError):compile_selection(library,proposal,board_id='grid_3x3',piece_budget=2)


def test_arc_preserved_and_compiles(library,proposal):
    proposal.update(board_id='arc',template_id='arc_00')
    result=compile_selection(library,proposal,board_id='arc')
    assert sum(p.get('locked',False) for p in result['design']['pieces'])==5
    assert validate_library(library)


def test_import_exact_both_bases_read_only(tmp_path,library):
    roots=[]
    for board in ('grid_3x3','arc'):
        root=tmp_path/board;root.mkdir();roots.append(root)
        atomic_json(root/'builder.json',library['boards'][board]['templates'][0]['design'])
        atomic_json(root/'fixed.json',{'results':[{'synthetic_fixture':True}]})
        atomic_json(root/'manifest.json',{'schema':'jimu_task_manifest_v1','builder_scene_json':'builder.json',
            'sam6d_fixed_scene_result_file':'fixed.json','builder':{'use_design_parent_targets':True}})
    before={str(p):p.read_bytes() for r in roots for p in r.iterdir()}
    out=tmp_path/'new_library.json'
    import_main(['--grid-task',str(roots[0]),'--arc-task',str(roots[1]),'--output',str(out)])
    output=validate_library(read_json(out))
    assert set(output['boards'])=={'grid_3x3','arc'}
    assert before=={str(p):p.read_bytes() for r in roots for p in r.iterdir()}
    for board in ('grid_3x3','arc'):
        assert output['boards'][board]['templates'][0]['design']==library['boards'][board]['templates'][0]['design']
    with pytest.raises(FileExistsError):import_main(['--grid-task',str(roots[0]),'--arc-task',str(roots[1]),'--output',str(out)])


def test_no_configured_arc_is_explicit_error(profile):
    profile['magnetic']['design_library']='/nonexistent/original/library.json'
    with pytest.raises(FileNotFoundError):load_library(profile)


def test_native_recipe_digest_invalidates_old_proof(library,proposal):
    data=compile_selection(library,proposal,board_id='grid_3x3')
    library['boards']['grid_3x3']['templates'][0]['native_recipe']['native_args']=['--jimu-apriltag-base-id','2']
    with pytest.raises(ValueError):validate_generated_request(library,data['design'],data['proof'])
