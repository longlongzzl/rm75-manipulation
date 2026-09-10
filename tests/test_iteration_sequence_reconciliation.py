"""Regression for eca8ece native outputs consumed by fae8a5e UI workflows.

No vendor imports, robot SDK, GPU initialization or network connections.
"""
import copy
import hashlib
import ast
from pathlib import Path
from types import SimpleNamespace
from collections import Counter

import pytest
from rm75_app.workcell.iteration_workflows import prepare_request, completion_summary
from rm75_app.workcell.sequence_evidence import summarize_sequence


def outcome(name, success=True, foreground=True, prefetch=False):
    return dict(source=name, success=success, foreground=foreground,
                prefetch_capture_only=prefetch)


def native_result(requested=('bi','tennis'), rows=None, passed=True):
    data = dict(passed=passed, input_unchanged=True, requested_sources=list(requested),
                requested_sources_completed=passed, all_sources_transport_observed=True,
                independent_clearance_passed=True,
                source_outcomes=rows if rows is not None else [outcome(n) for n in requested])
    return dict(command_success=passed, task_success=None, frozen_world_validation=data)


def test_actual_native_result_key_counts_partial_progress():
    names=['shuazi','bi','lvmukuai','carriot','tennis','gluestick','hongshupian']
    result=native_result(names,[outcome(n,n not in names[-2:]) for n in names],False)
    original=copy.deepcopy(result)
    report=completion_summary(result,names)
    assert report['completed_count']==5
    assert report['pending']==['gluestick','hongshupian']
    assert report['evidence_key']=='frozen_world_validation'
    assert not report['all_requested_verified'] and report['independent_physical_success'] is None
    assert result==original


def test_verified_native_chain_never_claims_physical_success():
    report=summarize_sequence(native_result(),['bi','tennis'])
    assert report['all_requested_verified'] and report['completed_count']==2
    assert report['same_scene_native_sequence'] and report['independent_physical_success'] is None


@pytest.mark.parametrize('gate',['passed','input_unchanged','requested_sources_completed',
                                'all_sources_transport_observed','independent_clearance_passed'])
def test_missing_or_false_gate_is_not_all_verified(gate):
    result=native_result();result['frozen_world_validation'].pop(gate)
    assert not summarize_sequence(result,['bi','tennis'])['all_requested_verified']


@pytest.mark.parametrize('row',[outcome('tennis',foreground=False),outcome('tennis',prefetch=True),
                              outcome('other'),outcome('tennis',success=None),
                              {'source':'tennis','success':True,'foreground':True}])
def test_prefetch_missing_flags_and_substitutes_do_not_complete_requested_source(row):
    result=native_result(rows=[outcome('bi'),row])
    report=summarize_sequence(result,['bi','tennis'])
    assert report['native_completed']==['bi'] and report['pending']==['tennis']
    assert not report['all_requested_verified']


def test_duplicate_success_is_visible_and_does_not_verify():
    result=native_result(rows=[outcome('bi'),outcome('bi'),outcome('tennis')])
    report=summarize_sequence(result,['bi','tennis'])
    assert report['completed_count']==2 and report['duplicate_success_sources']=={'bi':2}
    assert not report['all_requested_verified']


def test_real_key_wins_over_obsolete_fixture_key():
    result=native_result(rows=[outcome('bi')],passed=False)
    result['native_full_world_validation']=native_result()['frozen_world_validation']
    report=summarize_sequence(result,['bi','tennis'])
    assert report['completed_count']==1 and report['evidence_is_native_contract']
    result.pop('frozen_world_validation');result['command_success']=True
    legacy=summarize_sequence(result,['bi','tennis'])
    assert legacy['completed_count']==2 and not legacy['all_requested_verified']
    assert not legacy['evidence_is_native_contract']


@pytest.mark.parametrize('value',[True,'yes',1,None])
def test_native_command_bool_and_contract_are_required(value):
    result=native_result();result['command_success']=value
    report=summarize_sequence(result,['bi','tennis'])
    assert report['all_requested_verified'] is (value is True)


def test_empty_result_never_vacuously_succeeds():
    report=summarize_sequence({'command_success':True},['bi'])
    assert report['pending']==['bi'] and not report['all_requested_verified']
    with pytest.raises(ValueError):summarize_sequence({},[])


def test_one_selected_source_does_not_send_invalid_native_sequence():
    profile={'pickplace':{'fixed_scene_format':'native_world','frozen_source_order':['bi','tennis']}}
    source=copy.deepcopy(profile)
    spec={'task':'pickplace','mode':'sim','parameters':{'object_names':['tennis'],'automatic_order':True}}
    result,effective=prepare_request(spec,profile)
    assert result['parameters']=={'object_name':'tennis'}
    assert 'frozen_source_order' not in effective['pickplace'] and profile==source


def test_multiple_selected_sources_keep_same_scene_contract():
    profile={'pickplace':{'fixed_scene_format':'native_world'}}
    spec={'task':'pickplace','mode':'sim','parameters':{'object_names':['tennis','bi'],'automatic_order':True}}
    result,effective=prepare_request(spec,profile)
    assert result['parameters']=={'object_name':'bi'}
    assert effective['pickplace']['frozen_source_order']==['bi','tennis']


def test_consumer_against_real_producer_method_without_importing_native_dependencies(tmp_path):
    # Compile only the real producer method's AST, not the native import graph.
    source=Path(__file__).resolve().parents[1]/'rm75_app/workcell/native_frozen_world.py'
    tree=ast.parse(source.read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='FrozenWorldValidation')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='result')
    scope={'hashlib':hashlib,
           'requested_sources_completed':lambda rows,sources:Counter(r['source'] for r in rows
                if r['success'] is True and r['foreground'] is True and r['prefetch_capture_only'] is False)==Counter(sources),
           'full_chain_observed':lambda rows,sources:True}
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),scope)
    path=tmp_path/'frozen.json';path.write_text('{}')
    producer=SimpleNamespace(contract=dict(source='bi',source_order=('bi','tennis'),path=path,
        sha256=hashlib.sha256(path.read_bytes()).hexdigest(),names=('bi','tennis')),
        outcomes=[outcome('bi'),outcome('tennis')],worlds=[])
    result=scope['result'](producer,{'native_full_chain_passed':True},
                           [{'all_valid':True,'samples':2} for _ in range(2)])
    assert result['frozen_world_validation']['passed']
    report=summarize_sequence(result,['bi','tennis'])
    assert report['all_requested_verified'] and report['completed_count']==2
