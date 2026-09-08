from copy import deepcopy
import ast
import threading
from types import SimpleNamespace as NS

import pytest

from rm75_app.workcell.pickplace_world_coverage import (
    FrozenWorldIncomplete, coverage_row, full_chain_observed, install, requested_sources_completed,
)
from tools.run_native_pickplace import ROOT, frozen_scene_names


def planner(names=('desk', 'other')):
    return NS(_world=NS(objects=[NS(name='scene_obstacle_'+n) for n in names]),
              _disabled_world_obstacles=set(), _disabled_collision_links=set(),
              collision_enabled=True, config=NS(self_collision_check=True))


def row(p=None, **kwargs):
    return coverage_row(p or planner(), scene_names=('source', 'desk', 'other'),
                        source='source', label=kwargs.pop('label', 'transport'), **kwargs)


def test_complete_world_is_read_only_and_not_physical_qualification():
    p=planner(); before=deepcopy(p)
    result=row(p)
    assert result['complete'] and result['missing_names']==[]
    assert result['physical_geometry_qualified'] is False
    assert p==before


def test_only_desk_cannot_qualify_full_frozen_world():
    result=row(planner(('desk',)))
    assert not result['complete'] and result['missing_names']==['other']


@pytest.mark.parametrize('disabled',['other','scene_obstacle_other'])
def test_present_but_disabled_object_is_missing(disabled):
    p=planner();p._disabled_world_obstacles.add(disabled)
    assert row(p)['missing_names']==['other']


@pytest.mark.parametrize('label',['transport','post_grasp_lift','joint_start_1'])
def test_loaded_exclusions_cannot_hide_missing_neighbor(label):
    result=row(planner(('desk',)),label=label,excluded=('source','other'))
    assert not result['complete'] and result['missing_names']==['other']
    assert 'loaded_non_source_exclusion' in result['rejection_reasons']


def test_native_nontransport_explicit_exclusion_is_reported_not_transport_certified():
    result=row(planner(('desk',)),label='final_contact',excluded=('other',))
    assert result['complete'] and not result['transport_scope']
    assert result['explicitly_excluded_non_source_names']==['other']
    assert not full_chain_observed([result],('source',))


def test_active_source_is_not_expected_as_a_world_neighbor():
    assert row(excluded=('source',))['complete']


@pytest.mark.parametrize('flag',['world','self','link'])
def test_transport_checks_cannot_be_disabled(flag):
    p=planner()
    if flag=='world':p.collision_enabled=False
    if flag=='self':p.config.self_collision_check=False
    if flag=='link':p._disabled_collision_links.add('left_pad')
    assert not row(p)['complete']


def test_unknown_source_fails_closed():
    with pytest.raises(FrozenWorldIncomplete):
        coverage_row(planner(),scene_names=('desk',),source='unknown',label='transport')


def test_no_vacuous_or_partial_chain_coverage():
    good=row()
    assert full_chain_observed([good],('source',))
    assert not full_chain_observed([],('source',))
    assert not full_chain_observed([good],())
    assert not full_chain_observed([good],('source','second'))
    assert not full_chain_observed([good,{**good,'complete':False}],('source',))


def test_installed_observer_preserves_return_and_kwargs():
    calls=[]; emitted=[]
    def refresh(*args,**kwargs):calls.append(kwargs);return 'original_result'
    direct=NS(_refresh_curobo_world=refresh,_CUROBO_GPU_LOCK=threading.RLock(),
              _current_source_object_name=lambda args:'source')
    install(direct,('source','desk','other'),emitted.append)
    assert direct._refresh_curobo_world(planner(),None,NS(),label='transport')=='original_result'
    assert calls==[{'label':'transport'}] and len(emitted)==1
    with pytest.raises(FrozenWorldIncomplete):
        direct._refresh_curobo_world(planner(('desk',)),None,NS(),label='transport')
    assert not emitted[-1]['complete']
    with pytest.raises(FrozenWorldIncomplete):
        direct._refresh_curobo_world(planner(),None,NS(execute_real=True),label='transport')
    assert len(calls)==2


def native_function(relative, name, scope):
    # Compile just the verified original function, never import the robot entry.
    path=ROOT/'rm75_app/_vendor/working_snapshot/pick_jiaobang'/relative
    tree=ast.parse(path.read_text())
    node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==name)
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),scope)
    return scope[name]


@pytest.mark.parametrize('selected',['gluestick','bi','shuazi'])
def test_original_retry_and_active_filter_keep_every_nonactive_object(selected):
    names=frozen_scene_names('legacy_gluestick')
    derive=native_function('rm75_jiaobang_pick_place_targeted.py','_derive_cycle_obstacle_names',
        dict(normalize_object_name=lambda x:x,_dedupe_names=lambda xs:list(dict.fromkeys(xs)),
             get_required_place_target_names=lambda xs:['desk']))
    requested=derive(NS(tracked_scene_object_names=list(names)),1,selected,[],names)
    registry={name:{'actor':NS(name=name)} for name in names}
    demo=NS(scene_obstacles=[],env=NS(unwrapped=NS(_scene_obstacle_actors=[])))
    scope=dict(curobo_wrapper=NS(normalize_object_name=lambda x:x),
        _single_scene_registry=lambda d:registry,
        _single_scene_remove_planner_object=lambda *a:None,
        _single_scene_build_obstacle_entry=lambda d,a,n,m,c:{'object_name':n},
        _single_scene_set_planner_obstacle=lambda *a:None)
    sync=native_function('rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place.py',
                        '_single_scene_sync_obstacles',scope)
    sync(demo,NS(),selected,requested,{})
    actual=[entry['object_name'] for entry in demo.scene_obstacles]
    assert len(actual)==8 and set(actual)==set(names)-{selected}
    assert selected not in actual


@pytest.mark.parametrize('outcomes,sources,expected',[
    ([('gluestick',True)],('gluestick',),True),
    ([('gluestick',False),('bi',True)],('gluestick',),False),
    ([('gluestick',False),('gluestick',True)],('gluestick',),True),
    ([('gluestick',True),('gluestick',True)],('gluestick',),False),
    ([('gluestick',None)],('gluestick',),False),
    ([],('gluestick',),False),
    ([],(),False),
])
def test_completed_task_binds_requested_sources(outcomes,sources,expected):
    assert requested_sources_completed([dict(source=s,success=v,foreground=True,
        prefetch_capture_only=False) for s,v in outcomes],sources) is expected


def test_episode_observation_preserves_native_return_and_exception():
    def episode(d,b,r,args):
        if getattr(args,'fail',False):raise ValueError('native failure')
        return True
    direct=NS(_refresh_curobo_world=lambda *a,**k:None,_CUROBO_GPU_LOCK=threading.RLock(),
        _current_source_object_name=lambda a:a.object_name,
        run_targeted_place_episode_curobo_direct=episode)
    outcomes=[]
    install(direct,('source','desk','other'),lambda r:None,outcomes.append)
    assert direct.run_targeted_place_episode_curobo_direct(None,None,None,NS(object_name='source')) is True
    with pytest.raises(ValueError,match='native failure'):
        direct.run_targeted_place_episode_curobo_direct(None,None,None,NS(object_name='other',fail=True))
    assert outcomes==[dict(source='source',success=True,foreground=True,prefetch_capture_only=False),
        dict(source='other',success=None,error_type='ValueError',foreground=True,prefetch_capture_only=False)]


@pytest.mark.parametrize('scope',[dict(foreground=False,prefetch_capture_only=True),
    dict(foreground=False,prefetch_capture_only=False),dict(foreground=True,prefetch_capture_only=True)])
def test_background_or_capture_only_cannot_supply_task_or_transport_success(scope):
    assert not full_chain_observed([{**row(),**scope}],('source',))
    assert not requested_sources_completed([dict(source='source',success=True,**scope)],('source',))


def test_installed_observer_marks_native_prefetch_and_other_threads():
    direct=NS(_refresh_curobo_world=lambda *a,**k:None,_CUROBO_GPU_LOCK=threading.RLock(),
        _current_source_object_name=lambda a:'source',
        run_targeted_place_episode_curobo_direct=lambda *a:True)
    worlds=[]; outcomes=[]
    install(direct,('source','desk','other'),worlds.append,outcomes.append)
    def call(args):
        direct._refresh_curobo_world(planner(),None,args,label='transport')
        direct.run_targeted_place_episode_curobo_direct(None,None,None,args)
    call(NS(_planning_prefetch_capture_only=True))
    thread=threading.Thread(target=call,args=(NS(),));thread.start();thread.join(5)
    assert not thread.is_alive()
    assert len(worlds)==len(outcomes)==2
    assert all(not r['foreground'] for r in worlds+outcomes)
    assert [r['prefetch_capture_only'] for r in outcomes]==[True,False]
    assert not requested_sources_completed(outcomes,('source',))
