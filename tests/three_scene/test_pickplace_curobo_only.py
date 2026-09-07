import ast
import threading
from types import SimpleNamespace as NS
import pytest
from rm75_app.workcell.pickplace_curobo_only import (
    CuroboDemoPlanner,CuroboOnlyUnsupported,transform_source,source_adapter)


def test_source_transform_removes_external_import_and_constructor():
    tree=transform_source('import mplib\nfrom mplib import collision_detection as mplib_cd\nx=mplib.Planner(urdf="unused")','test.py')
    text=ast.unparse(tree)
    assert 'from mplib' not in text and 'import mplib' not in text
    assert 'mplib.CuroboDemoPlanner(self)' in text


def test_reverse_reuse_predicate_keeps_original_branch_bodies_and_short_circuit():
    source='''
def choose(reverse_endpoint_valid):
    if reverse_endpoint_valid:
        return 'reuse'
    else:
        return 'original_candidate_loop'
'''
    filename='rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place.py'
    before=ast.parse(source).body[0].body[0]
    tree=transform_source(source,filename);after=tree.body[0].body[0]
    assert ast.dump(ast.Module(body=before.body+before.orelse,type_ignores=[]))==ast.dump(
        ast.Module(body=after.body+after.orelse,type_ignores=[]))
    calls=[];scope={name:object() for name in ('planner','demo','args','reverse_clearance_path')}
    scope['_rm75_clearance_reverse_path_reusable']=lambda *args:calls.append(args) or False
    exec(compile(tree,filename,'exec'),scope)
    assert scope['choose'](False)=='original_candidate_loop' and not calls
    assert scope['choose'](True)=='original_candidate_loop' and len(calls)==1
    scope['_rm75_clearance_reverse_path_reusable']=lambda *args:True
    assert scope['choose'](True)=='reuse'


def test_missing_reviewed_reverse_boundary_fails_closed():
    with pytest.raises(RuntimeError,match='reuse boundary changed'):
        transform_source('pass','rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place.py')


def test_original_mesh_tilde_paths_use_existing_snapshot_bytes_only(tmp_path):
    path=tmp_path/'pick_jiaobang/meshs/plate.glb'
    path.parent.mkdir(parents=True);path.write_bytes(b'original mesh bytes')
    source="asset='~/Desktop/lerobot/pick_jiaobang/meshs/plate.glb'\nscale=0.1199\nother='~/Desktop/unrelated/file'"
    tree=transform_source(source,'object_specs.py',snapshot_root=tmp_path)
    scope={};exec(compile(tree,'object_specs.py','exec'),scope)
    assert scope['asset']==str(path) and scope['scale']==0.1199
    assert scope['other']=='~/Desktop/unrelated/file'
    assert path.read_bytes()==b'original mesh bytes'
    with pytest.raises(FileNotFoundError,match='Missing vendored object asset'):
        transform_source("asset='~/Desktop/lerobot/pick_jiaobang/meshs/missing.glb'",'object_specs.py',snapshot_root=tmp_path)


def test_all_24_original_object_mesh_paths_exist_in_snapshot():
    from pathlib import Path
    root=Path(__file__).resolve().parents[2]/'rm75_app/_vendor/working_snapshot'
    path=root/'pick_jiaobang/object_specs.py';source=path.read_text()
    before={n.value for n in ast.walk(ast.parse(source)) if isinstance(n,ast.Constant)
            and isinstance(n.value,str) and n.value.startswith('~/Desktop/lerobot/pick_jiaobang/meshs/')}
    assert len(before)==24
    tree=transform_source(source,str(path),snapshot_root=root)
    assert not any(isinstance(n,ast.Constant) and n.value in before for n in ast.walk(tree))


def test_loader_restores_after_exception(tmp_path):
    from importlib.machinery import SourceFileLoader
    original=SourceFileLoader.get_code
    with pytest.raises(ValueError):
        with source_adapter(tmp_path):
            assert SourceFileLoader.get_code is not original
            raise ValueError()
    assert SourceFileLoader.get_code is original


def test_external_mplib_import_is_fail_closed_and_guard_is_restored(tmp_path):
    import sys
    import importlib
    before=sys.meta_path[:]
    with source_adapter(tmp_path):
        with pytest.raises(CuroboOnlyUnsupported,match='external MPLib import forbidden'):
            importlib.import_module('mplib')
    assert sys.meta_path==before and 'mplib' not in sys.modules


def test_jimu_legacy_empty_collision_shim_is_replaced_with_real_curobo_boundary():
    from rm75_app.workcell.pickplace_curobo_only import install_jimu_binding
    def obsolete(*a):raise AssertionError('External MPLib bookkeeping must never run')
    portable=NS(_JimuNoopMplibPlanner=obsolete,_install_jimu_no_mplib_collision_detection=obsolete,
                _restore_jimu_no_mplib_collision_detection=obsolete)
    install_jimu_binding(portable)
    portable._install_jimu_no_mplib_collision_detection(NS())
    portable._restore_jimu_no_mplib_collision_detection()
    planner=portable._JimuNoopMplibPlanner(NS(robot=object()))
    assert isinstance(planner,CuroboDemoPlanner)
    with pytest.raises(CuroboOnlyUnsupported,match='world has not been bound'):
        planner.check_for_self_collision([0]*7)


@pytest.mark.parametrize('status', [None,'WORLD_COLLISION','SELF_COLLISION','UNKNOWN'])
def test_legacy_collision_queries_use_actual_curobo_result(status):
    adapter=CuroboDemoPlanner(NS(robot=object()))
    calls=[]
    def check(q):
        calls.append(q)
        return status is None,status
    adapter.native=NS(collision_enabled=True,config=NS(self_collision_check=True),
        _disabled_collision_links=set(),attached_object_active=True,check_start_state=check)
    adapter.lock=threading.RLock()
    collisions=adapter.check_for_self_collision([1])+adapter.check_for_env_collision([1],use_attach=True)
    assert bool(collisions) is (status is not None)
    assert calls==[[1],[1]]


def test_unbound_and_removed_planning_fail_closed():
    adapter=CuroboDemoPlanner(NS(robot=object()))
    for call in (lambda:adapter.check_for_self_collision([0]*7),lambda:adapter.IK(None)):
        with pytest.raises(CuroboOnlyUnsupported): call()


def test_all_native_pickplace_mplib_imports_are_in_adapter_scope():
    from pathlib import Path
    root=Path(__file__).resolve().parents[2]/'rm75_app/_vendor/working_snapshot/pick_jiaobang'
    expected={'rm75_jiaobang_pick_move_v10_perpendicular_to_object.py',
        'rm75_jiaobang_pick_real_with_foundationpose.py','rm75_jiaobang_pick_place_targeted.py',
        'rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place.py'}
    found=set()
    def external_import(node):
        return ((isinstance(node,ast.Import) and any(a.name=='mplib' for a in node.names))
            or (isinstance(node,ast.ImportFrom) and node.module=='mplib'))
    for path in root.glob('*.py'):
        source=path.read_text()
        if any(external_import(n) for n in ast.walk(ast.parse(source))):
            found.add(path.name)
            assert not any(external_import(n) for n in ast.walk(transform_source(source,str(path))))
    assert found==expected


def test_sim_action_conversion_preserves_values_and_device():
    import numpy as np
    import torch
    from rm75_app.workcell.pickplace_curobo_only import as_sim_action
    values=np.arange(9,dtype=np.float32)
    result=as_sim_action(NS(base_env=NS(device='cpu')),values)
    assert torch.is_tensor(result) and result.dtype==torch.float32
    np.testing.assert_array_equal(result.numpy(),values)


def test_sim_step_failure_cannot_be_swallowed_as_warning():
    from rm75_app.workcell.pickplace_curobo_only import step_sim
    def fail(action): raise ValueError('bad simulator state')
    with pytest.raises(CuroboOnlyUnsupported) as caught:
        step_sim(NS(env=NS(step=fail),base_env=NS(device='cpu')),[0.]*9)
    assert isinstance(caught.value.__cause__,ValueError)


def test_sim_reward_geometry_is_converted_without_changing_values():
    import numpy as np
    import torch
    from rm75_app.workcell.pickplace_curobo_only import step_sim
    env=NS(device='cpu',obj_xy_shortest_edge_vector=np.array([[.1,.2,.3]]))
    demo=NS(base_env=env,env=NS(step=lambda action:env.obj_xy_shortest_edge_vector))
    value=step_sim(demo,[0.]*9)
    assert torch.is_tensor(value)
    np.testing.assert_allclose(value.numpy(),[[.1,.2,.3]])


@pytest.mark.parametrize('helper',['_print_transport_motiongen_failure_diagnostics','print_failure_diagnostics'])
def test_print_only_failure_diagnostics_do_not_abort_original_retry(helper):
    from rm75_app.workcell.pickplace_curobo_only import isolate_print_only_diagnostics
    def unavailable(*args,**kwargs):raise CuroboOnlyUnsupported('attached collision model missing')
    base=NS(print_failure_diagnostics=unavailable)
    direct=NS(targeted=NS(base=base),_print_transport_motiongen_failure_diagnostics=unavailable)
    isolate_print_only_diagnostics(direct)
    assert getattr(base if helper=='print_failure_diagnostics' else direct,helper)() is None
    assert direct._curobo_diagnostic_rejections[0]['collision_state_qualified'] is False
    with pytest.raises(CuroboOnlyUnsupported):unavailable()


def test_print_diagnostic_does_not_swallow_other_native_exceptions():
    from rm75_app.workcell.pickplace_curobo_only import isolate_print_only_diagnostics
    def failed():raise RuntimeError('unexpected native failure')
    direct=NS(targeted=NS(base=NS(print_failure_diagnostics=failed)))
    isolate_print_only_diagnostics(direct)
    with pytest.raises(RuntimeError):direct.targeted.base.print_failure_diagnostics()
