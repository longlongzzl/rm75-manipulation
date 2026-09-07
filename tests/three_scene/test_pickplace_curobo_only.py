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


def test_loader_restores_after_exception(tmp_path):
    from importlib.machinery import SourceFileLoader
    original=SourceFileLoader.get_code
    with pytest.raises(ValueError):
        with source_adapter(tmp_path):
            assert SourceFileLoader.get_code is not original
            raise ValueError()
    assert SourceFileLoader.get_code is original


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
