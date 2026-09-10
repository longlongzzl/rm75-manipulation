import ast
import json
from pathlib import Path
import threading
from types import SimpleNamespace as NS

import numpy as np
import pytest

from rm75_app.workcell import ik_candidate_gallery as gallery


@pytest.mark.parametrize('task,mode,fmt',[('pickplace','real','native_world'),('pusht','sim','native_world'),
    ('pickplace','sim','sam6d'),('pickplace','sim',None)])
def test_gallery_refuses_hardware_or_live_inputs(task,mode,fmt):
    with pytest.raises(ValueError):gallery.require_gallery(dict(task=task,mode=mode),{'pickplace':{'fixed_scene_format':fmt}})


def test_gallery_accepts_only_explicit_frozen_sim():
    gallery.require_gallery(dict(task='pickplace',mode='sim'),{'pickplace':{'fixed_scene_format':'native_world'}})


def test_visual_cleanup_removes_both_registries():
    calls=[];actor=NS(name='ghost',remove_from_scene=lambda:calls.append('remove'))
    registry={'ghost':actor,'real':object()}
    scene=NS(actors=registry.copy(),remove_from_state_dict_registry=lambda a:registry.pop(a.name))
    gallery.remove_visual(scene,actor)
    assert calls==['remove'] and set(scene.actors)==set(registry)=={'real'}


def test_context_preserves_blocked_hover_release_order():
    records=[dict(grasp_candidate={'label':f'g{i}'},place_candidate={'label':f'p{i}'}) for i in range(3)]
    frame=NS(f_code=NS(co_name='_fast_chain_evaluate_paired_relation_records'),f_locals={'demo':'demo','records':records},f_back=None)
    demo,rows=gallery.query_context(frame,[None]*6)
    assert demo=='demo' and [r[0] for r in rows]==['hover']*3+['release']*3
    assert [r[1]['label'] for r in rows]==['g0','g1','g2']*2
    with pytest.raises(ValueError):gallery.query_context(frame,[None]*5)


@pytest.mark.parametrize('phase',['pregrasp','grasp'])
def test_context_uses_original_goal_list_identity(phase):
    goals=[object()];v={'demo':'demo','candidates':[{'label':'g'}],phase+'_poses':goals}
    frame=NS(f_code=NS(co_name='_fast_chain_preselect_grasp_place_pair'),f_locals=v,f_back=None)
    assert gallery.query_context(frame,goals)[1][0][0]==phase
    assert gallery.query_context(frame,goals.copy()) is None


def test_render_failure_preserves_solver_result_and_robot_state(tmp_path,monkeypatch):
    state={'q':np.zeros((1,7)),'v':np.ones((1,7))}
    robot=NS(get_qpos=lambda:state['q'].copy(),get_qvel=lambda:state['v'].copy(),
        set_qpos=lambda x:state.update(q=x.copy()),set_qvel=lambda x:state.update(v=x.copy()))
    scene=NS(actors={},state_dict_registry=NS(actors={}))
    demo=NS(robot=robot,env=NS(unwrapped=NS(scene=scene)),get_obj_pose=lambda:[np.zeros(3),[1,0,0,0]])
    result=[NS(success=False,goal_joint=None)]
    calls=[]
    def original(*a,**kw):calls.append('solver');return result
    direct=NS(_profile_fast_chain_solve_batch_start_goal_ik=original,_CUROBO_GPU_LOCK=threading.RLock(),
        _current_source_object_name=lambda options:'gluestick')
    monkeypatch.setattr(gallery,'query_context',lambda *a:(demo,[('grasp',{'label':'g'},None)]))
    monkeypatch.setattr(gallery,'result_rows',lambda *a:([{'joints':[0.]*7}],0))
    monkeypatch.setattr(gallery,'_state',lambda planner:{'immutable':True})
    def fail(*a):raise RuntimeError('intentional renderer failure')
    monkeypatch.setattr(gallery,'build_goal_tool',fail)
    # If optional simulator imports are unavailable, the same safe failure path applies.
    close=gallery.install(direct,tmp_path/'gallery',lambda row:None)
    assert direct._profile_fast_chain_solve_batch_start_goal_ik(NS(execute_real=False),NS(),[[0.]*7],[object()],num_seeds=32) is result
    close();assert calls==['solver']
    np.testing.assert_array_equal(state['q'],np.zeros((1,7)));np.testing.assert_array_equal(state['v'],np.ones((1,7)))
    record=json.loads((tmp_path/'gallery/batch_001/evidence.json').read_text())
    assert all(record[k] for k in ('robot_state_restored','planner_state_restored','actor_registry_restored','returned_rows_unchanged'))
    assert record['render_error'] and not json.loads((tmp_path/'gallery/manifest.json').read_text())['complete']
    assert direct._profile_fast_chain_solve_batch_start_goal_ik is original


def test_empty_gallery_is_not_a_success(tmp_path):
    direct=NS(_profile_fast_chain_solve_batch_start_goal_ik=lambda:None)
    gallery.install(direct,tmp_path/'gallery',lambda row:None)()
    assert json.loads((tmp_path/'gallery/manifest.json').read_text())['complete'] is False


def test_gallery_has_no_physics_step_or_collision_shape_creation():
    tree=ast.parse(Path(gallery.__file__).read_text())
    names=[ast.unparse(node.func) for node in ast.walk(tree) if isinstance(node,ast.Call)]
    assert not any(n.endswith('.step') or 'add_box_collision' in n or '_set_obstacle_enabled' in n for n in names)
