from types import SimpleNamespace as NS
import numpy as np
import pytest

from rm75_app.workcell.pickplace_ik_review import center_closure, fix_selected_debug, install, yaw_midpoints


def test_yaw_augmentation_preserves_all_originals_and_only_adds_midpoints():
    original=[0,-45,45,-90,90,180]
    values=yaw_midpoints(original)
    assert values[:6]==original and len(values)==12
    assert set(values[6:])=={22.5,67.5,135,-135,-67.5,-22.5}


def test_sphere_center_closure_detects_lateral_and_signed_axis_mismatch():
    relation=np.eye(4);relation[:3,3]=[.004,0,-.02]
    tcp=np.eye(4);tcp[:3,:3]=np.diag([1,-1,-1]);tcp[:3,3]=[.3,0,.02]
    row=center_closure(relation,tcp,[.3,0,0])
    assert row['lateral_tcp_offset_m']==pytest.approx(.004)
    assert row['signed_axial_offset_m']==pytest.approx(-.02)
    assert row['center_residual_m']==pytest.approx(np.hypot(.004,.04))


def test_centered_positive_axis_relation_is_closed():
    relation=np.eye(4);relation[2,3]=.02
    tcp=np.eye(4);tcp[:3,:3]=np.diag([1,-1,-1]);tcp[:3,3]=[.3,0,.02]
    assert center_closure(relation,tcp,[.3,0,0])['center_residual_m']==0


def test_selected_single_ik_debug_uses_chosen_success_row_not_row_zero():
    raw=NS(solution=np.array([[[0.]*7,[1.]*7]]),success=np.array([[False,True]]),
           position_error=np.array([[.01,.00001]]),rotation_error=np.array([[.2,.00002]]))
    result=NS(success=True,goal_joint=np.ones(7),raw_result=raw,debug={'ik_success_count':1})
    assert fix_selected_debug(result) is result
    assert result.debug['selected_raw_index']==1
    assert result.debug['position_error']==.00001 and result.debug['rotation_error']==.00002
    assert result.debug['ik_success_count']==1 and not raw.success[0,0]
    np.testing.assert_array_equal(result.goal_joint,np.ones(7))


def test_failed_ik_is_never_promoted_by_debug_repair():
    result=NS(success=False,goal_joint=None)
    assert fix_selected_debug(result) is result


def test_mismatched_selected_ik_row_is_rejected():
    result=NS(success=True,goal_joint=np.ones(7),raw_result=NS(solution=np.zeros((1,1,7)),success=np.ones((1,1))))
    with pytest.raises(ValueError,match='identity'):fix_selected_debug(result)


def test_missing_ee_link_is_not_silently_replaced_with_tcp():
    def paired(planner,demo,args,records,start_q,*,source_name,label,disabled_world_collision_links):pass
    direct=NS(_convert_demo_tcp_pose_to_curobo_ee_pose=lambda *a,**k:pytest.fail('silent fallback'),
        _get_robot_link_by_name=lambda *a:None,_profile_fast_chain_solve_batch_start_goal_ik=lambda *a,**k:None,
        _fast_chain_evaluate_paired_relation_records=paired)
    class Planner:
        def solve_ik(self,*a,**k):pass
    close=install(direct,Planner,lambda row:None,strategy='baseline')
    try:
        with pytest.raises(ValueError,match='Missing actual'):
            direct._convert_demo_tcp_pose_to_curobo_ee_pose(None,None,ee_link_name='missing')
    finally:close()


def test_unknown_ik_trial_cannot_install_new_strategy():
    with pytest.raises(ValueError):install(None,None,None,strategy='more-seeds')
