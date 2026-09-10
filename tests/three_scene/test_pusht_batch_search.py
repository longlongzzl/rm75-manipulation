from dataclasses import replace

import numpy as np
import pytest

from rm75_app.pusht.model import Config,Push,predict,valid_pose,choose_push
from rm75_app.pusht.batch_search import action_normals,action_arrays,predict_batch,valid_pose_batch,rank_pushes


def test_vectorized_predictions_match_independent_scalar_model():
    config=Config();poses=np.array([[.35,-.18,0.],[.4,.03,.6],[.3,-.03,-2.7]])
    centers,directions,lengths,_=action_arrays(poses,config)
    normals=action_normals(directions,config)
    batch=predict_batch(poses,centers,directions,lengths,config,config.friction_scales)
    for b,pose in enumerate(poses):
        for a,length in enumerate(lengths):
            push=Push(tuple(centers[b,a]),tuple(directions[b,a]),float(length),config.speed_mps,tuple(normals[b,a]))
            for j,scale in enumerate(config.friction_scales):
                np.testing.assert_allclose(batch[b,a,j],predict(pose,push,config,scale),atol=1e-14)


def test_vectorized_workspace_and_obstacles_match_scalar_geometry():
    config=Config(obstacles=((.46,.07,.02),(.3,-.22,.015)))
    poses=np.random.default_rng(4).uniform([.1,-.35,-np.pi],[.7,.35,np.pi],(1000,3))
    np.testing.assert_array_equal(valid_pose_batch(poses,config),[valid_pose(p,config) for p in poses])


def test_search_compares_long_and_short_actions_without_claiming_global_optimum():
    config=Config();rows=rank_pushes((.35,-.18,0),(.38,-.18,0),config)
    assert len(rows)>50
    far=rank_pushes((.35,-.18,0),(.45,-.18,0),config)
    assert len({round(p.length_m,6) for p,_ in far})==6
    assert rows[0][0].length_m>=.03
    report=rows[0][1]
    assert report['evaluated_friction_rollouts']>=9000
    assert not report['global_optimum_proven'] and not report['prediction_is_observation']
    assert all(a[1]['predicted_cost']<=b[1]['predicted_cost'] for a,b in zip(rows,rows[1:]))
    short=choose_push((.35,-.18,0),(.358,-.18,0),config)[0]
    assert short.length_m<rows[0][0].length_m


@pytest.mark.parametrize('values',[{'maximum_push_length_m':.5},{'push_length_scales':[]},
    {'push_length_scales':[float('nan')]},{'maximum_push_length_m':.01},
    {'push_action_cost_m':-.01},{'intermediate_cost_weight':2.}])
def test_invalid_search_configuration_is_rejected(values):
    with pytest.raises(ValueError):Config.from_dict(values)


def test_ranked_futures_remain_inside_workspace_under_all_friction_scales():
    config=Config(obstacles=((.45,.1,.02),))
    pose=(.58,-.03,.2);target=(.57,.02,.1)
    for push,_ in rank_pushes(pose,target,config,limit=24):
        for fraction in (.25,.5,.75,1.):
            for scale in config.friction_scales:
                assert valid_pose(predict(pose,replace(push,length_m=push.length_m*fraction),config,scale),config)


def test_geometry_mask_filters_current_and_future_contact_templates():
    from rm75_app.pusht.response import contact_feature
    config=Config();mask=np.zeros(60,dtype=bool);mask[4]=True
    pose=(.35,-.18,0.)
    rows=rank_pushes(pose,(.45,-.18,0.),config,contact_mask=mask)
    assert rows and all(contact_feature(pose,p,config)==1 for p,_ in rows)
    assert all(np.allclose(p.direction,[1,0]) for p,_ in rows)
    assert rows[0][1]['future_contact_filter']=='current_pose_tool_geometry'
    with pytest.raises(RuntimeError,match='no_valid_future'):
        rank_pushes(pose,(.45,-.18,0.),config,contact_mask=np.zeros(60,dtype=bool))


def test_near_goal_action_is_bounded_by_remaining_pose_correction():
    config=Config();pose=(.35,-.18,0.)
    for p,_ in rank_pushes(pose,(.36,-.18,0.),config):
        assert p.length_m<=.01+config.position_tolerance_m+1e-12
