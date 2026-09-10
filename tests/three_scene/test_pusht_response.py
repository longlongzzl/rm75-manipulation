from dataclasses import asdict,replace

import numpy as np
import pytest

from rm75_app.pusht.model import Config,Push,predict
from rm75_app.pusht.batch_search import action_normals,action_arrays,predict_batch
from rm75_app.pusht.response import ResponseEstimator,contact_feature,response_gains


def test_fit_uses_observed_translation_side_slip_and_yaw_then_matches_batch():
    cfg=Config();pose=np.array([.35,-.18,.1])
    centers,directions,lengths,_=action_arrays(pose[None],cfg)
    normals=action_normals(directions,cfg)
    push=Push(tuple(centers[0,3]),tuple(directions[0,3]),float(lengths[3]),cfg.speed_mps,tuple(normals[0,3]))
    nominal=predict(pose,push,cfg);d=np.asarray(push.direction)
    actual=pose.copy();actual[:2]+=(nominal[:2]-pose[:2])*.8+np.array([-d[1],d[0]])*push.length_m*(-.1)
    actual[2]+=(nominal[2]-pose[2])*2.5
    estimator=ResponseEstimator(cfg);row=estimator.update(pose,push,actual)
    np.testing.assert_allclose(row['gains'],[.8,-.1,2.5],atol=1e-12)
    calibrated=Config.from_dict(asdict(estimator.config()))
    np.testing.assert_allclose(predict(pose,push,calibrated),actual,atol=1e-12)
    batch=predict_batch(pose[None],centers,directions,lengths,calibrated,[.7,1.,1.3])
    for i,length in enumerate(lengths):
        action=Push(tuple(centers[0,i]),tuple(directions[0,i]),float(length),cfg.speed_mps,tuple(normals[0,i]))
        for j,scale in enumerate((.7,1.,1.3)):
            np.testing.assert_allclose(batch[0,i,j],predict(pose,action,calibrated,scale),atol=1e-12)
    assert estimator.config().response_fits[0]['samples']==1


def test_contact_feature_accepts_both_pusher_center_and_bound_surface():
    cfg=Config();pose=np.array([.35,-.18,.2]);c,d,l,_=action_arrays(pose[None],cfg)
    normals=action_normals(d,cfg)
    action=Push(tuple(c[0,9]),tuple(d[0,9]),float(l[9]),cfg.speed_mps,tuple(normals[0,9]))
    surface=replace(action,contact=tuple(np.asarray(action.contact)+normals[0,9]*cfg.pusher_radius_m))
    assert contact_feature(pose,action,cfg)==contact_feature(pose,surface,cfg)


@pytest.mark.parametrize('gains',[[0,0,1],[1,2,1],[1,0,20],[float('nan'),0,1]])
def test_invalid_response_cannot_enter_planning(gains):
    with pytest.raises(ValueError):Config.from_dict({'response_fits':[dict(feature=0,gains=gains,samples=1,sum_xy=[0,0,0],sum_xx=[1,1,1])]})
