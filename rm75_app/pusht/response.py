"""Fit per-contact planar response from completed, observed pushes only."""
from dataclasses import replace

import numpy as np


def response_gains(config):
    from .batch_search import contact_features
    points,normals=contact_features(config)
    gains=np.tile([1.,0.,1.],(len(points),1))
    for feature in range(len(points)):
        compatible=[row for row in config.response_fits
                    if np.array_equal(normals[int(row['feature'])],normals[feature])]
        if compatible:
            nearest=min(compatible,key=lambda row:np.linalg.norm(points[int(row['feature'])]-points[feature]))
            gains[feature]=nearest['gains']
    for row in config.response_fits:gains[int(row['feature'])]=row['gains']
    return gains


def contact_feature(pose,push,config):
    from .model import rotation
    from .batch_search import contact_features
    local=rotation(pose[2]).T@(np.asarray(push.contact)+np.asarray(push.normal if push.normal is not None else push.direction)*config.pusher_radius_m-np.asarray(pose)[:2])
    points,_=contact_features(config)
    surface_local=rotation(pose[2]).T@(np.asarray(push.contact)-np.asarray(pose)[:2])
    distance=np.minimum(np.linalg.norm(points-local,axis=1),np.linalg.norm(points-surface_local,axis=1))
    index=int(np.argmin(distance))
    if distance[index]>.0001:raise ValueError('Observed push is not a configured contact feature')
    return index


class ResponseEstimator:
    def __init__(self,config):
        self.base=replace(config,response_fits=())
        self.rows={int(row['feature']):dict(row) for row in config.response_fits}

    def update(self,before,push,after):
        from .model import predict,wrap
        feature=contact_feature(before,push,self.base)
        predicted=predict(before,push,self.base);direction=np.asarray(push.direction)
        lateral=np.array([-direction[1],direction[0]])
        actual=np.asarray(after)[:2]-np.asarray(before)[:2]
        expected=predicted[:2]-np.asarray(before)[:2]
        expected_yaw=wrap(predicted[2]-before[2]);actual_yaw=wrap(after[2]-before[2])
        x=np.array([float(expected@direction),push.length_m,expected_yaw])
        y=np.array([float(actual@direction),float(actual@lateral),actual_yaw])
        if not np.isfinite([*x,*y]).all():raise ValueError('Nonfinite observed push response')
        old=self.rows.get(feature,{})
        xy=np.asarray(old.get('sum_xy',[0.,0.,0.]))+x*y
        xx=np.asarray(old.get('sum_xx',[0.,0.,0.]))+x*x
        gains=np.divide(xy,xx,out=np.array([1.,0.,1.]),where=xx>1e-9)
        gains=np.clip(gains,[.1,-1.,-12.],[2.,1.,12.])
        self.rows[feature]=dict(feature=feature,gains=gains.tolist(),samples=int(old.get('samples',0))+1,
                                sum_xy=xy.tolist(),sum_xx=xx.tolist())
        return dict(self.rows[feature])

    def config(self):
        return replace(self.base,response_fits=tuple(self.rows[k] for k in sorted(self.rows)))
