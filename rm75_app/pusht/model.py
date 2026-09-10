"""CPU receding-horizon PushT model. Not a learned policy or a physics benchmark.

The bounded quasi-static contact surrogate must be calibrated locally. Predictions
are used to rank short pushes, never as observed proof that an action succeeded.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np
from rm75_app.workcell.io import finite, integer
from rm75_app.workcell.transforms import vector


def wrap(angle):
    return (float(angle) + math.pi) % (2 * math.pi) - math.pi


def rotation(yaw):
    c,s = math.cos(yaw), math.sin(yaw)
    return np.array([[c,-s],[s,c]])


@dataclass(frozen=True)
class Config:
    bar_width_m: float = .10
    bar_height_m: float = .03
    stem_width_m: float = .03
    stem_height_m: float = .07
    pusher_radius_m: float = .005
    push_length_m: float = .012  # Retained as one baseline candidate, not a fixed action.
    maximum_push_length_m: float = .050
    push_length_scales: tuple = (.12, .24, .4, .6, .8, 1.)
    push_direction_angles_rad: tuple = (-.35,0.,.35)
    push_action_cost_m: float = .002
    intermediate_cost_weight: float = .25
    response_fits: tuple = ()
    approach_gap_m: float = .010
    speed_mps: float = .015
    max_steps: int = 60
    success_observations: int = 3
    success_dwell_s: float = .3
    horizon: int = 3
    beam_width: int = 12
    position_tolerance_m: float = .006
    yaw_tolerance_rad: float = .10
    max_observation_age_s: float = 1.5
    stagnation_steps: int = 8
    minimum_improvement: float = .0002
    workspace: tuple = (.15, .65, -.30, .30)
    # Circular no-go objects [base_x,base_y,radius], supplied by calibrated scene.
    obstacles: tuple = ()
    friction_scales: tuple = (.7, 1.0, 1.3)

    @classmethod
    def from_dict(cls, raw):
        if not isinstance(raw, dict):
            raise ValueError('PushT config must be an object')
        unknown = set(raw)-set(cls.__dataclass_fields__)
        if unknown:
            raise ValueError(f'Unknown PushT parameters: {sorted(unknown)}')
        data = dict(raw)
        for key in ('workspace','obstacles','friction_scales','push_length_scales','push_direction_angles_rad','response_fits'):
            if key in data:
                data[key] = tuple(data[key])
        result = cls(**data)
        for name in ('bar_width_m','bar_height_m','stem_width_m','stem_height_m'):
            finite(getattr(result,name),name,.002,.5)
        if result.stem_width_m > result.bar_width_m:
            raise ValueError('Stem must not be wider than the T crossbar')
        for name,low,high in [('pusher_radius_m',.001,.03),('push_length_m',.001,.08),
                              ('maximum_push_length_m',.006,.08),('push_action_cost_m',0,.02),
                              ('intermediate_cost_weight',0,1),
                              ('approach_gap_m',.001,.05),('speed_mps',.001,.05),
                              ('position_tolerance_m',.0001,.05),('yaw_tolerance_rad',.005,.5),
                              ('max_observation_age_s',.05,5),('minimum_improvement',0,.05)]:
            finite(getattr(result,name),name,low,high)
        if result.push_length_m>result.maximum_push_length_m:
            raise ValueError('Baseline push length exceeds maximum push length')
        if not result.push_length_scales or len(result.push_length_scales)>12:
            raise ValueError('Expected 1..12 push length scales')
        for scale in result.push_length_scales:
            finite(scale,'push_length_scale',.02,1.)
        if not result.push_direction_angles_rad or len(result.push_direction_angles_rad)>5:
            raise ValueError('Expected 1..5 push direction angles')
        for angle in result.push_direction_angles_rad:finite(angle,'push_direction_angle',-.6,.6)
        features=set()
        for row in result.response_fits:
            index=integer(row['feature'],'response_feature',0,19)
            if index in features:raise ValueError('Duplicate response feature')
            features.add(index)
            gains=vector(row['gains'],3,'response_gains')
            if not (.1<=gains[0]<=2 and abs(gains[1])<=1 and abs(gains[2])<=12):
                raise ValueError('Invalid observed response gains')
            integer(row['samples'],'response_samples',1,10000)
            vector(row['sum_xy'],3,'response_sum_xy')
            if np.any(vector(row['sum_xx'],3,'response_sum_xx')<0):raise ValueError('Invalid response fit')
        integer(result.max_steps,'max_steps',1,500)
        integer(result.success_observations,'success_observations',2,10)
        finite(result.success_dwell_s,'success_dwell_s',.1,2)
        integer(result.horizon,'horizon',1,3)
        integer(result.beam_width,'beam_width',1,12)
        integer(result.stagnation_steps,'stagnation_steps',2,50)
        w = vector(result.workspace,4,'workspace')
        if w[0] >= w[1] or w[2] >= w[3]:
            raise ValueError('Workspace must be [xmin,xmax,ymin,ymax]')
        if not result.friction_scales or len(result.friction_scales)>7:
            raise ValueError('Expected 1..7 friction scales')
        for scale in result.friction_scales:
            finite(scale,'friction_scale',.1,3)
        for obstacle in result.obstacles:
            v = vector(obstacle,3,'obstacle')
            if v[2] <= 0:
                raise ValueError('Obstacle radius must be positive')
        return result

    @property
    def length_scale(self):
        return max(self.bar_width_m,self.stem_height_m+self.bar_height_m) / 2


def rectangles(config):
    # Origin is area centroid, not the marker center and not crossbar center.
    bw,bh,sw,sh = config.bar_width_m,config.bar_height_m,config.stem_width_m,config.stem_height_m
    com_y = -(sw*sh)*(bh/2+sh/2)/(bw*bh+sw*sh)
    return [(0., -com_y, bw, bh), (0., -bh/2-sh/2-com_y, sw, sh)]


def vertices(pose, config):
    pose = vector(pose,3,'pose')
    r = rotation(pose[2])
    output=[]
    for x,y,w,h in rectangles(config):
        output.append(np.array([[x-w/2,y-h/2],[x+w/2,y-h/2],
                                [x+w/2,y+h/2],[x-w/2,y+h/2]]) @ r.T + pose[:2])
    return output


def valid_pose(pose,config):
    try:
        polygons = vertices(pose,config)
    except (ValueError, TypeError):
        return False
    w=config.workspace
    points=np.concatenate(polygons)
    if not ((points[:,0]>=w[0]) & (points[:,0]<=w[1]) & (points[:,1]>=w[2]) & (points[:,1]<=w[3])).all():
        return False
    # Conservative circle-vs-each-rectangle test in object coordinates.
    local_r=rotation(float(pose[2])).T
    for ox,oy,radius in config.obstacles:
        local=local_r@(np.array([ox,oy])-np.asarray(pose)[:2])
        for x,y,ww,hh in rectangles(config):
            nearest=np.clip(local,[x-ww/2,y-hh/2],[x+ww/2,y+hh/2])
            if np.linalg.norm(local-nearest) <= radius:
                return False
    return True


@dataclass(frozen=True)
class Push:
    contact: tuple[float,float]
    direction: tuple[float,float]
    length_m: float
    speed_mps: float
    normal: tuple[float,float] | None = None

    def as_dict(self):
        return {'contact':list(self.contact),'direction':list(self.direction),
                'length_m':self.length_m,'speed_mps':self.speed_mps,
                **({'normal':list(self.normal)} if self.normal is not None else {})}


def candidates(pose,config):
    from .batch_search import action_arrays,action_normals
    centers,directions,lengths,valid=action_arrays(np.asarray(pose,dtype=float).reshape(1,3),config)
    normals=action_normals(directions,config)
    for index in np.flatnonzero(valid[0]):
        yield Push(tuple(centers[0,index]),tuple(directions[0,index]),float(lengths[index]),config.speed_mps,tuple(normals[0,index]))


def predict(pose,push,config,scale=1.):
    pose=vector(pose,3,'pose'); d=np.asarray(push.direction)*push.length_m*scale
    lever=np.asarray(push.contact)-pose[:2]
    ell=config.length_scale
    torque=(lever[0]*d[1]-lever[1]*d[0])/(ell*ell+np.dot(lever,lever))
    translation=d*(ell*ell/(ell*ell+np.dot(lever,lever)*.35))
    if config.response_fits:
        from .response import contact_feature,response_gains
        along,lateral,angular=response_gains(config)[contact_feature(pose,push,config)]
        translation=translation*along+np.array([-push.direction[1],push.direction[0]])*push.length_m*scale*lateral
        torque*=angular
    result=pose.copy();result[:2]+=translation;result[2]=wrap(pose[2]+torque)
    return result


def error(pose,target,config):
    pose=vector(pose,3,'pose');target=vector(target,3,'target')
    return float(np.linalg.norm(pose[:2]-target[:2])+config.length_scale*abs(wrap(pose[2]-target[2])))


def reached(pose,target,config):
    return (np.linalg.norm(np.asarray(pose)[:2]-np.asarray(target)[:2]) <= config.position_tolerance_m
            and abs(wrap(pose[2]-target[2])) <= config.yaw_tolerance_rad)


def choose_push(pose,target,config):
    from .batch_search import rank_pushes
    return rank_pushes(pose,target,config,limit=1)[0]
