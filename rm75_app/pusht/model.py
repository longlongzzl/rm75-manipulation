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
    push_length_m: float = .012
    approach_gap_m: float = .010
    speed_mps: float = .015
    max_steps: int = 60
    success_observations: int = 3
    success_dwell_s: float = .3
    horizon: int = 2
    beam_width: int = 5
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
        for key in ('workspace','obstacles','friction_scales'):
            if key in data:
                data[key] = tuple(data[key])
        result = cls(**data)
        for name in ('bar_width_m','bar_height_m','stem_width_m','stem_height_m'):
            finite(getattr(result,name),name,.002,.5)
        if result.stem_width_m > result.bar_width_m:
            raise ValueError('Stem must not be wider than the T crossbar')
        for name,low,high in [('pusher_radius_m',.001,.03),('push_length_m',.001,.025),
                              ('approach_gap_m',.001,.05),('speed_mps',.001,.05),
                              ('position_tolerance_m',.0001,.05),('yaw_tolerance_rad',.005,.5),
                              ('max_observation_age_s',.05,5),('minimum_improvement',0,.05)]:
            finite(getattr(result,name),name,low,high)
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

    def as_dict(self):
        return {'contact':list(self.contact),'direction':list(self.direction),
                'length_m':self.length_m,'speed_mps':self.speed_mps}


def candidates(pose,config):
    r=rotation(float(pose[2])); origin=np.asarray(pose)[:2]
    bar,stem=rectangles(config)
    x,y,bw,bh=bar; _,sy,sw,sh=stem
    # Exposed edges only: skip the internal crossbar/stem interface.
    contacts=[((-bw/2,y),(1,0)),((bw/2,y),(-1,0)),
              ((-bw*.3,y+bh/2),(0,-1)),((0,y+bh/2),(0,-1)),((bw*.3,y+bh/2),(0,-1)),
              ((-bw*.35,y-bh/2),(0,1)),((bw*.35,y-bh/2),(0,1)),
              ((-sw/2,sy-sh*.3),(1,0)),((sw/2,sy-sh*.3),(-1,0)),
              ((0,sy-sh/2),(0,1))]
    for p,n in contacts:
        direction=r@np.asarray(n,dtype=float)
        # Contact is pusher center tangent to edge, rather than a center in object.
        center=origin+r@np.asarray(p)-direction*config.pusher_radius_m
        begin=center-direction*config.approach_gap_m
        end=center+direction*config.push_length_m
        w=config.workspace; margin=config.pusher_radius_m
        if not all(w[0]+margin<=p[0]<=w[1]-margin and w[2]+margin<=p[1]<=w[3]-margin for p in (begin,end)):
            continue
        safe=True
        for ox,oy,rad in config.obstacles:
            delta=end-begin
            t=np.clip(np.dot(np.array([ox,oy])-begin,delta)/np.dot(delta,delta),0,1)
            if np.linalg.norm(begin+t*delta-[ox,oy]) <= rad+margin:
                safe=False;break
        if safe:
            yield Push(tuple(center),tuple(direction),config.push_length_m,config.speed_mps)


def predict(pose,push,config,scale=1.):
    pose=vector(pose,3,'pose'); d=np.asarray(push.direction)*push.length_m*scale
    lever=np.asarray(push.contact)-pose[:2]
    ell=config.length_scale
    torque=(lever[0]*d[1]-lever[1]*d[0])/(ell*ell+np.dot(lever,lever))
    translation=d*(ell*ell/(ell*ell+np.dot(lever,lever)*.35))
    result=pose.copy();result[:2]+=translation;result[2]=wrap(pose[2]+torque)
    return result


def error(pose,target,config):
    pose=vector(pose,3,'pose');target=vector(target,3,'target')
    return float(np.linalg.norm(pose[:2]-target[:2])+config.length_scale*abs(wrap(pose[2]-target[2])))


def reached(pose,target,config):
    return (np.linalg.norm(np.asarray(pose)[:2]-np.asarray(target)[:2]) <= config.position_tolerance_m
            and abs(wrap(pose[2]-target[2])) <= config.yaw_tolerance_rad)


def choose_push(pose,target,config):
    if not valid_pose(pose,config) or not valid_pose(target,config):
        raise ValueError('Observed/target T geometry crosses the workspace or an obstacle')
    beam=[(error(pose,target,config),np.asarray(pose,dtype=float),None,[])]
    for depth in range(config.horizon):
        expanded=[]
        for _,state,first,history in beam:
            for push in candidates(state,config):
                futures=[predict(state,push,config,s) for s in config.friction_scales]
                if not all(valid_pose(f,config) for f in futures):
                    continue
                nominal=predict(state,push,config)
                costs=[error(f,target,config) for f in futures]
                cost=max(costs)+.15*float(np.std(costs))+.00005*(depth+1)
                expanded.append((cost,nominal,first or push,history+[nominal.tolist()]))
        if not expanded:
            break
        beam=sorted(expanded,key=lambda row:row[0])[:config.beam_width]
    best=min((item for item in beam if item[2] is not None),key=lambda row:row[0],default=None)
    if best is None:
        raise RuntimeError('no_valid_future')
    return best[2], {'model':'quasi_static_surrogate_v1','predicted_cost':best[0],
                     'nominal_future':best[3], 'prediction_is_observation':False}
