"""Vectorized finite-candidate PushT search using the current T geometry.

All candidate/contact/length/friction rows share NumPy batches. Only the first
action is executed; predictions are never observations or a global optimum.
"""
import time

import numpy as np


def contact_features(config):
    from .model import rectangles
    (_,y,bw,bh),(_,sy,sw,sh)=rectangles(config)
    contacts=[];normals=[]
    def add(point,normal):
        contacts.append(point);normals.append(normal)
    for x,normal in ((-bw/2,(1,0)),(bw/2,(-1,0))):
        for yy in (y-bh*.4,y,y+bh*.4):add((x,yy),normal)
    for xx in (-bw*.3,0,bw*.3):add((xx,y+bh/2),(0,-1))
    for xx in (-bw*.35,bw*.35):add((xx,y-bh/2),(0,1))
    for x,normal in ((-sw/2,(1,0)),(sw/2,(-1,0))):
        for yy in sorted(set((sy-sh*.3,sy,sy+sh*.3,float(np.clip(0,sy-sh*.4,sy+sh*.4))))):
            add((x,yy),normal)
    add((0,sy-sh/2),(0,1))
    return np.asarray(contacts),np.asarray(normals)


def action_arrays(poses,config):
    """Return [state, contact*length] world actions and pusher sweep validity."""
    points,normals=contact_features(config)
    lengths=np.unique(np.r_[np.asarray(config.push_length_scales)*config.maximum_push_length_m,
                            config.push_length_m])
    lengths=lengths[lengths<=config.maximum_push_length_m+1e-12]
    c=np.cos(poses[:,2]);s=np.sin(poses[:,2])
    rotation=np.stack((c,-s,s,c),axis=-1).reshape(-1,2,2)
    directions=np.einsum('bij,fj->bfi',rotation,normals)
    surface=poses[:,None,:2]+np.einsum('bij,fj->bfi',rotation,points)
    centers=surface-directions*config.pusher_radius_m
    angles=np.asarray(config.push_direction_angles_rad)
    dx=directions[:,:,None,0]*np.cos(angles)-directions[:,:,None,1]*np.sin(angles)
    dy=directions[:,:,None,0]*np.sin(angles)+directions[:,:,None,1]*np.cos(angles)
    directions=np.stack((dx,dy),axis=-1).reshape(len(poses),-1,2)
    centers=np.repeat(centers,len(angles)*len(lengths),axis=1)
    directions=np.repeat(directions,len(lengths),axis=1)
    distances=np.tile(lengths,len(points)*len(angles))
    begin=centers-directions*config.approach_gap_m
    end=centers+directions*distances[None,:,None]
    xmin,xmax,ymin,ymax=config.workspace;r=config.pusher_radius_m
    valid=np.ones(centers.shape[:2],dtype=bool)
    for xyz in (begin,end):
        valid&=(xyz[...,0]>=xmin+r)&(xyz[...,0]<=xmax-r)&(xyz[...,1]>=ymin+r)&(xyz[...,1]<=ymax-r)
    delta=end-begin
    for ox,oy,radius in config.obstacles:
        obstacle=np.array([ox,oy])
        t=np.clip(np.sum((obstacle-begin)*delta,axis=-1)/np.sum(delta*delta,axis=-1),0,1)
        valid&=np.linalg.norm(begin+t[...,None]*delta-obstacle,axis=-1)>radius+r
    return centers,directions,distances,valid


def action_normals(directions,config):
    count=len(config.push_direction_angles_rad)
    lengths=directions.shape[1]//len(contact_features(config)[0])//count
    angles=np.tile(np.repeat(config.push_direction_angles_rad,lengths),len(contact_features(config)[0]))
    c=np.cos(angles);s=np.sin(angles)
    return np.stack((c*directions[...,0]+s*directions[...,1],-s*directions[...,0]+c*directions[...,1]),axis=-1)


def predict_batch(poses,centers,directions,lengths,config,scales):
    """[B, A, S, 3] predictions, equivalent to scalar predict at every row."""
    lever=centers-poses[:,None,:2]
    squared=np.sum(lever*lever,axis=-1);ell2=config.length_scale**2
    delta=directions[:,:,None,:]*np.asarray(lengths)[None,:,None,None]*np.asarray(scales)[None,None,:,None]
    translation=delta*(ell2/(ell2+squared*.35))[:,:,None,None]
    torque=(lever[:,:,None,0]*delta[...,1]-lever[:,:,None,1]*delta[...,0])/(ell2+squared)[:,:,None]
    if config.response_fits:
        from .response import response_gains
        gains=response_gains(config)
        gains=np.repeat(gains,len(lengths)//len(gains),axis=0)
        perpendicular=np.stack((-delta[...,1],delta[...,0]),axis=-1)
        translation=translation*gains[None,:,None,0,None]+perpendicular*gains[None,:,None,1,None]
        torque*=gains[None,:,None,2]
    result=np.broadcast_to(poses[:,None,None,:],(*delta.shape[:-1],3)).copy()
    result[...,:2]+=translation
    result[...,2]=(result[...,2]+torque+np.pi)%(2*np.pi)-np.pi
    return result


def valid_pose_batch(poses,config):
    from .model import rectangles
    poses=np.asarray(poses);c=np.cos(poses[...,2]);s=np.sin(poses[...,2])
    xmin,xmax,ymin,ymax=config.workspace
    valid=np.isfinite(poses).all(axis=-1)
    for x,y,w,h in rectangles(config):
        corners=np.array([[x-w/2,y-h/2],[x+w/2,y-h/2],[x+w/2,y+h/2],[x-w/2,y+h/2]])
        px=poses[...,0,None]+c[...,None]*corners[:,0]-s[...,None]*corners[:,1]
        py=poses[...,1,None]+s[...,None]*corners[:,0]+c[...,None]*corners[:,1]
        valid&=((px>=xmin)&(px<=xmax)&(py>=ymin)&(py<=ymax)).all(axis=-1)
        for ox,oy,radius in config.obstacles:
            dx=ox-poses[...,0];dy=oy-poses[...,1]
            local=np.stack((c*dx+s*dy,-s*dx+c*dy),axis=-1)
            nearest=np.clip(local,[x-w/2,y-h/2],[x+w/2,y+h/2])
            valid&=np.linalg.norm(local-nearest,axis=-1)>radius
    return valid


def error_batch(poses,target,config):
    yaw=(poses[...,2]-target[2]+np.pi)%(2*np.pi)-np.pi
    return np.linalg.norm(poses[...,:2]-target[:2],axis=-1)+config.length_scale*np.abs(yaw)


def rank_pushes(pose,target,config,*,limit=None,contact_mask=None):
    """Rank distinct first actions from a bounded vectorized beam search."""
    from .model import Push,valid_pose
    tick=time.perf_counter();target=np.asarray(target,dtype=float)
    if not valid_pose(pose,config) or not valid_pose(target,config):
        raise ValueError('Observed/target T geometry crosses the workspace or an obstacle')
    beam=[(np.asarray(pose,dtype=float),None,[],0.)]
    best_by_first={};counts=[]
    for depth in range(config.horizon):
        states=np.stack([row[0] for row in beam])
        centers,directions,lengths,valid=action_arrays(states,config)
        normals=action_normals(directions,config)
        if contact_mask is not None:
            mask=np.asarray(contact_mask,dtype=bool)
            expected=len(contact_features(config)[0])*len(config.push_direction_angles_rad)
            if mask.shape!=(expected,):raise ValueError('Invalid contact direction mask')
            valid&=np.repeat(mask,len(lengths)//expected)[None,:]
        distance=np.linalg.norm(states[:,:2]-target[:2],axis=-1)
        yaw=np.abs((states[:,2]-target[2]+np.pi)%(2*np.pi)-np.pi)
        # Near the goal, do not issue a stroke much longer than the remaining
        # pose correction. A poorly fitted contact must not cause a 50 mm nudge.
        correction_limit=np.maximum(distance+config.position_tolerance_m,config.length_scale*yaw)
        valid&=lengths[None,:]<=correction_limit[:,None]+1e-12
        futures=predict_batch(states,centers,directions,lengths,config,config.friction_scales)
        nominal=predict_batch(states,centers,directions,lengths,config,[1.])[:,:,0,:]
        # Check uncertainty scenarios along the entire predicted sweep.
        for fraction in (.25,.5,.75,1.):
            swept=predict_batch(states,centers,directions,lengths*fraction,config,config.friction_scales)
            valid&=valid_pose_batch(swept,config).all(axis=-1)
        errors=error_batch(futures,target,config)
        stage_cost=errors.max(axis=-1)+.15*errors.std(axis=-1)
        history_cost=np.asarray([row[3] for row in beam])[:,None]
        costs=stage_cost+history_cost+config.push_action_cost_m*(depth+1)
        costs[~valid]=np.inf
        counts.append(dict(depth=depth+1,candidate_actions=int(valid.size),valid_actions=int(valid.sum()),
                           friction_rollouts=int(valid.size*len(config.friction_scales))))
        if not valid.any():break
        # Save the best finite future for each first action; the beam only
        # limits future expansion, not the availability of current alternatives.
        for b,a in zip(*np.nonzero(valid)):
            first=beam[b][1] or Push(tuple(centers[b,a]),tuple(directions[b,a]),float(lengths[a]),config.speed_mps,tuple(normals[b,a]))
            history=beam[b][2]+[nominal[b,a].tolist()]
            key=(*first.contact,*first.direction,first.length_m)
            row=(float(costs[b,a]),first,history)
            if key not in best_by_first or row[0]<best_by_first[key][0]:best_by_first[key]=row
        next_beam=[];per_first={}
        for flat in np.argsort(costs,axis=None,kind='stable'):
            b,a=np.unravel_index(flat,costs.shape)
            if not np.isfinite(costs[b,a]):break
            first=beam[b][1] or Push(tuple(centers[b,a]),tuple(directions[b,a]),float(lengths[a]),config.speed_mps,tuple(normals[b,a]))
            key=(*first.contact,*first.direction,first.length_m)
            if per_first.get(key,0)>=2:continue
            per_first[key]=per_first.get(key,0)+1
            next_beam.append((nominal[b,a],first,beam[b][2]+[nominal[b,a].tolist()],
                              float(history_cost[b,0]+config.intermediate_cost_weight*stage_cost[b,a])))
            if len(next_beam)>=config.beam_width:break
        beam=next_beam
    if not best_by_first:raise RuntimeError('no_valid_future')
    ranked=sorted(best_by_first.values(),key=lambda row:row[0])
    report=dict(model='quasi_static_surrogate_v1',search='numpy_vectorized_contact_length_beam',
        prediction_is_observation=False,global_optimum_proven=False,
        future_contact_filter='current_pose_tool_geometry' if contact_mask is not None else None,
        contact_features=len(contact_features(config)[0]),direction_offsets_rad=list(config.push_direction_angles_rad),search_batches=counts,
        evaluated_friction_rollouts=sum(row['friction_rollouts'] for row in counts),
        maximum_push_length_m=config.maximum_push_length_m,planning_wall_s=time.perf_counter()-tick,
        ranked_first_actions=len(ranked))
    output=[]
    for rank,(cost,push,history) in enumerate(ranked[:limit]):
        output.append((push,{**report,'rank':rank,'predicted_cost':cost,'nominal_future':history}))
    return output
