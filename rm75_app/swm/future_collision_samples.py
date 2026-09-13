"""Bind original candidate joints to dense predicted target poses for auditing.

This prepares the original collision auditor's inputs; it does not issue an audit.
"""
import numpy as np
from .scene import digest,transform,pose_error,SceneInvalid


def future_collision_samples(motion,prediction,snapshot,object_id):
    canonical=dict(motion);claimed=canonical.pop('future_motion_digest')
    if digest(canonical)!=claimed:raise SceneInvalid('Future motion changed before path audit')
    if (prediction.get('schema')!='rm75_future_prediction_v1' or prediction.get('mode')!='future_prediction' or
            prediction.get('valid') is not True or prediction.get('engine_domain')!='physics' or
            prediction.get('identification_eligible') is not False or prediction.get('object_id')!=object_id or
            prediction.get('initial_snapshot_id')!=snapshot['snapshot_id'] or
            motion['source_snapshot_id']!=snapshot['snapshot_id'] or
            prediction.get('future_motion_digest')!=claimed or
            prediction.get('source_plan_digest')!=motion['source_plan_digest'] or
            prediction.get('native_tool_geometry_digest')!=motion['geometry_digest']):
        raise SceneInvalid('Future collision evidence identity mismatch')
    rows=motion['samples'];times=[r['time_s'] for r in rows]
    action=dict(source='planned_trajectory',time_s=times,T_world_tcp=[r['T_world_tcp'] for r in rows],
                stages=[r['stage'] for r in rows])
    if prediction.get('plan_action_digest')!=digest(action):raise SceneInvalid('Future candidate action changed')
    if not np.array_equal(prediction['time_s'],times) or len(prediction['T_world_object'])!=len(rows):
        raise SceneInvalid('Dense target prediction at every candidate joint sample required')
    if not 2<=len(rows)<=100000 or np.any(np.diff(times)<=0):raise SceneInvalid('Invalid future audit sample times')
    q=np.asarray([r['positions'] for r in rows],float)
    if (q.shape!=(len(rows),7) or not np.isfinite(q).all() or
            np.max(np.abs(q[0]-np.asarray(snapshot['robot']['positions'])))>1e-5 or
            np.max(np.abs(np.diff(q,axis=0)))>.01):
        raise SceneInvalid('Future collision samples require measured start and <=0.01 rad spacing')
    poses=[transform(pose).tolist() for pose in prediction['T_world_object']]
    p,r=pose_error(poses[0],snapshot['objects'][object_id]['measured']['T_world_object'])
    if p>1e-6 or r>1e-6:raise SceneInvalid('Future collision scene starts from another target pose')
    return [dict(time_s=row['time_s'],stage=row['stage'],positions=row['positions'],T_world_object=pose)
            for row,pose in zip(rows,poses)]
