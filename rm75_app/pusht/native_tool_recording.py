"""Native collision geometry and measured per-link motion, without replay claims."""
import numpy as np
from rm75_app.swm.native_body_mirror import shape_state
from rm75_app.swm.scene import digest,pose_error,transform,SceneInvalid
from rm75_app.swm.physical_recording import _clock


def export_tool_geometry(links):
    geometry={}
    for link in links:
        if len(link._bodies)!=1:raise SceneInvalid('One native body per tool link required')
        shapes=[shape_state(shape,np.eye(4)) for shape in link._bodies[0].collision_shapes]
        if shapes:geometry[link.name]=shapes
    if not geometry:raise SceneInvalid('Original native tool collision geometry missing')
    return dict(schema='rm75_native_tool_geometry_v1',frame='each_original_link',links=geometry,
                source='original_PhysX_collision_shape_readback',hardware_connected=False)


def bind_tool_motion(transition,geometry,rows):
    """Check one-to-one physical-time/TCP agreement with the existing transition."""
    identity,start=_clock(transition['initial_snapshot']['robot']['simulation_clock'])
    links=set(geometry['links']);samples=[];previous=None
    for row in rows:
        current,stamp=_clock(row['simulation_clock'])
        if current!=identity:raise SceneInvalid('Native tool clock identity changed')
        if set(row['link_poses'])!=links:raise SceneInvalid('Native tool link coverage changed')
        poses={name:transform(value).tolist() for name,value in row['link_poses'].items()}
        sample=dict(time_s=stamp-start,T_world_tcp=transform(row['T_world_tcp']).tolist(),
                    link_poses=poses,gripper_joint_positions=row['gripper_joint_positions'])
        if previous is not None and sample['time_s']<=previous['time_s']:
            if sample['time_s']!=previous['time_s']:raise SceneInvalid('Native tool time regressed')
            for name in links:
                p,r=pose_error(poses[name],previous['link_poses'][name])
                if p>1e-9 or r>1e-7:raise SceneInvalid('Native tool moved at unchanged physical time')
            continue
        samples.append(sample);previous=sample
    action=transition['actual_action']
    if len(samples)!=len(action['time_s']):raise SceneInvalid('Native tool samples do not cover measured action')
    for row,stamp,tcp in zip(samples,action['time_s'],action['T_world_tcp']):
        p,r=pose_error(row['T_world_tcp'],tcp)
        if abs(row['time_s']-stamp)>1e-9 or p>1e-9 or r>1e-7:
            raise SceneInvalid('Native tool motion differs from measured TCP action')
    result=dict(schema='rm75_native_tool_motion_v1',action_id=transition['action_id'],
        action_digest=transition['action_digest'],transition_digest=transition['transition_digest'],
        geometry_digest=digest(geometry),source='native_link_pose_readback',samples=samples,
        replay_qualified=False,hardware_connected=False)
    result['motion_digest']=digest(result)
    return result
