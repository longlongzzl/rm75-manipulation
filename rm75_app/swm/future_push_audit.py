"""Original PushT collision checks against dense private predicted scenes."""
from types import SimpleNamespace
import numpy as np
from .scene import SceneInvalid,pose_error,transform,digest
from .future_collision_samples import future_collision_samples


def audit_future_push(executor,motion,prediction,snapshot,object_id, *, progress=lambda row:None):
    from dataclasses import asdict
    from .physics_replay import compound_cuboid_parts
    from rm75_app.workcell.transforms import quaternion_matrix
    from rm75_app.pusht.retreat_contact import validate_contact_escape
    samples=future_collision_samples(motion,prediction,snapshot,object_id)
    if any(not obj['fixed'] for oid,obj in snapshot['objects'].items() if oid!=object_id):
        raise SceneInvalid('Dynamic non-target future scene adapter required')
    def observation(matrix,index):
        matrix=transform(matrix)
        if abs(matrix[2,3]-executor.profile['object_centroid_z_m'])>1e-5 or np.linalg.norm(matrix[:3,2]-[0,0,1])>1e-4:
            raise SceneInvalid('Original planar PushT collision model cannot represent predicted target pose')
        return SimpleNamespace(pose=[matrix[0,3],matrix[1,3],float(np.arctan2(matrix[1,0],matrix[0,0]))],
            session_id='private_future_'+motion['future_motion_digest'],sequence=index)
    initial=executor._scene(observation(snapshot['objects'][object_id]['measured']['T_world_object'],0))
    expected_names={oid for oid in snapshot['objects'] if oid!=object_id}|{'pusht_target_0','pusht_target_1'}
    if {obj.name for obj in initial.objects}!=expected_names:raise SceneInvalid('Original future audit scene coverage differs')
    native_objects={obj.name:obj for obj in initial.objects}
    for oid,obj in snapshot['objects'].items():
        asset=snapshot['assets'][obj['asset_id']]
        if asset['collision_kind']!='compound_cuboids':raise SceneInvalid('Original future cuboid geometry adapter required')
        parts=compound_cuboid_parts(asset)
        names=['pusht_target_0','pusht_target_1'] if oid==object_id else [oid]
        if len(parts)!=len(names):raise SceneInvalid('Original future part count differs')
        for name,(local,dimensions) in zip(names,parts):
            original=native_objects[name];matrix=np.eye(4)
            matrix[:3,3]=original.pose.position;matrix[:3,:3]=quaternion_matrix(original.pose.quaternion_wxyz)
            expected=transform(obj['measured']['T_world_object'])@transform(asset.get('T_object_collision',np.eye(4)))@local
            p,r=pose_error(matrix,expected)
            if original.kind!='cuboid' or p>1e-6 or r>1e-6 or not np.allclose(original.dimensions,dimensions,atol=1e-7,rtol=1e-6):
                raise SceneInvalid('Original future collision geometry differs from SWM')
    backend=executor.backend;backend.update_scene(initial)
    backend.set_measured_gripper_collision_state(snapshot['robot']['gripper_joint_positions'])
    planner=backend._ensure_planner();mods=backend._import_modules()
    if tuple(planner.joint_names)!=tuple(executor.names):raise SceneInvalid('Original audit joint order changed')
    limits=planner.kinematics.get_joint_limits().position.detach().cpu().numpy()
    if limits.shape!=(2,7):raise SceneInvalid('Original audit joint limits unavailable')
    retreat=[];counts={}
    try:
        for index,row in enumerate(samples):
            executor.stop.check();q=np.asarray(row['positions'],float)[None,:];stage=row['stage']
            progress(dict(sample=index,stage=stage,time_s=row['time_s']))
            if np.any(q<limits[0]) or np.any(q>limits[1]):raise SceneInvalid('Future candidate exceeds original joint limits')
            backend.update_scene(executor._scene(observation(row['T_world_object'],index+1)))
            # Original link allowlist applies only in the original contact phases.
            executor._audit(q,contact=stage in ('contact','push','retreat'))
            if stage=='retreat':
                state=mods['JointState'].from_position(mods['torch'].as_tensor(q,
                    device=planner.device_cfg.device,dtype=planner.device_cfg.dtype),joint_names=list(executor.names))
                retreat.append(backend._collision_diagnostics_for_states(planner,{'path':state}))
            counts[stage]=counts.get(stage,0)+1
        escape=validate_contact_escape(retreat,executor.allowed)
    finally:
        backend.update_scene(initial)
        backend.set_measured_gripper_collision_state(snapshot['robot']['gripper_joint_positions'])
    return dict(source='original_CuroboPushExecutor_dynamic_scene_collision_audit',samples=len(samples),
        source_snapshot_id=snapshot['snapshot_id'],source_plan_digest=motion['source_plan_digest'],
        future_motion_digest=motion['future_motion_digest'],prediction_digest=digest(prediction),
        parameters_digest=digest(prediction['parameters']),model_digest=digest(asdict(executor.config)),
        motion_profile_digest=digest(executor.profile),object_id=object_id,
        stages=counts,retreat=escape,allowed_contact_links=sorted(executor.allowed),
        original_joint_limits_checked=True,collision_samples_checked=True,
        full_primitive_audit_issued=False,execution_authorized=False)
