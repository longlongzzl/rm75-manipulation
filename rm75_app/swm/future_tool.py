"""Original URDF FK for future push candidates, never measured-action evidence."""
import hashlib
from pathlib import Path
import numpy as np
from .scene import digest,pose_error,SceneInvalid


def compile_future_tool_motion(plan,snapshot,geometry,urdf):
    import sapien
    from .native_body_mirror import shape_state,compare_native_state
    from .native_tool_replay import apply_attached_shape_properties
    if (plan.get('complete_chain') is not True or plan.get('validation_success') is not True or
            plan.get('execute_real') is not False or plan.get('hardware_connected') is not False):
        raise SceneInvalid('Original complete nonhardware push candidate required')
    canonical=dict(snapshot);sid=canonical.pop('snapshot_id')
    if digest(canonical)!=sid or not snapshot['valid'] or not snapshot['robot']['idle']:
        raise SceneInvalid('Valid immutable idle initial snapshot required')
    robot_state=snapshot['robot'];arm_names=[f'joint_{i}' for i in range(1,8)]
    if robot_state['joint_names']!=arm_names or robot_state['holding']!='empty':
        raise SceneInvalid('Original empty-tool seven-joint push state required')
    stages=plan['stages']
    if [s['stage'] for s in stages]!=['approach','descend','contact','push','retreat']:
        raise SceneInvalid('Original five push stages required')
    samples=[];elapsed=0.;last=np.asarray(robot_state['positions'],float)
    for stage in stages:
        q=np.asarray(stage['positions'],float);t=np.asarray(stage['times'],float)
        if (q.ndim!=2 or q.shape[1]!=7 or not 2<=len(q)<=100000 or t.shape!=(len(q),) or
                not np.isfinite(q).all() or not np.isfinite(t).all() or abs(t[0])>1e-9 or
                np.any(np.diff(t)<=0) or np.max(np.abs(q[0]-last))>1e-5):
            raise SceneInvalid('Future candidate timing or measured-start continuity invalid')
        for index,(stamp,positions) in enumerate(zip(t,q)):
            if samples and index==0:continue
            samples.append(dict(time_s=elapsed+float(stamp),stage=stage['stage'],positions=positions.tolist()))
        elapsed+=float(t[-1]);last=q[-1]
    if len(samples)>100000 or elapsed>900:raise SceneInvalid('Future motion budget exceeded')
    path=Path(urdf).resolve(strict=True);urdf_digest=hashlib.sha256(path.read_bytes()).hexdigest()
    scene=sapien.Scene()
    try:
        loader=scene.create_urdf_loader();loader.fix_root_link=True
        loader.load_multiple_collisions_from_file=True
        robot=loader.load(str(path));links={link.name:link for link in robot.get_links()}
        names=[joint.name for joint in robot.get_active_joints()]
        jaws=robot_state['gripper_joint_positions']
        if set(names)!=set(arm_names)|set(jaws):raise SceneInvalid('Original robot joint identity changed')
        for name,expected in geometry['links'].items():
            shapes=links[name].collision_shapes
            if len(shapes)!=len(expected):raise SceneInvalid('Original FK tool shape count changed')
            for shape,state in zip(shapes,expected):
                shape.physical_material=sapien.physx.PhysxMaterial(**state['material'])
                shape.set_collision_groups(state['collision_groups'])
                apply_attached_shape_properties(shape,state['properties'])
            actual=[shape_state(shape,np.eye(4)) for shape in shapes]
            compare_native_state(expected,actual,name)
        for row in samples:
            values={**jaws,**dict(zip(arm_names,row['positions']))}
            robot.set_qpos(np.asarray([values[name] for name in names],dtype=np.float32))
            row['T_world_tcp']=links['gripper_tcp'].pose.to_transformation_matrix().tolist()
            row['link_poses']={name:links[name].pose.to_transformation_matrix().tolist() for name in geometry['links']}
        p,r=pose_error(samples[0]['T_world_tcp'],robot_state['T_world_tcp'])
        if p>1e-5 or r>1e-4:raise SceneInvalid('Original planned FK does not match measured initial TCP')
    finally:
        scene.clear()
    result=dict(schema='rm75_future_tool_motion_v1',source='planned_joint_trajectory_original_native_FK',
        source_snapshot_id=sid,source_plan_digest=digest(plan),geometry_digest=digest(geometry),
        urdf=str(path),urdf_sha256=urdf_digest,samples=samples,
        gripper_joint_positions=jaws,gripper_prediction='hold_initial_measured_joint_positions',
        initial_tcp_position_error_m=p,initial_tcp_rotation_error_rad=r,
        measured_action=False,dynamics_predicted=False,execution_authorized=False,hardware_connected=False)
    result['future_motion_digest']=digest(result)
    return result
