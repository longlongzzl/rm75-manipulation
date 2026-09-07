#!/usr/bin/env python3
"""Read-only cuRobo2 tool-envelope evidence for saved authored PushT fixtures.

GPU FK supplies the actual closed simulated gripper geometry. Rigidly placing
that geometry at nominal requested TCP poses is NOT an IK solution or a path:
it only tests a necessary tool/world condition, with no arm/self qualification.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from rm75_app.planning.backends.curobo2 import Curobo2Backend,Curobo2BackendConfig
from rm75_app.planning.contracts import JointConfiguration,Pose
from rm75_app.pusht.model import Config,choose_push
from rm75_app.pusht.motion import CuroboPushExecutor
from rm75_app.pusht.observation import Observation
from rm75_app.workcell.io import atomic_json
from rm75_app.workcell.pickplace_clearance_audit import sphere_box_contacts
from rm75_app.workcell.transforms import quaternion_matrix


def local_spheres(spheres,pose):
    spheres=np.asarray(spheres,dtype=float)
    if spheres.ndim!=2 or spheres.shape[1]!=4 or not len(spheres) or not np.isfinite(spheres).all():
        raise ValueError('Expected finite native sphere geometry')
    result=spheres.copy()
    result[:,:3]=(spheres[:,:3]-pose.position)@quaternion_matrix(pose.quaternion_wxyz)
    return result


def place_spheres(local,pose):
    result=np.asarray(local,dtype=float).copy()
    result[:,:3]=result[:,:3]@quaternion_matrix(pose.quaternion_wxyz).T+pose.position
    return result


def fixture_observation(data):
    allowed={'baseline','orthogonal_tool','rotated','neighbor_blocked','rotated_orthogonal',
             'orthogonal_neighbor_blocked','yaw_matched_tool'}
    if (data.get('scenario')!='authored_simulation_fixture' or data.get('hardware_connected') is not False
            or data.get('execute_real') is not False or data.get('case') not in allowed
            or data.get('fixture') not in ('low_table','raised_table')):
        raise ValueError('Only saved authored no-hardware GPU fixtures are accepted')
    # The original generator did not store observation.pose. Reconstruct its
    # exact named fixture and verify its selected push, never infer a live pose.
    yaw=.3 if data['case'] in ('rotated','rotated_orthogonal','yaw_matched_tool') else 0.
    pose=(.35,-.18,yaw);config=Config.from_dict(data['config'])
    push,_=choose_push(pose,(.38,-.18,yaw),config)
    if push.as_dict()!=data['push']:raise ValueError('Saved push does not match original named fixture')
    return config,push,Observation('authored_envelope_diagnostic',1,time.time(),pose,'simulation')


def tool_geometry(backend):
    planner=backend._ensure_planner();mods=backend._import_modules()
    cfg=planner.kinematics.config.kinematics_config
    ids=cfg.link_sphere_idx_map.detach().cpu().tolist()
    names={int(value):name for name,value in cfg.link_name_to_idx_map.items()}
    links=[str(names[int(index)]) for index in ids]
    joints=tuple(planner.joint_names)
    q=planner.default_joint_state.position.reshape(-1)[:7].detach().cpu().numpy()
    other=q.copy();other[3]+=.1  # FK-only rigidity check, NEVER a requested movement.
    values=[]
    for reference in (q,other):
        state=mods['JointState'].from_position(mods['torch'].as_tensor(reference,
            device=planner.device_cfg.device,dtype=planner.device_cfg.dtype).reshape(1,-1),joint_names=list(joints))
        spheres=planner.kinematics.compute_kinematics(planner.kinematics.get_active_js(state)).robot_spheres
        spheres=spheres.detach().cpu().numpy().reshape(-1,4)
        if len(spheres)!=len(links):raise ValueError('Native sphere mapping changed')
        pose=backend.tool_pose_for_configuration(JointConfiguration(joints,reference),'gripper_tcp')
        values.append(local_spheres(spheres,pose))
    mask=np.array([(name.startswith('gripper_') or name in ('left_pad','right_pad'))
                   and values[0][i,3]>0 for i,name in enumerate(links)])
    if not mask.any():raise ValueError('Closed native gripper geometry missing')
    error=float(np.max(np.linalg.norm(values[0][mask,:3]-values[1][mask,:3],axis=1)))
    if error>1e-6 or not np.array_equal(values[0][mask,3],values[1][mask,3]):
        raise ValueError('Tool spheres are not rigid relative to requested TCP')
    return values[0][mask],[name for name,keep in zip(links,mask) if keep],error


def diagnose(backend,data):
    config,push,observation=fixture_observation(data)
    executor=CuroboPushExecutor(backend,None,config,data['motion'],None,None,None)
    scene=executor._scene(observation)
    if any(obj.kind!='cuboid' for obj in scene.objects):raise ValueError('Envelope comparison requires original cuboids')
    backend.update_scene(scene);backend.set_gripper_collision_state(closed=True)
    local,links,rigid_error=tool_geometry(backend)
    entry=np.asarray(push.contact)-np.asarray(push.direction)*config.approach_gap_m
    z=data['motion']['push_tcp_z_m'];height=data['motion']['hover_clearance_m']
    count=max(1,int(np.ceil(height/.005)))
    poses=[Pose([*entry,value],data['motion']['tool_quaternion_wxyz']) for value in np.linspace(z+height,z,count+1)]
    spheres=np.asarray([place_spheres(local,pose) for pose in poses])
    objects=[NS(name=obj.name,dims=obj.dimensions,pose=obj.pose.as_curobo_list()) for obj in scene.objects]
    contacts=[sphere_box_contacts(row,links,objects) for row in spheres]
    cpu_mask=np.zeros(spheres.shape[:2],dtype=bool)
    for index,rows in enumerate(contacts):
        for row in rows:cpu_mask[index,row['sphere_index']]=True
    planner=backend._ensure_planner();mods=backend._import_modules();torch=mods['torch']
    from curobo._src.geom.collision.buffer_collision import CollisionBuffer
    enabled={obj.name:backend._obstacle_enabled(obj.name) for obj in scene.objects}
    if not all(enabled.values()):raise ValueError('Original world has disabled obstacles')
    before=planner.kinematics.config.kinematics_config.link_spheres.clone()
    tensor=torch.as_tensor(spheres,device=planner.device_cfg.device,dtype=planner.device_cfg.dtype).unsqueeze(1)
    buffer=CollisionBuffer.from_shape(tensor.shape,planner.device_cfg)
    distance=planner.scene_collision_checker.get_sphere_distance_raw(tensor,buffer,
        torch.ones(1,device=tensor.device),torch.zeros(1,device=tensor.device))
    gpu_mask=distance.detach().cpu().numpy().reshape(cpu_mask.shape)>0
    unchanged=(torch.equal(before,planner.kinematics.config.kinematics_config.link_spheres)
        and enabled=={obj.name:backend._obstacle_enabled(obj.name) for obj in scene.objects})
    if not unchanged:raise RuntimeError('Read-only envelope query changed native geometry/world')
    invalid=np.flatnonzero(gpu_mask.any(axis=1)).tolist()
    row=dict(case=data['case'],fixture=data['fixture'],diagnostic_only=True,complete_chain=False,
        hardware_connected=False,execute_real=False,hardware_profile_qualified=False,
        native_tool_spheres=len(links),rigidity_error_m=rigid_error,nominal_descend_intervals=count,
        collision_samples=invalid,first_collision_sample=invalid[0] if invalid else None,
        cpu_gpu_masks_equal=bool(np.array_equal(cpu_mask,gpu_mask)),state_unchanged=unchanged,
        sampled_tool_world_clear=not invalid,arm_self_or_ik_qualified=False,
        contacts=[{'sample':index,'pairs':[{key:pair[key] for key in ('link','obstacle','overlap_m')}
                  for pair in pairs]} for index,pairs in enumerate(contacts) if pairs])
    row['diagnostic_complete']=row['cpu_gpu_masks_equal'] and unchanged
    return row,dict(local_tool_spheres=local.tolist(),links=links,nominal_tcp_poses=[pose.as_curobo_list() for pose in poses])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs',type=Path,nargs='+',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--summary-output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    report=dict(schema='rm75.pusht_tool_envelope/v1',gpu_backend='curobo2',
        diagnostic_only=True,execute_real=False,hardware_connected=False,physical_success=None,
        raw_artifacts_local_only=True,runs=[])
    backend=Curobo2Backend(Curobo2BackendConfig());started=time.monotonic()
    try:
        for path in args.inputs:
            raw=path.read_bytes();data=json.loads(raw)
            row,details=diagnose(backend,data)
            row['input_sha256']=hashlib.sha256(raw).hexdigest();report['runs'].append(row)
            atomic_json(args.output/(data['case']+'.json'),{'summary':row,**details})
    except Exception as exc:
        report['error_type']=type(exc).__name__
        atomic_json(args.output/'error.json',{'error':str(exc)})
        raise
    finally:
        backend.__exit__(None,None,None);report['elapsed_s']=time.monotonic()-started
        atomic_json(args.summary_output,report)
    print(json.dumps({'runs':len(report['runs']),'elapsed_s':report['elapsed_s'],
        'all_diagnostics_complete':all(row['diagnostic_complete'] for row in report['runs'])}))
    return 0 if all(row['diagnostic_complete'] for row in report['runs']) else 42


if __name__=='__main__':raise SystemExit(main())
