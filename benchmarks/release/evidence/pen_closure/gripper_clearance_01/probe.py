"""Advisory exact-convex clearance at the recorded native measured jaw state."""
import hashlib
import json
import sys
from pathlib import Path
from contextlib import ExitStack
import numpy as np
import yaml
sys.path.append('/home/zhangzhao/anaconda3/envs/realman/lib/python3.11/site-packages')
from rm75_app.swm.native_robot_mirror import SapienRobotStatePort
from rm75_app.swm.native_body_mirror import shape_state
from rm75_app.planning.convex_clearance import certify_convex_clearance

root = Path.cwd().resolve()
source = root/'runtime_data/swm_release_pen_worker_64/swm_native/checkpoints/0004.json'
config = root/'assets/curobo_rm75_config/rm75.yml'
snapshot = json.loads(source.read_text())
robot_state = snapshot['robot']
if snapshot.get('valid') is not True or robot_state.get('idle') is not True or robot_state.get('holding') != 'bi':
    raise ValueError('Original measured held idle snapshot required')
positions = dict(zip(robot_state['joint_names'], robot_state['positions']))
positions.update(robot_state['gripper_positions'])
kin = yaml.safe_load(config.read_text())['robot_cfg']['kinematics']
report = dict(domain='advisory_native_convex_clearance', execution_authorized=False,
    policy_override_installed=False, policy_change_approval='NOT_GRANTED', physics_steps=0,
    primary_world_created=False, hardware_connected=False,
    snapshot_id=snapshot['snapshot_id'], source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    config_sha256=hashlib.sha256(config.read_bytes()).hexdigest(), pairs=[])
with ExitStack() as resources:
    port = SapienRobotStatePort('assets/robot_models/RM75_gripper/RM75-B/urdf/RM75-B.urdf', resources=resources)
    report['urdf_sha256'] = port.urdf_sha256
    names = [joint.name for joint in port._robot.get_active_joints()]
    if len(names) != 13 or set(names) != set(positions): raise ValueError('Canonical measured joint identity required')
    q = np.asarray([positions[name] for name in names], dtype=np.float32)
    port._robot.set_qpos(q)
    if not np.array_equal(q, np.asarray(port._robot.get_qpos())): raise ValueError('Native FK readback differs')
    meshes = {}
    for link in port._robot.get_links():
        if link.name not in ('gripper_Left_Support_Link','gripper_Right_Support_Link'): continue
        meshes[link.name] = []
        for shape in link.collision_shapes:
            data = shape_state(shape, np.eye(4))
            if data['kind'] != 'ConvexMesh': raise ValueError('Native convex support shape required')
            points = np.asarray(data['geometry']['vertices'])*np.asarray(data['geometry']['scale'])
            frame = np.asarray(link.entity.pose.to_transformation_matrix()) @ np.asarray(data['T_object_shape'])
            meshes[link.name].append((np.c_[points,np.ones(len(points))] @ frame.T)[:,:3])
    left,right='gripper_Left_Support_Link','gripper_Right_Support_Link'
    if set(meshes) != {left,right} or not all(meshes.values()): raise ValueError('Both complete native meshes required')
    margins = dict(global_first=float(kin['collision_sphere_buffer']),global_second=float(kin['collision_sphere_buffer']),
        self_first=float(kin['self_collision_buffer'][left]),self_second=float(kin['self_collision_buffer'][right]))
    for i,a in enumerate(meshes[left]):
        for j,b in enumerate(meshes[right]):
            report['pairs'].append(dict(left_shape=i,right_shape=j,
                certificate=certify_convex_clearance(a,b,margins=margins)))
report['private_robot_closed'] = port.closed
print(json.dumps(report,indent=2),flush=True)
