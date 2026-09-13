"""Read native collision mesh extrema at recorded private states, not contact calibration."""
import json
import sys
from contextlib import ExitStack
from pathlib import Path
import numpy as np
sys.path.append('/home/zhangzhao/anaconda3/envs/realman/lib/python3.11/site-packages')
from rm75_app.swm.native_robot_mirror import SapienRobotStatePort
from rm75_app.swm.native_body_mirror import shape_state
from rm75_app.swm.scene import SceneInvalid

inputs = sorted(Path('runtime_data/swm_release_pen_worker_43/swm_native/closure_priority').resolve().glob('*.json'))
if len(inputs) != 34:
    raise SceneInvalid('Expected frozen worker 43 prediction inventory')
report = dict(domain='native_private_collision_mesh_FK', physics_steps=0,
              primary_world_created=False, contact_calibration_qualified=False, rows=[])
with ExitStack() as resources:
    port = SapienRobotStatePort('assets/robot_models/RM75_gripper/RM75-B/urdf/RM75-B.urdf', resources=resources)
    report['urdf_sha256'] = port.urdf_sha256
    names = [joint.name for joint in port._robot.get_active_joints()]
    links = {link.name: link for link in port._robot.get_links()}
    supports = {name: link for name, link in links.items() if 'Support' in name}
    if len(supports) != 2:
        raise SceneInvalid('Expected original two support links')
    for path in inputs:
        evidence = json.loads(path.read_text())
        opening = evidence['candidate_open_preparation']
        incoming = opening['joint_names']
        if len(incoming) != len(names) or set(incoming) != set(names):
            raise SceneInvalid('Recorded/native joint identities differ')
        for stage, values in [('open_prepared', opening['positions']), ('last_rejected_state', evidence['steps'][-1]['q'])]:
            q = np.asarray([values[incoming.index(name)] for name in names], dtype=np.float32)
            port._robot.set_qpos(q)
            if np.max(np.abs(np.asarray(port._robot.get_qpos()) - q)) > 1e-6:
                raise SceneInvalid('Private FK readback mismatch')
            tcp = np.asarray(links['gripper_tcp'].entity.pose.to_transformation_matrix(), dtype=float)
            for name, link in supports.items():
                if not link.collision_shapes:
                    raise SceneInvalid('Support collision geometry missing')
                for index, shape in enumerate(link.collision_shapes):
                    state = shape_state(shape, np.eye(4))
                    if state['kind'] != 'ConvexMesh':
                        raise SceneInvalid('Probe only supports actual convex support meshes')
                    vertices = np.asarray(state['geometry']['vertices']) * np.asarray(state['geometry']['scale'])
                    if not len(vertices) or not np.isfinite(vertices).all():
                        raise SceneInvalid('Invalid native mesh vertices')
                    frame = np.asarray(link.entity.pose.to_transformation_matrix()) @ np.asarray(state['T_object_shape'])
                    world = np.c_[vertices, np.ones(len(vertices))] @ frame.T
                    local = world @ np.linalg.inv(tcp).T
                    report['rows'].append(dict(source_file=str(path), stage=stage, link=name, shape_index=index,
                        vertex_count=len(vertices), tcp_z_min_m=float(local[:, 2].min()),
                        tcp_z_max_m=float(local[:, 2].max()), base_z_min_m=float(world[:, 2].min()),
                        base_z_max_m=float(world[:, 2].max()), contact_offset_m=state['properties']['contact_offset']))
report['private_robot_closed'] = port.closed
print('SWM_COLLISION_GEOMETRY ' + json.dumps(report), flush=True)
