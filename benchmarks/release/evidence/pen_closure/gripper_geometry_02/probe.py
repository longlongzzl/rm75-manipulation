"""Owned native FK geometry comparison at worker_64 measured postclosure state."""
import hashlib
import json
import sys
from contextlib import ExitStack
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial import ConvexHull
sys.path.append('/home/zhangzhao/anaconda3/envs/realman/lib/python3.11/site-packages')
from rm75_app.swm.native_robot_mirror import SapienRobotStatePort
from rm75_app.swm.native_body_mirror import shape_state
from rm75_app.swm.scene import SceneInvalid
from rm75_app.planning.gripper_collision import gripper_link_transforms

root = Path.cwd().resolve()
source = root / 'runtime_data/swm_release_pen_worker_64/swm_native/checkpoints/0004.json'
snapshot = json.loads(source.read_text())
observed = snapshot['robot']
if snapshot.get('valid') is not True or observed.get('holding') != 'bi' or observed.get('idle') is not True:
    raise SceneInvalid('Frozen independently observed held idle pen required')
positions = dict(zip(observed['joint_names'], observed['positions']))
positions.update(observed['gripper_positions'])
config_path = root / 'assets/curobo_rm75_config/rm75.yml'
kin = yaml.safe_load(config_path.read_text())['robot_cfg']['kinematics']
spheres_path = config_path.parent / kin['collision_spheres']
spheres = yaml.safe_load(spheres_path.read_text())
spheres = spheres.get('collision_spheres', spheres)
urdf = root / 'assets/robot_models/RM75_gripper/RM75-B/urdf/RM75-B.urdf'
report = dict(domain='native_private_FK_and_configuration_geometry', physics_steps=0,
    primary_world_created=False, hardware_connected=False, model_agreement=False,
    source_snapshot_id=snapshot['snapshot_id'], source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
    spheres_sha256=hashlib.sha256(spheres_path.read_bytes()).hexdigest(), links={}, mesh_pair_separation=[])
world_spheres = {}
meshes = {}
with ExitStack() as resources:
    port = SapienRobotStatePort(urdf, resources=resources)
    robot = port._robot
    names = [joint.name for joint in robot.get_active_joints()]
    if len(names) != 13 or set(names) != set(positions):
        raise SceneInvalid('Complete canonical measured native joint identity required')
    q = np.asarray([positions[name] for name in names], dtype=np.float32)
    robot.set_qpos(q)
    if not np.array_equal(np.asarray(robot.get_qpos()), q):
        raise SceneInvalid('Private FK did not apply exact measured native positions')
    links = {link.name: link for link in robot.get_links()}
    base = np.asarray(links['gripper_base_link'].entity.pose.to_transformation_matrix(), dtype=float)
    analytical = gripper_link_transforms(urdf, observed['gripper_positions'])
    for side in ('Left', 'Right'):
        name = f'gripper_{side}_Support_Link'
        link = links[name]
        frame = np.asarray(link.entity.pose.to_transformation_matrix(), dtype=float)
        error = float(np.max(np.abs(np.linalg.inv(base) @ frame - analytical[name])))
        if error > 1e-5:
            raise SceneInvalid('Native and original URDF measured jaw FK disagree')
        rows = []
        meshes[name] = []
        for index, shape in enumerate(link.collision_shapes):
            data = shape_state(shape, np.eye(4))
            if data['kind'] != 'ConvexMesh':
                raise SceneInvalid('Expected original native convex support collision shape')
            vertices = np.asarray(data['geometry']['vertices']) * np.asarray(data['geometry']['scale'])
            matrix = frame @ np.asarray(data['T_object_shape'])
            transformed = (np.c_[vertices, np.ones(len(vertices))] @ matrix.T)[:, :3]
            if not np.isfinite(transformed).all():
                raise SceneInvalid('Finite native support mesh required')
            meshes[name].append(transformed)
            rows.append(dict(shape_index=index, vertex_count=len(vertices),
                base_z_min_m=float(transformed[:, 2].min()), base_z_max_m=float(transformed[:, 2].max()),
                contact_offset_m=data['properties']['contact_offset']))
        if not rows:
            raise SceneInvalid('Native support mesh missing')
        entries = spheres[name]
        centers = np.asarray([row['center'] for row in entries])
        centers = (np.c_[centers, np.ones(len(centers))] @ frame.T)[:, :3]
        radii = np.asarray([row['radius'] for row in entries])
        world_spheres[name] = (centers, radii)
        report['links'][name] = dict(native_vs_original_urdf_FK_max_error=error,
            native_shapes=rows, configured_sphere_count=len(entries),
            configured_sphere_lowest_base_z_m=float(np.min(centers[:, 2]-radii)),
            globally_buffered_sphere_lowest_base_z_m=float(np.min(centers[:, 2]-radii)-kin['collision_sphere_buffer']))
    left, right = 'gripper_Left_Support_Link', 'gripper_Right_Support_Link'
    for i, a in enumerate(meshes[left]):
        for j, b in enumerate(meshes[right]):
            axes = np.vstack((np.eye(3), ConvexHull(a).equations[:, :3], ConvexHull(b).equations[:, :3]))
            axes /= np.linalg.norm(axes, axis=1)[:, None]
            pa, pb = a @ axes.T, b @ axes.T
            gaps = np.maximum(pb.min(axis=0)-pa.max(axis=0), pa.min(axis=0)-pb.max(axis=0))
            index = int(np.argmax(gaps))
            report['mesh_pair_separation'].append(dict(left_shape=i, right_shape=j,
                separation_proven=bool(gaps[index] > 0), separating_axis=axes[index].tolist(),
                gap_lower_bound_m=float(gaps[index]), exact_minimum_distance=False))
    ca, ra = world_spheres[left]
    cb, rb = world_spheres[right]
    overlaps = ra[:, None] + rb[None, :] - np.linalg.norm(ca[:, None, :] - cb[None, :, :], axis=2)
    pair = np.unravel_index(np.argmax(overlaps), overlaps.shape)
    raw = float(overlaps[pair])
    global_padding = 2 * float(kin['collision_sphere_buffer'])
    self_padding = sum(float(kin['self_collision_buffer'][name]) for name in (left,right))
    report['configured_sphere_pair'] = dict(left_index=int(pair[0]), right_index=int(pair[1]),
        raw_overlap_m=raw, with_global_buffer_overlap_m=raw+global_padding,
        with_global_and_self_buffers_overlap_m=raw+global_padding+self_padding,
        global_pair_buffer_m=global_padding, self_pair_buffer_m=self_padding,
        source='original_yaml_spheres_transformed_by_actual_native_link_FK_not_GPU_tensor_readback')
report['private_robot_closed'] = port.closed
report['all_native_mesh_pairs_proven_separated'] = all(row['separation_proven'] for row in report['mesh_pair_separation'])
print(json.dumps(report, indent=2), flush=True)
