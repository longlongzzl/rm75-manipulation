"""Map the planar push intent to the unchanged CLOSED gripper geometry.

The surrogate circle center is not a gripper TCP. Use a real leading sphere
to align the intended object surface contact, and the whole vertical tool sweep
to set the descend position. These necessary geometry checks do not replace IK,
arm/self/world audits, physical calibration, or fresh observations.
"""
from dataclasses import dataclass
import numpy as np

from rm75_app.planning.contracts import JointConfiguration
from rm75_app.workcell.transforms import quaternion_matrix
from .model import vertices
from .cartesian_ik import PushPathRejected


@dataclass(frozen=True)
class ToolGeometry:
    spheres: np.ndarray
    links: tuple
    rigidity_error_m: float = 0.


def native_tool_geometry(backend, reference_q):
    if backend._gripper_collision_state != 'closed' or not backend.config.dynamic_gripper_collision:
        raise PushPathRejected('Closed native gripper collision geometry is required')
    planner = backend._ensure_planner(); mods = backend._import_modules()
    cfg = planner.kinematics.config.kinematics_config
    ids = cfg.link_sphere_idx_map.detach().cpu().tolist()
    names = {int(value): name for name, value in cfg.link_name_to_idx_map.items()}
    links = [str(names[int(index)]) for index in ids]
    joints = tuple(planner.joint_names)
    reference_q = np.asarray(reference_q, dtype=float)
    other = reference_q.copy(); other[3] += .1  # FK only, never a movement.
    values = []
    for reference in (reference_q, other):
        state = mods['JointState'].from_position(mods['torch'].as_tensor(reference,
            device=planner.device_cfg.device, dtype=planner.device_cfg.dtype).reshape(1, -1), joint_names=list(joints))
        spheres = planner.kinematics.compute_kinematics(planner.kinematics.get_active_js(state)).robot_spheres
        spheres = spheres.detach().cpu().numpy().reshape(-1, 4).astype(float)
        pose = backend.tool_pose_for_configuration(JointConfiguration(joints, reference), 'gripper_tcp')
        spheres[:, :3] = (spheres[:, :3] - pose.position) @ quaternion_matrix(pose.quaternion_wxyz)
        values.append(spheres)
    mask = np.array([(name.startswith('gripper_') or name in ('left_pad', 'right_pad'))
                     and values[0][i, 3] > 0 for i, name in enumerate(links)])
    if not mask.any():
        raise PushPathRejected('Native closed gripper spheres are missing')
    error = float(np.max(np.linalg.norm(values[0][mask, :3] - values[1][mask, :3], axis=1)))
    if error > 1e-6 or not np.array_equal(values[0][mask, 3], values[1][mask, 3]):
        raise PushPathRejected('Tool geometry is not rigid relative to gripper_tcp')
    return ToolGeometry(values[0][mask], tuple(name for name, keep in zip(links, mask) if keep), error)


def sphere_box_clearances(spheres, obj):
    if obj.kind != 'cuboid':
        raise PushPathRejected('Closed-tool nominal audit needs an explicit cuboid scene')
    local = (spheres[:, :3] - obj.pose.position) @ quaternion_matrix(obj.pose.quaternion_wxyz)
    delta = np.abs(local) - np.asarray(obj.dimensions) / 2
    return np.linalg.norm(np.maximum(delta, 0), axis=1) + np.minimum(delta.max(axis=1), 0) - spheres[:, 3]


def _bind_contact(push, observation, config, profile, geometry, scene, anchor):
    local = np.asarray(geometry.spheres, dtype=float)
    if (local.shape != (len(geometry.links), 4) or not len(local) or not np.isfinite(local).all()
            or np.any(local[:, 3] <= 0)):
        raise PushPathRejected('Invalid complete tool geometry')
    d = np.asarray(push.direction, dtype=float)
    if d.shape != (2,) or not np.isfinite(d).all() or abs(np.linalg.norm(d) - 1) > 1e-6:
        raise PushPathRejected('Invalid push direction')
    r = quaternion_matrix(profile['tool_quaternion_wxyz'])
    offset = local[:, :3] @ r.T
    z = float(profile['push_tcp_z_m']); hover = float(profile['hover_clearance_m'])
    bottom = profile['object_centroid_z_m'] - profile['object_height_m'] / 2
    top = profile['object_centroid_z_m'] + profile['object_height_m'] / 2
    sphere_z = offset[:, 2] + z
    # Contact must be on an allowed finger/pad and a vertical face, not a
    # palm or an above/below edge that would induce an unmodelled vertical force.
    allowed = set(profile['pusher_contact_links'])
    if not (bottom <= sphere_z[anchor] <= top and geometry.links[anchor] in allowed):
        raise PushPathRejected('No allowed closed-gripper leading contact on a vertical target face')
    surface = np.asarray(push.contact) + d * config.pusher_radius_m
    contact_xy = surface - offset[anchor, :2] - d * local[anchor, 3]
    contact = np.r_[contact_xy, z]
    # Use ALL spheres intersecting the target's height during the ENTIRE descent.
    swept = (sphere_z - local[:, 3] <= top) & (sphere_z + hover + local[:, 3] >= bottom)
    sweep_reach = float(np.max((offset[:, :2] @ d + local[:, 3])[swept]))
    target_front = float(np.min(np.concatenate(vertices(observation.pose, config)) @ d))
    entry_projection = min(float(contact_xy @ d) - config.approach_gap_m,
                           target_front - sweep_reach - config.approach_gap_m)
    entry = contact.copy(); entry[:2] += d * (entry_projection - contact_xy @ d)
    above = entry + [0, 0, hover]
    end = contact + np.r_[d * push.length_m, 0]
    retreat = end + [0, 0, hover]
    # The original surrogate used a disk margin. Retain that workspace intent
    # using the full swept low-tool footprint at the actual mapped endpoints.
    w = config.workspace
    for xyz in (above, entry, contact, end, retreat):
        xy = offset[swept, :2] + xyz[:2]; radii = local[swept, 3]
        if (np.any(xy[:, 0] - radii < w[0]) or np.any(xy[:, 0] + radii > w[1])
                or np.any(xy[:, 1] - radii < w[2]) or np.any(xy[:, 1] + radii > w[3])):
            raise PushPathRejected('closed_tool_footprint_outside_workspace')
    placed = local.copy(); placed[:, :3] = offset + contact
    targets = [obj for obj in scene.objects if obj.name in ('pusht_target_0', 'pusht_target_1')]
    actual = np.r_[surface, sphere_z[anchor]]
    if (len(targets) != 2 or min(abs(sphere_box_clearances(np.array([[*actual, 0.]]), obj)[0])
                                for obj in targets) > 1e-7):
        raise PushPathRejected('Requested planar contact is not on the original T surface')
    # Original scene, full tool, 1 mm sampling; no allowed contacts until the
    # nominal tangent endpoint. Full-arm GPU audits still run on the actual path.
    audit = []
    for name, start, goal in (('descend', above, entry), ('contact', entry, contact)):
        count = max(1, int(np.ceil(np.linalg.norm(goal - start) / .001)))
        minimum = float('inf')
        for xyz in np.linspace(start, goal, count + 1):
            placed[:, :3] = offset + xyz
            for obj in scene.objects:
                clearance = sphere_box_clearances(placed, obj)
                minimum = min(minimum, float(clearance.min()))
                # Numeric equality at a constructed tangent, NOT a collision tolerance.
                if np.any(clearance < -1e-9):
                    i = int(np.argmin(clearance))
                    raise PushPathRejected(f'closed_tool_nominal_{name}_collision:{geometry.links[i]}:{obj.name}')
        audit.append(dict(stage=name, samples=count + 1, minimum_clearance_m=minimum))
    points = [('approach', above, False, False), ('descend', entry, True, False),
              ('contact', contact, True, True), ('push', end, True, True), ('retreat', retreat, True, True)]
    evidence = dict(tool_state='closed', tool_spheres=len(local), rigidity_error_m=geometry.rigidity_error_m,
                    model_contact_is_tcp=False, model_contact_xy=list(push.contact),
                    surface_contact_xyz=actual.tolist(), tcp_contact_xyz=contact.tolist(),
                    tcp_descend_xyz=entry.tolist(), contact_link=geometry.links[anchor],
                    descend_backoff_m=float((contact[:2] - entry[:2]) @ d),
                    tcp_mapping_offset_m=float(np.linalg.norm(contact[:2] - push.contact)),
                    geometry_unchanged=True, nominal_audits=audit, physical_qualified=False)
    return points, evidence


def bind_push(push, observation, config, profile, geometry, scene):
    """Try every original contact sphere; keep the requested T surface point.

    An infinite-plane support test can falsely reject a finite T: the most
    forward sphere can pass beside its narrow crossbar. Therefore test ALL
    eligible original contact features against the entire unchanged scene.
    This is geometric TCP calibration, not additional IK seeds or MPC tuning.
    """
    good=[];failures=[]
    for anchor in range(len(geometry.links)):
        try:
            points,row=_bind_contact(push,observation,config,profile,geometry,scene,anchor)
            row['contact_sphere_index']=anchor
            good.append((points,row))
        except PushPathRejected as exc:
            failures.append(str(exc))
    if not good:
        reason=next((item for item in failures if not item.startswith('No allowed')),None)
        raise PushPathRejected('closed_gripper_mapping_failed: '+(reason or (failures[0] if failures else 'missing tool geometry')))
    points,row=min(good,key=lambda item:(item[1]['tcp_mapping_offset_m'],item[1]['contact_sphere_index']))
    row.update(contact_features_tested=len(geometry.links),valid_contact_features=len(good),
               rejected_contact_features=len(failures))
    return points,row
