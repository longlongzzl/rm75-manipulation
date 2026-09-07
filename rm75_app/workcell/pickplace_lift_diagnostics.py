"""Bounded SIM evidence at the original, still-attached failed lift IK query.

No new solver call, collision ablation, candidate selection or result promotion.
Box contacts are analytic geometry evidence, not a replacement GPU validity test.
"""
import contextvars
import functools
import hashlib
import json

import numpy as np

from .contact_audit import _plain, scene_evidence
from .pickplace_clearance_audit import sphere_box_contacts
from .transforms import quaternion_matrix
from .world_only_contact import WorldOnlyContactUnsupported


def _array(value):
    if hasattr(value, 'detach'):
        value = value.detach().cpu().numpy()
    return np.asarray(value)


def _state(planner):
    owners = []
    for owner in (planner.motion_gen, planner.ik_solver):
        rollouts = []
        for rollout in owner.get_all_rollout_instances():
            cfg = rollout.kinematics.kinematics_config
            rollouts.append({
                'link_spheres': _plain(cfg.link_spheres),
                'fixed_transforms': _plain(cfg.fixed_transforms),
                'enabled': {name: bool(getattr(rollout, name).enabled)
                            for name in ('primitive_collision_cost', 'primitive_collision_constraint',
                                         'robot_self_collision_cost', 'robot_self_collision_constraint')
                            if getattr(rollout, name, None) is not None}})
        owners.append(rollouts)
    model = {'robot_cfg': _plain(planner.robot_cfg_dict), 'owners': owners}
    encoded = json.dumps(model, sort_keys=True, allow_nan=False, separators=(',', ':'))
    return {**scene_evidence(planner), 'attached': bool(planner.attached_object_active),
            'payload_spheres': int(planner.get_attached_sphere_count()),
            'collision_model_sha256': hashlib.sha256(encoded.encode()).hexdigest()}


def _configuration_evidence(planner, joints):
    joints = np.asarray(joints, dtype=float)
    if joints.shape != (7,) or not np.isfinite(joints).all():
        raise ValueError('diagnostic requires exactly seven finite joints')
    valid, status = planner.check_start_state(joints)
    spheres = planner._compute_world_link_spheres(joints)
    links = planner._collision_sphere_link_names()
    if spheres.shape != (len(links), 4) or not np.isfinite(spheres).all():
        raise ValueError('invalid diagnostic sphere/link geometry')
    self_detail = planner.diagnose_start_state_self_collision(joints, top_k=0)
    if self_detail.get('error'):
        raise ValueError('native self collision diagnostic: ' + str(self_detail['error']))
    return {'joints': joints.tolist(), 'valid': bool(valid), 'native_status': str(status),
            'fk': _plain(planner.fk(joints)),
            'box_contacts': sphere_box_contacts(spheres, links, planner._world.objects,
                                                planner._disabled_world_obstacles),
            'self_collision': _plain(self_detail),
            'non_box_objects_not_analytically_audited': [obj.name for obj in planner._world.objects
                                                       if getattr(obj, 'dims', None) is None]}


def _returned_rows(raw):
    """Keep each native seed's success/errors associated with that SAME seed."""
    solution = _array(raw.solution)
    if solution.ndim < 2 or solution.shape[-1] != 7:
        raise ValueError('unexpected native IK solution shape')
    solution = solution.reshape(-1, 7).copy()
    if not len(solution):
        raise ValueError('missing native IK returned rows')
    columns = {name: _array(getattr(raw, name)).reshape(-1).copy()
               for name in ('success', 'position_error', 'rotation_error')}
    if any(len(values) != len(solution) for values in columns.values()):
        raise ValueError('native IK per-seed result shape mismatch')
    rows = []
    for index, joints in enumerate(solution):
        finite = np.isfinite(joints).all() and all(
            np.isfinite(columns[name][index]) for name in ('position_error', 'rotation_error'))
        rows.append({'index': index, 'native_success': bool(columns['success'][index]),
                     'finite': bool(finite),
                     'position_error_m': float(columns['position_error'][index]) if finite else None,
                     'rotation_error_native': float(columns['rotation_error'][index]) if finite else None,
                     'joints': joints.tolist() if finite else None})
    return rows


def nominal_payload_base(planner, start_q, goal, comparison_q):
    """Place the unchanged rigid payload at the commanded EE pose, not a seed.

    This checks only payload/base sphere pairs, with the native self buffers and
    ignore pairs. It does not certify arm, world, path, or real geometry.
    """
    spheres = planner._compute_world_link_spheres(start_q)
    links = planner._collision_sphere_link_names()
    if spheres.shape != (len(links), 4) or not np.isfinite(spheres).all():
        raise ValueError('invalid nominal payload geometry')
    payload = [i for i, name in enumerate(links) if name == 'attached_object' and spheres[i, 3] > 0]
    base = [i for i, name in enumerate(links) if name == 'base_link' and spheres[i, 3] > 0]
    if not payload or not base:
        raise ValueError('nominal payload/base spheres missing')
    comparison = planner._compute_world_link_spheres(comparison_q)
    if comparison.shape != spheres.shape:
        raise ValueError('nominal base comparison shape mismatch')
    base_delta = float(np.max(np.abs(comparison[base] - spheres[base])))
    if not np.isfinite(base_delta) or base_delta > 1e-7:
        raise ValueError('base spheres changed between start and returned configuration')
    start = planner.fk(start_q)
    local = (spheres[payload, :3] - np.asarray(start['position'])) @ quaternion_matrix(start['quaternion'])
    nominal = local @ quaternion_matrix(goal['quaternion']).T + np.asarray(goal['position'])
    if not np.isfinite(nominal).all():
        raise ValueError('invalid nominal goal pose')
    buffers = planner._self_collision_buffers()
    ignored = ('attached_object', 'base_link') in planner._self_collision_ignore_pairs()
    overlaps = []
    for payload_index, center in zip(payload, nominal):
        for base_index in base:
            overlap = float(spheres[payload_index, 3] + spheres[base_index, 3]
                + buffers.get('attached_object', 0.) + buffers.get('base_link', 0.)
                - np.linalg.norm(center - spheres[base_index, :3]))
            if overlap > 0:
                overlaps.append({'link_a': 'attached_object', 'link_b': 'base_link',
                    'sphere_i': payload_index, 'sphere_j': base_index, 'overlap_m': overlap,
                    'pair_ignored': ignored})
    return {'payload_spheres': len(payload), 'base_spheres': len(base),
            'base_comparison_max_delta_m': base_delta, 'contacts': overlaps,
            'geometric_necessary_condition_only': True, 'physical_geometry_qualified': False}


def install_lift_diagnostics(direct, emit, *, per_source_limit=1):
    if type(per_source_limit) is not int or per_source_limit < 1:
        raise ValueError('positive integer per-source diagnostic limit required')
    original_segment = direct._plan_short_linear_segment_via_goal_ik
    original_query = direct._profile_solve_ik
    context = contextvars.ContextVar('pickplace_lift_diagnostic', default=None)
    records, counts = [], {}

    @functools.wraps(original_segment)
    def segment(planner, demo, args, start_q, pose_start, pose_goal, *, label, **kwargs):
        eligible = str(label).startswith('joint_start_') and 'lift' in str(label)
        token = context.set((args, label) if eligible else None)
        try:
            return original_segment(planner, demo, args, start_q, pose_start, pose_goal,
                                    label=label, **kwargs)
        finally:
            context.reset(token)

    @functools.wraps(original_query)
    def query(planner, start_q, goal_pose, *args, **kwargs):
        # Keep query and observation in the same native GPU critical section.
        with direct._CUROBO_GPU_LOCK:
            result = original_query(planner, start_q, goal_pose, *args, **kwargs)
            current = context.get()
            if current is None or bool(result.success) or not planner.attached_object_active:
                return result
            options, label = current
            if getattr(options, 'execute_real', False):
                return result
            source = direct._current_source_object_name(options)
            if counts.get(source, 0) >= per_source_limit:
                return result
            counts[source] = counts.get(source, 0) + 1
            row = {'event': 'pickplace_failed_lift_ik_diagnostic', 'source': source,
                   'step_id': label, 'diagnostic_only': True, 'execution_guard': False,
                   'native_success': False, 'native_status': str(result.status),
                   'requested_num_seeds': kwargs.get('num_seeds'),
                   'returned_rows': []}
            before = None
            try:
                before = _state(planner)
                row.update(before)
                # Copy all returned rows BEFORE any FK/check call touches GPU scratch buffers.
                rows = _returned_rows(result.raw_result)
                position, quaternion = planner._extract_pose_components(goal_pose)
                row['goal_pose'] = {'position': _plain(position), 'quaternion': _plain(quaternion)}
                row['start'] = _configuration_evidence(planner, start_q)
                finite_rows = [item for item in rows if item['finite']]
                if not finite_rows:
                    raise ValueError('no finite native row for nominal geometry comparison')
                row['nominal_goal_payload_base'] = nominal_payload_base(
                    planner, start_q, row['goal_pose'], finite_rows[0]['joints'])
                for item in rows:
                    if item['finite']:
                        item['configuration'] = _configuration_evidence(planner, item.pop('joints'))
                    row['returned_rows'].append(item)
                row['diagnostic_complete'] = all(item['finite'] for item in rows)
            except Exception as exc:
                row.update(diagnostic_complete=False, diagnostic_error=f'{type(exc).__name__}: {exc}')
            finally:
                row['state_unchanged'] = before is not None and _state(planner) == before
                records.append(row)
                emit(row)
                if not row['state_unchanged']:
                    raise WorldOnlyContactUnsupported('lift IK diagnostic collision state not preserved')
            return result

    direct._plan_short_linear_segment_via_goal_ik = segment
    direct._profile_solve_ik = query
    return records
