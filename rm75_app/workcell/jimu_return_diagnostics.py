"""Bounded read-only evidence at the ORIGINAL return query's refreshed world."""
import contextvars
import functools
import inspect
import numpy as np
from .contact_audit import _plain, scene_evidence
from .transport_contact import jimu_collision_details


def _state(planner):
    raw = planner.robot_cfg_dict
    locks = raw.get('robot_cfg', raw)['kinematics'].get('lock_joints')
    rollout = planner.motion_gen.rollout_fn
    return {**scene_evidence(planner), 'gripper_lock_joints': _plain(locks),
            'attached': bool(planner.attached_object_active),
            'world_constraint_enabled': bool(rollout.primitive_collision_constraint.enabled),
            'self_constraint_enabled': bool(rollout.robot_self_collision_constraint.enabled)}


def install_return_diagnostics(portable, emit, *, limit=1):
    """Observe failed native returns without changing results, search or filters.

    Install before portable.main captures these functions. Its retry wrapper
    still calls the original return body and this exact profile query. Evidence
    is collected before the transaction restores the caller's old world.
    """
    if type(limit) is not int or limit < 1:
        raise ValueError('Return diagnostic limit must be a positive integer')
    direct = portable.direct
    original_return = direct._plan_return_to_start_joint_curobo
    original_query = direct._profile_plan_to_joint_state
    context = contextvars.ContextVar('jimu_return_diagnostic', default=None)
    records = []

    @functools.wraps(original_return)
    def returning(planner, demo, args, start_q, goal_q, *, label, **kwargs):
        token = context.set((demo, args, label))
        try:
            return original_return(planner, demo, args, start_q, goal_q, label=label, **kwargs)
        finally:
            context.reset(token)

    @functools.wraps(original_query)
    def query(planner, start_q, goal_q, *args, **kwargs):
        result = original_query(planner, start_q, goal_q, *args, **kwargs)
        current = context.get()
        if current is None or bool(result.success) or len(records) >= limit:
            return result
        demo, options, label = current
        if getattr(options, 'execute_real', False):
            return result  # This diagnostic never participates in real execution.
        with direct._CUROBO_GPU_LOCK:
            before = _state(planner)
            row = dict(event='jimu_return_query_diagnostic', step_id=label,
                       source=direct._current_source_object_name(options),
                       native_success=False, native_status=str(result.status),
                       diagnostic_mode='read_only_failed_return_query',
                       released=bool(getattr(options, '_episode_place_released', False)),
                       endpoints={}, **before)
            try:
                row['sim_joint_names'] = list(demo.active_joint_names)
                row['sim_qpos'] = _plain(demo.robot.get_qpos())
                for name, joints in (('start', start_q), ('goal', goal_q)):
                    valid, status = planner.check_start_state(joints)
                    row['endpoints'][name] = dict(valid=bool(valid), status=str(status),
                        **jimu_collision_details(portable, planner, joints))
                row['diagnostic_complete'] = all(
                    endpoint['geometry_detail_recorded'] for endpoint in row['endpoints'].values())
            except Exception as exc:
                row.update(diagnostic_complete=False, diagnostic_error=f'{type(exc).__name__}: {exc}')
            finally:
                row['state_unchanged'] = _state(planner) == before
                records.append(row)
                emit(row)
                if not row['state_unchanged']:
                    from .world_only_contact import WorldOnlyContactUnsupported
                    raise WorldOnlyContactUnsupported('return diagnostic changed native collision state')
        return result

    direct._plan_return_to_start_joint_curobo = returning
    direct._profile_plan_to_joint_state = query
    return records


def install_release_execution_observer(portable, emit, *, synchronize=False):
    """Observe the real Jimu pose-execution boundary AFTER partial-open setup.

    With synchronize=True, first apply the existing released-SIM joint-state
    synchronization to all native kinematics owners. Geometry/radii and actual
    gripper commands are unchanged. Path observations remain diagnostic only;
    do not substitute PickPlace's different contact policy or call this a gate.
    """
    original=portable._jimu_execute_pose_path_stage_base
    signature=inspect.signature(original);direct=portable.direct
    records=[]

    @functools.wraps(original)
    def execute(*args,**kwargs):
        bound=signature.bind(*args,**kwargs);bound.apply_defaults();values=bound.arguments
        label=str(values['label']);options=values['args']
        if (not label.startswith('post_place_clearance') or values['real_exec'] is not None
                or getattr(options,'execute_real',False) or getattr(options,'_planning_prefetch_capture_only',False)):
            return original(*args,**kwargs)
        demo=values['demo'];planner=getattr(demo.planner,'native',None)
        row=dict(event='jimu_release_execution_observation',step_id=label,
                 diagnostic_only=True,execution_guard=False,physical_success=None)
        with direct._CUROBO_GPU_LOCK:
            if planner is None:
                if synchronize:
                    from .pickplace_curobo_only import CuroboOnlyUnsupported
                    raise CuroboOnlyUnsupported('Jimu release model sync requires bound native planner')
                row.update(diagnostic_complete=False,error='native planner not bound')
            else:
                row['model_sync_requested']=bool(synchronize)
                if synchronize:
                    if not getattr(options,'_episode_place_released',False):
                        from .pickplace_curobo_only import CuroboOnlyUnsupported
                        raise CuroboOnlyUnsupported('Jimu release model sync requires released state')
                    from .pickplace_gripper_state import update_from_demo
                    row['gripper_locks_before_sync']=_state(planner)['gripper_lock_joints']
                    update_from_demo(planner,demo,options)
                before=_state(planner)
                row.update(before,source=direct._current_source_object_name(options),
                           released=bool(getattr(options,'_episode_place_released',False)),
                           requested_gripper_pos=float(values['gripper_pos']),
                           table_present=any(obj.name=='virtual_table_plane' for obj in planner._world.objects))
                try:
                    joints=np.asarray(_plain(demo.robot.get_qpos()),dtype=float).reshape(-1)
                    names=list(demo.active_joint_names)
                    if len(joints)!=len(names) or len(set(names))!=len(names) or not np.isfinite(joints).all():
                        raise ValueError('invalid simulated joint identity/state')
                    locks=before['gripper_lock_joints']
                    row['sim_gripper_joints']={name:float(joints[names.index(name)]) for name in locks}
                    row['max_gripper_model_error_rad']=max(abs(locks[name]-value)
                        for name,value in row['sim_gripper_joints'].items())
                    path=np.asarray(values['q_path'],dtype=float)
                    if path.ndim!=2 or path.shape[1]!=7 or len(path)<2 or not np.isfinite(path).all():
                        raise ValueError('missing/invalid released path')
                    dense=[path[0]]
                    for start,end in zip(path[:-1],path[1:]):
                        count=max(1,int(np.ceil(np.max(abs(end-start))/.01)))
                        dense.extend(np.linspace(start,end,count+1)[1:])
                    invalid=[]
                    for index,joints in enumerate(dense):
                        valid,status=planner.check_start_state(joints)
                        if not valid:
                            detail={'index':index,'status':str(status)}
                            if len(invalid)<3:
                                detail.update(jimu_collision_details(portable,planner,joints))
                            invalid.append(detail)
                    row.update(path_points=len(path),audited_samples=len(dense),invalid_samples=len(invalid),
                               first_invalid=invalid[:3],native_all_valid=not invalid,
                               diagnostic_complete=all(item['geometry_detail_recorded'] for item in invalid[:3]))
                except Exception as exc:
                    row.update(diagnostic_complete=False,error=f'{type(exc).__name__}: {exc}')
                finally:
                    row['state_unchanged']=_state(planner)==before
                    if not row['state_unchanged']:
                        emit(row)
                        from .world_only_contact import WorldOnlyContactUnsupported
                        raise WorldOnlyContactUnsupported('release observation changed native collision state')
            records.append(row);emit(row)
        return original(*args,**kwargs)

    portable._jimu_execute_pose_path_stage_base=execute
    return records
