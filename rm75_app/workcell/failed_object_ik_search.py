"""Opt-in SIM retries at unchanged failed-object IK goals and collision state."""
import contextvars
import copy
import functools
import time

from .contact_audit import _plain
from .jimu_roof_ik_diagnostics import result_rows
from .pickplace_lift_diagnostics import _state, _configuration_evidence

SOURCES = frozenset(('gluestick', 'hongshupian'))


def require_search(spec, profile):
    section = profile.get('pickplace', {})
    if (spec.get('task') != 'pickplace' or spec.get('mode') != 'sim'
            or section.get('fixed_scene_format') != 'native_world'
            or section.get('simulation_contact_policy') != 'transport_world_checked_compatibility'):
        raise ValueError('Failed-object IK search requires checked frozen-world PickPlace SIM')


def merge_results(original, retries, valid):
    if not len(original) == len(retries) == len(valid):
        raise ValueError('IK retry candidate identity/count mismatch')
    return [old if old.success or not (new.success and checked) else new
            for old, new, checked in zip(original, retries, valid)]


def freeze_retry_result(result):
    """Detach saved results before the next single-goal query reuses GPU buffers."""
    result = copy.copy(result)
    result.raw_result = copy.copy(result.raw_result)
    for name in ('solution', 'success', 'position_error', 'rotation_error'):
        value = getattr(result.raw_result, name)
        setattr(result.raw_result, name, value.clone() if hasattr(value, 'clone') else value.copy())
    if result.goal_joint is not None:
        result.goal_joint = result.goal_joint.copy()
    result.debug = dict(result.debug or {})
    return result


def install(direct, emit, *, num_seeds=128):
    if num_seeds not in (128, 256):
        raise ValueError('Failed-object IK search supports bounded 128/256 seed trials')
    original_batch = direct._profile_fast_chain_solve_batch_start_goal_ik
    original_single = direct._profile_solve_ik
    original_segment = direct._plan_short_linear_segment_via_goal_ik
    context = contextvars.ContextVar('failed_object_ik_segment', default=None)

    def eligible(options):
        if getattr(options, 'execute_real', False):
            raise PermissionError('Failed-object IK search cannot run with real execution')
        return (direct._current_source_object_name(options) in SOURCES
                and not getattr(options, '_planning_prefetch_capture_only', False))

    def audit(planner, options, starts, goals, old, new, before, started, phase):
        # Preserve seed/joint/error association before FK and GPU checks.
        captured = [result_rows(r, i, len(goals), starts[i])
                    for i, r in enumerate(new)]
        accepted = []
        rows = []
        for i, (baseline, retry, goal, (seeds, nearest)) in enumerate(zip(old, new, goals, captured)):
            details = None
            if retry.success and retry.goal_joint is not None:
                details = _configuration_evidence(planner, retry.goal_joint)
            valid = details is not None and details['valid']
            accepted.append(valid)
            position, quaternion = planner._extract_pose_components(goal)
            rows.append(dict(index=i,original_success=bool(baseline.success),
                retry_success=bool(retry.success),retry_configuration_valid=bool(valid),
                newly_accepted=not baseline.success and bool(retry.success) and bool(valid),
                goal=dict(position=_plain(position),quaternion=_plain(quaternion)),
                explicit_start_q=_plain(starts[i]),raw_rows=seeds,
                nearest_seed_index=nearest,configuration=details))
        unchanged = _state(planner) == before
        raw_unchanged = captured == [result_rows(r,i,len(goals),starts[i]) for i,r in enumerate(new)]
        emit(dict(event='failed_object_ik_search',source=direct._current_source_object_name(options),
            phase=phase,requested_num_seeds=num_seeds,goal_count=len(goals),
            original_success_count=sum(bool(r.success) for r in old),
            newly_accepted=sum(r['newly_accepted'] for r in rows),rows=rows,
            state_before=before,state_unchanged=unchanged,raw_results_unchanged=raw_unchanged,
            goals_changed=False,collision_exemptions_added=False,elapsed_s=time.perf_counter()-started,
            endpoint_only=True,full_chain_success=False))
        if not unchanged or not raw_unchanged:
            raise RuntimeError('IK retry changed collision state or raw result identity')
        return merge_results(old,new,accepted)

    @functools.wraps(original_batch)
    def batch(options, planner, starts, goals, *, num_seeds):
        with direct._CUROBO_GPU_LOCK:
            old = original_batch(options,planner,starts,goals,num_seeds=num_seeds)
            if not eligible(options) or all(r.success for r in old):
                return old
            before = _state(planner)
            started = time.perf_counter()
            # Keep GPU allocation bounded by one goal. A broad 80-goal x 128
            # retry can exhaust this workstation's 8 GiB GPU/16 GiB RAM.
            retry = list(old)
            for index, baseline in enumerate(old):
                if baseline.success:
                    continue
                one = original_batch(options,planner,[starts[index]],[goals[index]],num_seeds=retry_seeds)
                if len(one) != 1:
                    raise ValueError('Single-goal IK retry returned an unexpected count')
                saved = freeze_retry_result(one[0])
                saved.debug.update(ik_search_goal_count=1,original_goal_index=index)
                retry[index] = saved
            return audit(planner,options,starts,goals,old,retry,before,started,'native_batch_single_goal_retries')

    @functools.wraps(original_segment)
    def segment(planner, demo, args, start_q, pose_start, pose_goal, **kwargs):
        token = context.set(args if eligible(args) else None)
        try:
            return original_segment(planner,demo,args,start_q,pose_start,pose_goal,**kwargs)
        finally:
            context.reset(token)

    @functools.wraps(original_single)
    def single(planner, start_q, goal_pose, **kwargs):
        goal = goal_pose
        with direct._CUROBO_GPU_LOCK:
            old = original_single(planner,start_q,goal,**kwargs)
            options = context.get()
            if options is None or old.success:
                return old
            before = _state(planner)
            started = time.perf_counter()
            retry = original_single(planner,start_q,goal,**dict(kwargs,num_seeds=retry_seeds))
            return audit(planner,options,[start_q],[goal],[old],[retry],before,started,'short_linear_endpoint')[0]

    retry_seeds = num_seeds
    direct._profile_fast_chain_solve_batch_start_goal_ik = batch
    direct._profile_solve_ik = single
    direct._plan_short_linear_segment_via_goal_ik = segment

    def close():
        direct._profile_fast_chain_solve_batch_start_goal_ik = original_batch
        direct._profile_solve_ik = original_single
        direct._plan_short_linear_segment_via_goal_ik = original_segment
    return close
