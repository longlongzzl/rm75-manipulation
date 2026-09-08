"""Observe existing current-table failures without selecting new paths or IK."""
import functools
import sys
import threading

from .pickplace_curobo_only import CuroboOnlyUnsupported
from .pickplace_clearance_audit import validate_clearance_path
from .pickplace_lift_diagnostics import _state, install_lift_diagnostics
from .pickplace_release_contact import release_target, audit_release_path


def install_grasp_diagnostic(direct, emit):
    """One foreground gluestick grasp batch, in its original collision scope."""
    from .jimu_roof_ik_diagnostics import result_rows
    from .pickplace_lift_diagnostics import _configuration_evidence
    local = threading.local()
    original_refresh = direct._refresh_curobo_world
    original_query = direct._profile_fast_chain_solve_batch_start_goal_ik
    seen = False

    @functools.wraps(original_refresh)
    def refresh(planner, demo, args, **kwargs):
        result = original_refresh(planner, demo, args, **kwargs)
        local.scope = (id(planner), kwargs.get('label'))
        return result

    @functools.wraps(original_query)
    def query(options, planner, starts, goals, *, num_seeds):
        nonlocal seen
        with direct._CUROBO_GPU_LOCK:
            result = original_query(options, planner, starts, goals, num_seeds=num_seeds)
            if (seen or getattr(options, 'execute_real', False)
                    or getattr(options, '_planning_prefetch_capture_only', False)
                    or direct._current_source_object_name(options) != 'gluestick'
                    or getattr(local, 'scope', None) != (id(planner), 'winner_chain_ik_preselect_grasp_contact')):
                return result
            seen = True
            before = _state(planner)
            row = dict(event='pickplace_grasp_batch_diagnostic', source='gluestick',
                phase='grasp', diagnostic_only=True, new_solver_calls=0,
                goal_count=len(goals), requested_num_seeds=num_seeds,
                native_success_count=sum(bool(r.success) for r in result), goals=[])
            try:
                if not len(goals) or len(starts)!=len(goals) or len(result)!=len(goals):
                    raise ValueError('Original grasp batch identity/count mismatch')
                copied = [result_rows(r, i, len(goals), starts[i]) for i,r in enumerate(result)]
                for index,(seeds,nearest) in enumerate(copied):
                    selected = seeds[nearest]
                    row['goals'].append(dict(goal_index=index,
                        native_success=bool(result[index].success), seeds=seeds,
                        nearest_seed_index=nearest,
                        configuration=_configuration_evidence(planner, selected['joints'])))
                after_rows = [result_rows(r, i, len(goals), starts[i]) for i,r in enumerate(result)]
                row['returned_rows_unchanged'] = after_rows == copied
                row['diagnostic_complete'] = all(s['finite'] for seeds,_ in copied for s in seeds)
            except Exception as exc:
                row.update(diagnostic_complete=False, error_type=type(exc).__name__)
            finally:
                row['state_unchanged'] = _state(planner) == before
                emit(row)
            if not row['state_unchanged'] or row.get('returned_rows_unchanged') is False:
                raise CuroboOnlyUnsupported('read-only grasp diagnostic changed native state/results')
            return result

    direct._refresh_curobo_world = refresh
    direct._profile_fast_chain_solve_batch_start_goal_ik = query


def observe_reverse(direct, planner, args, path, emit):
    """Audit the already generated reverse path, even if native endpoint rejected it."""
    if getattr(args, 'execute_real', False) or direct._current_source_object_name(args) != 'tennis':
        raise ValueError('Existing reverse diagnostic is tennis SIM only')
    row = {'event': 'pickplace_existing_reverse_diagnostic', 'source': 'tennis',
           'diagnostic_only': True, 'selected': False, 'new_solver_calls': 0,
           'path_points': len(path), 'accepted_by_same_full_path_gate': False}
    with direct._CUROBO_GPU_LOCK:
        before = _state(planner)
        try:
            try:
                evidence = validate_clearance_path(planner, path)
            except CuroboOnlyUnsupported:
                target = release_target(planner, args, direct._current_source_object_name(args))
                evidence = audit_release_path(planner, path, target)
            row.update(accepted_by_same_full_path_gate=True, audit=evidence)
        except (CuroboOnlyUnsupported, ValueError, RuntimeError) as exc:
            row.update(error_type=type(exc).__name__, reason=str(exc))
        finally:
            row['state_unchanged'] = _state(planner) == before
            emit(row)
        if not row['state_unchanged']:
            raise CuroboOnlyUnsupported('read-only reverse diagnostic changed collision state')
    return row


def install(direct, planner_class, emit, *, requested_source):
    if requested_source not in ('gluestick', 'hongshupian', 'tennis'):
        raise ValueError('Focused diagnostic is limited to the three reviewed sources')
    install_lift_diagnostics(direct, emit)
    if requested_source == 'gluestick':
        install_grasp_diagnostic(direct, emit)
    original = planner_class.diagnose_start_state_world_collision
    seen = False

    @functools.wraps(original)
    def diagnose(planner, *pos, **kwargs):
        nonlocal seen
        result = original(planner, *pos, **kwargs)
        caller = sys._getframe(1)
        local = caller.f_locals
        # Match the one original reverse endpoint call. Never capture arbitrary
        # planner queries or call the original candidate generator/solver again.
        if (not seen and requested_source == 'tennis'
                and caller.f_code.co_name == 'run_targeted_place_episode_curobo_direct'
                and 'reverse_clearance_path' in local and 'args' in local):
            args = local['args']
            if (not getattr(args, 'execute_real', False)
                    and not getattr(args, '_planning_prefetch_capture_only', False)
                    and direct._current_source_object_name(args) == 'tennis'):
                seen = True
                observe_reverse(direct, planner, args, local['reverse_clearance_path'], emit)
        return result

    planner_class.diagnose_start_state_world_collision = diagnose
    return lambda: setattr(planner_class, 'diagnose_start_state_world_collision', original)
