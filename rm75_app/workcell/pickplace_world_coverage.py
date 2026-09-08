"""Read-only coverage gate for the opt-in full frozen PickPlace SIM harness.

Install before the transport adapter so this observes its effective refresh
arguments. This never inserts objects, changes geometry, or changes a query.
"""
import functools
import threading
from collections import Counter

from .contact_audit import scene_evidence
from .transport_contact import is_transport


class FrozenWorldIncomplete(BaseException):
    """Abort native broad Exception retry handlers on an evidence gap."""


def coverage_row(planner, *, scene_names, source, label, excluded=(), foreground=True,
                 prefetch_capture_only=False):
    names = set(scene_names)
    if not names or source not in names:
        raise FrozenWorldIncomplete('unknown_frozen_scene_source')
    evidence = scene_evidence(planner)
    present = {row['name'] for row in evidence['world_objects']}
    disabled = set(evidence['disabled_objects'])
    excluded = set(excluded or ())
    loaded = is_transport(label) or 'joint_start_' in label or 'post_grasp_lift' in label
    permitted = set() if loaded else excluded - {source}
    expected = names - {source} - permitted
    missing = sorted(name for name in expected
                     if 'scene_obstacle_' + name not in present
                     or 'scene_obstacle_' + name in disabled or name in disabled)
    reasons = []
    if missing:
        reasons.append('missing_or_disabled_non_source_objects')
    if loaded and excluded - {source}:
        reasons.append('loaded_non_source_exclusion')
    if not planner.collision_enabled or not planner.config.self_collision_check:
        reasons.append('collision_checks_disabled')
    if is_transport(label) and evidence['disabled_links']:
        reasons.append('transport_link_exemption')
    return dict(event='frozen_world_coverage', source=source, step_id=label,
                foreground=foreground, prefetch_capture_only=prefetch_capture_only,
                frozen_object_count=len(names), expected_non_source_names=sorted(expected),
                explicitly_excluded_non_source_names=sorted(permitted),
                present_world_names=sorted(present), disabled_objects=sorted(disabled),
                disabled_links=evidence['disabled_links'], missing_names=missing,
                loaded_scope=loaded, transport_scope=is_transport(label),
                scene_fingerprint=evidence['scene_fingerprint'],
                complete=not reasons, rejection_reasons=reasons,
                physical_geometry_qualified=False)


def install(direct, scene_names, emit, emit_episode=None):
    scene_names = tuple(scene_names)
    original = direct._refresh_curobo_world
    owner=threading.get_ident()

    def execution_scope(args):
        prefetch=bool(getattr(args,'_planning_prefetch_capture_only',False))
        return dict(prefetch_capture_only=prefetch,
                    foreground=threading.get_ident()==owner and not prefetch)

    @functools.wraps(original)
    def refresh(planner, demo, args, **kwargs):
        if bool(getattr(args, 'execute_real', False)):
            raise FrozenWorldIncomplete('simulation_only_frozen_world_audit')
        with direct._CUROBO_GPU_LOCK:
            result = original(planner, demo, args, **kwargs)
            row = coverage_row(planner, scene_names=scene_names,
                               source=direct._current_source_object_name(args),
                               label=str(kwargs.get('label', '')),
                               excluded=kwargs.get('exclude_object_names') or (),**execution_scope(args))
            emit(row)
            if not row['complete']:
                raise FrozenWorldIncomplete(','.join(row['rejection_reasons']))
            return result

    direct._refresh_curobo_world = refresh
    if emit_episode is not None:
        original_episode=direct.run_targeted_place_episode_curobo_direct

        @functools.wraps(original_episode)
        def episode(demo, bridge, real_exec, args, *pos, **kwargs):
            if bool(getattr(args,'execute_real',False)):
                raise FrozenWorldIncomplete('simulation_only_frozen_source_audit')
            source=direct._current_source_object_name(args)
            scope=execution_scope(args)
            try:
                result=original_episode(demo,bridge,real_exec,args,*pos,**kwargs)
            except BaseException as exc:
                emit_episode(dict(source=source,success=None,error_type=type(exc).__name__,**scope))
                raise
            emit_episode(dict(source=source,success=bool(result),**scope))
            return result

        direct.run_targeted_place_episode_curobo_direct=episode


def full_chain_observed(rows, sources):
    """No vacuous PASS for an unobserved/failed source or incomplete refresh."""
    if not rows or not sources or any(row['complete'] is not True for row in rows):
        return False
    observed = {row['source'] for row in rows if row['transport_scope'] and row.get('foreground') is True
                and row.get('prefetch_capture_only') is False
                and not row['explicitly_excluded_non_source_names']}
    return set(sources) <= observed


def requested_sources_completed(outcomes, sources):
    """A native retry can select another object; it cannot substitute success."""
    return bool(sources) and Counter(row['source'] for row in outcomes
        if row['success'] is True and row.get('foreground') is True
        and row.get('prefetch_capture_only') is False)==Counter(sources)
