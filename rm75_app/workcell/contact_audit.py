"""Fail-closed paper boundary; the hash-verified native snapshot stays unchanged.

cuRobo1's reviewed adapter exposes link/world toggles, not qualified link/object
pairs. Compatibility auditing observes that behavior without certifying it.
"""
from __future__ import annotations

import contextvars
import functools
import hashlib
import json


class StrictContactNotSupported(BaseException):
    """Abort the native retry ladder, including its broad except Exception paths."""

    code = "strict_contact_not_supported"

    def __init__(self, evidence):
        self.evidence = evidence
        super().__init__(self.code)


def _plain(value):
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, dict):
        return {str(k): _plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    if hasattr(value, "tolist"):
        return _plain(value.tolist())
    # Do not fingerprint object reprs containing process-local memory addresses.
    return {k: _plain(getattr(value, k, None))
            for k in ("name", "pose", "dims", "radius", "file_path", "scale")}


def scene_evidence(planner):
    objects = _plain(list(getattr(getattr(planner, "_world", None), "objects", []) or []))
    state = {"world_objects": objects,
             "world_signature": _plain(getattr(planner, "_persistent_world_signature", None)),
             "disabled_links": sorted(getattr(planner, "_disabled_collision_links", set())),
             "disabled_objects": sorted(getattr(planner, "_disabled_world_obstacles", set()))}
    encoded = json.dumps(state, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return {**state, "scene_fingerprint": hashlib.sha256(encoded.encode()).hexdigest()}


def install_contact_audit(direct, emit, *, strict=True, tray_final_descent=False):
    """Install before portable.main patches the native module; process-local only.

    strict never calls the broad toggle. A table omission in final approach also
    aborts before world refresh. This is a rejection boundary, NOT a pair-local
    collision implementation or certification of other native stages.
    """
    context = contextvars.ContextVar("native_final_contact", default={})
    original_final = direct._apply_deferred_two_step_final_approach
    original_toggle = direct._set_world_collision_for_links
    original_refresh = direct._refresh_curobo_world
    original_episode = direct.run_targeted_place_episode_curobo_direct
    original_segment = getattr(direct, '_plan_constrained_linear_segment', None)
    if tray_final_descent and (not strict or not callable(original_segment)):
        raise ValueError('tray exception requires strict outer boundary and native segment hook')
    mode = 'tray_final_descent_compatibility' if tray_final_descent else ('strict' if strict else 'compatibility')

    def record(planner, label, links, reason, **extra):
        row = {"event": "native_contact_audit", "mode": mode,
               "piece_id": None, "permitted_contact_target": None,
               "permitted_contact_support": None,
               **context.get(), **scene_evidence(planner), "step_id": label,
               "requested_links": list(links), "changed_links": [], "reason": reason,
               "pair_local_supported": False, "verified_task_success": None, **extra}
        emit(row)
        return row

    @functools.wraps(original_episode)
    def episode(*args, **kwargs):
        episode_args = kwargs.get("args", args[3] if len(args) > 3 else None)
        piece = direct._current_source_object_name(episode_args) if episode_args else None
        token = context.set({"piece_id": piece, "permitted_contact_target": piece})
        try:
            return original_episode(*args, **kwargs)
        finally:
            context.reset(token)

    @functools.wraps(original_final)
    def final(planner, demo, args, grasp_choice, pregrasp_lookup):
        target = direct._current_source_object_name(args)
        cache = getattr(demo, 'scene_capture_cache_ref', None) or {}
        entry = (cache.get('objects') or {}).get(target) or {}
        slot = entry.get('jimu_tray_slot_index')
        qualified_tray = (type(slot) is int and slot >= 0 and not entry.get('placed', False)
                          and not bool(getattr(args, 'execute_real', False)))
        token = context.set({"final_contact": True, "piece_id": target, "candidate_id": grasp_choice.get("label"),
                             'tray_qualified': qualified_tray, 'tray_slot_index': slot,
                             "permitted_contact_target": target,
                             "permitted_contact_support": None,
                             "support_qualification": "not_confirmed"})
        try:
            return original_final(planner, demo, args, grasp_choice, pregrasp_lookup)
        finally:
            context.reset(token)

    @functools.wraps(original_toggle)
    def toggle(planner, link_names, *, enabled, label):
        links = direct._normalize_disabled_world_collision_links(planner, link_names)
        reason = "broad_link_world_retry" if "gripper_world_relaxed" in label else "link_world_toggle"
        required = (True if reason == "broad_link_world_retry" and not enabled else None)
        row = record(planner, label, links, reason, enabled=enabled,
                     relaxed_branch_required=required,
                     requirement_evidence="constrained attempt returned None" if required else "not_established")
        if (tray_final_descent and context.get().get('tray_qualified')
                and label == f"{context.get().get('candidate_id')}_final_approach_gripper_world_relaxed"):
            # Native wants to mutate shared robot spheres. Suppress BOTH its
            # disable and restore; only the scoped world-cost input is filtered.
            emit({**row, 'event': 'native_shared_sphere_toggle_suppressed'})
            return []
        if strict and links and not enabled:
            raise StrictContactNotSupported(row)
        result = original_toggle(planner, link_names, enabled=enabled, label=label)
        emit({**row, "event": "native_contact_toggle_applied", "changed_links": list(result),
              "scene_after": scene_evidence(planner)})
        return result

    @functools.wraps(original_refresh)
    def refresh(planner, demo, args, **kwargs):
        if tray_final_descent and context.get().get('tray_qualified'):
            if not getattr(args, 'curobo_table_collision', True) or kwargs.get('exclude_object_names'):
                raise StrictContactNotSupported(record(planner, kwargs.get('label'), [], 'unqualified_world_exclusions'))
            # Keep the target and table visible to every NON-finger link too.
            kwargs = {**kwargs, 'include_table': True, 'include_active_object': True}
            record(planner, kwargs.get('label'), [], 'table_and_target_retained')
            return original_refresh(planner, demo, args, **kwargs)
        if context.get().get("final_contact") and (not kwargs.get("include_table", False)
                              or not getattr(args, "curobo_table_collision", True)):
            row = record(planner, kwargs.get("label"), [], "final_contact_table_omission",
                         relaxed_branch_required=None, requested_include_table=False)
            if strict:
                raise StrictContactNotSupported(row)
        return original_refresh(planner, demo, args, **kwargs)

    direct._apply_deferred_two_step_final_approach = final
    direct._set_world_collision_for_links = toggle
    direct._refresh_curobo_world = refresh
    direct.run_targeted_place_episode_curobo_direct = episode
    if tray_final_descent:
        @functools.wraps(original_segment)
        def segment(planner, demo, args, start_q, pose_start, pose_goal, *, label, **kwargs):
            expected = f"{context.get().get('candidate_id')}_final_approach_gripper_world_relaxed"
            if not context.get().get('tray_qualified') or label != expected:
                return original_segment(planner, demo, args, start_q, pose_start, pose_goal, label=label, **kwargs)
            import numpy as np
            from .world_only_contact import world_only_fingers, WorldOnlyContactUnsupported
            start = direct._pose_to_matrix_from_pose_obj(pose_start)[:3, 3]
            goal = direct._pose_to_matrix_from_pose_obj(pose_goal)[:3, 3]
            delta = goal - start
            # Reject lateral/ascending/long segments; do not enlarge native tolerances.
            if (not np.isfinite(delta).all() or not -0.080 <= delta[2] < 0
                    or np.linalg.norm(delta[:2]) > 0.001):
                raise StrictContactNotSupported(record(planner, label, [], 'not_short_vertical_tray_descent'))
            try:
                with direct._CUROBO_GPU_LOCK, world_only_fingers(planner) as evidence:
                    emit({'event': 'tray_world_only_filter_enter', 'mode': mode, **context.get(), **evidence})
                    try:
                        result = original_segment(planner, demo, args, start_q, pose_start, pose_goal, label=label, **kwargs)
                        if not evidence['world_filter_calls'] or getattr(planner, '_cuda_graph_batch_ik_solvers', {}):
                            raise WorldOnlyContactUnsupported('world_filter_not_observed_or_graph_created')
                        return result
                    finally:
                        emit({'event': 'tray_world_only_filter_end', 'mode': mode, **evidence})
            except WorldOnlyContactUnsupported as exc:
                raise StrictContactNotSupported(record(planner, label, [], str(exc))) from exc
        direct._plan_constrained_linear_segment = segment
