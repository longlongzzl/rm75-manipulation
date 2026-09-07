"""Simulation compatibility: retain contact stages, fully check loaded transport.

All modifications are process-local adapters around the verified snapshot.
World exceptions never remove shared robot spheres or disable self checks.
"""
import copy
import functools
import threading
import numpy as np
from .contact_audit import StrictContactNotSupported, scene_evidence
from .world_only_contact import FINGER_LINKS, world_only_links, WorldOnlyContactUnsupported


def payload_contact_config(data):
    """Only attached-object <-> finger/pad pairs may ignore grasp contact.

    Preserve unrelated robot adjacency exclusions, buffers, and all geometry.
    In particular remove the old payload/base and payload/gripper-base ignores.
    """
    data = copy.deepcopy(data)
    kin = data.get('robot_cfg', data)['kinematics']
    if 'attached_object' not in kin.get('collision_link_names', []):
        raise WorldOnlyContactUnsupported('attached_collision_link_missing')
    ignores = kin.get('self_collision_ignore') or {}
    cleaned = {a: [b for b in bs if b != 'attached_object']
               for a, bs in ignores.items() if a != 'attached_object'}
    for finger in sorted(FINGER_LINKS.intersection(kin['collision_link_names'])):
        cleaned.setdefault(finger, []).append('attached_object')
    kin['self_collision_ignore'] = cleaned
    return data


def is_transport(label):
    label = str(label)
    return ('transport' in label and not any(x in label for x in
            ('final_contact', 'release_precheck', 'paired_relation_ik')))


def install_payload_pairs(planner_class):
    original = planner_class._load_robot_cfg_dict
    @functools.wraps(original)
    def load(self):
        return payload_contact_config(original(self))
    planner_class._load_robot_cfg_dict = load


def preserve_start_check_flags(motion_gen):
    """Local cuRobo WORLD_COLLISION early return forgets to re-enable self cost.

    Preserve the original result/exception and restore both entry flags even
    on an early return. Do not turn an invalid query into a valid one.
    """
    original = motion_gen.check_start_state
    @functools.wraps(original)
    def check(*args, **kwargs):
        costs = [motion_gen.rollout_fn.primitive_collision_constraint,
                 motion_gen.rollout_fn.robot_self_collision_constraint]
        before = [(cost, bool(cost.enabled)) for cost in costs]
        try:
            return original(*args, **kwargs)
        finally:
            for cost, enabled in before:
                if enabled:
                    cost.enable_cost()
                else:
                    cost.disable_cost()
    motion_gen.check_start_state = check
    return motion_gen


def install_start_check_restoration(planner_class):
    original = planner_class._build_motion_gen
    @functools.wraps(original)
    def build(self, *args, **kwargs):
        return preserve_start_check_flags(original(self, *args, **kwargs))
    planner_class._build_motion_gen = build


def install_read_only_jimu_diagnostics(portable, emit):
    """Replace diagnostic-only collision ablations, never a planning result.

    Reviewed native consumers only print/store this dictionary. The old helper
    clears the world and removes payload/link geometry, and its BaseException
    escape can leave the world empty. Query the unchanged current state instead.
    No 'valid after removal' claim is fabricated; planning/retry code is untouched.
    """
    original = portable._jimu_start_collision_diagnosis

    @functools.wraps(original)
    def diagnose(planner, args, candidates, label, disabled_world_collision_links):
        candidates = list(candidates or ())
        if planner is None or not candidates or candidates[0].get('start_q') is None:
            return None
        q = np.asarray(candidates[0]['start_q'], dtype=np.float32).reshape(-1)[:7]
        if q.size != 7 or not np.isfinite(q).all():
            return {'valid': False, 'status': 'INVALID_DIAGNOSTIC_JOINTS',
                    'diagnostic_mode': 'read_only_current_collision_state'}
        with portable.direct._CUROBO_GPU_LOCK:
            valid, status = planner.check_start_state(q)
            row = {'valid': bool(valid), 'status': str(status),
                   'diagnostic_mode': 'read_only_current_collision_state',
                   'world_obstacle_names': [obj.name for obj in planner._world.objects],
                   'valid_after_removing': [], 'ablation': [], 'group_ablation': [],
                   'empty_world_diag': {'not_run': 'world_must_remain_present'},
                   'attached_disabled_diag': {'not_run': 'payload_must_remain_present'},
                   'link_ablation': [], 'requested_disabled_links': list(disabled_world_collision_links or ()),
                   'attached_sphere_summary': {
                       'active': bool(planner.attached_object_active),
                       'count': int(planner.get_attached_sphere_count())},
                   'candidate_label': str(candidates[0].get('label', '')),
                   **scene_evidence(planner)}
        emit({'event': 'jimu_read_only_collision_diagnostic', 'step_id': label, **row})
        return row

    portable._jimu_start_collision_diagnosis = diagnose


def install_transport_contact(direct, planner_class, emit):
    """Explicit no-motion runner only; do not install in a real worker."""
    install_payload_pairs(planner_class)
    install_start_check_restoration(planner_class)
    original_batch_ik = planner_class.solve_batch_start_goal_ik
    @functools.wraps(original_batch_ik)
    def eager_ik(self, *args, **kwargs):
        # Replay must not retain an exception from a prior contact stage.
        # Same candidates/seeds/thresholds; only execution acceleration changes.
        kwargs['use_cuda_graph_batch'] = False
        return original_batch_ik(self, *args, **kwargs)
    planner_class.solve_batch_start_goal_ik = eager_ik
    direct._transport_attached_contact_disabled_links = lambda planner, base_links=None: []
    original_relief=getattr(direct,'_single_obstacle_start_collision_relief',None)
    if callable(original_relief):
        @functools.wraps(original_relief)
        def no_world_object_relief(planner,*args,**kwargs):
            obstacle=original_relief(planner,*args,**kwargs)
            if obstacle:
                emit({'event':'contact_obstacle_relief_refused','obstacle':obstacle,
                      'reason':'keep_non_source_obstacle_in_world',**scene_evidence(planner)})
            # Native caller continues its SAME lift/retreat/candidate ladder,
            # with the unrelated obstacle present. No candidate is fabricated
            # or removed; an unsafe whole-object exclusion is never attempted.
            return None
        direct._single_obstacle_start_collision_relief=no_world_object_relief
    original_refresh = direct._refresh_curobo_world
    active = {}
    excluded_sources = {}

    def failure(planner, label, reason):
        row = {'event': 'transport_policy_rejected', 'step_id': label, 'reason': reason,
               'verified_task_success': None, **scene_evidence(planner)}
        emit(row)
        return StrictContactNotSupported(row)

    def toggle(planner, link_names, *, enabled, label):
        links = direct._normalize_disabled_world_collision_links(planner, link_names)
        key = (threading.get_ident(), id(planner), label)
        if enabled:
            pending = active.get(key, [])
            if pending:
                manager, evidence = pending.pop()
                try:
                    manager.__exit__(None, None, None)
                finally:
                    direct._CUROBO_GPU_LOCK.release()
                emit({'event': 'contact_world_filter_restored', 'step_id': label, **evidence})
            return links
        if not links:
            return []
        if is_transport(label):
            raise failure(planner, label, 'transport_world_exemption_forbidden')
        # A table omitted by the original NON-transport scene can leave a
        # disabled cache entry. This never permits disabling a present table
        # or omitting a table in transport; the evaluator checks both.
        allowed = {'active_target_object'}
        present = {obj.name for obj in planner._world.objects}
        if 'virtual_table_plane' not in present:
            allowed.add('virtual_table_plane')
        source = excluded_sources.get(id(planner))
        if (source and planner.attached_object_active
                and planner.get_attached_sphere_count() > 0):
            # A source-specific mesh cache entry is the same object now
            # represented by attached spheres, not an unrelated obstacle.
            # Require explicit exclusion by the most recent world refresh.
            allowed.update(name for name in (source, 'scene_obstacle_' + source)
                           if name not in present)
        manager = world_only_links(planner, links, allowed_disabled_objects=allowed)
        direct._CUROBO_GPU_LOCK.acquire()
        try:
            evidence = manager.__enter__()
        except WorldOnlyContactUnsupported as exc:
            direct._CUROBO_GPU_LOCK.release()
            raise failure(planner, label, str(exc)) from exc
        except BaseException:
            direct._CUROBO_GPU_LOCK.release()
            raise
        active.setdefault(key, []).append((manager, evidence))
        emit({'event': 'contact_world_filter_enter', 'step_id': label, **evidence,
              'verified_task_success': None})
        return links
    direct._set_world_collision_for_links = toggle

    @functools.wraps(original_refresh)
    def refresh(planner, demo, args, **kwargs):
        if bool(getattr(args, 'execute_real', False)):
            raise failure(planner, kwargs.get('label'), 'simulation_only_policy')
        label=str(kwargs.get('label',''))
        free_loaded=(is_transport(label) or 'joint_start_' in label or 'post_grasp_lift' in label)
        if free_loaded:
            if not getattr(args, 'curobo_table_collision', True):
                raise failure(planner, kwargs.get('label'), 'transport_table_check_disabled')
            kwargs['include_table'] = True
            if planner.attached_object_active:
                source=direct._current_source_object_name(args)
                requested=set(kwargs.get('exclude_object_names') or ())
                restored=requested-{source}
                if restored:
                    emit({'event':'loaded_world_exclusions_restored','step_id':label,
                          'restored_objects':sorted(restored),'attached_source':source})
                kwargs['exclude_object_names']=requested & {source}
        with direct._CUROBO_GPU_LOCK:
            excluded_sources.pop(id(planner), None)
            result = original_refresh(planner, demo, args, **kwargs)
            source = direct._current_source_object_name(args)
            if source and source in (kwargs.get('exclude_object_names') or ()):
                excluded_sources[id(planner)] = source
            return result
    direct._refresh_curobo_world = refresh

    def guarded_evaluator(original):
        @functools.wraps(original)
        def evaluate(planner, demo, args, *pos, **kwargs):
            label = kwargs.get('label', '')
            if not is_transport(label):
                return original(planner, demo, args, *pos, **kwargs)
            if bool(getattr(args, 'execute_real', False)):
                raise failure(planner, label, 'simulation_only_policy')
            source = direct._current_source_object_name(args)
            allowed_excludes = {source}
            requested=set(kwargs.get('exclude_object_names') or [])
            if requested-allowed_excludes:
                emit({'event':'loaded_world_exclusions_restored','step_id':label,
                      'restored_objects':sorted(requested-allowed_excludes),'attached_source':source})
            kwargs = {**kwargs, 'disabled_world_collision_links': [], 'include_table': True,
                      'exclude_object_names':requested & allowed_excludes}
            with direct._CUROBO_GPU_LOCK:
                if not planner.collision_enabled or not planner.config.self_collision_check:
                    raise failure(planner, label, 'transport_collision_checks_disabled')
                if any(k[1] == id(planner) and v for k, v in active.items()):
                    raise failure(planner, label, 'contact_mask_leaked_into_transport')
                if not planner.attached_object_active or planner.get_attached_sphere_count() <= 0:
                    raise failure(planner, label, 'transport_payload_collision_missing')
                results = original(planner, demo, args, *pos, **kwargs)
                # Release prechecks may rebuild the world. Restore transport
                # world before the independent exhaustive sampled-path check.
                refresh(planner, demo, args, label=label + '_full_world_audit', include_table=True,
                        include_active_object=False, exclude_object_names=kwargs.get('exclude_object_names'))
                if (planner._disabled_collision_links or
                    set(planner._disabled_world_obstacles) - {'active_target_object', source, 'scene_obstacle_' + source}):
                    raise failure(planner, label, 'transport_collision_state_not_restored')
                if any(k[1] == id(planner) and v for k, v in active.items()):
                    raise failure(planner, label, 'contact_mask_leaked_after_precheck')
                if not planner.attached_object_active or planner.get_attached_sphere_count() <= 0:
                    raise failure(planner, label, 'transport_payload_lost_during_planning')
                table_name = direct._build_virtual_table_cuboid(args)['name']
                if table_name not in {obj.name for obj in planner._world.objects}:
                    raise failure(planner, label, 'transport_table_missing')
                samples = 0
                for result in results:
                    path = result.get('q_path')
                    if path is None or len(path) < 2:
                        raise failure(planner, label, 'transport_path_missing')
                    for q in path:
                        valid, status = planner.check_start_state(q)
                        samples += 1
                        if not valid:
                            raise failure(planner, label, 'transport_sample_collision:' + str(status))
                emit({'event': 'transport_full_world_audit', 'step_id': label, 'samples': samples,
                      'candidate_paths': len(results), 'payload_spheres': planner.get_attached_sphere_count(),
                      'world_exempt_links': [], 'verified_task_success': None, **scene_evidence(planner)})
                return results
        return evaluate
    for name in ('_evaluate_curobo_pose_candidates', '_evaluate_curobo_pose_candidates_goalset',
                 '_evaluate_curobo_pose_candidates_multi_start'):
        setattr(direct, name, guarded_evaluator(getattr(direct, name)))

    def close():
        for key, pending in active.items():
            if key[0] != threading.get_ident() and pending:
                raise RuntimeError('contact scope still active on another native thread')
            while pending:
                manager, _ = pending.pop()
                try:
                    manager.__exit__(None, None, None)
                finally:
                    direct._CUROBO_GPU_LOCK.release()
    return close
