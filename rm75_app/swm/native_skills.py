"""Native phase solvers using the shared PickPlace coordinator and push planner.

No legacy episode is called. The caller must supply the original trusted task
compiler, current-scene full-path auditor, and feedback-producing executor.
Incomplete installations must not advertise these bindings in a worker.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import copy
import uuid

import numpy as np

from .adapters import NativeAtomicBinding
from .scene import SceneInvalid, digest, transform, pose_error
from .skills import ExecutionReceipt, PlannedSkill


@dataclass(frozen=True)
class NativeStageState:
    """Predicted collision state, not evidence that a grasp actually succeeded."""
    gripper_closed: bool | None
    holding: str
    T_tcp_object: object = None
    released_object_pose: object = None
    gripper_positions: object = None

    def __post_init__(self):
        if (not self.holding or (type(self.gripper_closed) is not bool
                and not (self.gripper_closed is None and self.gripper_positions is not None))):
            raise ValueError("Explicit jaw geometry and attachment required")
        if self.gripper_positions is not None:
            positions = dict(self.gripper_positions)
            required = {f'gripper_{side}_{part}_Joint' for side in ('Left', 'Right')
                        for part in ('1', '2', 'Support')}
            if set(positions) != required or not np.isfinite(list(positions.values())).all():
                raise ValueError('Complete finite six-joint jaw geometry required')
            object.__setattr__(self, 'gripper_positions', tuple(
                (name, float(positions[name])) for name in sorted(positions)))
        if (self.holding != 'empty') != (self.T_tcp_object is not None):
            raise ValueError("Held geometry requires a TCP-relative transform")
        if self.holding != 'empty' and self.released_object_pose is not None:
            raise ValueError("An attached object cannot also be released")
        for key in ('T_tcp_object', 'released_object_pose'):
            value = getattr(self, key)
            if value is not None:
                object.__setattr__(self, key, tuple(tuple(row) for row in transform(value).tolist()))

    def as_dict(self):
        value = dict(gripper_closed=self.gripper_closed, holding=self.holding,
                     T_tcp_object=self.T_tcp_object, released_object_pose=self.released_object_pose)
        if self.gripper_positions is not None:
            value['gripper_positions'] = dict(self.gripper_positions)
        return value


@dataclass(frozen=True)
class NativeStage:
    name: str
    trajectory: object
    gripper_after: bool | None = None
    state_before: NativeStageState | None = None
    state_after: NativeStageState | None = None
    contact_objects: tuple[str, ...] = ()
    allow_start_contact_escape: bool = False


@dataclass(frozen=True)
class NativePrimitive:
    skill: str
    object_id: str
    stages: tuple[NativeStage, ...]

    def fingerprint(self):
        return digest(dict(skill=self.skill, object_id=self.object_id, stages=[
            dict(name=s.name, joint_names=list(s.trajectory.joint_names),
                 positions=np.asarray(s.trajectory.positions).tolist(),
                 dt=None if s.trajectory.dt is None else np.asarray(s.trajectory.dt).tolist(),
                 gripper_after=s.gripper_after,
                 state_before=None if s.state_before is None else s.state_before.as_dict(),
                 state_after=None if s.state_after is None else s.state_after.as_dict(),
                 contact_objects=s.contact_objects,
                 allow_start_contact_escape=s.allow_start_contact_escape) for s in self.stages]))


class SharedPrimitiveExecutor:
    """Dispatch native stages to the SAME trajectory/gripper sink.

    Completion is checked against fresh feedback after the final stage. Closing
    a gripper does not set holding; only the following SWM observation may do so.
    ``feedback`` must read the existing executor/simulator, never a command cache.
    """

    def __init__(self, executor, feedback, *, clock, stop, endpoint_tolerance_rad=.08,
                 recorder=None, recording_context=None):
        if not 0 < endpoint_tolerance_rad <= .08:
            raise ValueError("Bounded measured endpoint tolerance required")
        self.executor, self.feedback = executor, feedback
        self.clock, self.stop = clock, stop
        self.tolerance = endpoint_tolerance_rad
        if (recorder is None) != (recording_context is None) or (
                recording_context is not None and not callable(recording_context)):
            raise ValueError('Recorder and trusted measured support context must be installed together')
        self.recorder, self.recording_context = recorder, recording_context

    def __call__(self, plan, audit):
        primitive = plan.payload
        if not isinstance(primitive, NativePrimitive) or primitive.fingerprint() != plan.payload_digest:
            raise SceneInvalid("Native primitive changed since audit")
        if audit.payload_digest != plan.payload_digest:
            raise SceneInvalid("Audit belongs to another native primitive")
        if not primitive.stages:
            raise SceneInvalid("Empty native primitive")
        started = self.clock()
        action_id = uuid.uuid4().hex
        handle = None
        if self.recorder is not None and primitive.skill == 'push':
            handle = self.recorder.begin_action(action_id, started,
                **self.recording_context(primitive))
        try:
            for stage in primitive.stages:
                self.stop.check()
                self.executor.execute_trajectory(stage.name, stage.trajectory)
                if stage.gripper_after is not None:
                    self.stop.check()
                    self.executor.set_gripper(stage.gripper_after)
            observed = self.feedback()
            ended = self.clock()
            final = primitive.stages[-1].trajectory
            q = np.asarray(observed['positions'], dtype=float)
            if (observed.get('source') != 'measured_feedback' or observed.get('idle') is not True
                    or not started <= observed['captured_at'] <= ended
                    or tuple(observed['joint_names']) != tuple(final.joint_names)
                    or q.shape != (7,) or not np.isfinite(q).all()
                    or np.max(np.abs(q - final.positions[-1])) > self.tolerance):
                raise SceneInvalid("Native executor lacks fresh completed-motion feedback")
            receipt = ExecutionReceipt(True, started, ended, True, action_id)
            return receipt if handle is None else self.recorder.finish_command(handle, receipt)
        except BaseException:
            if handle is not None:
                handle.close()
            raise


class PickPlaceNativePhases:
    """Independent grasp/lift and measured-attachment place/retreat solvers.

    ``build_task(request, snapshot)`` reuses the installed object-specific or
    Jimu compiler. It must preserve the original collision/placement policies
    and return a task in base_link with stable instance ids. The restricted
    first migration supports exact rigid targets; symmetry/containment tasks
    need their original goal-verifier adapter before installation.
    """

    def __init__(self, coordinator, build_task, audit, execute, *, closure_screen=None, emit=None,
                 candidate_priority=None):
        self.coordinator = coordinator
        self.build_task = build_task
        self.last_relation_screen = {}
        if closure_screen is not None and not callable(closure_screen):
            raise TypeError("Trusted closure screening callback required")
        self.closure_screen = closure_screen
        if candidate_priority is not None and not callable(candidate_priority):
            raise TypeError("Trusted candidate priority callback required")
        self.candidate_priority = candidate_priority
        self.emit = emit or (lambda **row: None)
        self.relation_screen_history = []
        self.audit = audit
        self.execute = execute

    def bindings(self):
        return {skill: NativeAtomicBinding(self.plan, self.audit, self.execute, boundary)
                for skill, boundary in (('grasp', 'approach->grasp->close->lift'),
                                        ('place', 'measured_attachment->preplace->place->release->retreat'))}

    def plan(self, request, snapshot):
        from rm75_app.pickplace.coordinator import (
            _approach_offset_candidates, _offset_candidates, _place_approach_candidates,
            _reverse_trajectory, _pose_matrix)
        from rm75_app.pickplace.cached_scene import matrix_to_quaternion_wxyz
        from rm75_app.planning.contracts import Pose, PoseCandidate

        if request.skill not in ('grasp', 'place'):
            raise NotImplementedError("Only native grasp and place are installed")
        task = self.build_task(request, copy.deepcopy(snapshot))
        if (task.object_name != request.object_id or task.scene.revision != snapshot['snapshot_id']
                or tuple(task.current.names) != tuple(snapshot['robot']['joint_names'])
                or not np.array_equal(task.current.positions, snapshot['robot']['positions'])):
            raise SceneInvalid("Native task compiler returned another scene/start/instance")
        if {o.name for o in task.scene.objects} != set(snapshot['objects']):
            raise SceneInvalid("Native task omitted obstacles or already placed objects")
        for obj in task.scene.objects:
            p, r = pose_error(obj.metadata.get("swm_object_pose", _pose_matrix(obj.pose)), snapshot['objects'][obj.name]['measured']['T_world_object'])
            if p > 1e-8 or r > 1e-6:
                raise SceneInvalid("Native task compiler did not use measured object poses")
        c = self.coordinator
        measured_jaw = snapshot['robot'].get('gripper_positions')
        stages = None
        if request.skill == 'grasp':
            if snapshot['robot']['holding'] != 'empty':
                raise SceneInvalid("Native grasp requires an observed empty gripper")
            if measured_jaw is not None:
                c.planner.set_measured_gripper_collision_state(measured_jaw)
            else:
                c.planner.set_gripper_collision_state(False)
            for grasp_candidate, screened, task in self._grasp_relations(task, measured_jaw, snapshot):
                pre = _approach_offset_candidates((grasp_candidate,), abs(task.grasp_approach_offset))[0]
                approach = c._plan_pose_stage(stage='pregrasp', current=task.current, candidates=(pre,), task=task)
                if approach is None or approach.trajectory is None:
                    continue
                grasp = c._plan_linear_stage(stage='grasp', current=c._end_configuration(approach.trajectory),
                    candidate=grasp_candidate, task=task, ignore_object_name=task.object_name)
                if grasp is None or grasp.trajectory is None:
                    continue
                grasp_q = c._end_configuration(grasp.trajectory)
                if self.closure_screen is not None:
                    permitted = self.closure_screen(grasp_candidate, snapshot, grasp_q)
                    if type(permitted) is not bool:
                        raise SceneInvalid("Closure screen must explicitly accept or reject a candidate")
                    if not permitted:
                        continue
                c.planner.attach_object(task.object_name, grasp_q)
                try:
                    origin = PoseCandidate(grasp_candidate.candidate_id,
                        c.planner.tool_pose_for_configuration(grasp_q, task.tool_frame))
                    options = ((_offset_candidates((origin,), abs(task.lift_height), 'lift_world_z')[0], False),
                               (_approach_offset_candidates((origin,), abs(task.lift_height), 'lift_tool_z')[0], True))
                    for candidate, project in options:
                        lift = c._plan_linear_stage(stage='lift', current=grasp_q, candidate=candidate,
                            task=task, allow_start_contact_escape=True, axis='z', project_distance_to_goal=project)
                        if lift is not None and lift.trajectory is not None:
                            relative = np.linalg.inv(_pose_matrix(origin.pose)) @ transform(
                                snapshot['objects'][request.object_id]['measured']['T_world_object'])
                            lifted_q = c._end_configuration(lift.trajectory)
                            # Screen placement with the same original candidates and
                            # measured/predicted attachment. Discard these paths: the
                            # place skill MUST solve again after its fresh observation.
                            if self._placement_stages(task, lifted_q, relative, None,
                                    candidates=screened.places_by_grasp[grasp_candidate.candidate_id]) is None:
                                continue
                            empty = NativeStageState(None if measured_jaw is not None else False,
                                'empty', gripper_positions=measured_jaw)
                            held = NativeStageState(True, task.object_name, relative)
                            stages = (NativeStage('approach', approach.trajectory,
                                          state_before=empty, state_after=empty),
                                      NativeStage('grasp', grasp.trajectory, True, empty, held,
                                          (task.object_name,)),
                                      NativeStage('lift', lift.trajectory, state_before=held,
                                          state_after=held, allow_start_contact_escape=True))
                            expected = _pose_matrix(c.planner.tool_pose_for_configuration(
                                lifted_q, task.tool_frame)) @ relative
                            break
                finally:
                    # detach alone does not restore moved proxy poses or the
                    # complete obstacle set after hypothetical candidate planning.
                    c.planner.detach_object(task.object_name)
                    c.planner.update_scene(task.scene)
                    if measured_jaw is not None:
                        c.planner.set_measured_gripper_collision_state(measured_jaw)
                    else:
                        c.planner.set_gripper_collision_state(False)
                if stages is not None:
                    break
        else:
            if snapshot['robot']['holding'] != request.object_id:
                raise SceneInvalid("Native place requires measured holding of this instance")
            relative = np.linalg.inv(transform(snapshot['robot']['T_world_tcp'])) @ transform(
                snapshot['objects'][request.object_id]['measured']['T_world_object'])
            try:
                c.planner.update_attached_object_pose(task.object_name, task.current, relative)
                placement = self._placement_stages(task, task.current, relative, request,
                    jaw_positions=measured_jaw)
                if placement is not None:
                    stages, expected = placement
            finally:
                # Restore the observed held state, including jaw geometry, on
                # success, infeasibility and exceptions. Audit starts from here.
                c.planner.detach_object(task.object_name)
                c.planner.update_scene(task.scene)
                c.planner.attach_object(task.object_name, task.current)
                c.planner.update_attached_object_pose(task.object_name, task.current, relative)
                if measured_jaw is not None:
                    c.planner.set_measured_gripper_collision_state(measured_jaw)
                else:
                    c.planner.set_gripper_collision_state(True)
        if stages is None:
            raise RuntimeError('Native atomic phase has no feasible trajectory')
        primitive = NativePrimitive(request.skill, request.object_id, stages)
        return PlannedSkill(digest(request.as_dict()), snapshot['snapshot_id'], primitive.fingerprint(),
                            primitive, expected.tolist(), 'shared_PickPlaceCoordinator_phase_solvers')


    def _grasp_relations(self, task, measured_jaw, source_snapshot=None):
        """Share one motion budget across discrete and original axis searches.

        Pool-local IDs keep changed axis geometry eligible; an exact source/pose
        fingerprint prevents repeating an already attempted geometric candidate.
        Full task pairing and scene state travel with each yielded candidate.
        """
        initial_task = task
        budget = task.max_motion_candidates
        used = 0
        attempted = set()
        attempted_geometry = set()
        axis_used = False
        self.relation_screen_history = []
        while used < budget:
            original_ids = {candidate.candidate_id for candidate in task.grasp_candidates}
            options = dict(initial_gripper_positions=measured_jaw)
            if attempted:
                options['excluded_grasp_ids'] = tuple(sorted(attempted))
            screened = self.coordinator.screen_relations(task, **options)
            self.last_relation_screen = copy.deepcopy(dict(screened.diagnostics))
            if self.candidate_priority is None or not screened.grasp_candidates:
                ranked = self.coordinator.rank_grasp_relations(
                    task, screened.grasp_candidates, screened.grasp_scores) if screened.grasp_candidates else ()
            else:
                priorities = self.candidate_priority(task, screened.grasp_candidates, source_snapshot)
                expected_ids = {candidate.candidate_id for candidate in screened.grasp_candidates}
                if (not isinstance(priorities, dict) or set(priorities) != expected_ids
                        or any(type(value) is not int or value not in (0, 1, 2)
                               for value in priorities.values())):
                    raise SceneInvalid('Complete explicit candidate priority classifications required')
                # Keep original distance/score/source-diversity ordering within
                # each heuristic tier. No candidate is declared safe or removed.
                ranked = tuple(candidate for level in (0, 1, 2)
                    for candidate in self.coordinator.rank_grasp_relations(task,
                        tuple(item for item in screened.grasp_candidates
                              if priorities[item.candidate_id] == level), screened.grasp_scores))
            remaining = budget - used
            ranked = tuple(ranked)[:remaining]
            ids = [candidate.candidate_id for candidate in ranked]
            if (len(ids) != len(set(ids)) or set(ids) & attempted
                    or not set(ids) <= original_ids):
                raise SceneInvalid('Relation continuation returned repeated or foreign grasp identity')
            evidence = dict(round=len(self.relation_screen_history)+1,
                snapshot_id=task.scene.revision, excluded_grasp_ids=sorted(attempted),
                original_candidate_count=len(initial_task.grasp_candidates),
                pool_candidate_count=len(task.grasp_candidates),
                search_mode='continuous_axis' if axis_used else 'discrete',
                screened_count=len(screened.grasp_candidates), ranked_ids=ids,
                total_motion_budget=budget, remaining_motion_budget=remaining,
                relation_diagnostics=copy.deepcopy(self.last_relation_screen))
            self.relation_screen_history.append(copy.deepcopy(evidence))
            self.emit(kind='swm_native_relation_search', **evidence)
            if not ranked:
                if axis_used or not screened.axis_requested:
                    return
                axis_used = True
                fallback, counts = self.coordinator.resolve_axis_fallback_task(initial_task)
                self.emit(kind='swm_native_axis_candidates_built',
                    snapshot_id=task.scene.revision, **counts,
                    eligible_count=0 if fallback is None else len(fallback.grasp_candidates),
                    remaining_motion_budget=remaining, whole_episode_called=False)
                if fallback is None:
                    return
                task = fallback
                attempted = set()
                continue
            for candidate in ranked:
                attempted.add(candidate.candidate_id)
                geometry = digest(dict(
                    source_id=str(candidate.metadata.get('source_grasp_candidate_id', candidate.candidate_id)),
                    position=np.asarray(candidate.pose.position).tolist(),
                    quaternion_wxyz=np.asarray(candidate.pose.quaternion_wxyz).tolist()))
                if geometry in attempted_geometry:
                    self.emit(kind='swm_native_duplicate_candidate_geometry_skipped',
                        candidate_id=candidate.candidate_id, snapshot_id=task.scene.revision)
                    continue
                attempted_geometry.add(geometry)
                used += 1
                yield candidate, screened, task


    def _placement_stages(self, task, current, relative, request, *, jaw_positions=None, candidates=None):
        """Shared feasibility screening and post-observation place planning.

        These are candidate paths only. In particular, the reversed retreat
        still requires the installed auditor under its RELEASED/OPEN state.
        """
        from rm75_app.pickplace.coordinator import _place_approach_candidates, _reverse_trajectory
        from rm75_app.pickplace.cached_scene import matrix_to_quaternion_wxyz
        from rm75_app.planning.contracts import Pose

        c = self.coordinator
        held = NativeStageState(None if jaw_positions is not None else True,
                               task.object_name, relative, gripper_positions=jaw_positions)
        # During grasp lookahead retain the ORIGINAL per-grasp pairing before
        # applying its candidate budget. Global rows include duplicate symmetry
        # targets from other grasps and can crowd out the selected relation.
        pool = task.place_candidates if candidates is None else candidates
        for original in sorted(pool, key=lambda x: x.score, reverse=True)[:task.max_motion_candidates]:
            raw_target = original.metadata.get('planning_target_object_pose')
            if raw_target is None:
                raise SceneInvalid("Original placement compiler must provide a world object target")
            if request is not None:
                p, r = pose_error(raw_target, request.target)
                if p > request.position_tolerance_m or r > request.rotation_tolerance_rad:
                    continue
            tcp = transform(raw_target) @ np.linalg.inv(relative)
            candidate = replace(original, pose=Pose(tcp[:3, 3], matrix_to_quaternion_wxyz(tcp[:3, :3])),
                metadata={**original.metadata, 'T_tcp_object': relative.tolist()})
            for pre in _place_approach_candidates(candidate, task.place_clearance):
                preplace = c._plan_pose_stage(stage='preplace', current=current, candidates=(pre,), task=task)
                if preplace is None or preplace.trajectory is None:
                    continue
                place = c._plan_linear_stage(stage='place', current=c._end_configuration(preplace.trajectory),
                    candidate=candidate, task=task, ignore_object_name=task.place_contact_object_name,
                    project_distance_to_goal=True)
                if place is None or place.trajectory is None:
                    continue
                released = NativeStageState(False, 'empty', released_object_pose=raw_target)
                contacts = () if task.place_contact_object_name is None else (task.place_contact_object_name,)
                stages = (NativeStage('preplace', preplace.trajectory, state_before=held, state_after=held),
                          NativeStage('place', place.trajectory, False, held, released, contacts),
                          NativeStage('retreat', _reverse_trajectory(place.trajectory),
                              state_before=released, state_after=released))
                return stages, transform(raw_target)
        return None


class PushNativePhase:
    """One original prepared long-push segment; never a complete PushT session.

    The native scene compiler must include every SWM instance (including moving
    obstacles) before this adapter is installed. ``select_segment`` uses the
    original local search and returns a segment plus its predicted WORLD pose.
    """

    def __init__(self, planner, select_segment, compile_observation, audit, execute):
        self.planner, self.select_segment = planner, select_segment
        self.compile_observation, self.audit, self.execute = compile_observation, audit, execute

    def binding(self):
        return NativeAtomicBinding(self.plan, self.audit, self.execute, 'approach->descend->contact->push->retreat')

    def plan(self, request, snapshot):
        from rm75_app.planning.contracts import JointTrajectory

        if request.skill != 'push':
            raise NotImplementedError('Only a native push segment is installed')
        observation = self.compile_observation(snapshot, request.object_id)
        push, expected = self.select_segment(request, copy.deepcopy(snapshot), observation)
        prepared = self.planner.plan_push(push, observation)
        if not np.array_equal(prepared.start_q, snapshot['robot']['positions']):
            raise SceneInvalid('Push planner did not start from measured SWM joints')
        names = tuple(self.planner.names)
        stages = tuple(NativeStage(stage, JointTrajectory(names, path, dt=np.diff(times)))
                       for stage, path, times in prepared.stages)
        primitive = NativePrimitive('push', request.object_id, stages)
        return PlannedSkill(digest(request.as_dict()), snapshot['snapshot_id'], primitive.fingerprint(),
                            primitive, transform(expected).tolist(), 'shared_CuroboPushExecutor.plan_push')
