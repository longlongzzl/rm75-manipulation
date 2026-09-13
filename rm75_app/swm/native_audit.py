"""Sequential stage auditing against the already owned cuRobo collision model.

No planner, simulator, camera or executor is constructed here. This auditor is
not a worker installation or evidence of a successful native simulation.
"""
from __future__ import annotations

import numpy as np

from .native_skills import NativePrimitive, NativeStageState
from .scene import SceneInvalid, pose_error, transform
from .skills import PlanAudit, REQUIRED_AUDITS


class CuroboNativeStageAuditor:
    """Audit paths and jaw/attachment transitions, restoring the idle model.

    Contact permissions come from the trusted native phase compiler. They are
    bound into the primitive digest, not supplied by a browser or model. Escape
    contacts retain the backend's link allowlist and 1 mm depth tolerance.
    The worker must first synchronize the complete native scene and provide
    independently observed six-joint jaw geometry; holding is not a proxy.
    """

    def __init__(self, backend, *, interpolation_step_rad=.01, max_samples=20000):
        if not 0 < interpolation_step_rad <= .01:
            raise ValueError('Collision interpolation must not exceed 0.01 rad')
        if type(max_samples) is not int or not 2 <= max_samples <= 20000:
            raise ValueError('Bounded native audit sample budget required')
        self.backend = backend
        self.step = interpolation_step_rad
        self.max_samples = max_samples
        self.last_evidence = []
        self.last_collision_evidence = None
        self._collision_holding = "empty"
        self._native_audit = False
        self.last_duplicate_exclusion = None

    def __call__(self, plan, snapshot, *, execution_state=None):
        from rm75_app.planning.contracts import JointConfiguration

        self.last_evidence = []
        self.last_collision_evidence = None
        primitive = plan.payload
        if (not isinstance(primitive, NativePrimitive) or not primitive.stages
                or primitive.fingerprint() != plan.payload_digest):
            raise SceneInvalid('Stage audit requires an unchanged native primitive')
        if primitive.skill not in ('grasp', 'place'):
            raise SceneInvalid('Native contact auditor not installed for this skill')
        robot = snapshot['robot']
        approved_state = None
        if execution_state is not None:
            feedback=execution_state['feedback'];approved_state=execution_state['state']
            if (execution_state.get('source')!='approved_grasp_place_prediction'
                    or execution_state.get('snapshot_id')!=snapshot['snapshot_id']
                    or execution_state.get('payload_digest')!=plan.payload_digest
                    or not isinstance(approved_state,NativeStageState)
                    or feedback.get('source')!='measured_feedback' or feedback.get('idle') is not True
                    or tuple(feedback['joint_names'])!=tuple(robot['joint_names'])):
                raise SceneInvalid('Invalid nonvisual execution audit context')
            robot=dict(robot,positions=feedback['positions'],gripper_positions=feedback['gripper_positions'],
                holding=approved_state.holding,T_world_tcp=feedback['T_world_tcp'])
        measured_jaw = robot.get('gripper_positions')
        native = snapshot.get('observation_domain') in ('real', 'physics')
        self._native_audit = native
        if robot.get('idle') is not True or (measured_jaw is None and (
                native or type(robot.get('gripper_closed')) is not bool)):
            raise SceneInvalid('Independent idle jaw observation required for native audit')
        scene = self.backend._scene
        if (scene is None or scene.revision != snapshot['snapshot_id']
                or {obj.name for obj in scene.objects} != set(snapshot['objects'])):
            raise SceneInvalid('Native audit requires the complete current collision scene')
        holding = robot['holding']
        self._collision_holding = holding
        relative = approved_state.T_tcp_object if approved_state is not None else None if holding == 'empty' else (
            np.linalg.inv(transform(robot['T_world_tcp'])) @ transform(
                snapshot['objects'][holding]['measured']['T_world_object']))
        initial = NativeStageState(None if measured_jaw is not None else robot['gripper_closed'],
                                   holding, relative, gripper_positions=measured_jaw)
        if approved_state is not None:self._same_state(initial,approved_state)
        names = tuple(robot['joint_names'])
        current = JointConfiguration(names, robot['positions'])
        previous = initial
        active_object = holding
        pending_escape = None
        try:
            for index, stage in enumerate(primitive.stages):
                if stage.state_before is None or stage.state_after is None:
                    raise SceneInvalid('Native stage lacks explicit pre/post collision state')
                self._same_state(previous, stage.state_before)
                if (not set(stage.contact_objects) <= set(snapshot['objects'])
                        or (stage.gripper_after is not None
                            and stage.gripper_after != stage.state_after.gripper_closed)
                        or (stage.gripper_after is None
                            and (stage.state_before.gripper_closed != stage.state_after.gripper_closed
                                 or stage.state_before.gripper_positions != stage.state_after.gripper_positions))):
                    raise SceneInvalid('Native stage has inconsistent jaw/contact transition')
                path = stage.trajectory
                q = np.asarray(path.positions, dtype=float)
                if (tuple(path.joint_names) != names or q.ndim != 2 or q.shape[1] != 7
                        or len(q) < 2 or not np.isfinite(q).all()
                        or not np.allclose(q[0], current.positions, atol=1e-6, rtol=0)):
                    raise SceneInvalid('Native stage path is discontinuous or starts at another state')
                reuse_post = (index > 0 and previous.as_dict() == stage.state_before.as_dict()
                              and np.array_equal(q[0], current.positions))
                if not reuse_post:
                    active_object = self._apply(stage.state_before, current, primitive.object_id,
                                                scene, active_object)
                if pending_escape is not None:
                    beginning = self._contacts(q[:1], names, set(stage.contact_objects))
                    if not self._same_contacts(pending_escape, beginning):
                        raise SceneInvalid('Pending support contact changed before lift audit')
                evidence = self._check_path(stage, q)
                if pending_escape is not None:
                    if not evidence['escape']:
                        raise SceneInvalid('Pending support contact lacks audited escape')
                    self.last_evidence[-1]['post_transition_escape_verified_by'] = stage.name
                    pending_escape = None
                end = JointConfiguration(names, q[-1])
                active_object = self._apply(stage.state_after, end, primitive.object_id,
                                            scene, active_object)
                # Opening or closing changes collision geometry at the endpoint.
                # Audit that endpoint in the POST state too, not just the descent.
                contacts = self._contacts(q[-1:], names, set(stage.contact_objects))
                if contacts:
                    following = primitive.stages[index + 1] if index + 1 < len(primitive.stages) else None
                    if not self._initial_payload_escape(stage, following, contacts, end,
                                                        primitive.object_id, snapshot):
                        raise SceneInvalid(f'{stage.name}: post-transition collision: {contacts[:3]}')
                    pending_escape = contacts
                self.last_evidence.append(dict(stage=stage.name, **evidence,
                    reused_previous_post_state=reuse_post,
                    state_before=stage.state_before.as_dict(), state_after=stage.state_after.as_dict(),
                    post_transition_contacts=contacts,
                    duplicate_world_proxy=self.last_duplicate_exclusion))
                current, previous = end, stage.state_after
            if pending_escape is not None:
                raise SceneInvalid('Unverified post-transition support contact')
        finally:
            # Restore even if attachment application failed partway through;
            # detach_object clears the backend's actual single attachment slot.
            self.backend.detach_object(primitive.object_id)
            if native:
                from .native_scene import CuroboScenePort
                CuroboScenePort(self.backend).apply_idle_snapshot(snapshot)
            else:
                self.backend.update_scene(scene)
                q0 = JointConfiguration(names, robot['positions'])
                self._apply(initial, q0, primitive.object_id, scene, 'empty')
        return PlanAudit(plan.payload_digest, snapshot['snapshot_id'], REQUIRED_AUDITS)

    @staticmethod
    def _same_contacts(expected, actual):
        def depths(rows):
            result = {}
            for row in rows:
                key = (row.get('collision_type'), row.get('robot_link'), row.get('world_object'))
                result[key] = max(result.get(key, 0.), float(row['penetration_m']))
            return result
        before, after = depths(expected), depths(actual)
        return before.keys() == after.keys() and all(abs(before[key] - after[key]) <= 1e-5 for key in before)

    def _initial_payload_escape(self, stage, following, contacts, q, object_id, snapshot):
        """Defer only the original table/payload overlap to its immediate lift.

        This is not a collision waiver: the next stage must re-observe the
        SAME overlap and pass the complete sampled escape audit before any
        PlanAudit can be returned. Robot-link closing collisions remain fatal.
        """
        from rm75_app.core.frames import MANISKILL_TABLE_COLLISION_NAME
        from rm75_app.pickplace.coordinator import _pose_matrix

        table = MANISKILL_TABLE_COLLISION_NAME
        limit = float(getattr(self.backend.config, 'retreat_start_contact_max_penetration_m', 0.))
        if (not 0 < limit <= .020 or stage.name != 'grasp' or stage.gripper_after is not True
                or stage.state_before.holding != 'empty' or stage.state_after.holding != object_id
                or following is None or following.name != 'lift'
                or following.allow_start_contact_escape is not True
                or following.gripper_after is not None or table in following.contact_objects
                or not snapshot['objects'].get(table, {}).get('fixed')
                or 'attached_object' not in self.backend.config.retreat_escape_contact_links):
            return False
        if any(row.get('collision_type') != 'world' or row.get('robot_link') != 'attached_object'
               or row.get('world_object') != table or not 0 < float(row['penetration_m']) <= limit
               for row in contacts):
            return False
        self._same_state(stage.state_after, following.state_before)
        # Switching to an attachment must not move the observed payload into
        # a surface. Its original fitted geometry stays at the measured pose.
        tcp = _pose_matrix(self.backend.tool_pose_for_configuration(q, 'gripper_tcp'))
        represented = tcp @ transform(stage.state_after.T_tcp_object)
        measured = snapshot['objects'][object_id]['measured']['T_world_object']
        position, rotation = pose_error(represented, measured)
        return position <= 1e-5 and rotation <= 1e-5

    @staticmethod
    def _same_state(expected, actual):
        if (expected.gripper_closed != actual.gripper_closed
                or expected.holding != actual.holding):
            raise SceneInvalid('Stage jaw/attachment chain differs from current state')
        if expected.gripper_positions != actual.gripper_positions:
            raise SceneInvalid('Stage measured jaw geometry changed')
        for key in ('T_tcp_object', 'released_object_pose'):
            a, b = getattr(expected, key), getattr(actual, key)
            if (a is None) != (b is None):
                raise SceneInvalid('Stage geometry transition is missing')
            if a is not None:
                p, r = pose_error(a, b)
                if p > 1e-6 or r > 1e-5:
                    raise SceneInvalid('Stage attachment/released geometry changed')

    def _apply(self, state, q, object_id, scene, active_object):
        from rm75_app.planning.contracts import Pose
        from rm75_app.pickplace.cached_scene import matrix_to_quaternion_wxyz

        if active_object != 'empty':
            self.backend.detach_object(active_object)
        self.backend.update_scene(scene)
        if state.released_object_pose is not None:
            matrix = transform(state.released_object_pose)
            self.backend.detach_object(object_id, Pose(
                matrix[:3, 3], matrix_to_quaternion_wxyz(matrix[:3, :3])))
            # Native detach(released_pose=...) updates the proxy, but does not
            # itself re-enable it. The released object MUST be an obstacle.
            self.backend.enable_object_collision(object_id)
        if state.holding != 'empty':
            self.backend.attach_object(state.holding, q)
            self.backend.update_attached_object_pose(state.holding, q, transform(state.T_tcp_object))
        if state.gripper_positions is not None:
            self.backend.set_measured_gripper_collision_state(dict(state.gripper_positions))
        else:
            self.backend.set_gripper_collision_state(state.gripper_closed)
        self._collision_holding = state.holding
        return state.holding

    def _verified_contact_ignores(self, ignored):
        """Exclude only a verified payload's duplicate world representation.

        The shared diagnostic intentionally enables each world obstacle to
        inspect it. A held object's world proxy must instead stay excluded
        while its actual GPU attached spheres participate in every query.
        Disabled unrelated obstacles are an error, not implicit permissions.
        """
        self.last_duplicate_exclusion = None
        ignored = set(ignored)
        holding = self._collision_holding
        if holding == 'empty':
            return ignored
        enabled = getattr(self.backend, '_obstacle_enabled', None)
        storage_reader = getattr(self.backend, 'read_attachment_collision_state', None)
        if not callable(enabled) or not callable(storage_reader):
            if self._native_audit:
                raise NotImplementedError('Native duplicate exclusion requires GPU attachment and enable readback')
            return ignored  # Explicit non-native stage-order fixtures only.
        flags = {obj.name: enabled(obj.name) for obj in self.backend._scene.objects}
        if holding not in flags or flags[holding] or any(not value for oid, value in flags.items() if oid != holding):
            raise SceneInvalid('Held proxy must be the only disabled world obstacle')
        if getattr(self.backend, '_attachment_fit_report', {}).get('object_name') != holding:
            raise SceneInvalid('Native attachment identity differs from audited holding')
        storage = storage_reader()
        owners = storage.get('owners', [])
        counts = [count for owner in owners for count in owner.get('active_counts', [])]
        if (storage.get('source') != 'native_GPU_attachment_link_spheres'
                or storage.get('consumers_consistent') is not True
                or not {'trajectory', 'planner', 'ik'} <= {owner.get('owner') for owner in owners}
                or any(not owner.get('active_counts') for owner in owners)
                or not counts or any(type(count) is not int or count <= 0 for count in counts)
                or len(set(counts)) != 1):
            raise SceneInvalid('Native GPU payload geometry is absent or inconsistent')
        ignored.add(holding)
        self.last_duplicate_exclusion = dict(object_id=holding, source=storage['source'],
            active_spheres=counts[0], consumers=[owner['owner'] for owner in owners],
            world_proxy_enabled=False, unrelated_obstacles_enabled=True)
        return ignored

    def _contacts(self, positions, names, ignored):
        backend = self.backend
        ignored = self._verified_contact_ignores(ignored)
        planner = backend._ensure_planner()
        modules = backend._import_modules()
        tensor = modules['torch'].as_tensor(positions, dtype=planner.device_cfg.dtype,
                                           device=planner.device_cfg.device)
        state = modules['JointState'].from_position(tensor, joint_names=list(names))
        return backend._collision_diagnostics_for_states(
            planner, {'path': state}, ignored_world_objects=ignored)

    def _check_path(self, stage, q):
        planner = self.backend._ensure_planner()
        names = tuple(stage.trajectory.joint_names)
        if tuple(planner.joint_names) != names:
            raise SceneInvalid('Native joint-limit order differs from trajectory')
        limits = planner.kinematics.get_joint_limits().position.detach().cpu().numpy()
        if limits.shape != (2, 7) or np.any(q < limits[0]) or np.any(q > limits[1]):
            raise SceneInvalid('Native stage exceeds actual model joint limits')
        pieces = []
        count = 1
        for a, b in zip(q[:-1], q[1:]):
            n = max(1, int(np.ceil(np.max(np.abs(b - a)) / self.step)))
            count += n
            if count > self.max_samples:
                raise SceneInvalid('Native collision audit sample budget exhausted')
            pieces.append(np.linspace(a, b, n + 1)[:-1])
        samples = np.concatenate((*pieces, q[-1:]), axis=0)
        ignored = set(stage.contact_objects)
        contacts = []
        for offset in range(0, len(samples), 64):
            for row in self._contacts(samples[offset:offset + 64], names, ignored):
                contacts.append({**row, 'candidate_index': int(row['candidate_index']) + offset})
        self.last_collision_evidence = dict(stage=stage.name, samples=len(samples),
            contact_count=len(contacts), contacts=contacts[:64],
            truncated=len(contacts) > 64,
            allowed_escape_links=list(self.backend.config.retreat_escape_contact_links),
            allowed_start_contact_escape=stage.allow_start_contact_escape,
            ignored_world_objects=sorted(ignored),
            source='native_collision_diagnostics_for_sampled_path',
            skill_verified=False)
        if not contacts:
            return dict(samples=len(samples), contacts=0, escape=False)
        if not stage.allow_start_contact_escape:
            raise SceneInvalid(f'{stage.name}: native path collision: {contacts[:3]}')
        allowed_links = set(self.backend.config.retreat_escape_contact_links)
        start = [row for row in contacts if row['candidate_index'] == 0]
        escape_objects = {row.get('world_object') for row in start
                          if row.get('collision_type') == 'world' and row.get('robot_link') in allowed_links}
        previous = {oid: 0. for oid in escape_objects}
        for row in start:
            oid = row.get('world_object')
            if oid in previous:
                previous[oid] = max(previous[oid], float(row['penetration_m']))
        limit = float(getattr(self.backend.config, 'retreat_start_contact_max_penetration_m', 0.))
        if not 0 < limit <= .020 or any(value > limit for value in previous.values()):
            raise SceneInvalid('Native escape exceeds original initial-contact limit')
        grouped = {}
        for row in contacts:
            grouped.setdefault(row['candidate_index'], []).append(row)
        for index in range(len(samples)):
            depth = {oid: 0. for oid in escape_objects}
            for row in grouped.get(index, ()):
                oid = row.get('world_object')
                if (row.get('collision_type') != 'world' or oid not in depth
                        or row.get('robot_link') not in allowed_links):
                    raise SceneInvalid('Native escape introduced a new collision/link')
                depth[oid] = max(depth[oid], float(row['penetration_m']))
            if any(depth[oid] > previous[oid] + .001 for oid in depth):
                raise SceneInvalid('Native escape penetration increased')
            previous = depth
        if any(value > .001 for value in previous.values()):
            raise SceneInvalid('Native escape did not clear initial contact')
        return dict(samples=len(samples), contacts=len(contacts), escape=True)
