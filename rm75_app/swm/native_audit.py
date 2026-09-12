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

    def __call__(self, plan, snapshot):
        from rm75_app.planning.contracts import JointConfiguration

        self.last_evidence = []
        primitive = plan.payload
        if (not isinstance(primitive, NativePrimitive) or not primitive.stages
                or primitive.fingerprint() != plan.payload_digest):
            raise SceneInvalid('Stage audit requires an unchanged native primitive')
        if primitive.skill not in ('grasp', 'place'):
            raise SceneInvalid('Native contact auditor not installed for this skill')
        robot = snapshot['robot']
        measured_jaw = robot.get('gripper_positions')
        native = snapshot.get('observation_domain') in ('real', 'physics')
        if robot.get('idle') is not True or (measured_jaw is None and (
                native or type(robot.get('gripper_closed')) is not bool)):
            raise SceneInvalid('Independent idle jaw observation required for native audit')
        scene = self.backend._scene
        if (scene is None or scene.revision != snapshot['snapshot_id']
                or {obj.name for obj in scene.objects} != set(snapshot['objects'])):
            raise SceneInvalid('Native audit requires the complete current collision scene')
        holding = robot['holding']
        relative = None if holding == 'empty' else (
            np.linalg.inv(transform(robot['T_world_tcp'])) @ transform(
                snapshot['objects'][holding]['measured']['T_world_object']))
        initial = NativeStageState(None if measured_jaw is not None else robot['gripper_closed'],
                                   holding, relative, gripper_positions=measured_jaw)
        names = tuple(robot['joint_names'])
        current = JointConfiguration(names, robot['positions'])
        previous = initial
        active_object = holding
        try:
            for stage in primitive.stages:
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
                active_object = self._apply(stage.state_before, current, primitive.object_id,
                                            scene, active_object)
                evidence = self._check_path(stage, q)
                end = JointConfiguration(names, q[-1])
                active_object = self._apply(stage.state_after, end, primitive.object_id,
                                            scene, active_object)
                # Opening or closing changes collision geometry at the endpoint.
                # Audit that endpoint in the POST state too, not just the descent.
                contacts = self._contacts(q[-1:], names, set(stage.contact_objects))
                if contacts:
                    raise SceneInvalid(f'{stage.name}: post-transition collision: {contacts[:3]}')
                self.last_evidence.append(dict(stage=stage.name, **evidence,
                    state_before=stage.state_before.as_dict(), state_after=stage.state_after.as_dict(),
                    post_transition_contacts=contacts))
                current, previous = end, stage.state_after
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
        return state.holding

    def _contacts(self, positions, names, ignored):
        backend = self.backend
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
