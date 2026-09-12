"""Owned CPU PhysX robot mirror, expressed in the SWM base_link frame.

This class creates its own articulation. It cannot adopt the primary robot or
an executor, and never steps physics or sends a robot/gripper command.
"""
from __future__ import annotations

import hashlib
from pathlib import Path

import numpy as np

from .native_bootstrap import _array
from .scene import SceneInvalid, pose_error, transform


ARM_JOINTS = tuple(f'joint_{i}' for i in range(1, 8))
GRIPPER_JOINTS = tuple(
    f'gripper_{side}_{part}_Joint'
    for side in ('Left', 'Right') for part in ('1', '2', 'Support'))


def constraint_state(drive, links):
    """Read actual native constraint storage, not constructor arguments."""
    from .native_body_mirror import pose_matrix
    parent = [name for name, link in links.items() if drive.parent is link]
    child = [name for name, link in links.items() if drive.entity is link.entity]
    if len(parent) != 1 or len(child) != 1:
        raise SceneInvalid('Gripper constraint endpoint is outside the owned articulation')
    return dict(parent=parent[0], child=child[0],
        T_parent_constraint=pose_matrix(drive.pose_in_parent).tolist(),
        T_child_constraint=pose_matrix(drive.pose_in_child).tolist(),
        limits={axis: list(getattr(drive, 'get_limit_' + axis)())
                for axis in ('x', 'y', 'z', 'twist', 'cone', 'pyramid')},
        properties={axis: list(getattr(drive, 'get_drive_property_' + axis)())
                    for axis in ('x', 'y', 'z', 'twist', 'swing', 'slerp')},
        target=pose_matrix(drive.get_drive_target()).tolist(),
        velocity_target=[_array(value).tolist() for value in drive.get_drive_velocity_target()])


def measured_joint_vectors(observation, names):
    """Never synthesize jaw position or velocity from a holding boolean."""
    if observation.get('idle') is not True or observation.get('units') != 'rad':
        raise SceneInvalid('Measured idle robot state in radians is required')
    arm_names = observation.get('joint_names', [])
    if (len(arm_names) != 7 or set(arm_names) != set(ARM_JOINTS)
            or len(names) != 13 or set(names) != set(ARM_JOINTS + GRIPPER_JOINTS)):
        raise SceneInvalid('Complete native arm and six-joint gripper identity required')
    q = np.asarray(observation.get('positions'), dtype=float)
    v = np.asarray(observation.get('velocities'), dtype=float)
    jaw = observation.get('gripper_positions', {})
    jaw_v = observation.get('gripper_velocities', {})
    if q.shape != (7,) or v.shape != (7,) or set(jaw) != set(GRIPPER_JOINTS) or set(jaw_v) != set(GRIPPER_JOINTS):
        raise SceneInvalid('Independent measured q/qdot for all thirteen joints required')
    positions = dict(zip(arm_names, q)); positions.update(jaw)
    velocities = dict(zip(arm_names, v)); velocities.update(jaw_v)
    q = np.asarray([positions[name] for name in names], dtype=float)
    v = np.asarray([velocities[name] for name in names], dtype=float)
    if not np.isfinite(q).all() or not np.isfinite(v).all() or np.max(np.abs(v)) > .001:
        raise SceneInvalid('Nonfinite or moving native joint feedback')
    return q, v


class SapienRobotStatePort:
    """A private native articulation with explicit close and actual readback."""

    def __init__(self, urdf_path, *, resources):
        import sapien

        self._scene = None
        self._robot = None
        self.closed = False
        self.acknowledgement = None
        self.physics_acknowledgement = None
        self._physics_state = None
        self._constraints = []
        self.urdf_path = Path(urdf_path).expanduser().resolve(strict=True)
        self.urdf_sha256 = hashlib.sha256(self.urdf_path.read_bytes()).hexdigest()
        resources.callback(self.close)
        # Original URDF visual components require a passive render system.
        # No camera, hardware driver, controller, or executor is constructed.
        self._scene = sapien.Scene([
            sapien.physx.PhysxCpuSystem(), sapien.render.RenderSystem()])
        loader = self._scene.create_urdf_loader()
        loader.fix_root_link = True
        self._robot = loader.load(str(self.urdf_path))
        if self._robot is None:
            raise SceneInvalid('Private CPU PhysX robot URDF load failed')

    def synchronize_primary_physics(self, primary):
        """Initialize only the private robot from the original native policy."""
        import sapien
        from rm75_app.execution.maniskill_scene import create_rm75_gripper_constraints
        from .native_body_mirror import shape_state, compare_native_state, SHAPE_SCALARS

        if self.closed or self._physics_state is not None or self._constraints:
            raise SceneInvalid('Robot physics policy requires a fresh private articulation')
        agent = primary.env.unwrapped.agent
        source = {}
        for name, wrapper in agent.robot.links_map.items():
            if len(wrapper._objs) != 1:
                raise SceneInvalid('One primary native robot link per name is required')
            source[name] = wrapper._objs[0]
        private = {link.name: link for link in self._robot.get_links()}
        if set(source) != set(private) or any(source[n] is private[n] for n in source):
            raise SceneInvalid('Independent complete native robot link inventory required')
        expected_shapes = {}
        for name, link in source.items():
            expected = [shape_state(shape, np.eye(4)) for shape in link.collision_shapes]
            actual_shapes = private[name].collision_shapes
            if len(actual_shapes) != len(expected):
                raise SceneInvalid('Original robot collision shape count differs: ' + name)
            for shape, state in zip(actual_shapes, expected):
                actual = shape_state(shape, np.eye(4))
                for key in ('kind', 'geometry', 'T_object_shape'):
                    compare_native_state(state[key], actual[key], name + '.' + key)
                shape.physical_material = sapien.physx.PhysxMaterial(**state['material'])
                shape.set_collision_groups(state['collision_groups'])
                for key in SHAPE_SCALARS:
                    if getattr(shape, key) != state['properties'][key]:
                        setattr(shape, key, state['properties'][key])
            expected_shapes[name] = expected
        groups = (getattr(agent, '_rm75_planar_gripper_constraints', ()),
                  getattr(agent, '_rm75_pad_parallel_constraints', ()))
        if tuple(map(len, groups)) != (2, 1):
            raise SceneInvalid('Original two closures and parallel-pad constraint required')
        source_constraints = []
        for group in groups:
            for drive in group:
                if len(drive._objs) != 1:
                    raise SceneInvalid('One primary native component per gripper constraint required')
                source_constraints.append(constraint_state(drive._objs[0], source))
        closures, parallel = create_rm75_gripper_constraints(private, self._scene.create_drive)
        self._constraints = [*closures, *parallel]
        if (len(closures), len(parallel)) != (2, 1):
            raise SceneInvalid('Private gripper closure construction incomplete')
        self._physics_state = dict(shapes=expected_shapes, constraints=source_constraints)
        return self.read_physics_policy()

    def read_physics_policy(self):
        from .native_body_mirror import shape_state, compare_native_state
        from .scene import digest

        self.physics_acknowledgement = None
        if self.closed or self._physics_state is None:
            raise SceneInvalid('Native robot physics policy has not been initialized')
        links = {link.name: link for link in self._robot.get_links()}
        if any(link.entity.scene is not self._scene for link in links.values()):
            raise SceneInvalid('Native robot physics policy escaped the owned scene')
        actual = dict(shapes={name: [shape_state(shape, np.eye(4))
            for shape in link.collision_shapes] for name, link in links.items()},
            constraints=[constraint_state(drive, links) for drive in self._constraints])
        compare_native_state(self._physics_state, actual, 'robot_physics_policy')
        self.physics_acknowledgement = dict(source='native_robot_shape_and_constraint_readback',
            link_count=len(links), shape_count=sum(map(len, actual['shapes'].values())),
            constraint_count=len(self._constraints), constraints=actual['constraints'],
            expected_digest=digest(self._physics_state), actual_digest=digest(actual),
            collision_shapes_and_groups_aligned=True, gripper_constraint_configuration_aligned=True,
            dynamics_stepping_qualified=False, attachment_qualified=False, hardware_qualified=False)
        return self.physics_acknowledgement

    def apply_idle_state(self, observation):
        from rm75_app.execution.maniskill_scene import _pose_matrix

        self.acknowledgement = None
        if self.closed or self._robot is None:
            raise SceneInvalid('Private robot mirror is closed')
        robot = self._robot
        names = [joint.name for joint in robot.get_active_joints()]
        q, v = measured_joint_vectors(observation, names)
        expected_tcp = transform(observation['T_world_tcp'])
        expected_links = observation.get('gripper_links_in_base', {})
        from rm75_app.planning.gripper_collision import DYNAMIC_GRIPPER_LINKS
        required_links = set(DYNAMIC_GRIPPER_LINKS) | {'gripper_base_link'}
        if set(expected_links) != required_links:
            raise SceneInvalid('Complete measured gripper link geometry required')
        links = {link.name: link for link in robot.get_links()}
        if not required_links | {'gripper_tcp'} <= set(links):
            raise SceneInvalid('Private URDF link identity differs from measured robot')
        for matrix in expected_links.values():
            transform(matrix)
        root_pose = _pose_matrix(robot.get_root_pose())
        p, r = pose_error(root_pose, np.eye(4))
        if p > 1e-6 or r > 1e-3:
            raise SceneInvalid('Private robot world must be the SWM base_link frame')
        robot.set_qpos(q.astype(np.float32))
        robot.set_qvel(v.astype(np.float32))
        actual_q = _array(robot.get_qpos()).reshape(-1)
        actual_v = _array(robot.get_qvel()).reshape(-1)
        if (actual_q.shape != q.shape or actual_v.shape != v.shape
                or not np.allclose(actual_q, q, atol=1e-6, rtol=0)
                or not np.allclose(actual_v, v, atol=1e-6, rtol=0)):
            raise SceneInvalid('Private native joint q/qdot readback mismatch')
        errors = {}
        for name, expected in {**expected_links, 'gripper_tcp': expected_tcp}.items():
            actual = _pose_matrix(links[name].pose)
            p, r = pose_error(actual, expected)
            if p > .001 or r > .005:
                raise SceneInvalid(f'Private native link mismatch: {name}, {p} m, {r} rad')
            errors[name] = dict(position_error_m=p, rotation_error_rad=r)
        self.acknowledgement = dict(
            source='native_CPU_PhysX_articulation_readback',
            world_role='private_mirror', coordinate_frame='base_link',
            joint_names=names, positions=actual_q.tolist(), velocities=actual_v.tolist(),
            link_errors=errors, urdf_sha256=self.urdf_sha256,
            robot_state_aligned=True, primary_world_mutated=False,
            attachment_qualified=False, hardware_qualified=False)
        if self._physics_state is not None:
            self.acknowledgement['physics_policy'] = self.read_physics_policy()
        return self.acknowledgement

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.acknowledgement = None
        self.physics_acknowledgement = None
        self._physics_state = None
        self._constraints.clear()
        self._robot = None
        if self._scene is not None:
            self._scene.clear()
        self._scene = None
