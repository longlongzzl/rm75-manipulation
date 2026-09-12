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
        original_drives = [component for name in sorted(source)
            for component in source[name].entity.components
            if isinstance(component, sapien.physx.PhysxDriveComponent)]
        source_constraints = []
        recipes = []
        for drive in original_drives:
            matches = [recipe for recipe in primary.constraint_recipes.values() if recipe.drive is drive]
            if len(matches) != 1:
                raise SceneInvalid('Exactly one original construction recipe per robot constraint required')
            recipes.append(matches[0])
            source_constraints.append(constraint_state(drive, source))
        required = {(f'gripper_{side}_2_Link', f'gripper_{side}_Support_Link')
                    for side in ('Left', 'Right')}
        edges = {(row['parent'], row['child']) for row in source_constraints}
        optional = {('gripper_Left_Support_Link', 'gripper_Right_Support_Link')}
        if not required <= edges or not edges <= required | optional or len(edges) != len(recipes):
            raise SceneInvalid('Original complete gripper closure topology required')
        for recipe in recipes:
            self._constraints.append(recipe.build(self._scene, source, private))
        if len(agent.robot._objs) != 1:
            raise SceneInvalid('One original native articulation required')
        source_joints = {}
        for name in ARM_JOINTS + GRIPPER_JOINTS:
            wrapper = agent.robot.joints_map.get(name)
            if wrapper is None or len(wrapper._objs) != 1:
                raise SceneInvalid('One native source joint per canonical wrapper required')
            source_joints[name] = wrapper._objs[0]
        drives = align_articulation_drive_policy(agent.robot._objs[0], self._robot,
            source_joints=source_joints)
        bodies = align_link_body_policy(source, private)
        self._physics_state = dict(shapes=expected_shapes, constraints=source_constraints,
            articulation_drive=drives, link_bodies=bodies)
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
            constraints=[constraint_state(drive, links) for drive in self._constraints],
            articulation_drive=articulation_drive_policy(self._robot),
            link_bodies={name: link_body_policy(link) for name, link in links.items()})
        compare_native_state(self._physics_state, actual, 'robot_physics_policy')
        self.physics_acknowledgement = dict(source='native_robot_shape_and_constraint_readback',
            link_count=len(links), shape_count=sum(map(len, actual['shapes'].values())),
            constraint_count=len(self._constraints), constraints=actual['constraints'],
            expected_digest=digest(self._physics_state), actual_digest=digest(actual),
            collision_shapes_and_groups_aligned=True, gripper_constraint_configuration_aligned=True,
            articulation_drive_parameters_aligned=True,
            link_body_parameters_aligned=True, link_bodies=actual['link_bodies'],
            articulation_drive=actual['articulation_drive'],
            drive_target_replay_qualified=False,
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


def articulation_drive_policy(robot, *, joint_map=None):
    """Read native static drive policy, not controller configs or command targets."""
    joints = _native_drive_joints(robot, joint_map)
    rows = {}
    for name, joint in joints.items():
        limits = _array(joint.limits)
        armature = _array(joint.armature).reshape(-1)
        values = {key: float(getattr(joint, key)) for key in
            ('stiffness', 'damping', 'force_limit', 'friction')}
        if (int(joint.dof) != 1 or joint.type not in ('revolute', 'revolute_unwrapped')
                or joint.drive_mode not in ('force', 'acceleration')
                or limits.shape != (1, 2) or not np.isfinite(limits).all()
                or limits[0, 0] > limits[0, 1]
                or armature.shape != (1,) or not np.isfinite(armature).all()
                or np.any(armature < 0)
                or any(not np.isfinite(value) or value < 0 for value in values.values())):
            raise SceneInvalid('Invalid native joint drive policy: ' + joint.name)
        rows[name] = dict(type=joint.type, dof=1, limits=limits.tolist(),
            armature=armature.tolist(), drive_mode=joint.drive_mode, **values)
    position = robot.solver_position_iterations
    velocity = robot.solver_velocity_iterations
    sleep = float(robot.sleep_threshold)
    if (type(position) is not int or not 1 <= position <= 255
            or type(velocity) is not int or not 0 <= velocity <= 255
            or not np.isfinite(sleep) or sleep < 0):
        raise SceneInvalid('Invalid native articulation solver policy')
    return dict(joints=rows, solver_position_iterations=position,
        solver_velocity_iterations=velocity, sleep_threshold=sleep)


def align_articulation_drive_policy(source, private, *, source_joints=None):
    """Copy to an independent mirror only; never change source q/qdot or drives.

    This does not copy body inertia/gravity policy, controller state, target
    timelines, or simulation stepping, and cannot qualify dynamics replay.
    """
    from .native_body_mirror import compare_native_state

    if source is private:
        raise SceneInvalid('Drive policy destination must be an independent articulation')
    expected = articulation_drive_policy(source, joint_map=source_joints)
    before = articulation_drive_policy(private)
    source_joints = _native_drive_joints(source, source_joints)
    joints = {joint.name: joint for joint in private.get_active_joints()}
    for name, joint in joints.items():
        if joint is source_joints[name]:
            raise SceneInvalid('Drive policy destination aliases a primary joint')
        identity = ('type', 'dof', 'limits')
        compare_native_state({key: expected['joints'][name][key] for key in identity},
            {key: before['joints'][name][key] for key in identity}, name + '.drive_identity')
    for key in ('solver_position_iterations', 'solver_velocity_iterations', 'sleep_threshold'):
        getattr(private, 'set_' + key)(expected[key])
    for name, joint in joints.items():
        row = expected['joints'][name]
        joint.set_drive_properties(row['stiffness'], row['damping'],
            row['force_limit'], row['drive_mode'])
        joint.set_friction(row['friction'])
        joint.set_armature(np.asarray(row['armature'], dtype=np.float32))
    compare_native_state(expected, articulation_drive_policy(private), 'articulation_drive_policy')
    return expected


def _native_drive_joints(robot, joint_map=None):
    """Canonical names must resolve bijectively to this articulation's handles.

    ManiSkill namespaces native joint names. Its trusted wrapper map supplies
    canonical identities; suffix stripping or position-based matching cannot.
    """
    native = list(robot.get_active_joints())
    joints = ({joint.name: joint for joint in native}
        if joint_map is None else dict(joint_map))
    required = set(ARM_JOINTS + GRIPPER_JOINTS)
    if (len(native) != 13 or len({id(joint) for joint in native}) != 13
            or set(joints) != required or len({id(joint) for joint in joints.values()}) != 13
            or any(not any(joint is member for member in native) for joint in joints.values())):
        raise SceneInvalid('Complete independent thirteen-joint drive inventory required')
    return joints


def link_body_policy(link):
    """Read native link-local inertial, damping and gravity properties."""
    from .native_body_mirror import pose_matrix
    values = {key: float(getattr(link, key)) for key in
        ('mass', 'linear_damping', 'angular_damping')}
    inertia = _array(link.inertia)
    cmass = transform(pose_matrix(link.cmass_local_pose))
    if (any(not np.isfinite(value) or value < 0 for value in values.values())
            or inertia.shape != (3,) or not np.isfinite(inertia).all()
            or np.any(inertia < 0)):
        raise SceneInvalid('Invalid native link inertial policy')
    return dict(**values, inertia=inertia.tolist(), T_link_cmass=cmass.tolist(),
        auto_compute_mass=bool(link.auto_compute_mass), disable_gravity=bool(link.disable_gravity))


def align_link_body_policy(source, private):
    """Align independent link bodies without changing automatic mass semantics.

    The enclosing mirror validates the full URDF link inventory and shapes.
    Automatic mass must already match after shape synchronization. Assigning
    manual inertia would silently disable that original native policy.
    """
    from .native_body_mirror import compare_native_state, native_pose
    if (not source or set(source) != set(private)
            or len({id(link) for link in source.values()}) != len(source)
            or len({id(link) for link in private.values()}) != len(private)
            or any(dst is src for dst in private.values() for src in source.values())):
        raise SceneInvalid('Independent complete link-body inventory required')
    expected = {name: link_body_policy(link) for name, link in source.items()}
    before = {name: link_body_policy(link) for name, link in private.items()}
    inertial = ('mass', 'inertia', 'T_link_cmass', 'auto_compute_mass')
    for name, row in expected.items():
        if row['auto_compute_mass']:
            compare_native_state({key: row[key] for key in inertial},
                {key: before[name][key] for key in inertial}, name + '.automatic_mass')
    for name, row in expected.items():
        link = private[name]
        if not row['auto_compute_mass']:
            link.mass = row['mass']
            link.inertia = np.asarray(row['inertia'], dtype=np.float32)
            link.cmass_local_pose = native_pose(row['T_link_cmass'])
        link.disable_gravity = row['disable_gravity']
        link.linear_damping = row['linear_damping']
        link.angular_damping = row['angular_damping']
    compare_native_state(expected,
        {name: link_body_policy(link) for name, link in private.items()}, 'link_body_policy')
    return expected
