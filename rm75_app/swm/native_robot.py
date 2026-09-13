"""Independent primary robot/contact observations and shared-model readback.

No actuator commands. The original ManiSkill grasp predicate remains separate
from measured jaw geometry; simulation evidence never qualifies real hardware.
"""
from __future__ import annotations

import time

import numpy as np

from .native_bootstrap import _array
from .scene import ObservationUnavailable, SceneInvalid, pose_error, transform


def observe_primary_robot(primary):
    from rm75_app.execution.maniskill_task_bridge import _pose_matrix
    from rm75_app.planning.gripper_collision import DYNAMIC_GRIPPER_LINKS

    raw = primary.read_state()
    drive_state = read_primary_drive_state(primary)
    robot = primary.demo.robot
    agent = primary.env.unwrapped.agent
    names = tuple(raw['joint_names'])
    arm_names = tuple(f'joint_{index}' for index in range(1, 8))
    grip_names = tuple(f'gripper_{side}_{part}_Joint' for side in ('Left', 'Right')
                       for part in ('1', '2', 'Support'))
    if set(names) != set(arm_names + grip_names):
        raise ObservationUnavailable('Native robot identity differs from seven-arm/six-jaw contract')
    q, qdot = np.asarray(raw['positions']), np.asarray(raw['velocities'])
    if np.max(np.abs(qdot)) > .001:
        raise ObservationUnavailable('Native joint feedback is not idle')
    base = transform(_pose_matrix(robot))
    tcp = np.linalg.inv(base) @ transform(_pose_matrix(primary.demo.tcp))
    links = robot.links_map
    gripper_links = {name: (np.linalg.inv(base) @ transform(_pose_matrix(links[name]))).tolist()
                     for name in ('gripper_base_link', *DYNAMIC_GRIPPER_LINKS)}
    contacts, held = {}, []
    for oid, actor in primary.actors.items():
        left = _array(agent.scene.get_pairwise_contact_forces(agent.finger1_link, actor)).reshape(-1, 3)
        right = _array(agent.scene.get_pairwise_contact_forces(agent.finger2_link, actor)).reshape(-1, 3)
        result = _array(agent.is_grasping(actor, min_force=.5, max_angle=95)).reshape(-1)
        if left.shape != (1, 3) or right.shape != (1, 3) or result.shape != (1,):
            raise ObservationUnavailable('Native grasp feedback must refer to one primary world')
        accepted = bool(result[0])
        contacts[oid] = dict(left_force_n=left[0].tolist(), right_force_n=right[0].tolist(),
                             original_grasp_predicate=accepted)
        if accepted:
            held.append(oid)
    if len(held) > 1:
        raise ObservationUnavailable('Ambiguous multi-object native holding evidence')
    if not np.allclose(_array(robot.get_qpos()).reshape(-1), q, atol=1e-6, rtol=0):
        raise ObservationUnavailable('Robot moved during idle contact observation')
    if read_primary_drive_state(primary) != drive_state:
        raise ObservationUnavailable('Native drive targets changed during robot observation')
    finished = time.monotonic()
    return dict(source='native_joint_link_and_contact_feedback', domain='physics',
        captured_at=raw['capture_started_at'], capture_finished_at=finished,
        query_elapsed_s=finished - raw['capture_started_at'], primary_sequence=raw['sequence'],
        joint_names=list(arm_names), positions=[float(q[names.index(name)]) for name in arm_names],
        velocities=[float(qdot[names.index(name)]) for name in arm_names], units='rad', idle=True,
        T_world_base=base.tolist(), T_world_tcp=tcp.tolist(), tcp_coordinate_frame='base_link',
        gripper_positions={name: float(q[names.index(name)]) for name in grip_names},
        gripper_velocities={name: float(qdot[names.index(name)]) for name in grip_names},
        gripper_links_in_base=gripper_links, gripper_observed=True, native_drive_state=drive_state,
        native_velocity_evidence=raw.get("velocity_evidence"),
        holding=held[0] if held else 'empty', holding_evidence=dict(
            source='original_ManiSkill_is_grasping', min_force_n=.5, max_angle_deg=95, contacts=contacts),
        hardware_qualified=False)


def synchronize_robot_geometry(backend, observation):
    from rm75_app.pickplace.coordinator import _pose_matrix
    from rm75_app.planning.contracts import JointConfiguration
    from rm75_app.planning.gripper_collision import DYNAMIC_GRIPPER_LINKS, gripper_link_transforms

    if observation.get('domain') != 'physics' or observation.get('idle') is not True:
        raise SceneInvalid('Measured primary idle observation required')
    q = JointConfiguration(tuple(observation['joint_names']), observation['positions'])
    tcp = _pose_matrix(backend.tool_pose_for_configuration(q, 'gripper_tcp'))
    p, r = pose_error(tcp, observation['T_world_tcp'])
    if p > .001 or r > .005:
        raise SceneInvalid(f'Native/model TCP FK mismatch: translation={p:.9f} m rotation={r:.9f} rad')
    controller = backend._ensure_gripper_sphere_controller()
    expected_links = gripper_link_transforms(controller.urdf_path, observation['gripper_positions'])
    measured_links = observation['gripper_links_in_base']
    gripper_base = np.linalg.inv(transform(measured_links['gripper_base_link']))
    link_errors = {}
    for name in DYNAMIC_GRIPPER_LINKS:
        measured = gripper_base @ transform(measured_links[name])
        lp, lr = pose_error(measured, expected_links[name])
        if lp > .001 or lr > .005:
            raise SceneInvalid(f'Native/model gripper link mismatch: {name}: {lp:.9f} m {lr:.9f} rad')
        link_errors[name] = dict(position_error_m=lp, rotation_error_rad=lr)
    gripper = backend.set_measured_gripper_collision_state(observation['gripper_positions'])
    return dict(source='native_FK_and_GPU_gripper_geometry_readback',
        primary_sequence=observation['primary_sequence'], tcp_position_error_m=p,
        tcp_rotation_error_rad=r, gripper_link_errors=link_errors, gripper=gripper,
        robot_geometry_aligned=True, attachment_qualified=False, hardware_qualified=False)


def read_primary_drive_state(primary):
    """Native command targets, explicitly separate from measured q/qdot."""
    from .native_robot_mirror import (ARM_JOINTS, GRIPPER_JOINTS,
        _native_drive_joints, validate_native_drive_state)
    if primary.closed:
        raise ObservationUnavailable('Primary closed during drive target observation')
    primary.stop.check()
    env = primary.env.unwrapped
    robot = env.agent.robot
    if len(robot._objs) != 1:
        raise ObservationUnavailable('One source native articulation required')
    mapping = {}
    for name in ARM_JOINTS + GRIPPER_JOINTS:
        wrapper = robot.joints_map.get(name)
        if wrapper is None or len(wrapper._objs) != 1:
            raise ObservationUnavailable('Complete source drive target identity required')
        mapping[name] = wrapper._objs[0]
    joints = _native_drive_joints(robot._objs[0], mapping)
    positions, velocities = [], []
    for name in ARM_JOINTS + GRIPPER_JOINTS:
        primary.stop.check()
        p = _array(joints[name].drive_target).reshape(-1)
        v = _array(joints[name].drive_velocity_target).reshape(-1)
        if p.shape != (1,) or v.shape != (1,):
            raise ObservationUnavailable('Single-DOF native drive targets required')
        positions.append(float(p[0])); velocities.append(float(v[0]))
    state = dict(source='native_joint_drive_targets_readback',
        joint_names=list(ARM_JOINTS + GRIPPER_JOINTS), position_targets=positions,
        velocity_targets=velocities, physics_timestep_s=float(env.scene.px.timestep),
        simulation_frequency_hz=float(env.sim_freq), control_frequency_hz=float(env.control_freq))
    validate_native_drive_state(state)
    return state


def read_primary_tcp_feedback(primary):
    """Read moving simulation joints and TCP in one owned, unstepped interval.

    Native TCP link FK is read, not evaluated from requested command targets.
    Host read times are explicit; this is not a hardware clock calibration.
    """
    from rm75_app.execution.maniskill_task_bridge import _pose_matrix
    primary.stop.check()
    if primary.closed:
        raise ObservationUnavailable('Primary closed during measured TCP feedback')
    raw = primary.read_state()
    arm = tuple(f'joint_{i}' for i in range(1, 8))
    jaw = tuple(f'gripper_{side}_{part}_Joint' for side in ('Left','Right') for part in ('1','2','Support'))
    names = tuple(raw['joint_names'])
    q = np.asarray(raw['positions'], dtype=float)
    if (raw.get('domain') != 'physics' or raw.get('source') != 'native_actor_and_joint_readback'
            or len(names) != 13 or set(names) != set(arm+jaw)
            or q.shape != (13,) or not np.isfinite(q).all()):
        raise ObservationUnavailable('Complete actual primary joint feedback required')
    base = transform(_pose_matrix(primary.demo.robot))
    tcp = np.linalg.inv(base) @ transform(_pose_matrix(primary.demo.tcp))
    after_q = _array(primary.demo.robot.get_qpos()).reshape(-1)
    if after_q.shape != (13,) or not np.array_equal(after_q, q):
        raise ObservationUnavailable('Primary moved during measured TCP feedback')
    finished = time.monotonic()
    started = float(raw['capture_started_at'])
    if not np.isfinite([started, finished]).all() or finished < started:
        raise ObservationUnavailable('Invalid primary feedback query clock')
    return dict(source='measured_feedback', domain='physics',
        captured_at=started, query_completed_at=finished, query_elapsed_s=finished-started,
        clock_domain='host_monotonic_owned_simulation',
        timestamp_semantics='owned_simulation_read_interval_not_device_sample_clock',
        primary_sequence=raw['sequence'], joint_names=list(arm),
        positions=[float(q[names.index(name)]) for name in arm],
        gripper_positions={name: float(q[names.index(name)]) for name in jaw},
        T_world_tcp=tcp.tolist(), tcp_coordinate_frame='base_link', T_world_base=base.tolist(),
        tcp_source='original_native_TCP_link_pose_at_measured_joint_state',
        hardware_qualified=False, physical_time_axis_qualified=False)
