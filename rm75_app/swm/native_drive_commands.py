"""Owned original controller writes shared with private prediction semantics."""
from __future__ import annotations
import numpy as np
from .native_bootstrap import _array
from .native_robot_mirror import ARM_JOINTS, GRIPPER_JOINTS, _native_drive_joints, validate_native_drive_state
from .scene import SceneInvalid


def target_vector(value, count):
    result = _array(value)
    if result.shape not in ((count,), (1, count)):
        raise SceneInvalid('Single-world native controller target shape required')
    result = result.reshape(-1).astype(np.float32)
    if not np.isfinite(result).all():
        raise SceneInvalid('Native-precision drive targets must be finite')
    return result


def current_targets(joints):
    values = []
    for joint in joints:
        value = _array(joint.drive_target).reshape(-1)
        if value.shape != (1,):
            raise SceneInvalid('Single-DOF native position target required')
        values.append(value[0])
    return np.asarray(values)


def needs_group_write(joints, requested):
    return not np.array_equal(current_targets(joints), target_vector(requested, len(joints)))


class NativeDriveCommands:
    """Wrap only owned PD target setters; original controller caches still update."""
    def __init__(self, primary, *, resources, emit):
        from mani_skill.agents.controllers.passive_controller import PassiveController
        controllers = primary.env.unwrapped.agent.controller.controllers
        if tuple(controllers) != ('arm', 'gripper', 'gripper_passive'):
            raise SceneInvalid('Original ordered three-controller inventory required')
        if not isinstance(controllers['gripper_passive'], PassiveController):
            raise SceneInvalid('Original passive gripper controller required')
        robot = primary.env.unwrapped.agent.robot
        if len(robot._objs) != 1:
            raise SceneInvalid('One owned primary articulation required')
        mapping = {}
        for name in ARM_JOINTS + GRIPPER_JOINTS:
            wrapper = robot.joints_map.get(name)
            if wrapper is None or len(wrapper._objs) != 1:
                raise SceneInvalid('Original canonical drive wrapper required')
            mapping[name] = wrapper._objs[0]
        mapping = _native_drive_joints(robot._objs[0], mapping)
        inventories = {name: tuple(joint.name for joint in controller.joints)
                       for name, controller in controllers.items()}
        flat = sum(inventories.values(), ())
        if (len(flat) != 13 or len(set(flat)) != 13 or set(flat) != set(mapping)
                or set(inventories['arm']) != set(ARM_JOINTS)):
            raise SceneInvalid('Original controller groups must partition native joints')
        for name in ('arm', 'gripper'):
            cfg = controllers[name].config
            if cfg.use_delta or cfg.use_target or cfg.interpolate:
                raise SceneInvalid('Original absolute noninterpolating PD control required')
        self.groups = tuple((name, inventories[name]) for name in ('arm', 'gripper'))
        self.passive = inventories['gripper_passive']
        self.stats = {name: dict(written=0, exact_equal_skipped=0) for name, _ in self.groups}
        self._restore = []
        self.closed = False
        self.emit = emit
        resources.callback(self.close)
        for name, names in self.groups:
            controller = controllers[name]
            original = controller.set_drive_targets
            handles = tuple(mapping[item] for item in names)
            def write(value, *, _original=original, _handles=handles, _name=name):
                primary.stop.check()
                if primary.closed or self.closed:
                    raise SceneInvalid('Owned primary drive policy is closed')
                requested = target_vector(value, len(_handles))
                if not needs_group_write(_handles, requested):
                    self.stats[_name]['exact_equal_skipped'] += 1
                    return
                _original(value)
                if not np.array_equal(current_targets(_handles), requested):
                    raise SceneInvalid('Original controller target write failed native readback')
                self.stats[_name]['written'] += 1
            controller.set_drive_targets = write
            self._restore.append((controller, original))

    def close(self):
        if self.closed:
            return
        for controller, original in reversed(self._restore):
            controller.set_drive_targets = original
        self._restore.clear()
        self.closed = True
        self.emit(kind='swm_native_drive_commands_released', groups=self.groups,
            passive_unchanged=list(self.passive), counts=self.stats,
            source='exact_native_precision_group_targets', hardware_qualified=False)


def apply_private_control(robot, physics_system, state, groups):
    """Position-group control only; full state/timestep transfer is initialization."""
    names, p, v, dt = validate_native_drive_state(state)
    joints = _native_drive_joints(robot)
    if float(physics_system.timestep) != float(np.float32(dt)):
        raise SceneInvalid('Private control timestep differs from initialized source')
    if tuple(name for name, _ in groups) != ('arm', 'gripper'):
        raise SceneInvalid('Original ordered PD groups required')
    controlled = sum((tuple(items) for _, items in groups), ())
    if (len(set(controlled)) != len(controlled) or not set(controlled) <= set(names)
            or set(groups[0][1]) != set(ARM_JOINTS) or not groups[1][1]):
        raise SceneInvalid('Private PD group identity mismatch')
    # Neither velocity targets nor passive targets may change during control.
    for index, name in enumerate(names):
        actual_v = _array(joints[name].drive_velocity_target).reshape(-1)
        if actual_v.shape != (1,) or actual_v[0] != np.float32(v[index]):
            raise SceneInvalid('Private velocity target differs from source contract')
        if name not in controlled and current_targets([joints[name]])[0] != np.float32(p[index]):
            raise SceneInvalid('Private passive target differs from source contract')
    counts = {}
    for group, items in groups:
        handles = tuple(joints[name] for name in items)
        requested = target_vector(np.asarray([p[names.index(name)] for name in items]), len(items))
        write = needs_group_write(handles, requested)
        if write:
            for joint, value in zip(handles, requested):
                joint.set_drive_target(np.asarray([value], dtype=np.float32))
            if not np.array_equal(current_targets(handles), requested):
                raise SceneInvalid('Private position group write failed native readback')
        counts[group] = dict(written=write, exact_equal_skipped=not write)
    return counts
