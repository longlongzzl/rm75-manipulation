"""Separate native joint cache from verified all-link PhysX sleep kinematics.

Never sets q/qdot, sleeps a body, or changes thresholds. Raw cache is retained.
"""
from __future__ import annotations
import numpy as np
from .scene import SceneInvalid


def finite(value, shape):
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.isfinite(result).all():
        raise SceneInvalid('Complete finite native velocity evidence required')
    return result


class NativeArticulationVelocity:
    def __init__(self, robot):
        if len(robot._objs) != 1:
            raise SceneInvalid('One owned native articulation required')
        self.native = robot._objs[0]
        self.root = self.native.root
        self.links = tuple(self.native.get_links())
        from .native_robot_mirror import ARM_JOINTS, GRIPPER_JOINTS, _native_drive_joints
        mapping = {}
        for name in ARM_JOINTS + GRIPPER_JOINTS:
            wrapper = robot.joints_map.get(name)
            if wrapper is None or len(wrapper._objs) != 1:
                raise SceneInvalid('Complete canonical joint wrappers required')
            mapping[name] = wrapper._objs[0]
        self.mapping = _native_drive_joints(self.native, mapping)
        self.native_joints = tuple(self.native.get_active_joints())
        self.names = tuple(self.mapping)
        self.native_indices = {name: next(index for index, joint in enumerate(self.native_joints)
            if joint is handle) for name, handle in self.mapping.items()}
        if (len(self.links) != 20 or len({link.name for link in self.links}) != 20
                or len(self.names) != 13 or len(set(self.names)) != 13
                or self.root not in self.links or not self.root.is_root
                or any(link.articulation != self.native for link in self.links)):
            raise SceneInvalid('Complete original native articulation identity required')

    def _links(self):
        if self.native.root != self.root or tuple(self.native.get_links()) != self.links:
            raise SceneInvalid('Native articulation link identity changed')
        rows = []
        for link in self.links:
            sleeping = link.sleeping
            if type(sleeping) is not bool:
                raise SceneInvalid('Native sleeping state must be boolean')
            rows.append(dict(name=link.name, sleeping=sleeping,
                linear_velocity=finite(link.linear_velocity, (3,)).tolist(),
                angular_velocity=finite(link.angular_velocity, (3,)).tolist()))
        return rows

    def __call__(self, names, positions, velocities):
        names = tuple(names)
        if len(names) != 13 or len(set(names)) != 13 or set(names) != set(self.names):
            raise SceneInvalid('Native velocity joint identity mismatch')
        q = finite(positions, (13,))
        raw = finite(velocities, (13,))
        if tuple(self.native.get_active_joints()) != self.native_joints:
            raise SceneInvalid('Native active joint identity changed')
        indices = [self.native_indices[name] for name in names]
        native_q = finite(self.native.get_qpos(), (13,))[indices]
        native_v = finite(self.native.get_qvel(), (13,))[indices]
        if not np.array_equal(q, native_q) or not np.array_equal(raw, native_v):
            raise SceneInvalid('Native velocity evidence belongs to another state')
        before = self._links()
        after = self._links()
        if not np.array_equal(native_q, finite(self.native.get_qpos(), (13,))[indices]):
            raise SceneInvalid('Native joints moved during velocity observation')
        sleeping = all(row['sleeping'] for row in before + after)
        evidence = dict(source='native_joint_velocity_readback', joint_names=list(names),
            raw_native_velocities=raw.tolist(), all_links_sleeping_before_and_after=sleeping,
            joint_positions_unchanged=True, links_before=before, links_after=after,
            native_state_mutated=False)
        if sleeping:
            if any(value != 0. for row in before + after
                   for field in ('linear_velocity', 'angular_velocity') for value in row[field]):
                raise SceneInvalid('Sleeping native links have nonzero spatial velocity')
            evidence['source'] = 'native_PhysX_all_link_sleep_constraint'
            return np.zeros(13), evidence
        return raw.copy(), evidence
