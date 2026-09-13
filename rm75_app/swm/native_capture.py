"""Fresh, read-only primary PhysX observations for SWM skill boundaries.

Binding construction belongs to the trusted worker: each row names an existing
primary actor and its registered model frame. No camera, executor or alternate
simulator is constructed here. Captures never step or set the primary world.
"""
from __future__ import annotations

from dataclasses import dataclass
import threading
import time

import numpy as np

from .native_bootstrap import _array
from .native_robot import observe_primary_robot, read_primary_drive_state
from .native_robot_mirror import ARM_JOINTS, GRIPPER_JOINTS, measured_joint_vectors
from .scene import ObservationUnavailable, pose_error, transform


@dataclass(frozen=True)
class PrimaryActorBinding:
    actor: object
    mesh_sha256: str
    T_actor_object: object

    def __post_init__(self):
        if (len(self.mesh_sha256) != 64
                or any(c not in '0123456789abcdef' for c in self.mesh_sha256)):
            raise ValueError('Registered model SHA256 is required')
        matrix = transform(self.T_actor_object)
        matrix.flags.writeable = False
        object.__setattr__(self, 'T_actor_object', matrix)


class PrimaryObjectNotSettled(ObservationUnavailable):
    """Finite measured motion, distinct from missing or corrupt feedback."""
    def __init__(self, observation):
        self.observation = observation
        super().__init__("Primary collision object is not settled at the checkpoint: "
            + repr(observation))


def read_primary_actor(binding, T_base_world):
    from rm75_app.execution.maniskill_task_bridge import _pose_matrix as actor_pose

    actor = binding.actor
    T_world_actor = transform(actor_pose(actor))
    if getattr(actor, 'px_body_type', None) == 'static':
        import sapien
        entities = getattr(actor, '_objs', ())
        if len(entities) != 1 or entities[0].find_component_by_type(sapien.physx.PhysxRigidStaticComponent) is None:
            raise ObservationUnavailable('Static velocity needs a native fixed-body constraint')
        linear = angular = np.zeros(3)
        velocity_source = 'native_PhysX_static_body_constraint'
    else:
        linear = _array(actor.linear_velocity).reshape(-1)
        angular = _array(actor.angular_velocity).reshape(-1)
        if linear.shape != (3,) or angular.shape != (3,):
            raise ObservationUnavailable('Single-world native object velocity required')
        velocity_source = 'native_PhysX_rigid_body_velocity'
    if not np.isfinite(linear).all() or not np.isfinite(angular).all():
        raise ObservationUnavailable('Nonfinite native object velocity')
    # Velocities refer to the registered model origin, not an offset table actor.
    offset_world = T_world_actor[:3, :3] @ binding.T_actor_object[:3, 3]
    linear = T_base_world[:3, :3] @ (linear + np.cross(angular, offset_world))
    angular = T_base_world[:3, :3] @ angular
    observation = dict(T_world_object=(T_base_world @ T_world_actor @ binding.T_actor_object).tolist(),
                linear_velocity=linear.tolist(), angular_velocity=angular.tolist(),
                velocity_source=velocity_source)
    if np.linalg.norm(linear) > .001 or np.linalg.norm(angular) > .005:
        raise PrimaryObjectNotSettled(observation)
    return observation


class NativePrimaryCapture:
    def __init__(self, primary, bindings, *, T_world_base, calibration_id,
                 sensor_session, clock=time.monotonic):
        if not bindings or not all(isinstance(b, PrimaryActorBinding) for b in bindings.values()):
            raise ValueError('Complete trusted primary actor/model bindings required')
        if not set(primary.actors) <= set(bindings):
            raise ValueError('Primary movable inventory cannot be omitted')
        for oid, actor in primary.actors.items():
            if bindings[oid].actor is not actor:
                raise ValueError('Registered actor is not the observed primary instance')
        if not calibration_id or not sensor_session:
            raise ValueError('Calibration and primary sensor-session identity required')
        self.primary = primary
        self.bindings = dict(bindings)
        self.T_world_base = transform(T_world_base)
        self.T_base_world = np.linalg.inv(self.T_world_base)
        self.calibration_id = calibration_id
        self.sensor_session = sensor_session
        self.clock = clock
        self._last_sequence = -1
        self._last_stamp = float('-inf')
        self._lock = threading.Lock()

    def read_settle_state(self):
        """Complete read-only motion sample, never an accepted SWM checkpoint."""
        with self._lock:
            if self.primary.closed:
                raise ObservationUnavailable('Observed primary world is closed')
            started = self.clock()
            rows = {}
            for oid, binding in self.bindings.items():
                self.primary.stop.check()
                try:
                    observed = read_primary_actor(binding, self.T_base_world)
                    settled = True
                except PrimaryObjectNotSettled as exc:
                    observed, settled = exc.observation, False
                rows[oid] = dict(settled=settled, **observed)
            return dict(source='native_registered_object_velocity_readback',
                world_frame='base_link', objects=rows,
                idle=all(row['settled'] for row in rows.values()),
                capture_started_at=started, capture_finished_at=self.clock(),
                checkpoint_accepted=False, primary_world_mutated=False)

    def capture(self, ids, *, after, boundary):
        from rm75_app.execution.maniskill_task_bridge import _pose_matrix as actor_pose

        if not getattr(self.primary, 'object_observations_allowed', True):
            raise ObservationUnavailable('No object capture inside the grasp-place execution window')
        if len(ids) != len(self.bindings) or set(ids) != set(self.bindings):
            raise ObservationUnavailable('Every registered primary collision object is required')
        with self._lock:
            if self.primary.closed:
                raise ObservationUnavailable('Observed primary world is closed')
            self.primary.stop.check()
            started = self.clock()
            if started <= max(after, self._last_stamp):
                raise ObservationUnavailable('New capture must start after the requested boundary')
            robot = observe_primary_robot(self.primary)
            sequence = robot['primary_sequence']
            stamp = robot['captured_at']
            if sequence <= self._last_sequence or stamp <= max(after, self._last_stamp):
                raise ObservationUnavailable('Repeated primary feedback cannot form a new capture')
            # Consume failed batches too: recovery must acquire another sample.
            self._last_sequence, self._last_stamp = sequence, stamp
            q, _ = measured_joint_vectors(robot, ARM_JOINTS + GRIPPER_JOINTS)
            p, r = pose_error(robot['T_world_base'], self.T_world_base)
            if p > 1e-6 or r > 1e-3:
                raise ObservationUnavailable('Primary base calibration changed')
            rows = []
            for oid in ids:
                row_stamp = self.clock()
                try:
                    observed = read_primary_actor(self.bindings[oid], self.T_base_world)
                except PrimaryObjectNotSettled as exc:
                    raise ObservationUnavailable(f"{oid} at {boundary}: {exc}") from exc
                rows.append(dict(id=oid, mesh_sha256=self.bindings[oid].mesh_sha256,
                    captured_at=row_stamp, sequence=sequence, accepted=True,
                    tracking_state='tracking', source='native_primary_PhysX_readback',
                    position_uncertainty_m=0., rotation_uncertainty_rad=0., **observed))
            for row in rows:
                current = read_primary_actor(self.bindings[row['id']], self.T_base_world)
                p, r = pose_error(current['T_world_object'], row['T_world_object'])
                if p > 1e-6 or r > 1e-3:
                    raise ObservationUnavailable('Primary object changed during capture')
            native_robot = self.primary.demo.robot
            names = [joint.get_name() for joint in native_robot.get_active_joints()]
            actual_q = _array(native_robot.get_qpos()).reshape(-1)
            expected = dict(zip(ARM_JOINTS + GRIPPER_JOINTS, q))
            if (len(names) != 13 or set(names) != set(expected) or actual_q.shape != (13,)
                    or not np.allclose(actual_q, [expected[name] for name in names], atol=1e-6, rtol=0)):
                raise ObservationUnavailable('Primary robot changed during capture')
            p, r = pose_error(actor_pose(native_robot), self.T_world_base)
            if p > 1e-6 or r > 1e-3:
                raise ObservationUnavailable('Primary base changed during capture')
            if read_primary_drive_state(self.primary) != robot.get('native_drive_state'):
                raise ObservationUnavailable('Native drive targets changed during capture')
            physical_clock = robot.get("simulation_clock")
            if physical_clock is not None:
                if self.primary.simulation_clock.read() != physical_clock:
                    raise ObservationUnavailable("Primary physical clock changed during complete capture")
                import copy
                for row in rows:
                    row["simulation_clock"] = copy.deepcopy(physical_clock)
            self.primary.stop.check()
            finished = self.clock()
            return dict(schema='rm75_swm_observation_v1', world_frame='base_link',
                calibration_id=self.calibration_id, sensor_session=self.sensor_session,
                domain='physics', objects=rows, robot=robot, boundary=str(boundary),
                capture_started_at=started, capture_finished_at=finished,
                primary_world_mutated=False, hardware_connected=False)
