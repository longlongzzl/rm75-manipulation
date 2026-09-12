"""Concrete idle scene ports and batch capture for the existing native services.

These objects receive already owned native instances. Constructing a port never
opens a camera, creates a tracker, connects an arm, or initializes a GPU.
"""
from __future__ import annotations

import copy
import hashlib
import threading
from pathlib import Path

import numpy as np

from .scene import ObservationUnavailable, SceneInvalid, digest, transform, pose_error


class SharedRRTrackCapture:
    """Advance the existing per-instance trackers with ONE newly acquired frame.

    ``trackers`` maps stable SWM ids to the shared RRTracker instances, while
    ``bindings`` records the exact mesh loaded into each instance. Frame capture
    and exposure-aligned robot/holding feedback remain owned by that service.
    An invisible fixed object requires a real tracker/anchor adapter too.
    """

    def __init__(self, trackers, bindings, acquire_frame, robot_at_exposure, *, sensor_session):
        if set(trackers) != set(bindings) or not trackers or not sensor_session:
            raise ValueError("Complete shared tracker/model bindings are required")
        self.trackers = dict(trackers)
        self.bindings = copy.deepcopy(bindings)
        self.acquire_frame = acquire_frame
        self.robot_at_exposure = robot_at_exposure
        self.sensor_session = sensor_session
        self._last_stamp = float('-inf')
        self._last_sequence = -1
        self._lock = threading.Lock()

    def __call__(self, ids, *, after, boundary):
        from rm75_app.scenarios.rrtrack_bridge import RRTrackInstanceSample

        if set(ids) != set(self.trackers) or len(ids) != len(self.trackers):
            raise ObservationUnavailable("Checkpoint must include every shared instance")
        with self._lock:
            for binding in self.bindings.values():
                if hashlib.sha256(Path(binding['mesh_path']).read_bytes()).hexdigest() != binding['mesh_sha256']:
                    raise ObservationUnavailable("Shared tracker mesh changed; reload localization")
            frame = self.acquire_frame(after=max(after, self._last_stamp), boundary=boundary)
            stamp = frame.timestamp_s
            if (stamp is None or not np.isfinite(stamp) or stamp <= max(after, self._last_stamp)
                    or frame.frame_index <= self._last_sequence):
                raise ObservationUnavailable("Shared acquisition returned a cached/reordered frame")
            # Consume the sequence even on inference failure. Never reuse its
            # image under a fresh timestamp after partial multi-object inference.
            self._last_stamp, self._last_sequence = float(stamp), frame.frame_index
            samples = []
            for oid in ids:
                output = self.trackers[oid].step(frame)
                if output.frame_index != frame.frame_index:
                    raise ObservationUnavailable("Tracker returned another image's estimate")
                samples.append(RRTrackInstanceSample(
                    oid, self.bindings[oid]['asset_name'], output, stamp))
            robot = self.robot_at_exposure(stamp)
            if robot.get('captured_at') != stamp:
                raise ObservationUnavailable("Robot feedback is not aligned to frame exposure")
            return dict(samples=samples, sensor_session=self.sensor_session, robot=robot)


def planning_scene(snapshot):
    from rm75_app.planning.contracts import CollisionObject, PlanningScene, Pose
    from rm75_app.pickplace.cached_scene import matrix_to_quaternion_wxyz

    objects = []
    for oid, obj in snapshot['objects'].items():
        asset = snapshot['assets'][obj['asset_id']]
        matrix = transform(obj['measured']['T_world_object'])
        pose = Pose(matrix[:3, 3], matrix_to_quaternion_wxyz(matrix[:3, :3]))
        if asset.get('collision_kind') == 'cuboid':
            shape = dict(kind='cuboid', dimensions=asset['collision_dimensions_m'])
        elif asset.get('collision_kind') == 'convex_mesh':
            shape = dict(kind='mesh', mesh_path=asset['collision_path'], scale=[1., 1., 1.])
        else:
            raise SceneInvalid("Native world requires a registered metric collision proxy")
        objects.append(CollisionObject(oid, pose=pose, **shape,
            metadata=dict(asset_id=obj['asset_id'], mesh_sha256=asset['mesh_sha256'])))
    return PlanningScene(tuple(objects), revision=snapshot['snapshot_id'])


class CuroboScenePort:
    """Apply full measured geometry and the measured attachment to shared cuRobo."""

    def __init__(self, backend):
        self.backend = backend
        self.holding = 'empty'
        self.applied_snapshot_id = None
        self.physics = {}

    def apply_idle_snapshot(self, snapshot):
        from rm75_app.planning.contracts import JointConfiguration

        self.applied_snapshot_id = None
        if snapshot['robot']['idle'] is not True:
            raise SceneInvalid("Cannot update a moving native scene")
        scene = planning_scene(snapshot)
        if self.holding != 'empty':
            self.backend.detach_object(self.holding)
            self.holding = 'empty'
        self.backend.update_scene(scene)
        # update_scene alone can merely cache a scene before native initialization.
        # Require the real planner to exist before acknowledging this transaction.
        self.backend._ensure_planner()
        robot = snapshot['robot']
        holding = robot['holding']
        if holding != 'empty':
            q = JointConfiguration(tuple(robot['joint_names']), robot['positions'])
            self.backend.set_gripper_collision_state(True)
            self.backend.attach_object(holding, q)
            self.holding = holding
            relative = np.linalg.inv(transform(robot['T_world_tcp'])) @ transform(
                snapshot['objects'][holding]['measured']['T_world_object'])
            self.backend.update_attached_object_pose(holding, q, relative)
        if self.backend._scene.revision != snapshot['snapshot_id']:
            raise SceneInvalid("cuRobo did not retain the complete checkpoint scene")
        self.physics = copy.deepcopy(snapshot['physics'])
        self.applied_snapshot_id = snapshot['snapshot_id']
        return self.applied_snapshot_id


class SapienScenePort:
    """Update an existing PRIVATE simulator mirror, never the observation world.

    The native mirror's attachment and material managers must apply changes and
    return their actual state. Missing managers fail closed. No robot qpos is
    assigned here, and no held object is teleported as a separate free actor.
    """

    def __init__(self, actors, *, asset_bindings, set_attachment, read_attachment,
                 apply_physics, read_physics, make_pose):
        self.actors = dict(actors)
        self.assets = dict(asset_bindings)
        self.set_attachment = set_attachment
        self.read_attachment = read_attachment
        self.apply_physics = apply_physics
        self.read_physics = read_physics
        self.make_pose = make_pose
        self.applied_snapshot_id = None

    def apply_idle_snapshot(self, snapshot):
        from rm75_app.execution.maniskill_scene import _pose_matrix

        self.applied_snapshot_id = None
        if snapshot['robot']['idle'] is not True or set(self.actors) != set(snapshot['objects']):
            raise SceneInvalid("Idle complete simulator actor coverage is required")
        robot = snapshot['robot']
        holding = robot['holding']
        attachment = None
        for oid, obj in snapshot['objects'].items():
            asset = snapshot['assets'][obj['asset_id']]
            if self.assets.get(oid) != asset['collision_sha256']:
                raise SceneInvalid("Simulator collision asset identity differs from SWM")
            if oid == holding:
                attachment = dict(object_id=oid, T_tcp_object=(
                    np.linalg.inv(transform(robot['T_world_tcp'])) @ transform(
                        obj['measured']['T_world_object'])).tolist())
        self.set_attachment(attachment)
        if digest(self.read_attachment()) != digest(attachment):
            raise SceneInvalid("Simulator did not apply the measured attachment")
        for oid, obj in snapshot['objects'].items():
            if oid == holding:
                continue
            actor = self.actors[oid]
            expected = transform(obj['measured']['T_world_object'])
            actor.set_pose(self.make_pose(expected))
            actual = _pose_matrix(actor.pose)
            if actual is None:
                raise SceneInvalid("Simulator pose readback is unavailable")
            p, r = pose_error(actual, expected)
            if p > 1e-6 or r > 1e-3:
                raise SceneInvalid("Simulator did not apply measured object pose")
        self.apply_physics(copy.deepcopy(snapshot['physics']))
        if digest(self.read_physics()) != digest(snapshot['physics']):
            raise SceneInvalid("Simulator did not apply the physical belief revision")
        self.applied_snapshot_id = snapshot['snapshot_id']
        return self.applied_snapshot_id
