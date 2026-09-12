"""Scene World Model: identity/geometry, measured state and physics belief.

No camera, robot or simulator is imported here. Planned poses are deliberately
not a write path for measured poses. Every live state change needs a complete,
fresh, calibrated checkpoint, and increments the scene revision.
"""
from __future__ import annotations
import copy
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import threading
from typing import Callable, Mapping
import numpy as np


def digest(value) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    ensure_ascii=False, allow_nan=False).encode()).hexdigest()


def transform(value):
    a = np.array(value, dtype=float, copy=True)
    if a.shape != (4, 4) or not np.isfinite(a).all():
        raise ValueError('A finite 4x4 SE(3) transform is required')
    if not np.allclose(a[3], [0, 0, 0, 1], atol=1e-7, rtol=0):
        raise ValueError('Invalid homogeneous row')
    r = a[:3, :3]
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-5, rtol=0) or abs(np.linalg.det(r)-1)>1e-5:
        raise ValueError('Rotation must be proper and orthonormal; no scale/shear/reflection')
    return a


def pose_error(a, b):
    a, b = transform(a), transform(b)
    # Native float32 readbacks are valid up to transform()'s strict rigid
    # tolerance, but R.T @ R can have a trace slightly below 3. acos(trace)
    # then reports motion even for identical measurements. Compare the polar
    # rotation factors only; never rewrite the authoritative measured matrices.
    rotations = []
    for matrix in (a, b):
        u, _, vh = np.linalg.svd(matrix[:3, :3])
        rotations.append(u @ vh)
    relative = rotations[0].T @ rotations[1]
    sine = .5 * np.linalg.norm([relative[2, 1] - relative[1, 2],
                               relative[0, 2] - relative[2, 0],
                               relative[1, 0] - relative[0, 1]])
    cosine = np.clip((np.trace(relative) - 1) / 2, -1, 1)
    return (float(np.linalg.norm(a[:3, 3]-b[:3, 3])),
            float(np.arctan2(sine, cosine)))


def positive(value, name, *, zero=False):
    if type(value) not in (float, int) or not math.isfinite(value) or value < 0 or (not zero and value == 0):
        raise ValueError(f'{name} must be finite and {"nonnegative" if zero else "positive"}')
    return float(value)


def identifier(value):
    import re
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,100}', value):
        raise ValueError('Stable path-safe instance/asset identity required')
    return value


class ObservationUnavailable(RuntimeError):
    pass


class SceneInvalid(RuntimeError):
    pass


@dataclass(frozen=True)
class SyncPolicy:
    max_age_s: float = 1.5
    max_batch_span_s: float = .30
    max_position_uncertainty_m: float = .003
    max_rotation_uncertainty_rad: float = .05
    drift_position_m: float = .003
    drift_rotation_rad: float = .04
    joint_drift_rad: float = .02

    def __post_init__(self):
        for key, value in vars(self).items():
            positive(value, key)


def _asset_file_roles(asset):
    role = asset.get('collision_role')
    if role not in (None, 'physical', 'planning_proxy'):
        raise ValueError('Unknown collision asset role')
    roles = ['mesh', 'collision']
    for name in ('native_physics_geometry', 'native_physics_baseline'):
        if role == 'planning_proxy' or name + '_path' in asset or name + '_sha256' in asset:
            roles.append(name)
    return tuple(roles)


class SceneWorldModel:
    """A single authoritative measured scene, not a second simulator.

    Geometry is already in metres in the EXACT object frame used by FP/collision
    and functionality poses. Model replacement requires a new asset identity and
    new observations; it may not silently inherit an old pose or density fit.
    """
    def __init__(self, manifest: Mapping):
        value = copy.deepcopy(dict(manifest))
        if value.get('schema') != 'rm75_swm_v1' or value.get('world_frame') != 'base_link':
            raise ValueError('Expected rm75_swm_v1 in the calibrated base_link frame')
        if value.get('observation_domain') not in ('real', 'physics', 'fixture'):
            raise ValueError('Observation domain must be explicit')
        self._lock = threading.RLock()
        self._objects = {}
        assets = value.get('assets', {})
        for aid, asset in assets.items():
            identifier(aid)
            if asset.get('units') != 'm' or asset.get('metric_scale_verified') is not True:
                raise ValueError('Model metric scale must be validated before scene registration')
            if asset.get('source') not in ('sam3d', 'legacy', 'cad'):
                raise ValueError('Mesh source provenance is required')
            file_roles = _asset_file_roles(asset)
            for key in tuple(role + suffix for role in file_roles for suffix in ('_path', '_sha256')) + ('scale_evidence',):
                if not isinstance(asset.get(key), str) or not asset[key]:
                    raise ValueError(f'Missing asset {key}')
            for key in tuple(role + '_sha256' for role in file_roles):
                if len(asset[key]) != 64 or any(c not in '0123456789abcdef' for c in asset[key]):
                    raise ValueError('Expected full asset SHA256')
            if 'volume_m3' in asset: positive(asset['volume_m3'], 'volume_m3')
            for function in asset.get('functional_poses', []):
                identifier(function['id'])
                if function['skill'] not in ('grasp','place','push','pull','rotate'):
                    raise ValueError('Unknown functional pose skill')
                transform(function['T_object_function'])
                if not function.get('provenance'): raise ValueError('Functional pose needs provenance')
        for obj in value.get('objects', []):
            oid = identifier(obj['id'])
            if oid in self._objects or obj['asset_id'] not in assets:
                raise ValueError('Duplicate instance or unknown mesh asset')
            if not isinstance(obj.get('name'), str) or not obj['name']:
                raise ValueError('Each instance needs a human-readable name')
            if type(obj.get('fixed', False)) is not bool: raise ValueError('fixed must be boolean')
            self._objects[oid] = dict(id=oid, name=obj['name'], asset_id=obj['asset_id'],
                                     fixed=obj.get('fixed',False), measured=None, lifecycle='unknown')
        if not self._objects: raise ValueError('A scene must contain registered instances')
        self._assets = copy.deepcopy(assets)
        if not isinstance(value.get('calibration_id'),str) or not value['calibration_id']:
            raise ValueError('Nonempty calibration identity is required')
        self.calibration_id = value['calibration_id']
        self.domain = value['observation_domain']
        self.frame = value['world_frame']
        self.revision = 0
        self.physics_revision = 0
        self._physics = {}
        self._robot = None
        self._sensor_session = None
        self.valid = False
        self.invalid_reason = 'not_observed'
        self.last_checkpoint = None

    @property
    def assets(self):
        return copy.deepcopy(self._assets)

    def check_assets(self):
        for asset in self.assets.values():
            for name in _asset_file_roles(asset):
                path = Path(asset[name+'_path']).expanduser()
                if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != asset[name+'_sha256']:
                    raise SceneInvalid(f'{name} asset missing or changed')

    def snapshot(self):
        with self._lock:
            value = dict(schema='rm75_swm_snapshot_v1', revision=self.revision,
                         physics_revision=self.physics_revision, world_frame=self.frame,
                         observation_domain=self.domain, calibration_id=self.calibration_id,
                         sensor_session=self._sensor_session, valid=self.valid,
                         invalid_reason=self.invalid_reason, objects=self._objects,
                         assets=self.assets, robot=self._robot, physics=self._physics,
                         checkpoint=self.last_checkpoint)
            result = copy.deepcopy(value)
            result['snapshot_id'] = digest(value)
            return result

    def invalidate(self, reason):
        with self._lock:
            self.valid = False
            self.invalid_reason = str(reason)
            self.revision += 1

    def begin_new_sensor_session(self):
        """Explicit no-motion reinitialization, never automatic replay acceptance."""
        with self._lock:
            self._sensor_session = None
            for obj in self._objects.values(): obj['measured'] = None
            self._robot = None
            self.invalidate('sensor_session_reinitialization_required')

    def prepare_checkpoint(self, batch, *, after, now, policy: SyncPolicy, boundary):
        """Validate ALL instances and robot feedback before committing any of them."""
        value = copy.deepcopy(batch)
        if (value.get('schema') != 'rm75_swm_observation_v1' or value.get('world_frame') != self.frame
                or value.get('calibration_id') != self.calibration_id or value.get('domain') != self.domain):
            raise ObservationUnavailable('Observation domain/frame/calibration mismatch')
        session = value.get('sensor_session')
        if not isinstance(session,str) or not session or (self._sensor_session and session != self._sensor_session):
            raise ObservationUnavailable('Sensor session changed; explicit reinitialization required')
        rows = value.get('objects', [])
        by_id = {row['id']: row for row in rows}
        if len(rows) != len(by_id) or set(by_id) != set(self._objects):
            raise ObservationUnavailable('Full registered scene observation is required, including obstacles')
        stamps = []
        for oid, obj in self._objects.items():
            row = by_id[oid]
            stamp = row.get('captured_at')
            if type(stamp) not in (float,int) or not math.isfinite(stamp) or not after < stamp <= now or now-stamp > policy.max_age_s:
                raise ObservationUnavailable(f'{oid}: stale, future or non-capture timestamp')
            seq = row.get('sequence')
            if type(seq) is not int or seq < 0: raise ObservationUnavailable('Invalid frame sequence')
            previous = obj['measured']
            if previous and (seq <= previous['sequence'] or stamp <= previous['captured_at']):
                raise ObservationUnavailable(f'{oid}: repeated/reordered observation')
            if row.get('accepted') is not True or row.get('tracking_state') in ('lost','recovering'):
                raise ObservationUnavailable(f'{oid}: pose missing, ambiguous or tracker lost')
            if self.domain == 'real' and row.get('source') not in ('foundationpose','rrtrack_foundationpose'):
                raise ObservationUnavailable('Real checkpoint requires the shared FoundationPose observation path')
            if row.get('mesh_sha256') != self.assets[obj['asset_id']]['mesh_sha256']:
                raise ObservationUnavailable('Pose belongs to a different object model')
            row['T_world_object'] = transform(row['T_world_object']).tolist()
            pu = positive(row['position_uncertainty_m'],'position_uncertainty_m',zero=True)
            ru = positive(row['rotation_uncertainty_rad'],'rotation_uncertainty_rad',zero=True)
            if pu > policy.max_position_uncertainty_m or ru > policy.max_rotation_uncertainty_rad:
                raise ObservationUnavailable('Observation uncertainty exceeds the task budget')
            stamps.append(stamp)
        robot = value.get('robot', {})
        q = np.asarray(robot.get('positions'), dtype=float)
        names = robot.get('joint_names', [])
        if (len(names) != 7 or len(set(names)) != 7 or q.shape != (7,) or not np.isfinite(q).all()
                or robot.get('units') != 'rad' or robot.get('idle') is not True):
            raise ObservationUnavailable('Fresh idle robot feedback in radians is required')
        if self._robot and names != self._robot['joint_names']:
            raise ObservationUnavailable('Robot joint order changed')
        stamp = robot.get('captured_at')
        if type(stamp) not in (int,float) or not math.isfinite(stamp) or not after < stamp <= now or now-stamp > policy.max_age_s:
            raise ObservationUnavailable('Stale robot feedback')
        stamps.append(stamp)
        if max(stamps)-min(stamps) > policy.max_batch_span_s:
            raise ObservationUnavailable('Checkpoint capture is not temporally coherent')
        robot['T_world_tcp'] = transform(robot['T_world_tcp']).tolist()
        if robot.get('gripper_observed') is not True or robot.get('holding') not in ('empty', *self._objects):
            raise ObservationUnavailable('Holding state must be independently observed, not inferred from a command')
        if not robot.get('holding_evidence'):
            raise ObservationUnavailable('Missing holding/release evidence')
        if robot['holding'] != 'empty' and self._objects[robot['holding']]['fixed']:
            raise ObservationUnavailable('A fixed fixture cannot be held')
        return dict(batch=value, rows=by_id, boundary=str(boundary), base_revision=self.revision)

    def commit_checkpoint(self, prepared):
        with self._lock:
            if prepared['base_revision'] != self.revision: raise SceneInvalid('Concurrent scene update')
            self._robot = prepared['batch']['robot']
            self._sensor_session = prepared['batch']['sensor_session']
            for oid, row in prepared['rows'].items():
                self._objects[oid]['measured'] = row
                self._objects[oid]['lifecycle'] = 'held' if self._robot['holding'] == oid else 'free'
            self.revision += 1
            self.valid = True; self.invalid_reason = None
            self.last_checkpoint = prepared['boundary']
            return self.snapshot()

    def update_physics(self, oid, posterior, *, expected_physics_revision):
        with self._lock:
            if oid not in self._objects or expected_physics_revision != self.physics_revision:
                raise SceneInvalid('Unknown object or stale physical belief update')
            obj = self._objects[oid]
            if posterior.get('mesh_sha256') != self.assets[obj['asset_id']]['mesh_sha256']:
                raise SceneInvalid('Physical belief belongs to another mesh/scale')
            if posterior.get('schema') != 'rm75_physics_posterior_v1': raise ValueError('Invalid belief')
            if posterior.get('domain') != self.domain or posterior.get('object_id') != oid:
                raise SceneInvalid('Physics evidence domain/instance mismatch')
            old = self._physics.get(oid)
            if old and old.get('transition_digest') == posterior.get('transition_digest'):
                raise SceneInvalid('Do not fit the same execution evidence twice')
            support = self._objects.get(posterior.get('support_id'))
            if not support or not support['fixed'] or posterior.get('support_mesh_sha256') != self.assets[support['asset_id']]['mesh_sha256']:
                raise SceneInvalid('Physics evidence belongs to different support geometry')
            self._physics[oid] = copy.deepcopy(posterior)
            self.physics_revision += 1
            self.revision += 1


class CheckpointSynchronizer:
    """Non-realtime scene synchronization at explicitly requested boundaries.

    simulator.replace_scene is an idle-only transaction on the whole snapshot.
    It must not teleport a held payload independently of its measured attachment.
    Failure latches the SWM invalid; a later full checkpoint may recover it.
    """
    def __init__(self, world, observation_port, simulator, *, clock, policy=None, emit=None, store=None):
        self.world = world; self.source = observation_port; self.simulator = simulator
        self.clock = clock; self.policy = policy or SyncPolicy(); self.emit = emit or (lambda **kw: None)
        self.store = store

    def sync(self, boundary, *, after=None):
        after = self.clock() if after is None else after
        with self.world._lock:
            try:
                self.world.check_assets()
                batch = self.source.capture(tuple(self.world._objects), after=after, boundary=boundary)
                prepared = self.world.prepare_checkpoint(batch, after=after, now=self.clock(),
                                                         policy=self.policy, boundary=boundary)
                # Supply a new immutable-by-copy candidate to the simulator first.
                candidate = self.world.snapshot()
                for oid, row in prepared['rows'].items():
                    candidate['objects'][oid]['measured'] = copy.deepcopy(row)
                    candidate['objects'][oid]['lifecycle'] = 'held' if batch['robot']['holding']==oid else 'free'
                candidate['robot'] = copy.deepcopy(batch['robot'])
                candidate['revision'] = self.world.revision+1
                candidate['valid'] = True; candidate['invalid_reason'] = None
                candidate['sensor_session'] = batch['sensor_session']; candidate['checkpoint'] = boundary
                candidate.pop('snapshot_id',None); candidate['snapshot_id'] = digest(candidate)
                acknowledgement = self.simulator.replace_scene(copy.deepcopy(candidate))
                if acknowledgement != candidate['snapshot_id']:
                    raise SceneInvalid('Simulator did not acknowledge the exact complete measured scene')
                actual = self.world.commit_checkpoint(prepared)
                if actual['snapshot_id'] != candidate['snapshot_id']:
                    raise SceneInvalid('Simulator/SWM transaction digests differ')
                if self.store is not None:self.store.save(self.world)
                self.emit(kind='swm_checkpoint', boundary=boundary, snapshot_id=actual['snapshot_id'],
                          revision=actual['revision'], observed_ids=list(prepared['rows']),
                          observation_domain=self.world.domain)
                return actual
            except BaseException as exc:
                self.world.invalidate(type(exc).__name__)
                raise


def moved_between(old, new, policy: SyncPolicy):
    if old['physics_revision'] != new['physics_revision'] or old['assets'] != new['assets']:
        return True
    if set(old['objects']) != set(new['objects']) or old['calibration_id'] != new['calibration_id']:
        return True
    if old['robot']['holding'] != new['robot']['holding']:
        return True
    for oid in old['objects']:
        a,b = old['objects'][oid]['measured'], new['objects'][oid]['measured']
        if not a or not b: return True
        pos, rot = pose_error(a['T_world_object'], b['T_world_object'])
        if pos > policy.drift_position_m or rot > policy.drift_rotation_rad: return True
    return float(np.max(np.abs(np.asarray(old['robot']['positions'])-new['robot']['positions']))) > policy.joint_drift_rad
