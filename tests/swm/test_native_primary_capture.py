"""Explicit primary doubles; these are not native simulation qualification."""
import copy
from types import SimpleNamespace

import numpy as np
import pytest

from rm75_app.swm import native_capture
from rm75_app.swm.native_capture import NativePrimaryCapture, PrimaryActorBinding, read_primary_actor
from rm75_app.swm.native_robot_mirror import ARM_JOINTS, GRIPPER_JOINTS
from rm75_app.swm.scene import ObservationUnavailable, SceneInvalid, SceneWorldModel, SyncPolicy


@pytest.fixture
def capture_rig(rig, monkeypatch):
    manifest = copy.deepcopy(rig.manifest)
    manifest['observation_domain'] = 'physics'
    world = SceneWorldModel(manifest)
    names = ARM_JOINTS + GRIPPER_JOINTS
    robot = SimpleNamespace(pose=SimpleNamespace(to_transformation_matrix=lambda: np.eye(4)), get_qpos=lambda: np.zeros(13),
        get_active_joints=lambda: [SimpleNamespace(get_name=lambda name=name: name) for name in names])
    actors = {oid: SimpleNamespace(pose=SimpleNamespace(to_transformation_matrix=lambda: np.eye(4)), linear_velocity=np.zeros(3),
        angular_velocity=np.zeros(3), px_body_type='dynamic') for oid in world._objects}
    primary = SimpleNamespace(actors=actors, closed=False, stop=rig.stop,
        demo=SimpleNamespace(robot=robot), sequence=0)
    primary.drive_state = dict(source='native_joint_drive_targets_readback',
        joint_names=list(names), position_targets=[0.]*13, velocity_targets=[0.]*13,
        physics_timestep_s=.01, simulation_frequency_hz=100., control_frequency_hz=20.)
    monkeypatch.setattr(native_capture, 'read_primary_drive_state',
        lambda primary: copy.deepcopy(primary.drive_state))
    def clock():
        return rig.clock.tick(.001)
    def observe(primary):
        primary.sequence += 1
        return dict(primary_sequence=primary.sequence, captured_at=clock(), idle=True, units='rad',
            native_drive_state=copy.deepcopy(primary.drive_state),
            joint_names=list(ARM_JOINTS), positions=[0.] * 7, velocities=[0.] * 7,
            gripper_positions={name: 0. for name in GRIPPER_JOINTS},
            gripper_velocities={name: 0. for name in GRIPPER_JOINTS},
            T_world_base=np.eye(4).tolist(), T_world_tcp=np.eye(4).tolist(),
            gripper_observed=True, holding='empty', holding_evidence='explicit fixture')
    monkeypatch.setattr(native_capture, 'observe_primary_robot', observe)
    bindings = {oid: PrimaryActorBinding(actor, manifest['assets']['asset']['mesh_sha256'], np.eye(4))
        for oid, actor in actors.items()}
    source = NativePrimaryCapture(primary, bindings, T_world_base=np.eye(4),
        calibration_id=manifest['calibration_id'], sensor_session='fixture-primary', clock=clock)
    return SimpleNamespace(source=source, world=world, primary=primary, clock=clock, bindings=bindings)


def test_two_new_batches_are_accepted_by_existing_swm_contract(capture_rig):
    r = capture_rig
    stamps, sequences = [], []
    for boundary in ('before_grasp', 'preexecute_grasp'):
        after = r.clock()
        batch = r.source.capture(tuple(r.bindings), after=after, boundary=boundary)
        prepared = r.world.prepare_checkpoint(batch, after=after, now=r.clock(), policy=SyncPolicy(), boundary=boundary)
        r.world.commit_checkpoint(prepared)
        stamps.append(batch['capture_started_at'])
        sequences.append(batch['robot']['primary_sequence'])
        assert batch['primary_world_mutated'] is False
        assert all(row['velocity_source'] == 'native_PhysX_rigid_body_velocity' for row in batch['objects'])
    assert stamps[1] > stamps[0]
    assert sequences == [1, 2]


def test_incomplete_inventory_and_moving_object_are_rejected(capture_rig):
    r = capture_rig
    with pytest.raises(ObservationUnavailable, match='Every registered'):
        r.source.capture(('a',), after=r.clock(), boundary='before_grasp')
    r.primary.actors['a'].linear_velocity = np.array([.002, 0., 0.])
    with pytest.raises(ObservationUnavailable, match='not settled'):
        r.source.capture(tuple(r.bindings), after=r.clock(), boundary='before_grasp')


def test_reused_feedback_sequence_is_rejected(capture_rig):
    r = capture_rig
    r.source.capture(tuple(r.bindings), after=r.clock(), boundary='before_grasp')
    r.primary.sequence = 0
    with pytest.raises(ObservationUnavailable, match='Repeated'):
        r.source.capture(tuple(r.bindings), after=r.clock(), boundary='preexecute_grasp')


def test_robot_drift_during_capture_is_rejected(capture_rig):
    r = capture_rig
    r.primary.demo.robot.get_qpos = lambda: np.full(13, .01)
    with pytest.raises(ObservationUnavailable, match='robot changed'):
        r.source.capture(tuple(r.bindings), after=r.clock(), boundary='before_grasp')


def test_velocity_is_transformed_to_registered_offset_origin():
    actor = SimpleNamespace(pose=SimpleNamespace(to_transformation_matrix=lambda: np.eye(4)), linear_velocity=np.zeros(3),
        angular_velocity=np.array([0., 0., .001]), px_body_type='dynamic')
    local = np.eye(4); local[0, 3] = .1
    base = np.eye(4); base[:2, :2] = [[0., -1.], [1., 0.]]
    binding = PrimaryActorBinding(actor, 'a' * 64, local)
    row = read_primary_actor(binding, base)
    np.testing.assert_allclose(row['linear_velocity'], [-.0001, 0., 0.], atol=1e-12)
    np.testing.assert_allclose(row['T_world_object'], base @ local)


def test_drive_target_change_during_actor_capture_rejects_batch(capture_rig, monkeypatch):
    r = capture_rig
    original = native_capture.read_primary_actor
    def changing(*args):
        r.primary.drive_state['position_targets'][0] = .1
        return original(*args)
    monkeypatch.setattr(native_capture, 'read_primary_actor', changing)
    with pytest.raises(ObservationUnavailable, match='drive targets changed'):
        r.source.capture(tuple(r.bindings), after=r.clock(), boundary='before_grasp')


def test_settle_readback_preserves_full_inventory_and_measured_motion(capture_rig):
    r = capture_rig
    r.primary.actors['a'].angular_velocity = np.array([0., 0., .006])
    state = r.source.read_settle_state()
    assert set(state['objects']) == set(r.bindings)
    assert not state['idle'] and not state['objects']['a']['settled']
    assert state['objects']['a']['angular_velocity'] == [0., 0., .006]
    assert state['checkpoint_accepted'] is False
    assert state['primary_world_mutated'] is False
    assert r.primary.sequence == 0
    with pytest.raises(ObservationUnavailable, match='a at before_grasp'):
        r.source.capture(tuple(r.bindings), after=r.clock(), boundary='before_grasp')
    r.primary.actors['a'].angular_velocity[:] = 0.
    assert r.source.read_settle_state()['idle'] is True


def test_settle_readback_does_not_turn_nonfinite_feedback_into_wait(capture_rig):
    capture_rig.primary.actors['a'].linear_velocity = np.array([np.nan, 0., 0.])
    with pytest.raises(SceneInvalid, match='Nonfinite'):
        capture_rig.source.read_settle_state()
