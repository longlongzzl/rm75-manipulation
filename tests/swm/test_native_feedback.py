"""Measured feedback and native scene adapter checks with explicit test doubles."""
from types import SimpleNamespace
import copy
import numpy as np
import pytest

from rm75_app.swm.feedback import MeasuredFeedbackRecorder
from rm75_app.swm.native_scene import SapienScenePort, CuroboScenePort
from rm75_app.swm.scene import SceneInvalid
from rm75_app.swm.skills import SkillRequest
from .test_measurement_storage import recording
from .conftest import pose


def test_recorder_waits_for_after_exposure_and_rejects_command_samples(rig):
    before, after, trace, receipt = recording(rig)
    recorder = MeasuredFeedbackRecorder(domain='fixture', sensor_session='camera-session', calibration_id='extrinsics-v1')
    rows = [dict(source='measured_feedback', domain='fixture', sensor_session='camera-session', calibration_id='extrinsics-v1',
                 captured_at=t, T_world_tcp=p, stage=s)
            for t, p, s in zip(trace['tool_captured_at'], trace['T_world_tcp'], trace['stages'])]
    recorder.append(rows[0])
    recorder.append(rows[1])
    recorder.bind_action(receipt, support_id='table', initially_settled=True, settling_evidence='fixture')
    request = SkillRequest('push', 'a', pose(.35, z=.03))
    with pytest.raises(ValueError, match='BOTH object frames'):
        recorder.transition(request, receipt, before, after)
    recorder.append(rows[2])
    transition = recorder.transition(request, receipt, before, after)
    assert transition['observation_mode'] == 'endpoint_only'
    assert transition['actual_action']['time_s'][-1] == transition['object_time_s'][-1]
    with pytest.raises(SceneInvalid, match='No recorder'):
        recorder.transition(request, receipt, before, after)
    with pytest.raises(ValueError, match='Commanded'):
        recorder.append({**rows[2], 'source': 'planned_commands'})


def test_shared_realman_callback_reads_actual_joint_feedback_without_sdk():
    from rm75_app.execution.realman_executor import RealManTrajectoryExecutor, RealManExecutionConfig
    from rm75_app.planning.contracts import JointTrajectory

    class FakeSession:
        config = SimpleNamespace(joint_names=tuple(f'joint_{i}' for i in range(1, 8)))
        stop_available = False
        def __init__(self):
            self.q = np.zeros(7)
        def read_joint_radians(self):
            return self.q.copy()
        def send_joint_follow(self, requested):
            # Deliberate tracking error distinguishes feedback from commands.
            self.q = np.asarray(requested) + .002

    session = FakeSession()
    rows = []
    executor = RealManTrajectoryExecutor(session, RealManExecutionConfig(pace_commands=False,
        terminal_hold_s=0., endpoint_timeout_s=0.), feedback_observer=rows.append)
    executor._armed = True  # Test double only, no SDK/preflight/device connection.
    path = JointTrajectory(session.config.joint_names, np.stack([np.zeros(7), np.full(7, .01)]), dt=.02)
    executor.execute_trajectory('push', path)
    assert len(rows) >= 3
    np.testing.assert_allclose(rows[-1]['positions'], np.full(7, .012))
    assert rows[-1]['source'] == 'measured_feedback'
    assert rows[-1]['stage'] == 'push'
    assert rows[-1]['captured_at'] >= rows[0]['captured_at']


@pytest.mark.parametrize('fail_pose', [False, True])
def test_simulator_port_applies_and_reads_back_every_free_actor(rig, fail_pose):
    snapshot = rig.sync.sync('before_grasp')
    class Actor:
        def __init__(self):
            self.pose = np.eye(4)
            self.calls = 0
        def set_pose(self, value):
            self.calls += 1
            if not fail_pose:
                self.pose = np.asarray(value)
    actors = {oid: Actor() for oid in snapshot['objects']}
    state = {'attachment': None, 'physics': {}}
    port = SapienScenePort(actors,
        asset_bindings={oid: rig.world.assets['asset']['collision_sha256'] for oid in actors},
        set_attachment=lambda value: state.update(attachment=copy.deepcopy(value)),
        read_attachment=lambda: state['attachment'],
        apply_physics=lambda value: state.update(physics=copy.deepcopy(value)),
        read_physics=lambda: state['physics'], make_pose=lambda value: value)
    if fail_pose:
        with pytest.raises(SceneInvalid, match='did not apply'):
            port.apply_idle_snapshot(snapshot)
        assert port.applied_snapshot_id is None
    else:
        assert port.apply_idle_snapshot(snapshot) == snapshot['snapshot_id']
        assert all(actor.calls == 1 for actor in actors.values())


def test_curobo_port_updates_full_world_and_measured_attachment(rig):
    rig.source.holding = 'a'
    snapshot = rig.sync.sync('after_grasp')
    calls = []
    class Backend:
        def update_scene(self, scene):
            self._scene = scene
            calls.append(('scene', {o.name for o in scene.objects}))
        def _ensure_planner(self):
            calls.append(('native', True))
        def set_gripper_collision_state(self, closed):
            pass
        def attach_object(self, oid, q):
            calls.append(('attach', oid))
        def update_attached_object_pose(self, oid, q, relative):
            calls.append(('relative', relative))
        def detach_object(self, oid):
            calls.append(('detach', oid))
    port = CuroboScenePort(Backend())
    assert port.apply_idle_snapshot(snapshot) == snapshot['snapshot_id']
    assert ('scene', {'a', 'b', 'table'}) in calls
    assert ('attach', 'a') in calls[:3]
    expected = np.linalg.inv(np.asarray(snapshot['robot']['T_world_tcp'])) @ np.asarray(
        snapshot['objects']['a']['measured']['T_world_object'])
    np.testing.assert_allclose(calls[-1][1], expected)
