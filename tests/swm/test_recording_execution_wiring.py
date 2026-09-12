"""Actual SWM component wiring with fixture sensors/sink, not physical evidence."""
from dataclasses import replace

import numpy as np
import pytest

from rm75_app.planning.contracts import JointTrajectory
from rm75_app.swm.adapters import NativeAtomicBackend, NativeAtomicBinding
from rm75_app.swm.feedback import MeasuredFeedbackRecorder, MeasuredRecordingHandle
from rm75_app.swm.native_skills import NativePrimitive, NativeStage, SharedPrimitiveExecutor
from rm75_app.swm.scene import ObservationUnavailable, SceneInvalid, digest
from rm75_app.swm.skills import AtomicSkillRuntime, ExecutionReceipt, PlannedSkill, PlanAudit, REQUIRED_AUDITS, SkillRequest
from .conftest import pose


def recorder(max_samples=32):
    return MeasuredFeedbackRecorder(domain='fixture', sensor_session='camera-session',
        calibration_id='extrinsics-v1', max_samples=max_samples)


def sample(stamp, tcp=None, stage='push'):
    return dict(source='measured_feedback', domain='fixture', sensor_session='camera-session',
        calibration_id='extrinsics-v1', captured_at=stamp, T_world_tcp=pose() if tcp is None else tcp,
        stage=stage)


CONTEXT = dict(support_id='table', initially_settled=True, settling_evidence='fixture independent rest', intervened=False)


@pytest.mark.parametrize('failure', [None, 'command', 'after_observation'])
def test_execution_handle_reaches_post_observation_and_is_always_released(rig, failure):
    record = recorder()
    q = np.zeros(7)
    measured_tcp = pose(.3, z=.2)
    receipts, transitions = [], []

    def capture(batch):
        for row in batch['objects']:
            row['source'] = 'fixture'
        batch['robot']['positions'] = q.tolist()
        batch['robot']['T_world_tcp'] = measured_tcp
        after = rig.source.boundaries[-1] == 'after_push'
        if after and failure == 'after_observation':
            raise ObservationUnavailable('fixture after frame unavailable')
        record.append(sample(batch['robot']['captured_at'], measured_tcp,
            'post_settle' if after else 'approach'))

    rig.source.mutate = capture

    class Sink:
        def execute_trajectory(self, stage, path):
            nonlocal measured_tcp
            # Real component must establish the handle BEFORE dispatch.
            assert len(record._handles) == 1
            if failure == 'command':
                raise RuntimeError('fixture unknown command result')
            q[:] = path.positions[-1]
            measured_tcp = pose(.35, z=.2)
            rig.clock.tick(.2)
            record.append(sample(rig.clock(), measured_tcp, stage))
            rig.source.poses['a'] = pose(.35, z=.03)

        def set_gripper(self, closed):
            raise AssertionError('push primitive must not send a gripper command')

    runner = SharedPrimitiveExecutor(Sink(), lambda: dict(source='measured_feedback',
        idle=True, captured_at=rig.clock(), positions=q.tolist(),
        joint_names=[f'joint_{i}' for i in range(1, 8)]), clock=rig.clock, stop=rig.stop,
        recorder=record, recording_context=lambda primitive: CONTEXT)

    def plan(request, snapshot):
        names = tuple(snapshot['robot']['joint_names'])
        path = JointTrajectory(names, np.stack((q.copy(), np.full(7, .01))), dt=.2)
        payload = NativePrimitive('push', 'a', (NativeStage('push', path),))
        return PlannedSkill(digest(request.as_dict()), snapshot['snapshot_id'],
            payload.fingerprint(), payload, pose(.35, z=.03), 'fixture phase planner')

    def execute(plan, audit):
        result = runner(plan, audit)
        assert isinstance(result.observations, MeasuredRecordingHandle)
        assert result.actual_action_id in record._actions  # Still waiting for AFTER exposure.
        receipts.append(result)
        return result

    def on_transition(request, receipt, *, initial_snapshot, final_snapshot):
        transitions.append(record.transition(request, receipt, initial_snapshot, final_snapshot))

    backend = NativeAtomicBackend({'push': NativeAtomicBinding(plan,
        lambda p, s: PlanAudit(p.payload_digest, s['snapshot_id'], REQUIRED_AUDITS),
        execute, 'fixture push segment')}, execution_domain='fixture')
    runtime = AtomicSkillRuntime(rig.sync, backend, clock=rig.clock, stop=rig.stop,
        transition_observer=on_transition)
    request = SkillRequest('push', 'a', pose(.35, z=.03))
    if failure:
        with pytest.raises((RuntimeError, ObservationUnavailable)):
            runtime.run(request)
        assert transitions == []
    else:
        assert runtime.run(request).skill_verified
        assert len(transitions) == 1
        data = transitions[0]
        assert data['action_id'] == receipts[0].actual_action_id
        assert data['actual_action']['time_s'][-1] == data['object_time_s'][-1]
        assert data['actual_action']['source'] == 'measured_feedback'
        assert not any(row['kind'] == 'swm_physics_fit_skipped' for row in rig.events)
    assert record._actions == {} and record._handles == {}
    assert len(record._rows) <= 1


def test_active_window_never_evicted_but_idle_storage_is_bounded():
    record = recorder(max_samples=4)
    for stamp in range(20):
        record.append(sample(stamp))
    assert len(record._rows) <= 4
    handle = record.begin_action('action', 19., **CONTEXT)
    stamp = 20
    while len(record._rows) < 4:
        record.append(sample(stamp))
        stamp += 1
    first = record._rows[0]['captured_at']
    with pytest.raises(SceneInvalid, match='cannot drop action start'):
        record.append(sample(stamp))
    assert record._rows[0]['captured_at'] == first
    handle.close()
    assert len(record._rows) == 1
    record.append(sample(stamp))


def test_sampling_and_exact_command_receipt_required_before_trace_binding():
    record = recorder()
    with pytest.raises(SceneInvalid, match='sampling must start'):
        record.begin_action('action', 1., **CONTEXT)
    record.append(sample(1.))
    handle = record.begin_action('action', 1., **CONTEXT)
    receipt = ExecutionReceipt(True, 1., 2., True, 'action')
    with pytest.raises(SceneInvalid, match='actual completed command'):
        record.finish_command(handle, replace(receipt, actual_action_id='another'))
    result = record.finish_command(handle, receipt)
    assert result.observations is handle
    with pytest.raises(SceneInvalid, match='already completed'):
        record.finish_command(handle, receipt)
    handle.close()
    with pytest.raises(SceneInvalid, match='Duplicate'):
        record.begin_action('action', 3., **CONTEXT)
    record.close()
    with pytest.raises(SceneInvalid, match='closed'):
        record.append(sample(3.))
