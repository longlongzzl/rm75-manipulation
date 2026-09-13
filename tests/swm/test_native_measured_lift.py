"""Postclosure adapter contract fixtures, not native holding qualification."""
from types import SimpleNamespace
import numpy as np
import pytest
from rm75_app.planning.contracts import JointTrajectory
from rm75_app.swm.native_measured_lift import MeasuredLiftAudit
from rm75_app.swm.skills import PlanAudit, REQUIRED_AUDITS
from rm75_app.swm.scene import SceneInvalid


def setup():
    names = tuple(f'joint_{i}' for i in range(1, 8))
    snapshot = dict(valid=True, observation_domain='physics', snapshot_id='fresh',
        robot=dict(idle=True, holding='bi', joint_names=names, positions=[.001]*7,
            T_world_tcp=np.eye(4).tolist(), gripper_positions={
                f'gripper_{side}_{part}_Joint': .67 for side in ('Left','Right')
                for part in ('1','2','Support')}),
        objects={'bi': {'measured': {'T_world_object': np.eye(4).tolist()}}})
    calls = []
    class Auditor:
        last_evidence = [{'stage': 'lift', 'samples': 3}]
        def __call__(self, plan, observed):
            calls.append((plan, observed))
            return PlanAudit(plan.payload_digest, observed['snapshot_id'], REQUIRED_AUDITS)
    def sync(boundary, **kwargs):
        assert boundary == 'after_close_before_lift' and kwargs['after'] > 0
        return snapshot
    adapter = MeasuredLiftAudit(SimpleNamespace(sync=sync), Auditor(), target='bi',
        stop=SimpleNamespace(check=lambda: None), emit=lambda **row: None)
    path = JointTrajectory(names, np.array([[0.]*7, [.02]*7]), dt=.1)
    return adapter, path, snapshot, calls


def test_fresh_measured_jaw_and_start_reach_auditor_without_changing_endpoint():
    adapter, path, snapshot, calls = setup()
    result = adapter('lift', path)
    assert np.array_equal(result.positions[0], snapshot['robot']['positions'])
    assert np.array_equal(result.positions[-1], path.positions[-1])
    assert np.array_equal(path.positions[0], np.zeros(7))
    stage = calls[0][0].payload.stages[0]
    assert stage.state_before.gripper_positions == snapshot['robot']['gripper_positions']
    assert stage.state_before.holding == 'bi'
    assert stage.state_before.gripper_closed is None


@pytest.mark.parametrize('fault', ['empty', 'moving', 'missing_jaw', 'large_gap'])
def test_invalid_postclosure_state_never_reaches_audit(fault):
    adapter, path, snapshot, calls = setup()
    robot = snapshot['robot']
    if fault == 'empty': robot['holding'] = 'empty'
    if fault == 'moving': robot['idle'] = False
    if fault == 'missing_jaw': robot['gripper_positions'].pop(next(iter(robot['gripper_positions'])))
    if fault == 'large_gap': robot['positions'] = [.021]*7
    with pytest.raises(SceneInvalid): adapter('lift', path)
    assert not calls


def test_other_stage_and_incomplete_audit_do_not_authorize_path():
    adapter, path, snapshot, calls = setup()
    with pytest.raises(SceneInvalid): adapter('retreat', path)
    adapter.auditor = lambda plan, observed: PlanAudit(plan.payload_digest, 'stale', REQUIRED_AUDITS)
    with pytest.raises(SceneInvalid, match='current measured lift audit'):
        adapter('lift', path)
