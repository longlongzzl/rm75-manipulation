"""A bounded physical heuristic cannot replace exact endpoint auditing."""
from types import SimpleNamespace

import numpy as np
import pytest

from rm75_app.planning.contracts import JointConfiguration, Pose, PoseCandidate
from rm75_app.pickplace.coordinator import PickPlaceCoordinator
from rm75_app.swm import native_priority
from rm75_app.swm.native_closure import NativeClosureRejected
from rm75_app.swm.native_skills import PickPlaceNativePhases
from rm75_app.swm.skills import SkillRequest
from rm75_app.swm.scene import SceneInvalid
from .test_native_phases import PhasePlanner
from .test_original_relation_screen import make_task


@pytest.mark.parametrize('exact_failure', [False, True])
def test_priority_changes_selection_but_keeps_exact_endpoint_check(rig, exact_failure):
    snapshot = rig.sync.sync('initial')
    task = make_task(snapshot)  # Original total motion budget remains one.
    exact_calls = []
    def exact(candidate, observed, configuration):
        exact_calls.append(candidate.candidate_id)
        if exact_failure:
            raise SceneInvalid('exact endpoint rejected')
        return True
    phases = PickPlaceNativePhases(PickPlaceCoordinator(PhasePlanner(), SimpleNamespace()),
        lambda *args: task, None, None, closure_screen=exact,
        candidate_priority=lambda task, candidates, observed: {'high': 2, 'low': 0})
    if exact_failure:
        with pytest.raises(SceneInvalid, match='exact endpoint rejected'):
            phases.plan(SkillRequest('grasp', 'a'), snapshot)
    else:
        phases.plan(SkillRequest('grasp', 'a'), snapshot)
    assert exact_calls == ['low']
    assert phases.relation_screen_history[0]['total_motion_budget'] == 1


@pytest.mark.parametrize('priorities', [{'high': 0}, {'high': True, 'low': 0}, {'high': 0, 'low': 3}])
def test_incomplete_or_ambiguous_priority_does_not_reach_motion(rig, priorities):
    snapshot = rig.sync.sync('initial')
    planner = PhasePlanner()
    phases = PickPlaceNativePhases(PickPlaceCoordinator(planner, SimpleNamespace()),
        lambda *args: make_task(snapshot), None, None, candidate_priority=lambda *args: priorities)
    with pytest.raises(SceneInvalid, match='classifications'):
        phases.plan(SkillRequest('grasp', 'a'), snapshot)
    assert not planner.calls


def priority_fixture(tmp_path, monkeypatch, *, limit=2):
    names = tuple(f'joint_{i}' for i in range(1, 8))
    current = JointConfiguration(names, np.zeros(7))
    task = SimpleNamespace(scene=SimpleNamespace(revision='fresh'), current=current,
        tool_frame='gripper_tcp', object_name='a')
    candidates = tuple(PoseCandidate(name, Pose([.3, 0, .03], [1, 0, 0, 0])) for name in ('a', 'b', 'c'))
    monkeypatch.setattr(native_priority, '_cached_configuration', lambda *args: current)
    planner = SimpleNamespace(tool_pose_for_configuration=lambda *args: Pose([.304, 0, .03], [1, 0, 0, 0]))
    events = []
    predictor = native_priority.CachedClosurePriority(planner, None, None, None,
        directory=tmp_path, emit=lambda **row: events.append(row), max_predictions=limit)
    snapshot = {'snapshot_id': 'fresh', 'valid': True, 'observation_domain': 'physics'}
    return predictor, task, candidates, snapshot, events


def test_bounded_cache_retains_fk_uncertainty_and_never_authorizes(tmp_path, monkeypatch):
    priority, task, candidates, snapshot, events = priority_fixture(tmp_path, monkeypatch)
    calls = []
    monkeypatch.setattr(native_priority, 'reject_predicted_closure', lambda *args, **kwargs: calls.append(kwargs))
    assert priority(task, candidates, snapshot) == {'a': 0, 'b': 0, 'c': 1}
    assert len(calls) == 2
    assert priority(task, candidates, snapshot) == {'a': 0, 'b': 0, 'c': 1}
    assert len(calls) == 2
    assert all(event['fk_position_error_m'] == pytest.approx(.004) for event in events)
    assert all(event['execution_authorized'] is False and event['exact_endpoint_recheck_required'] for event in events)
    task.scene.revision = 'new'
    newer = {**snapshot, 'snapshot_id': 'new'}
    priority(task, candidates[:1], newer)
    assert len(calls) == 3


@pytest.mark.parametrize('failure,expected', [(NativeClosureRejected('contact'), 2),
    (SceneInvalid('missing model'), None), (RuntimeError('native error'), None)])
def test_only_physical_prediction_rejection_becomes_lower_priority(tmp_path, monkeypatch, failure, expected):
    priority, task, candidates, snapshot, events = priority_fixture(tmp_path, monkeypatch)
    def fail(*args, **kwargs):
        raise failure
    monkeypatch.setattr(native_priority, 'reject_predicted_closure', fail)
    if expected is None:
        with pytest.raises(type(failure), match=str(failure)):
            priority(task, candidates[:1], snapshot)
    else:
        assert priority(task, candidates[:1], snapshot) == {'a': expected}


def test_missing_original_cache_cannot_be_silently_prioritized(tmp_path, monkeypatch):
    priority, task, candidates, snapshot, events = priority_fixture(tmp_path, monkeypatch)
    monkeypatch.setattr(native_priority, '_cached_configuration', lambda *args: None)
    with pytest.raises(SceneInvalid, match='cached IK unavailable'):
        priority(task, candidates[:1], snapshot)
    assert not events
