"""Original axis construction reused without running a legacy episode."""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from rm75_app.pickplace.coordinator import PickPlaceCoordinator, _pose_matrix
from rm75_app.planning.contracts import Pose
from rm75_app.swm.native_skills import PickPlaceNativePhases
from rm75_app.swm.skills import SkillRequest
from .test_native_phases import PhasePlanner
from .test_original_relation_screen import make_task
from .conftest import pose


class AxisPlanner(PhasePlanner):
    def __init__(self, *, same_id=False, unchanged=False, unavailable=False):
        super().__init__()
        self.axis_calls = []
        self.same_id, self.unchanged, self.unavailable = same_id, unchanged, unavailable

    def resolve_axis_constrained_pose_candidates(self, candidates, scene, **kwargs):
        self.axis_calls.append((candidates, scene, kwargs))
        if self.unavailable:
            raise RuntimeError('axis model unavailable')
        source = candidates[0]
        position = source.pose.position.copy()
        if not self.unchanged:
            position[0] += .01
        return (replace(source,
            candidate_id=source.candidate_id if self.same_id else source.candidate_id+'__axis_solution_01',
            pose=Pose(position, source.pose.quaternion_wxyz),
            metadata={**source.metadata, 'source_grasp_candidate_id': source.candidate_id,
                      'axis_constrained_resolved': True}),)


def axis_task(snapshot, budget=2):
    original = make_task(snapshot)
    grasp = replace(original.grasp_candidates[0], metadata={'free_rotation_axis_local': 'y'})
    place = replace(original.place_candidates[0], metadata={
        **original.place_candidates[0].metadata, 'T_tcp_object': pose(), 'paired_grasp_id': grasp.candidate_id})
    return replace(original, grasp_candidates=(grasp,), place_candidates=(place,),
        place_candidates_by_grasp={grasp.candidate_id: (place,)}, max_motion_candidates=budget)


def phases_for(task, planner, screen, events):
    coordinator = PickPlaceCoordinator(planner, SimpleNamespace())
    coordinator.run = lambda *args: pytest.fail('Whole episode must never run in atomic fallback')
    phases = PickPlaceNativePhases(coordinator, lambda *args: task, None, None,
        closure_screen=screen, emit=lambda **row: events.append(row))
    return phases


def test_shared_builder_preserves_actual_relation_without_execution(rig):
    snapshot = rig.sync.sync('initial')
    task = axis_task(snapshot)
    planner = AxisPlanner()
    coordinator = PickPlaceCoordinator(planner, SimpleNamespace())
    coordinator.run = lambda *args: pytest.fail('Pure builder called episode')
    fallback, counts = coordinator.resolve_axis_fallback_task(task)
    assert counts == {'input_count': 1, 'resolved_count': 1}
    assert fallback.scene is task.scene and fallback.current is task.current
    assert fallback.enable_axis_fallback is False
    grasp = fallback.grasp_candidates[0]
    place = fallback.places_for_grasp(grasp.candidate_id)[0]
    assert place.metadata['paired_grasp_id'] == grasp.candidate_id
    assert np.allclose(_pose_matrix(grasp.pose) @ place.metadata['T_tcp_object'],
                       _pose_matrix(task.grasp_candidates[0].pose))
    assert planner.axis_calls[0][2]['ignore_object_names'] == ('a',)
    assert 'gripper_Left_Support_Link' in planner.axis_calls[0][2]['disable_collision_links']
    assert planner.calls == []


@pytest.mark.parametrize('same_id', [False, True])
def test_atomic_fallback_reaudits_changed_geometry_and_preserves_budget(rig, same_id):
    snapshot = rig.sync.sync('initial')
    task = axis_task(snapshot)
    planner = AxisPlanner(same_id=same_id)
    attempts, events = [], []
    def screen(candidate, observed, configuration):
        attempts.append(float(candidate.pose.position[0]))
        return candidate.metadata.get('axis_constrained_resolved', False)
    phases = phases_for(task, planner, screen, events)
    result = phases.plan(SkillRequest('grasp', 'a'), snapshot)
    assert attempts == pytest.approx([.3, .31])
    assert len(planner.axis_calls) == 1
    assert [stage.name for stage in result.payload.stages] == ['approach', 'grasp', 'lift']
    rounds = [e for e in events if e['kind'] == 'swm_native_relation_search']
    assert [e['remaining_motion_budget'] for e in rounds] == [2, 1, 1]
    assert rounds[-1]['search_mode'] == 'continuous_axis'
    assert planner.closed is False


@pytest.mark.parametrize('budget,accept_discrete', [(1, False), (2, True)])
def test_no_axis_after_budget_exhaustion_or_discrete_success(rig, budget, accept_discrete):
    snapshot = rig.sync.sync('initial')
    task = axis_task(snapshot, budget)
    planner = AxisPlanner()
    phases = phases_for(task, planner, lambda *args: accept_discrete, [])
    if accept_discrete:
        phases.plan(SkillRequest('grasp', 'a'), snapshot)
    else:
        with pytest.raises(RuntimeError, match='no feasible trajectory'):
            phases.plan(SkillRequest('grasp', 'a'), snapshot)
    assert not planner.axis_calls


def test_identical_axis_geometry_is_not_retried_under_new_id(rig):
    snapshot = rig.sync.sync('initial')
    planner = AxisPlanner(unchanged=True)
    calls, events = [], []
    def veto(candidate, *args):
        calls.append(candidate.candidate_id)
        return False
    phases = phases_for(axis_task(snapshot), planner, veto, events)
    with pytest.raises(RuntimeError, match='no feasible trajectory'):
        phases.plan(SkillRequest('grasp', 'a'), snapshot)
    assert calls == ['high']
    assert len(planner.axis_calls) == 1
    assert any(e['kind'] == 'swm_native_duplicate_candidate_geometry_skipped' for e in events)


def test_axis_model_failure_propagates_without_episode_or_repeated_fallback(rig):
    snapshot = rig.sync.sync('initial')
    planner = AxisPlanner(unavailable=True)
    phases = phases_for(axis_task(snapshot), planner, lambda *args: False, [])
    with pytest.raises(RuntimeError, match='axis model unavailable'):
        phases.plan(SkillRequest('grasp', 'a'), snapshot)
    assert len(planner.axis_calls) == 1
