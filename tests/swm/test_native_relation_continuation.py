"""Original tier graph survives vetoes, with one global motion budget."""
from dataclasses import replace
from types import SimpleNamespace

import pytest

from rm75_app.swm.native_skills import PickPlaceNativePhases
from rm75_app.swm.scene import SceneInvalid
from rm75_app.swm.skills import SkillRequest
from rm75_app.pickplace.coordinator import PickPlaceCoordinator
from .test_native_phases import PhasePlanner
from .test_original_relation_screen import make_task


def tiered_task(snapshot, budget=2):
    task = make_task(snapshot)
    high, low = task.grasp_candidates
    high = replace(high, metadata={'search_tier': 0})
    low = replace(low, metadata={'search_tier': 1})
    places = tuple(replace(p, metadata={**p.metadata, 'search_tier': i})
        for i, p in enumerate(task.place_candidates))
    return replace(task, grasp_candidates=(high, low), place_candidates=places,
        place_candidates_by_grasp={'high': (places[0],), 'low': (places[1],)},
        max_motion_candidates=budget)


def test_exclusion_resumes_original_later_tier_without_mutating_task(rig):
    snapshot = rig.sync.sync('initial')
    task = tiered_task(snapshot)
    coordinator = PickPlaceCoordinator(PhasePlanner(), SimpleNamespace())
    first = coordinator.screen_relations(task)
    assert [c.candidate_id for c in first.grasp_candidates] == ['high']
    second = coordinator.screen_relations(task, excluded_grasp_ids=('high',))
    assert [c.candidate_id for c in second.grasp_candidates] == ['low']
    assert [p.candidate_id for p in second.places_by_grasp['low']] == ['place_low']
    assert [c.candidate_id for c in task.grasp_candidates] == ['high', 'low']
    assert second.diagnostics['excluded_grasp_ids'] == ['high']
    with pytest.raises(ValueError, match='original task'):
        coordinator.screen_relations(task, excluded_grasp_ids=('foreign',))


def test_actual_phase_rescreens_after_closure_veto_preserving_pairing(rig):
    snapshot = rig.sync.sync('initial')
    task = tiered_task(snapshot)
    planner = PhasePlanner()
    calls = []
    events = []
    def veto(candidate, observed, configuration):
        calls.append(candidate.candidate_id)
        return candidate.candidate_id != 'high'
    phases = PickPlaceNativePhases(PickPlaceCoordinator(planner, SimpleNamespace()),
        lambda request, observed: task, None, None, closure_screen=veto,
        emit=lambda **row: events.append(row))
    result = phases.plan(SkillRequest('grasp', 'a'), snapshot)
    assert calls == ['high', 'low']
    assert [s.name for s in result.payload.stages] == ['approach', 'grasp', 'lift']
    assert any('place_low' in name for name in planner.calls)
    assert not any('place_high' in name for name in planner.calls)
    assert [e['ranked_ids'] for e in events] == [['high'], ['low']]
    assert [e['remaining_motion_budget'] for e in events] == [2, 1]
    assert planner.closed is False


def test_budget_is_global_across_relation_search_rounds(rig):
    snapshot = rig.sync.sync('initial')
    task = tiered_task(snapshot, budget=1)
    calls = []
    def veto(candidate, *args):
        calls.append(candidate.candidate_id)
        return False
    phases = PickPlaceNativePhases(PickPlaceCoordinator(PhasePlanner(), SimpleNamespace()),
        lambda request, observed: task, None, None, closure_screen=veto)
    with pytest.raises(RuntimeError, match='no feasible trajectory'):
        phases.plan(SkillRequest('grasp', 'a'), snapshot)
    assert calls == ['high']
    assert len(phases.relation_screen_history) == 1


def test_unavailable_prediction_does_not_resume_relations(rig):
    snapshot = rig.sync.sync('initial')
    task = tiered_task(snapshot)
    def unavailable(*args):
        raise SceneInvalid('native model unavailable')
    phases = PickPlaceNativePhases(PickPlaceCoordinator(PhasePlanner(), SimpleNamespace()),
        lambda request, observed: task, None, None, closure_screen=unavailable)
    with pytest.raises(SceneInvalid, match='native model unavailable'):
        phases.plan(SkillRequest('grasp', 'a'), snapshot)
    assert len(phases.relation_screen_history) == 1


def test_misbehaving_screen_cannot_repeat_excluded_candidate(rig):
    snapshot = rig.sync.sync('initial')
    task = tiered_task(snapshot)
    coordinator = PickPlaceCoordinator(PhasePlanner(), SimpleNamespace())
    original = coordinator.screen_relations
    coordinator.screen_relations = lambda task, **kwargs: original(task)
    phases = PickPlaceNativePhases(coordinator, lambda *args: task, None, None,
        closure_screen=lambda *args: False)
    with pytest.raises(SceneInvalid, match='repeated or foreign'):
        phases.plan(SkillRequest('grasp', 'a'), snapshot)
