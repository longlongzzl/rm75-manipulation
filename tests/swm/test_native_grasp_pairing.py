"""Per-grasp joint feasibility using original coordinator, fixture IK only."""
from types import SimpleNamespace
import pytest
from rm75_app.swm.native_skills import PickPlaceNativePhases
from rm75_app.swm.native_scene import planning_scene
from rm75_app.swm.skills import SkillRequest
from rm75_app.pickplace.coordinator import PickPlaceCoordinator, PickPlaceTask
from rm75_app.planning.contracts import JointConfiguration, Pose, PoseCandidate, CandidatePlan, BatchPlanningResult
from .test_native_phases import PhasePlanner
from .conftest import pose


@pytest.mark.parametrize('paired_feasible', [True, False])
def test_lookahead_uses_selected_grasp_pair_before_budget(rig, paired_feasible):
    class Planner(PhasePlanner):
        def plan_candidates(self, request):
            if not paired_feasible and any('paired_place' in c.candidate_id for c in request.candidates):
                self.calls.extend(c.candidate_id for c in request.candidates)
                return BatchPlanningResult(tuple(CandidatePlan(c.candidate_id, False, status='fixture_unreachable')
                    for c in request.candidates), backend='fixture_ik')
            return super().plan_candidates(request)
    planner = Planner()
    snap = rig.sync.sync('initial')
    selected = PoseCandidate('selected_grasp', Pose([.3,0,.03],[1,0,0,0]), score=2.)
    other = PoseCandidate('other_grasp', Pose([.31,0,.03],[1,0,0,0]), score=-100.)
    paired = PoseCandidate('paired_place', Pose([.4,0,.03],[1,0,0,0]), score=1.,
        metadata={'planning_target_object_pose':pose(.4,z=.03)})
    unrelated = PoseCandidate('unrelated_place', Pose([.5,0,.03],[1,0,0,0]), score=99.,
        metadata={'planning_target_object_pose':pose(.5,z=.03)})
    task = PickPlaceTask('a',JointConfiguration(snap['robot']['joint_names'],snap['robot']['positions']),
        (selected,other),(unrelated,paired),planning_scene(snap),max_motion_candidates=1,
        place_candidates_by_grasp={'selected_grasp':(paired,), 'other_grasp':(unrelated,)})
    phases = PickPlaceNativePhases(PickPlaceCoordinator(planner,SimpleNamespace()),
        lambda request,snapshot:task, None,None)
    if paired_feasible:
        result = phases.plan(SkillRequest('grasp','a'),snap)
        assert [stage.name for stage in result.payload.stages] == ['approach','grasp','lift']
    else:
        with pytest.raises(RuntimeError,match='no feasible trajectory'):
            phases.plan(SkillRequest('grasp','a'),snap)
    assert any('paired_place' in cid for cid in planner.calls)
    assert not any('unrelated_place' in cid for cid in planner.calls)
