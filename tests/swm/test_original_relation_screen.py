"""Original broad IK screening is shared without calling the episode runner."""
from types import SimpleNamespace
import numpy as np
import pytest
from rm75_app.swm.native_skills import PickPlaceNativePhases
from rm75_app.swm.native_scene import planning_scene
from rm75_app.swm.skills import SkillRequest
from rm75_app.pickplace.coordinator import PickPlaceCoordinator, PickPlaceTask
from rm75_app.planning.contracts import JointConfiguration, Pose, PoseCandidate
from .test_native_phases import PhasePlanner
from .conftest import pose


def make_task(snapshot):
    grasps=tuple(PoseCandidate(name,Pose([.3,0,.03],[1,0,0,0]),score=score)
        for name,score in [('high',10.),('low',1.)])
    places=tuple(PoseCandidate('place_'+g.candidate_id,Pose([.4,0,.03],[1,0,0,0]),
        metadata={'planning_target_object_pose':pose(.4,z=.03)}) for g in grasps)
    return PickPlaceTask('a',JointConfiguration(snapshot['robot']['joint_names'],snapshot['robot']['positions']),
        grasps,places,planning_scene(snapshot),max_motion_candidates=1,
        place_candidates_by_grasp={g.candidate_id:(p,) for g,p in zip(grasps,places)})


class ScreenPlanner(PhasePlanner):
    def __init__(self):
        super().__init__(); self.prepared=[]; self.measured=[]
    def prepare_pose_candidates(self,candidates,scene,**kwargs):
        self.prepared.extend(c.candidate_id for c in candidates)
    def feasible_pose_candidate_ids(self,candidates):
        return frozenset(c.candidate_id for c in candidates if 'low' in c.candidate_id)
    def set_measured_gripper_collision_state(self,positions):
        self.measured.append(dict(positions))


def test_native_phase_selects_complete_lower_score_relation_without_episode(rig):
    snap=rig.sync.sync('initial'); planner=ScreenPlanner()
    coordinator=PickPlaceCoordinator(planner,SimpleNamespace())
    coordinator.run=lambda *args: pytest.fail('Whole episode must not run')
    phases=PickPlaceNativePhases(coordinator,lambda request,snapshot:make_task(snapshot),None,None)
    planned=phases.plan(SkillRequest('grasp','a'),snap)
    assert planned.payload.skill=='grasp'
    assert 'high' in planner.prepared and 'low' in planner.prepared
    assert 'low' in planner.calls and 'high' not in planner.calls
    assert phases.last_relation_screen['complete_relation_count']==1


@pytest.mark.parametrize('fail', [False,True])
def test_screen_only_never_executes_and_restores_measured_jaw(rig,fail):
    snap=rig.sync.sync('initial'); planner=ScreenPlanner()
    jaw={'joint':.0123}
    if fail:
        def fail_prepare(*args,**kwargs): raise RuntimeError('IK probe failure')
        planner.prepare_pose_candidates=fail_prepare
    coordinator=PickPlaceCoordinator(planner,SimpleNamespace())
    if fail:
        with pytest.raises(RuntimeError,match='IK probe failure'):
            coordinator.screen_relations(make_task(snap),initial_gripper_positions=jaw)
    else:
        result=coordinator.screen_relations(make_task(snap),initial_gripper_positions=jaw)
        assert [c.candidate_id for c in result.grasp_candidates]==['low']
    assert planner.calls==[]
    assert planner.measured[-1]==jaw
