"""Candidate rejection preserves original phase planning and failure boundaries."""
from types import SimpleNamespace

import numpy as np
import pytest

from rm75_app.swm import native_closure
from rm75_app.swm.native_skills import PickPlaceNativePhases
from rm75_app.swm.native_scene import planning_scene
from rm75_app.swm.scene import SceneInvalid
from rm75_app.swm.skills import SkillRequest
from rm75_app.pickplace.coordinator import PickPlaceCoordinator, PickPlaceTask
from rm75_app.planning.contracts import JointConfiguration, Pose, PoseCandidate
from .test_native_phases import PhasePlanner
from .conftest import pose


def test_rejected_grasp_continues_original_candidate_loop_without_execution(rig):
    planner = PhasePlanner()
    snapshot = rig.sync.sync('initial')
    first = PoseCandidate('first', Pose([.3, 0, .03], [1, 0, 0, 0]), score=2.)
    second = PoseCandidate('second', Pose([.31, 0, .03], [1, 0, 0, 0]), score=1.)
    place = PoseCandidate('place', Pose([.4, 0, .03], [1, 0, 0, 0]),
        metadata={'planning_target_object_pose': pose(.4, z=.03)})
    task = PickPlaceTask('a', JointConfiguration(snapshot['robot']['joint_names'],
        snapshot['robot']['positions']), (first, second), (place,), planning_scene(snapshot),
        max_motion_candidates=2, place_candidates_by_grasp={'first': (place,), 'second': (place,)})
    calls = []
    def screen(candidate, observed, configuration):
        calls.append((candidate.candidate_id, observed['snapshot_id']))
        assert configuration.positions[0] == pytest.approx(candidate.pose.position[0])
        return candidate.candidate_id == 'second'
    phases = PickPlaceNativePhases(PickPlaceCoordinator(planner, SimpleNamespace()),
        lambda request, observed: task, None, None, closure_screen=screen)
    result = phases.plan(SkillRequest('grasp', 'a'), snapshot)
    assert calls == [('first', snapshot['snapshot_id']), ('second', snapshot['snapshot_id'])]
    assert result.payload.stages[1].trajectory.positions[-1, 0] == pytest.approx(.31)
    assert [s.name for s in result.payload.stages] == ['approach', 'grasp', 'lift']
    assert planner.closed is False


@pytest.mark.parametrize('failure,expected', [(native_closure.NativeClosureRejected('contact'), False),
    (SceneInvalid('missing native identity'), None), (RuntimeError('model unavailable'), None)])
def test_only_explicit_collision_is_candidate_infeasibility(tmp_path, monkeypatch, failure, expected):
    def reject(*args, **kwargs):
        raise failure
    monkeypatch.setattr(native_closure, 'reject_predicted_closure', reject)
    configuration = JointConfiguration(tuple(f'joint_{i}' for i in range(1, 8)), np.zeros(7))
    args = (None, None, None, SimpleNamespace(candidate_id='one'), {'snapshot_id': 'fresh'}, configuration)
    kwargs = dict(target='bi', emit=lambda **row: None, directory=tmp_path)
    if expected is None:
        with pytest.raises(type(failure), match=str(failure)):
            native_closure.screen_closure_candidate(*args, **kwargs)
    else:
        assert native_closure.screen_closure_candidate(*args, **kwargs) is expected


def private_robot():
    names = [f'joint_{i}' for i in range(7, 0, -1)] + [
        f'gripper_{side}_{part}_Joint' for side in ('Left', 'Right') for part in ('1', '2', 'Support')]
    state = {'q': np.zeros(13), 'v': np.ones(13)*.0001, 'writes': []}
    def write(key, value):
        state['writes'].append(key)
        state[key] = np.array(value)
    robot = SimpleNamespace(
        get_active_joints=lambda: [SimpleNamespace(name=n, limits=[[-3., 3.]]) for n in names],
        get_qpos=lambda: state['q'], get_qvel=lambda: state['v'],
        set_qpos=lambda q: write('q', q), set_qvel=lambda v: write('v', v))
    drives = dict(source='native_joint_drive_targets_readback', joint_names=names,
        position_targets=[0.]*13, velocity_targets=[0.]*13, physics_timestep_s=.01,
        simulation_frequency_hz=100., control_frequency_hz=20.)
    return robot, drives, state, names


def test_hypothesis_endpoint_uses_native_identity_and_never_claims_measured():
    robot, drives, state, names = private_robot()
    q = JointConfiguration(tuple(f'joint_{i}' for i in range(1, 8)), np.arange(7)*.1)
    result = native_closure.apply_candidate_endpoint(robot, q, drives)
    assert result['measured'] is False
    assert result['assumption'] == 'stationary_endpoint_not_executed_approach'
    assert np.array_equal(state['v'], np.zeros(13))
    for name, value in zip(q.names, q.positions):
        assert state['q'][names.index(name)] == pytest.approx(value)
        assert drives['position_targets'][names.index(name)] == pytest.approx(value)


@pytest.mark.parametrize('bad', ['limit', 'names'])
def test_bad_endpoint_rejected_before_any_private_write(bad):
    robot, drives, state, names = private_robot()
    q = JointConfiguration(tuple(f'joint_{i}' for i in range(1, 8)), np.zeros(7))
    if bad == 'limit':
        q = JointConfiguration(q.names, [4.]*7)
    else:
        q = JointConfiguration(tuple(reversed(q.names)), q.positions)
    with pytest.raises(SceneInvalid):
        native_closure.apply_candidate_endpoint(robot, q, drives)
    assert not state['writes']
