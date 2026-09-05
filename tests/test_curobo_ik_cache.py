from types import SimpleNamespace

import numpy as np
import torch

from rm75_app.planning.backends.curobo2 import Curobo2Backend


def make_backend():
    backend = Curobo2Backend.__new__(Curobo2Backend)
    backend.config = SimpleNamespace(device="cpu", position_tolerance=0.005,
                                     orientation_tolerance=0.05, coarse_ik_return_seeds=1)
    backend._pose_ik_cache = {}
    backend._pose_ik_metrics = {}
    backend._gripper_collision_state = "open"
    backend._import_modules = lambda: {
        "torch": torch,
        "DeviceCfg": lambda **kw: SimpleNamespace(device="cpu", dtype=torch.float32),
        "GoalToolPose": lambda **kw: SimpleNamespace(**kw),
    }
    return backend


def candidate(name, x=0.0):
    return SimpleNamespace(candidate_id=name, pose=SimpleNamespace(
        position=np.array([x, 0.0, 0.0]),
        quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0])))


def screen(backend, candidates, successes):
    count = len(candidates)
    result = SimpleNamespace(
        success=torch.tensor(successes).reshape(count, 1),
        solution=torch.arange(count * 7, dtype=torch.float32).reshape(count, 1, 7),
        position_error=torch.zeros(count, 1), rotation_error=torch.zeros(count, 1),
        feasible=torch.ones(count, 1, dtype=torch.bool))
    backend._solve_endpoint_pose_batch = lambda *args, **kwargs: result
    solver = SimpleNamespace(scene_collision_checker=object(), reset_seed=lambda: None,
                             kinematics=SimpleNamespace(kinematics_config=object()))
    return backend._prepare_pose_candidates_with_solver(
        tuple(candidates), solver=solver, batch_size=count, scene=None,
        tool_frame="tool", ignore_object_name=None, ignore_object_names=(), screen_kind="coarse")


def test_failed_rescreen_invalidates_old_success_but_not_other_poses():
    backend = make_backend()
    a, b = candidate("a"), candidate("b", 1.0)
    screen(backend, [a, b], [True, True])
    assert backend.feasible_pose_candidate_ids((a, b)) == {"a", "b"}
    rows = screen(backend, [a], [False])
    assert rows["a"]["success"] is False
    assert backend._pose_ik_metrics[backend._pose_cache_key(a)]["success"] is False
    assert backend.feasible_pose_candidate_ids((a, b)) == {"b"}
    screen(backend, [a], [True])
    assert backend.feasible_pose_candidate_ids((a, b)) == {"a", "b"}


def test_failure_without_prior_cache_is_safe_and_mixed_batch_keeps_success():
    backend = make_backend()
    a, b = candidate("a"), candidate("b", 1.0)
    screen(backend, [a, b], [False, True])
    screen(backend, [a], [False])
    assert backend.feasible_pose_candidate_ids((a, b)) == {"b"}


def test_rescreen_invalidates_same_pose_even_with_different_candidate_id():
    backend = make_backend()
    original, resolved = candidate("original"), candidate("resolved")
    screen(backend, [original], [True])
    screen(backend, [resolved], [False])
    assert backend.feasible_pose_candidate_ids((original, resolved)) == set()


def test_invalidation_does_not_cross_gripper_state_cache_keys():
    backend = make_backend()
    a = candidate("a")
    screen(backend, [a], [True])
    backend._gripper_collision_state = "closed"
    screen(backend, [a], [False])
    assert backend.feasible_pose_candidate_ids((a,)) == set()
    backend._gripper_collision_state = "open"
    assert backend.feasible_pose_candidate_ids((a,)) == {"a"}
