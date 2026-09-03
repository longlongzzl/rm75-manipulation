from __future__ import annotations

import numpy as np

from rm75_app.execution.maniskill_task_bridge import ManiSkillTaskBridge
from rm75_app.orchestration.multi_object_executor import AtomExecution, SceneObjectState, TaskSceneState
from rm75_app.tasks.manipulation_plan import (
    ManipulationAtom,
    ManipulationPrimitive,
    SuccessCriteria,
)


class FakeTensor:
    def __init__(self, value):
        self.value = np.asarray(value)

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.value


class FakePose:
    def __init__(self, matrix):
        self.matrix = matrix

    def to_transformation_matrix(self):
        return FakeTensor(self.matrix[None])


class FakeActor:
    def __init__(self, matrix):
        self.pose = FakePose(matrix)


class FakeRobot:
    def get_qpos(self):
        return FakeTensor([0.1, 0.2, 0.3])


class FakeEnv:
    def __init__(self):
        self.unwrapped = self
        self.agent = type("Agent", (), {"robot": FakeRobot()})()
        self.action_space = type("Space", (), {"shape": (4,)})()
        self.actions = []

    def step(self, action):
        self.actions.append(np.asarray(action).copy())


def test_maniskill_bridge_observes_settles_and_validates() -> None:
    target = np.eye(4)
    target[0, 3] = 0.2
    env = FakeEnv()
    bridge = ManiSkillTaskBridge(
        env,
        {"carrot_1": FakeActor(target)},
        settle_steps=3,
        hold_action=lambda: np.asarray([0.1, 0.2, 0.3, 0.0]),
    )
    atom = ManipulationAtom(
        "atom_01",
        ManipulationPrimitive.PICK_PLACE,
        "carrot_1",
        "carriot",
        target,
    )
    scene = TaskSceneState({"carrot_1": SceneObjectState("carrot_1", "carriot", np.eye(4))})
    validation = bridge.validate_atom(atom, AtomExecution(True, target), scene)
    assert validation.success
    assert len(env.actions) == 3
    np.testing.assert_allclose(bridge.observe_object_pose("carrot_1"), target)


def test_maniskill_bridge_validates_inside_as_containment_not_release_pose() -> None:
    support = np.asarray(
        [
            [-0.0107148, 0.01146237, 0.99987698, -0.29775435],
            [0.99993926, 0.00269823, 0.01068446, -0.28024864],
            [-0.00257546, 0.99993074, -0.01149056, 0.04883897],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    observed = support.copy()
    observed[:3, 3] = (
        support @ np.asarray([-0.0015, -0.0067, 0.0, 1.0])
    )[:3]
    release = observed.copy()
    release[2, 3] += 0.061
    atom = ManipulationAtom(
        "atom_inside",
        ManipulationPrimitive.PICK_PLACE,
        "tennis",
        "tennis",
        release,
        support_object_id="bitong",
        success=SuccessCriteria("inside", 0.04, None, 0),
    )
    scene = TaskSceneState(
        {
            "tennis": SceneObjectState("tennis", "tennis", observed),
            "bitong": SceneObjectState("bitong", "bitong", support),
        }
    )
    bridge = ManiSkillTaskBridge(
        FakeEnv(),
        {"tennis": FakeActor(observed), "bitong": FakeActor(support)},
        settle_steps=0,
    )

    validation = bridge.validate_atom(atom, AtomExecution(True, observed), scene)

    assert validation.success
    assert validation.position_error_m == 0.0
    assert validation.diagnostics["validation_mode"] == "geometric_containment"
    assert validation.diagnostics["release_pose_distance_m"] > 0.05


def test_maniskill_bridge_rejects_object_outside_container() -> None:
    support = np.eye(4)
    observed = np.eye(4)
    observed[0, 3] = 0.09
    atom = ManipulationAtom(
        "atom_outside",
        ManipulationPrimitive.PICK_PLACE,
        "tennis",
        "tennis",
        observed,
        support_object_id="bitong",
        success=SuccessCriteria("inside", 0.04, None, 0),
    )
    scene = TaskSceneState(
        {
            "tennis": SceneObjectState("tennis", "tennis", observed),
            "bitong": SceneObjectState("bitong", "bitong", support),
        }
    )
    bridge = ManiSkillTaskBridge(
        FakeEnv(),
        {"tennis": FakeActor(observed), "bitong": FakeActor(support)},
        settle_steps=0,
    )

    validation = bridge.validate_atom(atom, AtomExecution(True, observed), scene)

    assert not validation.success
    assert validation.position_error_m > 0.0
