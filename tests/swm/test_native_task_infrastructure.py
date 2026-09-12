import numpy as np
import pytest

from rm75_app.orchestration.multi_object_executor import SceneObjectState, TaskSceneState
from rm75_app.pickplace.atom_task_builder import FixedSceneAtomTaskBuilder
from rm75_app.swm.native_skills import NativeStageState


def test_fixed_native_wall_is_not_dropped_or_scaled_by_original_builder():
    state = SceneObjectState('virtual_side_wall', 'virtual_side_wall', np.eye(4), movable=False,
        metadata={'swm_native_infrastructure': dict(dimensions_m=[.03, 1.2, .8], T_object_collision=np.eye(4))})
    scene = FixedSceneAtomTaskBuilder()._planning_scene(TaskSceneState({state.object_id: state}))
    assert len(scene.objects) == 1
    assert scene.objects[0].name == state.object_id
    assert np.array_equal(scene.objects[0].dimensions, [.03, 1.2, .8])


def test_movable_object_cannot_use_native_infrastructure_override():
    state = SceneObjectState('virtual_side_wall', 'virtual_side_wall', np.eye(4), movable=True,
        metadata={'swm_native_infrastructure': dict(dimensions_m=[.03, 1.2, .8], T_object_collision=np.eye(4))})
    with pytest.raises(ValueError, match='Only bound fixed'):
        FixedSceneAtomTaskBuilder()._planning_scene(TaskSceneState({state.object_id: state}))


def test_measured_jaw_state_cannot_omit_a_joint_or_infer_it_from_holding():
    with pytest.raises(ValueError, match='six-joint'):
        NativeStageState(None, 'empty', gripper_positions={'gripper_Left_1_Joint': .1})
