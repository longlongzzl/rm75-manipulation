"""Original typed task/compiler bridge checks; no model or physical execution."""
from types import SimpleNamespace
import numpy as np
import pytest

from rm75_app.swm.native_tasks import CompiledNativeTask
from rm75_app.swm.adapters import to_task_scene_state
from rm75_app.swm.native_scene import planning_scene
from rm75_app.swm.scene import SceneInvalid
from rm75_app.swm.skills import SkillRequest
from rm75_app.pickplace.coordinator import PickPlaceTask
from rm75_app.planning.contracts import JointConfiguration, Pose, PoseCandidate
from rm75_app.scenarios.magnetic.backend import MagneticPickPlaceTaskBuilder
from rm75_app.tasks.manipulation_plan import ManipulationAtom, ManipulationPlan, ManipulationPrimitive
from .conftest import pose


def test_original_jimu_builder_preserves_inventory_supports_and_world_conversion(rig):
    rig.world._assets['asset']['native_asset_name'] = 'asset'
    snapshot = rig.sync.sync('initial')
    template = to_task_scene_state(snapshot)
    atom = ManipulationAtom('original_role', ManipulationPrimitive.SNAP_PLACE, 'a', 'asset', pose(1.4, 2., 3.03),
        metadata={'magnetic_support_object_ids': ['b', 'table'], 'magnetic_connection_id': 'original_slot'})
    plan = ManipulationPlan('original_proof_bound_plan', 'original_scene.json', (atom,))
    calls = []

    class BaseBuilder:
        config = SimpleNamespace(robot_base_world_xyz_m=[1., 2., 3.])
        def __call__(self, received_atom, scene):
            calls.append((received_atom, scene))
            grasp = PoseCandidate('grasp', Pose([.3, 0, .03], [1, 0, 0, 0]))
            place = PoseCandidate('place', Pose([.4, 0, .03], [1, 0, 0, 0]),
                metadata={'planning_target_object_pose': pose(.4, z=.03)})
            return PickPlaceTask('a', JointConfiguration(scene.joint_names, scene.joint_positions),
                (grasp,), (place,), planning_scene(snapshot), place_candidates_by_grasp={'grasp': (place,)})

    verified_scenes = []
    compiler = CompiledNativeTask('magnetic', plan, template, MagneticPickPlaceTaskBuilder(BaseBuilder()),
        structure_verifier=lambda plan, scene: verified_scenes.append(scene) or False)
    program = compiler.requests()
    grasp = next(program)
    built = compiler.build_task(grasp, snapshot)
    assert calls[0][0] is atom
    np.testing.assert_allclose(calls[0][1].objects['a'].pose[:3, 3], [1.3, 2., 3.03])
    assert built.place_candidates[0].metadata['magnetic_support_object_ids'] == ['b', 'table']
    assert built.place_candidates[0].metadata['magnetic_connection_id'] == 'original_slot'
    place = next(program)
    np.testing.assert_allclose(np.asarray(place.target)[:3, 3], [.4, 0., .03])
    with pytest.raises(SceneInvalid, match='trusted compiled target'):
        compiler.build_task(SkillRequest('place', 'a', pose(.5, z=.03)), snapshot)
    assert compiler.verify_goals(snapshot) is False
    assert len(verified_scenes) == 1


def test_magnetic_without_measured_structure_adapter_remains_unavailable(rig):
    snapshot = rig.sync.sync('initial')
    atom = ManipulationAtom('role', ManipulationPrimitive.SNAP_PLACE, 'a', 'asset', pose())
    plan = ManipulationPlan('proof', 'scene.json', (atom,))
    with pytest.raises(NotImplementedError, match='structure'):
        CompiledNativeTask('magnetic', plan, to_task_scene_state(snapshot), lambda *args: None)
