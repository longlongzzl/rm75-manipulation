"""Shared task/planning calibration contract; no native physics in fixtures."""
from types import SimpleNamespace

import numpy as np
import pytest

from rm75_app.core.frames import explicit_robot_base_transform
from rm75_app.pickplace.atom_task_builder import AtomTaskBuilderConfig, FixedSceneAtomTaskBuilder
from rm75_app.swm.adapters import to_task_scene_state
from rm75_app.swm.native_tasks import CompiledNativeTask
from rm75_app.tasks.manipulation_plan import ManipulationAtom, ManipulationPlan, ManipulationPrimitive
from .conftest import pose


@pytest.mark.parametrize('yaw', [np.pi / 2, -.4])
def test_builder_and_swm_targets_share_full_rigid_calibration(rig, yaw):
    rig.world._assets['asset']['native_asset_name'] = 'asset'
    snapshot = rig.sync.sync('initial')
    base = np.asarray(pose(-.3, .1, .02, yaw=yaw))
    local_target = np.asarray(pose(.4, -.2, .05, yaw=.2))
    world_target = base @ local_target
    config = AtomTaskBuilderConfig(robot_base_world_transform=base)
    builder = FixedSceneAtomTaskBuilder(config=config)
    atom = ManipulationAtom('place', ManipulationPrimitive.PICK_PLACE, 'a', 'asset', world_target)
    plan = ManipulationPlan('approved_plan', 'original_scene.json', (atom,))
    compiler = CompiledNativeTask('pickplace', plan, to_task_scene_state(snapshot), builder)
    program = compiler.requests()
    next(program)
    np.testing.assert_allclose(next(program).target, local_target, atol=1e-12)
    np.testing.assert_allclose(builder._to_planning_pose(world_target), local_target, atol=1e-12)
    world_scene = compiler.measured_native_scene(snapshot)
    for oid, obj in world_scene.objects.items():
        measured = np.asarray(snapshot['objects'][oid]['measured']['T_world_object'])
        np.testing.assert_allclose(obj.pose, base @ measured, atol=1e-12)
        np.testing.assert_allclose(builder._to_planning_pose(obj.pose), measured, atol=1e-12)
    # Frozen config must not retain the caller's mutable matrix.
    base[0, 3] = 99.
    assert config.robot_base_world_transform[0][3] == -.3


def test_translation_only_configuration_keeps_previous_meaning():
    builder = FixedSceneAtomTaskBuilder(config=AtomTaskBuilderConfig(robot_base_world_xyz_m=(1., 2., 3.)))
    np.testing.assert_allclose(builder._to_planning_pose(np.asarray(pose(1.4, 2., 3.05))), pose(.4, z=.05))
    np.testing.assert_array_equal(explicit_robot_base_transform(), np.eye(4))


@pytest.mark.parametrize('fault', ['scale', 'reflection', 'nan', 'last_row', 'both'])
def test_invalid_or_ambiguous_base_calibration_never_silently_falls_back(fault):
    matrix = np.eye(4)
    options = dict(robot_base_world_transform=matrix)
    if fault == 'scale':
        matrix[0, 0] = .9
    elif fault == 'reflection':
        matrix[0, 0] = -1.
    elif fault == 'nan':
        matrix[0, 3] = np.nan
    elif fault == 'last_row':
        matrix[3, 0] = 1.
    else:
        options['robot_base_world_xyz_m'] = (0., 0., 0.)
    with pytest.raises(ValueError):
        AtomTaskBuilderConfig(**options)
