"""Measured reference contracts; fixture evidence, not native execution."""
import copy
from types import SimpleNamespace
import numpy as np
import pytest
from rm75_app.swm.skills import SkillRequest
from rm75_app.swm.scene import SceneInvalid
from rm75_app.swm.native_tasks import CompiledNativeTask
from rm75_app.swm.adapters import to_task_scene_state
from rm75_app.tasks.manipulation_plan import ManipulationAtom, ManipulationPlan, ManipulationPrimitive
from .conftest import pose


def test_reference_uses_new_pose_and_absolute_does_not_drift(rig):
    snap = rig.sync.sync('initial')
    local = pose(.1, yaw=.2)
    relative = SkillRequest('push', 'a', local, target_reference_id='b')
    absolute = SkillRequest('push', 'a', pose(.6))
    snap['objects']['b']['measured']['T_world_object'] = pose(.4, .2, yaw=.6)
    np.testing.assert_allclose(relative.resolve(snap).target, np.asarray(pose(.4, .2, yaw=.6)) @ local)
    assert absolute.resolve(snap) is absolute
    assert relative.as_dict()['target_reference_id'] == 'b'
    assert relative.target == tuple(tuple(row) for row in local)


@pytest.mark.parametrize('fault', ['missing', 'unmeasured', 'invalid'])
def test_unobserved_reference_refuses_resolution(rig, fault):
    snap = rig.sync.sync('initial')
    if fault == 'missing': del snap['objects']['b']
    elif fault == 'unmeasured': snap['objects']['b']['measured'] = None
    else: snap['valid'] = False
    with pytest.raises(SceneInvalid, match='reference'):
        SkillRequest('place', 'a', pose(), target_reference_id='b').resolve(snap)


def test_runtime_replans_reference_motion(rig):
    req = SkillRequest('push', 'a', pose(.1), target_reference_id='b', position_tolerance_m=.001)
    targets = []
    def change_reference(request, snapshot):
        targets.append(request.target)
        if len(targets) == 1:
            rig.source.poses['b'] = pose(.302, z=.03)
    rig.backend.on_plan = change_reference
    result = rig.runtime.run(req)
    assert result.skill_verified
    assert len(targets) == 2 and rig.backend.executions == 1
    assert result.attempts[0]['reason'] == 'target_changed_during_planning'
    np.testing.assert_allclose(np.asarray(targets[-1])[:3, 3], [.402, 0., .03])


def test_post_reference_motion_cannot_verify_old_goal(rig):
    rig.source.holding = 'a'
    rig.backend.on_execute = lambda request: rig.source.poses.update(b=pose(.4, z=.03))
    result = rig.runtime.run(SkillRequest('place', 'a', pose(.1), target_reference_id='b'))
    assert not result.skill_verified
    assert result.status == 'replan_required'
    assert rig.backend.executions == 1


def test_inside_compiler_preserves_local_relation_under_full_base_rotation(rig):
    rig.world._assets['asset']['native_asset_name'] = 'asset'
    snap = rig.sync.sync('initial')
    base = np.asarray(pose(-.3, .1, yaw=.7))
    template = to_task_scene_state(snap)
    for state in template.objects.values(): state.pose = base @ state.pose
    local = np.asarray(pose(0, 0, .08, yaw=.2))
    atom = ManipulationAtom('insert', ManipulationPrimitive.INSERT, 'a', 'asset',
        template.objects['b'].pose @ local, support_object_id='b', semantic_operator='inside')
    builder = SimpleNamespace(config=SimpleNamespace(robot_base_world_transform=base))
    compiler = CompiledNativeTask('pickplace', ManipulationPlan('plan', 'scene.json', (atom,)), template, builder)
    program = compiler.requests()
    next(program)
    request = next(program)
    moved = copy.deepcopy(snap)
    moved['objects']['b']['measured']['T_world_object'] = pose(.45, .1, yaw=.4)
    expected = np.asarray(pose(.45, .1, yaw=.4)) @ local
    assert request.target_reference_id == 'b'
    np.testing.assert_allclose(request.resolve(moved).target, expected, atol=1e-12)
    np.testing.assert_allclose(compiler._base_target(atom, moved), expected, atol=1e-12)
