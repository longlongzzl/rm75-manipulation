"""Original geometry predicates on measured snapshots, not physical execution."""
from types import SimpleNamespace
import numpy as np
import pytest
from rm75_app.swm.skills import SkillRequest, SkillVerification
from rm75_app.swm.scene import SceneInvalid, digest
from rm75_app.swm.native_tasks import CompiledNativeTask
from rm75_app.swm.adapters import to_task_scene_state
from rm75_app.tasks.manipulation_plan import ManipulationAtom, ManipulationPlan, ManipulationPrimitive, SuccessCriteria
from .conftest import pose


def test_native_predicate_missing_refuses_motion(rig):
    with pytest.raises(NotImplementedError, match='verifier adapter'):
        rig.runtime.run(SkillRequest('push', 'a', pose(.4), goal_predicate='native_relation'))
    assert rig.backend.executions == 0


@pytest.mark.parametrize('fault', ['snapshot', 'request', 'untyped'])
def test_wrong_native_verification_provenance_is_rejected(rig, fault):
    req = SkillRequest('push', 'a', pose(.4), goal_predicate='native_relation')
    def verify(request, snapshot):
        if fault == 'untyped': return True
        return SkillVerification('wrong' if fault == 'request' else digest(request.as_dict()),
            'wrong' if fault == 'snapshot' else snapshot['snapshot_id'], True, {})
    rig.runtime.goal_verifier = verify
    with pytest.raises(SceneInvalid, match='provenance'):
        rig.runtime.run(req)
    assert rig.backend.executions == 1


def test_native_relation_cannot_override_holding_requirement(rig):
    rig.source.holding = 'a'
    rig.backend.on_execute = lambda request: setattr(rig.source, 'holding', 'a')
    rig.runtime.goal_verifier = lambda request, snapshot: SkillVerification(
        digest(request.as_dict()), snapshot['snapshot_id'], True, {})
    result = rig.runtime.run(SkillRequest('place', 'a', pose(.4), goal_predicate='native_relation'))
    assert not result.skill_verified
    assert rig.backend.executions == 1


def test_original_inside_geometry_used_for_skill_and_final_goal(rig, monkeypatch):
    from rm75_app.execution import maniskill_task_bridge as original
    rig.world._assets['asset']['native_asset_name'] = 'asset'
    snap = rig.sync.sync('initial')
    # Known local box vertices retain the ORIGINAL containment algorithm.
    vertices = np.array([[x,y,z] for x in (-.01,.01) for y in (-.01,.01) for z in (-.02,.02)])
    monkeypatch.setattr(original, '_scaled_asset_geometry', lambda name: (
        np.array([[-.05,-.05,-.05],[.05,.05,.05]]), vertices))
    atom = ManipulationAtom('inside', ManipulationPrimitive.INSERT, 'a', 'asset', pose(.3,z=.08),
        support_object_id='b', semantic_operator='inside', success=SuccessCriteria('inside', .006, 5.))
    compiler = CompiledNativeTask('pickplace', ManipulationPlan('plan','scene.json',(atom,)),
        to_task_scene_state(snap), SimpleNamespace(config=SimpleNamespace()))
    program = compiler.requests(); next(program); request = next(program)
    assert request.goal_predicate == 'native_relation'
    result = compiler.verify_skill(request, snap)
    assert result.reached and compiler.verify_goals(snap)
    assert result.diagnostics['geometry']['validation_mode'] == 'geometric_containment'
    # Object is contained after settling, although 5 cm from the nominal release.
    assert result.diagnostics['geometry']['release_pose_distance_m'] == pytest.approx(.05)
    snap['objects']['b']['measured']['T_world_object'] = pose(.5,z=.03)
    assert not compiler.verify_skill(request, snap).reached
    assert not compiler.verify_goals(snap)


def test_original_symmetry_predicate_ignores_sphere_rotation_only(rig):
    rig.world._assets['asset']['native_asset_name'] = 'asset'
    snap = rig.sync.sync('initial')
    atom = ManipulationAtom('sphere', ManipulationPrimitive.PICK_PLACE, 'a', 'tennis', pose(.3,z=.03),
                            success=SuccessCriteria('target_pose', .006, 5.))
    compiler = CompiledNativeTask('pickplace', ManipulationPlan('plan','scene.json',(atom,)),
        to_task_scene_state(snap), SimpleNamespace(config=SimpleNamespace()))
    program = compiler.requests(); next(program); request = next(program)
    snap['objects']['a']['measured']['T_world_object'] = pose(.3,z=.03,yaw=1.)
    assert compiler.verify_skill(request,snap).reached
    snap['objects']['a']['measured']['T_world_object'] = pose(.32,z=.03,yaw=1.)
    assert not compiler.verify_skill(request,snap).reached
