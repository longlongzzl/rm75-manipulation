"""State-order regression fixtures; these are not cuRobo/PhysX qualification."""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from rm75_app.planning.contracts import JointTrajectory
from rm75_app.swm.native_audit import CuroboNativeStageAuditor
from rm75_app.swm.native_skills import NativePrimitive, NativeStage, NativeStageState
from rm75_app.swm.scene import SceneInvalid
from rm75_app.swm.skills import PlannedSkill, REQUIRED_AUDITS


class Array:
    def __init__(self, value):
        self.value = np.asarray(value)

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.value


class Model:
    def __init__(self, snapshot):
        self._scene = SimpleNamespace(revision=snapshot['snapshot_id'],
            objects=[SimpleNamespace(name=oid) for oid in snapshot['objects']])
        self.config = SimpleNamespace(retreat_escape_contact_links=('finger',))
        self.names = tuple(snapshot['robot']['joint_names'])
        self.held = 'a'
        self.closed = True
        self.released = None
        self.enabled = set()
        self.reads = []
        self.collide_open = False

    def _ensure_planner(self):
        return SimpleNamespace(joint_names=self.names,
            device_cfg=SimpleNamespace(dtype=float, device='fixture'),
            kinematics=SimpleNamespace(get_joint_limits=lambda: SimpleNamespace(
                position=Array([[-3.] * 7, [3.] * 7]))))

    def _import_modules(self):
        return dict(torch=SimpleNamespace(as_tensor=lambda q, **kwargs: np.asarray(q)),
            JointState=SimpleNamespace(from_position=lambda q, **kwargs: q))

    def _collision_diagnostics_for_states(self, planner, states, **kwargs):
        self.reads.append((self.closed, self.held, self.released))
        if self.collide_open and not self.closed:
            return [dict(candidate_index=0, collision_type='world', robot_link='finger',
                         world_object='b', penetration_m=.01)]
        return []

    def update_scene(self, scene):
        self._scene = scene
        self.released = None

    def detach_object(self, oid, released_pose=None):
        self.held = 'empty'
        self.closed = False
        if released_pose is not None:
            self.released = tuple(released_pose.position)

    def attach_object(self, oid, q):
        self.held = oid

    def update_attached_object_pose(self, oid, q, relative):
        self.relative = relative

    def enable_object_collision(self, oid):
        self.enabled.add(oid)

    def set_gripper_collision_state(self, closed):
        self.closed = closed


def case():
    names = tuple(f'joint_{i}' for i in range(1, 8))
    pose = np.eye(4)
    target = pose.copy()
    target[0, 3] = .4
    snapshot = dict(snapshot_id='current', objects={
        'a': dict(measured=dict(T_world_object=pose.tolist())), 'b': {}},
        robot=dict(idle=True, gripper_closed=True, holding='a', T_world_tcp=pose.tolist(),
                   positions=[0.] * 7, joint_names=names))
    held = NativeStageState(True, 'a', pose)
    released = NativeStageState(False, 'empty', released_object_pose=target)
    path = JointTrajectory(names, np.array([[0.] * 7, [.02] * 7]), dt=.1)
    reverse = JointTrajectory(names, path.positions[::-1], dt=.1)
    primitive = NativePrimitive('place', 'a', (
        NativeStage('place', path, False, held, released),
        NativeStage('retreat', reverse, state_before=released, state_after=released)))
    plan = PlannedSkill('request', 'old', primitive.fingerprint(), primitive, target, 'fixture')
    return snapshot, plan


def test_release_is_enabled_open_and_audited_then_observed_state_restored():
    snapshot, plan = case()
    model = Model(snapshot)
    auditor = CuroboNativeStageAuditor(model)
    audit = auditor(plan, snapshot)
    assert audit.passed == REQUIRED_AUDITS
    assert audit.snapshot_id == 'current'
    assert (False, 'empty', (.4, 0., 0.)) in model.reads
    assert 'a' in model.enabled
    assert model.held == 'a' and model.closed is True and model.released is None
    assert [row['stage'] for row in auditor.last_evidence] == ['place', 'retreat']


def test_opening_collision_fails_and_restores_model_for_next_candidate():
    snapshot, plan = case()
    model = Model(snapshot)
    model.collide_open = True
    auditor = CuroboNativeStageAuditor(model)
    with pytest.raises(SceneInvalid, match='post-transition collision'):
        auditor(plan, snapshot)
    assert model.held == 'a' and model.closed is True and model.released is None
    model.collide_open = False
    assert auditor(plan, snapshot).passed == REQUIRED_AUDITS


def test_holding_does_not_substitute_for_jaw_observation():
    snapshot, plan = case()
    del snapshot['robot']['gripper_closed']
    with pytest.raises(SceneInvalid, match='jaw observation'):
        CuroboNativeStageAuditor(Model(snapshot))(plan, snapshot)


def test_state_mutation_invalidates_primitive_digest():
    snapshot, plan = case()
    stage = plan.payload.stages[-1]
    changed = replace(stage, contact_objects=('b',))
    payload = replace(plan.payload, stages=(*plan.payload.stages[:-1], changed))
    with pytest.raises(SceneInvalid, match='unchanged'):
        CuroboNativeStageAuditor(Model(snapshot))(replace(plan, payload=payload), snapshot)


def test_discontinuous_next_stage_is_rejected_and_restored():
    snapshot, plan = case()
    stage = plan.payload.stages[-1]
    bad_path = replace(stage.trajectory, positions=stage.trajectory.positions + .1)
    payload = replace(plan.payload, stages=(*plan.payload.stages[:-1], replace(stage, trajectory=bad_path)))
    changed = replace(plan, payload=payload, payload_digest=payload.fingerprint())
    model = Model(snapshot)
    with pytest.raises(SceneInvalid, match='discontinuous'):
        CuroboNativeStageAuditor(model)(changed, snapshot)
    assert model.held == 'a' and model.closed is True
