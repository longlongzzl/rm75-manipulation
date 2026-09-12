"""Native coordinator wiring with explicit fixture solvers/sensors, not GPU proof."""
from dataclasses import replace
from types import SimpleNamespace
import copy

import numpy as np
import pytest

from rm75_app.swm.adapters import NativeAtomicBackend
from rm75_app.swm.native_scene import SharedRRTrackCapture, planning_scene
from rm75_app.swm.native_skills import PickPlaceNativePhases, SharedPrimitiveExecutor
from rm75_app.swm.skills import AtomicSkillRuntime, SkillRequest, PlanAudit, REQUIRED_AUDITS
from rm75_app.swm.scene import ObservationUnavailable
from rm75_app.pickplace.coordinator import PickPlaceCoordinator, PickPlaceTask
from rm75_app.planning.contracts import (JointConfiguration, JointTrajectory, Pose,
    PoseCandidate, CandidatePlan, BatchPlanningResult)
from .conftest import pose


class PhasePlanner:
    """Fixture IK only; the coordinator's actual stage methods are exercised."""
    def __init__(self):
        self.calls = []
        self.relations = []

    def update_scene(self, scene):
        self.scene = scene

    def set_gripper_collision_state(self, closed):
        self.closed = closed

    def attach_object(self, name, q):
        pass

    def detach_object(self, name):
        pass

    def update_attached_object_pose(self, name, q, relative):
        self.relations.append(relative.copy())

    def tool_pose_for_configuration(self, q, tool_frame):
        return Pose(q.positions[:3], [1, 0, 0, 0])

    def plan_candidates(self, request):
        plans = []
        for candidate in request.candidates:
            self.calls.append(candidate.candidate_id)
            end = np.zeros(7)
            end[:3] = candidate.pose.position
            trajectory = JointTrajectory(request.current.names,
                np.stack([request.current.positions, end]), dt=.1)
            plans.append(CandidatePlan(candidate.candidate_id, True, trajectory=trajectory))
        return BatchPlanningResult(tuple(plans), backend='fixture_ik')

    def plan_linear_candidates(self, request, **kwargs):
        return self.plan_candidates(request)


def test_native_grasp_place_use_fresh_full_checkpoints_and_replan_place(rig):
    planner = PhasePlanner()
    q = np.zeros(7)
    executed = []
    gripper = {'closed': False}

    class Sink:
        def execute_trajectory(self, stage, trajectory):
            executed.append(stage)
            q[:] = trajectory.positions[-1]
            rig.clock.tick(.1)
            # Simulated fixture measurement is independently provided below.
            if stage == 'lift':
                rig.source.holding = 'a'
                rig.source.poses['a'] = pose(*q[:3])
            if stage == 'retreat':
                rig.source.holding = 'empty'
                rig.source.poses['a'] = pose(.4, z=.03)

        def set_gripper(self, closed):
            gripper['closed'] = closed

    def fresh_robot(batch):
        batch['robot']['positions'] = q.tolist()
        batch['robot']['T_world_tcp'] = pose(*q[:3])

    rig.source.mutate = fresh_robot
    compiler_snapshots = []

    def build(request, snapshot):
        compiler_snapshots.append(copy.deepcopy(snapshot))
        return PickPlaceTask('a', JointConfiguration(tuple(snapshot['robot']['joint_names']), snapshot['robot']['positions']),
            (PoseCandidate('original_grasp', Pose([.3, 0, .03], [1, 0, 0, 0])),),
            (PoseCandidate('original_place', Pose([.4, 0, .03], [1, 0, 0, 0]),
                metadata={'planning_target_object_pose': pose(.4, z=.03)}),),
            planning_scene(snapshot))

    feedback = lambda: dict(source='measured_feedback', idle=True, captured_at=rig.clock(),
        joint_names=[f'joint_{i}' for i in range(1, 8)], positions=q.tolist())
    runner = SharedPrimitiveExecutor(Sink(), feedback, clock=rig.clock, stop=rig.stop)
    audited = []
    def auditor(p, s):
        audited.append(p.payload)
        return PlanAudit(p.payload_digest, s['snapshot_id'], REQUIRED_AUDITS)
    phases = PickPlaceNativePhases(PickPlaceCoordinator(planner, Sink()), build, auditor, runner)
    backend = NativeAtomicBackend(phases.bindings(), execution_domain='fixture')
    runtime = AtomicSkillRuntime(rig.sync, backend, clock=rig.clock, stop=rig.stop)
    assert runtime.run(SkillRequest('grasp', 'a')).skill_verified
    assert executed == ['approach', 'grasp', 'lift']
    assert any('place' in name for name in planner.calls)  # Lookahead plans, never executes place.
    # Measured object-to-TCP relation changes between the two skills.
    rig.source.poses['a'][0][3] += .002
    assert runtime.run(SkillRequest('place', 'a', pose(.4, z=.03))).skill_verified
    assert executed == ['approach', 'grasp', 'lift', 'preplace', 'place', 'retreat']
    assert planner.relations[-1][0, 3] == pytest.approx(.002)
    assert audited[0].stages[-1].state_after.holding == 'a'
    retreat = audited[1].stages[-1]
    assert retreat.state_before.holding == 'empty'
    assert retreat.state_before.gripper_closed is False
    assert np.allclose(retreat.state_before.released_object_pose, pose(.4, z=.03))
    assert planner.closed is True  # Hypothetical place did not mutate observed held state.
    assert rig.source.boundaries == ['before_grasp', 'pre_execute_grasp', 'after_grasp',
                                     'before_place', 'pre_execute_place', 'after_place']
    assert len({row['snapshot_id'] for row in rig.mirror.rows}) == 6
    assert all(set(row['objects']) == {'a', 'b', 'table'} for row in rig.mirror.rows)
    assert compiler_snapshots[1]['revision'] > compiler_snapshots[0]['revision']
    assert runtime.capabilities()['pull'] is False
    assert runtime.capabilities()['rotate'] is False


def test_shared_rrtrack_acquires_once_for_all_instances_and_rejects_reused_frame(rig):
    from rm75_app.perception.rrtrack.models import FrameObservation
    frames = []
    calls = []
    frame = FrameObservation(np.zeros((2, 2, 3)), np.ones((2, 2)), np.eye(3), 1, 101.)
    agreement = SimpleNamespace(precision=1., support=1., entropy=0.)

    class Tracker:
        def step(self, incoming):
            calls.append(incoming)
            return SimpleNamespace(frame_index=incoming.frame_index, T_cam_obj=pose(),
                agreement=agreement, accepted=True, event='tracking', state='tracking')

    asset = rig.world.assets['asset']
    bindings = {oid: dict(asset_name='asset', mesh_path=asset['mesh_path'],
        mesh_sha256=asset['mesh_sha256']) for oid in ('a', 'b', 'table')}
    def capture(**kwargs):
        frames.append(kwargs)
        return frame
    service = SharedRRTrackCapture({oid: Tracker() for oid in bindings}, bindings, capture,
        lambda stamp: {'captured_at': stamp}, sensor_session='shared')
    result = service(tuple(bindings), after=100., boundary='before_grasp')
    assert len(frames) == 1 and len(calls) == 3
    assert all(item is frame for item in calls)
    assert {s.timestamp_s for s in result['samples']} == {101.}
    with pytest.raises(ObservationUnavailable, match='cached/reordered'):
        service(tuple(bindings), after=101., boundary='after_grasp')
    assert len(calls) == 3
