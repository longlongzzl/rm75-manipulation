"""Regression tests from the 2026-09-06 review; no GPU, robot or API calls."""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from rm75_app.scenarios.contracts import (
    ExecutionMode, PreparedStep, ProgramStep, ScenarioKind, ScenarioObservation,
    ScenarioProgram, SceneStamp, StepExecutionResult, StepStatus,
)
from rm75_app.scenarios.program_runner import ScenarioProgramRunner
from rm75_app.scenarios.pusht import (
    PushAction, PushTClosedLoopController, PushTControllerConfig, PushTGoal,
    PushTModelParameters, PushTObservation, PushTPose, PushTState,
    QuasiStaticPushTModel,
)
from rm75_app.scenarios.magnetic import (
    MagneticAssemblySpec, MagneticConnection, MagneticGeometryValidator,
    MagneticInventoryItem, MagneticJointType, MagneticPanelSpec, MagneticPiece,
    PanelEdge, PanelPoseClass, PanelRole, StrictMagneticAssemblyPlanner,
)
from rm75_app.scenarios.magnetic.contracts import pose_matrix
from rm75_app.scenarios.magnetic.planner import MagneticAssemblyPlanner, _edge_world_geometry
from rm75_app.scenarios.pickplace_program import (
    AtomBoundaryCommand, CompiledPickPlaceAtom, CompiledPickPlaceProgram,
    GripperCommand, PickPlaceProgramExecutor, TrajectoryCommand,
)
from rm75_app.orchestration.multi_object_executor import TaskSceneState
from rm75_app.tasks.manipulation_plan import ManipulationAtom, ManipulationPrimitive
from rm75_app.planning.contracts import JointConfiguration, JointTrajectory


@pytest.mark.parametrize('bad', [np.nan, np.inf, -np.inf])
@pytest.mark.parametrize('field', ['contact', 'direction', 'distance', 'speed', 'clearance', 'goal_tolerance'])
def test_push_nonfinite_inputs_fail_closed(bad, field):
    kw = dict(contact_local_xy=[-.05, 0.0], direction_world_xy=[1.0, 0.0],
              distance_m=.02, speed_mps=.04, approach_clearance_m=.02)
    if field == 'contact': kw['contact_local_xy'][0] = bad
    elif field == 'direction': kw['direction_world_xy'][1] = bad
    elif field == 'distance': kw['distance_m'] = bad
    elif field == 'speed': kw['speed_mps'] = bad
    elif field == 'clearance': kw['approach_clearance_m'] = bad
    with pytest.raises(ValueError):
        if field == 'goal_tolerance':
            PushTGoal(PushTPose(0, 0, 0), position_tolerance_m=bad)
        else:
            PushAction(**kw)


class Tracker:
    def __init__(self, samples):
        self.samples = iter(samples)
        self.calls = 0
    def observe(self):
        self.calls += 1
        return next(self.samples)


class PushExecutor:
    def __init__(self, success=True): self.calls, self.success = 0, success
    def execute_push(self, action, observation):
        self.calls += 1
        return {'success': self.success}


class FixedMPC:
    def plan(self, state, goal, parameters):
        return SimpleNamespace(action=PushAction((-.05, 0), (1, 0), .02))


def observation(x=0, yaw=0, stamp=0):
    return PushTObservation(PushTState(PushTPose(x, 0, yaw)), stamp)


def controller(samples, executor=None, **kwargs):
    tracker = Tracker(samples)
    executor = executor or PushExecutor()
    instance = PushTClosedLoopController(
        tracker, executor, QuasiStaticPushTModel(), FixedMPC(),
        config=PushTControllerConfig(settle_time_s=0, **kwargs),
    )
    return instance, tracker, executor


def test_rejected_push_is_not_fitted_or_retried():
    run, tracker, executor = controller([observation()], PushExecutor(False))
    report = run.run(PushTGoal(PushTPose(.1, 0, 0)))
    assert report.reason == 'execution_failed'
    assert not report.transitions and executor.calls == 1 and tracker.calls == 1


def test_stale_sensor_frame_cannot_claim_success():
    run, _, _ = controller([observation(), observation(.1, stamp=0)])
    report = run.run(PushTGoal(PushTPose(.1, 0, 0)))
    assert not report.success
    assert report.reason == 'tracking_timestamp_not_increasing'


def test_orientation_only_progress_does_not_stall():
    run, _, executor = controller(
        [observation(yaw=yaw, stamp=i) for i, yaw in enumerate([.8, .6, .4, .2, 0])],
        max_consecutive_stalls=1,
    )
    report = run.run(PushTGoal(PushTPose(0, 0, 0), yaw_tolerance_rad=.02))
    assert report.success and executor.calls == 4
    assert all(item.execution_diagnostics['pose_progress_m'] > 0 for item in report.transitions)


def test_planner_failure_returns_report_without_actuation():
    run, _, executor = controller([observation()])
    def fail(*args): raise RuntimeError('no feasible future')
    run.mpc.plan = fail
    report = run.run(PushTGoal(PushTPose(.1, 0, 0)))
    assert report.reason == 'planning_failed' and executor.calls == 0


def test_workspace_is_not_a_fictitious_wall():
    model = QuasiStaticPushTModel(workspace_bounds_xy=(-.3, .3, -.3, .3))
    result = model.step(PushTState(PushTPose(.29, 0, 0)),
                        PushAction((-.05, 0), (1, 0), .1), PushTModelParameters())
    assert result.pose.x > .3  # prediction is NOT clamped back into the workspace
    assert not model.state_is_valid(result)


class Runtime:
    supports_concurrent_planning = True
    def __init__(self, change_once=False, change_always=False, fail_first=False):
        self.revision = 0
        self.change_once, self.change_always, self.fail_first = change_once, change_always, fail_first
        self.executed, self.planned = [], []
    def observe(self):
        return ScenarioObservation(SceneStamp(self.revision, fingerprint=str(self.revision)), {})
    def plan_step(self, step, obs):
        self.planned.append(step.step_id)
        result = PreparedStep(step, obs.stamp, None, ScenarioObservation(
            SceneStamp(obs.stamp.revision+1, fingerprint=str(obs.stamp.revision+1)), {}))
        if self.change_always or self.change_once:
            self.revision += 1
            self.change_once = False
        return result
    def plan_is_compatible(self, plan, obs):
        return plan.source_stamp.compatible_with(obs.stamp, require_fingerprint=True)
    def execute_step(self, plan):
        assert self.plan_is_compatible(plan, self.observe()), 'stale command was sent'
        self.executed.append(plan.step.step_id)
        success = not (self.fail_first and plan.step.step_id == 'a')
        if success: self.revision += 1
        return StepExecutionResult(success, plan.step.step_id,
                                   StepStatus.SUCCEEDED if success else StepStatus.FAILED,
                                   observation=self.observe())


def program(mode):
    return ScenarioProgram('test', ScenarioKind.SORTING,
                           (ProgramStep('a','pick',{}), ProgramStep('b','pick',{},('a',))), mode)


@pytest.mark.parametrize('mode', list(ExecutionMode))
def test_scene_changes_during_planning_are_checked_before_first_execution(mode):
    runtime = Runtime(change_once=True)
    report = ScenarioProgramRunner().run(program(mode), runtime)
    assert report.success and report.replans >= 1
    assert runtime.executed == ['a', 'b']


@pytest.mark.parametrize('mode', list(ExecutionMode))
def test_changing_scene_aborts_after_bounded_replanning(mode):
    runtime = Runtime(change_always=True)
    report = ScenarioProgramRunner(max_replans_per_step=1).run(program(mode), runtime)
    assert not report.success and not runtime.executed
    assert report.results[-1].status is StepStatus.INVALIDATED
    assert len(runtime.planned) <= 4


@pytest.mark.parametrize('mode', list(ExecutionMode))
def test_failed_dependency_is_skipped_even_when_continuing(mode):
    runtime = Runtime(fail_first=True)
    request = replace(program(mode), steps=program(mode).steps+(ProgramStep('c','pick',{}),))
    report = ScenarioProgramRunner(stop_on_failure=False).run(request, runtime)
    assert not report.success and runtime.executed == ['a', 'c']
    assert {result.step_id: result.status for result in report.results}['b'] is StepStatus.SKIPPED


def wall_fixture():
    catalog={'p':MagneticPanelSpec('p',(.10,.06,.008))}
    pieces=(MagneticPiece('l','o0','p',PanelRole.BASE,PanelPoseClass.FLAT,(0,-.036,0)),
            MagneticPiece('r','o1','p',PanelRole.BASE,PanelPoseClass.FLAT,(0,.036,0)),
            MagneticPiece('w','o2','p',PanelRole.WALL,PanelPoseClass.VERTICAL))
    link=MagneticConnection('slot',MagneticJointType.VERTICAL_SLOT,'w',('l','r'),
                            (PanelEdge.POS_Y,PanelEdge.NEG_Y))
    return catalog, MagneticAssemblySpec('wall',pieces,(link,),np.eye(4)), tuple(
        MagneticInventoryItem(f'o{i}','p',f's{i}') for i in range(3))


def test_mag_panel_limit_is_not_controlled_by_untrusted_blueprint():
    _, spec, _ = wall_fixture()
    with pytest.raises(ValueError): replace(spec, max_pieces=100)


@pytest.mark.parametrize('rotation', [np.diag([1,1,-1]), np.diag([2,1,1])])
def test_mag_rejects_reflection_and_scale_as_rotation(rotation):
    pose=np.eye(4); pose[:3,:3]=rotation
    with pytest.raises(ValueError): pose_matrix(pose)


def test_slot_child_cannot_be_translated_away_from_valid_parents():
    catalog,spec,inventory=wall_fixture()
    placements=list(StrictMagneticAssemblyPlanner(catalog).resolve(spec,inventory))
    moved=placements[-1].target_pose.copy(); moved[0,3]+=.10
    placements[-1]=replace(placements[-1],target_pose=moved)
    report=MagneticGeometryValidator().validate(spec,placements,catalog)
    assert not report.valid
    assert 'slot_child_not_centered' in {item.code for item in report.violations}


@pytest.mark.parametrize('child_edge', list(PanelEdge))
@pytest.mark.parametrize('flip', [False,True])
def test_all_selected_child_edges_align_with_parent_edge(child_edge,flip):
    panel=MagneticPanelSpec('p',(.1,.06,.008))
    conn=MagneticConnection('j',MagneticJointType.RIGHT_ANGLE_EDGE,'c',('p',),
                            (PanelEdge.POS_X,),child_edge=child_edge,flip=flip)
    planner=MagneticAssemblyPlanner({'p':panel})
    pose,_=planner._right_angle_pose(conn,panel,np.eye(4),panel)
    parent=_edge_world_geometry(np.eye(4),panel,PanelEdge.POS_X)
    child=_edge_world_geometry(pose,panel,child_edge)
    assert abs(np.dot(parent[2],child[2])) > .999999
    assert abs(np.dot(parent[3],child[3])) < 1e-6
    assert np.isclose(np.linalg.det(pose[:3,:3]),1)


def compiled_program():
    atom=ManipulationAtom('a',ManipulationPrimitive.PICK_PLACE,'o','p',np.eye(4))
    trajectory=JointTrajectory(('j',),np.array([[0.],[.1]]),dt=.1)
    commands=(AtomBoundaryCommand('a',True),TrajectoryCommand('a','approach',trajectory),
              GripperCommand('a',True),GripperCommand('a',False),AtomBoundaryCommand('a',False,True))
    compiled=CompiledPickPlaceAtom(atom,commands,'g','p',0,1,0)
    return CompiledPickPlaceProgram('p',SceneStamp(0,fingerprint='s'),(compiled,),TaskSceneState({}),0)


class Sink:
    def __init__(self): self.commands=[]
    def set_gripper(self,closed): self.commands.append(('gripper',closed))
    def execute_trajectory(self,stage,trajectory): self.commands.append(('trajectory',stage))


def test_required_feedback_cannot_silently_disappear():
    sink=Sink()
    report=PickPlaceProgramExecutor(sink,require_state_feedback=True).execute(compiled_program())
    assert not report.success and not sink.commands


def test_source_scene_mismatch_blocks_all_commands():
    sink=Sink()
    report=PickPlaceProgramExecutor(sink,scene_stamp_provider=lambda:SceneStamp(1,fingerprint='changed')).execute(compiled_program())
    assert not report.success and not sink.commands


def test_failed_readiness_does_not_open_gripper_and_requests_stop():
    sink=Sink(); stopped=[]
    report=PickPlaceProgramExecutor(sink,pre_release_gate=lambda _:False,
                                    stop_callback=lambda:stopped.append(True)).execute(compiled_program())
    assert not report.success and report.failed_stage=='release_readiness'
    assert ('gripper',False) not in sink.commands and stopped==[True]


def test_dispatch_completion_is_not_physical_success_evidence():
    sink=Sink()
    report=PickPlaceProgramExecutor(sink).execute(compiled_program())
    assert report.success
    assert report.diagnostics['completion_kind']=='command_dispatch'
    assert not report.diagnostics['task_success_verified']


def test_failed_outcome_does_not_commit_atom_as_complete():
    sink=Sink()
    report=PickPlaceProgramExecutor(sink,atom_validator=lambda _:False).execute(compiled_program())
    assert not report.success and not report.completed_atoms
    assert report.failed_stage=='atom_outcome_validation'


def test_magnetic_support_top_handles_swapped_vertical_axis():
    from rm75_app.scenarios.magnetic.planner import _support_top
    panel = MagneticPanelSpec('p', (.1, .06, .008))
    pose = np.eye(4)
    # local +X is up, not the historical +Y convention
    pose[:3, :3] = [[0, 1, 0], [0, 0, 1], [1, 0, 0]]
    assert np.allclose(_support_top(pose, panel), [0, 0, .05])


def test_antiparallel_support_normals_are_rejected_without_validator_crash():
    catalog, spec, inventory = wall_fixture()
    placements = list(StrictMagneticAssemblyPlanner(catalog).resolve(spec, inventory))
    changed = placements[1].target_pose.copy()
    changed[:3, :3] = np.diag([1, -1, -1])
    placements[1] = replace(placements[1], target_pose=changed)
    report = MagneticGeometryValidator().validate(spec, placements, catalog)
    assert not report.valid


def test_unified_facade_preserves_opt_in_execution_feedback_gate():
    from rm75_app.scenarios.system import UnifiedManipulationSystem, UnifiedSystemConfig
    class Sink:
        def execute_trajectory(self, *args): raise AssertionError('not expected')
        def set_gripper(self, *args): raise AssertionError('not expected')
    system = UnifiedManipulationSystem(None, None, trajectory_sink=Sink(),
                                      config=UnifiedSystemConfig(require_execution_feedback=True))
    assert system.program_executor.require_state_feedback


def test_initial_gripper_open_is_not_misclassified_as_held_object_release():
    request = compiled_program()
    atom = request.atoms[0]
    commands = (atom.commands[0], GripperCommand('a', False), *atom.commands[1:])
    request = replace(request, atoms=(replace(atom, commands=commands),))
    called = []
    sink = Sink()
    report = PickPlaceProgramExecutor(sink, pre_release_gate=lambda x: called.append(x) or True).execute(request)
    assert report.success and called == ['a']
    assert sink.commands[0] == ('gripper', False)
