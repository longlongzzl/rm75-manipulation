from __future__ import annotations

import numpy as np

from rm75_app.planning.contracts import (
    BatchPlanningResult,
    CandidatePlan,
    JointConfiguration,
    JointTrajectory,
    PlanningScene,
)
from rm75_app.scenarios.pusht import (
    CuroboWaypointPushBackend,
    PushAction,
    PushTClosedLoopController,
    PushTControllerConfig,
    PushTGoal,
    PushTModelParameters,
    PushTMPC,
    PushTMPCConfig,
    PushTParameterEnsemble,
    PushTPose,
    PushTState,
    QuasiStaticPushTModel,
    SimulatedPushTWorld,
)


def test_quasi_static_model_moves_and_rotates_from_off_center_contact() -> None:
    model = QuasiStaticPushTModel()
    state = PushTState(PushTPose(0.0, 0.0, 0.0))
    action = PushAction((-0.05, -0.04), (1.0, 0.0), 0.04)

    result = model.step(state, action, PushTModelParameters())

    assert result.pose.x > state.pose.x
    assert abs(result.pose.y) < 1.0e-12
    assert result.pose.yaw > 0.0


def test_parameter_ensemble_selects_simulation_closest_to_observed_response() -> None:
    model = QuasiStaticPushTModel()
    true_parameters = PushTModelParameters(
        friction=0.25,
        translation_gain=0.96,
        rotation_gain=2.0,
        contact_efficiency=0.92,
    )
    wrong_parameters = PushTModelParameters(
        friction=0.9,
        translation_gain=0.55,
        rotation_gain=5.0,
        contact_efficiency=0.65,
    )
    ensemble = PushTParameterEnsemble(
        (wrong_parameters, true_parameters),
        observation_sigma_m=0.004,
    )
    state = PushTState(PushTPose(0.0, 0.0, 0.0))
    actions = (
        PushAction((-0.025, 0.0), (1.0, 0.0), 0.035),
        PushAction((0.025, 0.0), (0.8, 0.2), 0.030),
        PushAction((0.0, 0.06), (0.7, -0.3), 0.040),
    )

    for action in actions:
        after = model.step(state, action, true_parameters)
        ensemble.update(state, action, after, model)
        state = after

    assert ensemble.best() == true_parameters
    assert ensemble.weights[1] > 0.95


def test_closed_loop_controller_executes_one_push_then_reobserves() -> None:
    model = QuasiStaticPushTModel(
        workspace_bounds_xy=(-0.3, 0.3, -0.3, 0.3)
    )
    parameters = PushTModelParameters(
        friction=0.35,
        translation_gain=0.9,
        rotation_gain=2.2,
        contact_efficiency=0.95,
    )
    world = SimulatedPushTWorld(
        model,
        PushTState(PushTPose(0.0, 0.0, 0.0)),
        true_parameters=parameters,
    )
    mpc = PushTMPC(
        model,
        PushTMPCConfig(
            horizon=3,
            candidate_sequences=512,
            minimum_push_m=0.02,
            maximum_push_m=0.045,
            direction_noise_rad=np.deg2rad(12.0),
            goal_steering_fraction=0.85,
            seed=4,
        ),
    )
    controller = PushTClosedLoopController(
        world,
        world,
        model,
        mpc,
        parameters=parameters,
        config=PushTControllerConfig(
            max_steps=10,
            settle_time_s=0.0,
            max_consecutive_stalls=4,
        ),
        sleep=lambda _seconds: None,
    )
    goal = PushTGoal(
        PushTPose(0.07, 0.0, 0.0),
        position_tolerance_m=0.018,
        yaw_tolerance_rad=np.deg2rad(15.0),
    )

    report = controller.run(goal)

    assert report.transitions
    assert len(world.actions) == len(report.transitions)
    for previous, current in zip(
        report.transitions,
        report.transitions[1:],
    ):
        assert previous.after.timestamp_s == current.before.timestamp_s
    final_error = np.linalg.norm(
        report.final_observation.state.pose.xy - goal.pose.xy
    )
    assert final_error < 0.07


def test_curobo_push_backend_plans_complete_short_program_before_execution() -> None:
    events: list[tuple[str, str]] = []
    joints = ("j1", "j2")

    class FakePlanner:
        name = "fake"

        def _result(self, request, kind: str) -> BatchPlanningResult:
            candidate = request.candidates[0]
            start = np.asarray(request.current.positions, dtype=np.float64)
            end = start + np.asarray([0.01, 0.005], dtype=np.float64)
            events.append(("plan", f"{kind}:{candidate.candidate_id}"))
            return BatchPlanningResult(
                (
                    CandidatePlan(
                        candidate.candidate_id,
                        True,
                        JointTrajectory(joints, np.stack((start, end))),
                        status="success",
                    ),
                ),
                backend="fake",
            )

        def plan_candidates(self, request):
            return self._result(request, "pose")

        def plan_linear_candidates(self, request, **kwargs):
            assert kwargs["disable_collision_links"] is None
            assert kwargs["ignore_object_name"] is None
            return self._result(request, f"linear_{kwargs['axis']}")

    class FakeSink:
        def execute_trajectory(self, stage, trajectory):
            del trajectory
            events.append(("execute", stage))

    def scene_provider(contact: bool) -> PlanningScene:
        return PlanningScene(revision="contact" if contact else "free")

    backend = CuroboWaypointPushBackend(
        FakePlanner(),
        FakeSink(),
        lambda: JointConfiguration(joints, [0.0, 0.0]),
        scene_provider,
    )

    result = backend.execute_cartesian_push(
        approach_xy=np.asarray([0.0, 0.0]),
        contact_xy=np.asarray([0.02, 0.0]),
        end_xy=np.asarray([0.08, 0.0]),
        speed_mps=0.04,
        metadata={"test": True},
    )

    first_execute = next(
        index for index, event in enumerate(events) if event[0] == "execute"
    )
    assert all(event[0] == "plan" for event in events[:first_execute])
    assert result["segment_count"] >= 5
    assert result["program_diagnostics"]["contact_scene_policy"] == (
        "omit_only_t_object"
    )
    assert backend.last_program is not None
    assert backend.last_program.segments[-1].stage == "push_retract"
