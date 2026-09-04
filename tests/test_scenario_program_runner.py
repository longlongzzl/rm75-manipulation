from __future__ import annotations

from dataclasses import dataclass

from rm75_app.scenarios.contracts import (
    ExecutionMode,
    PreparedStep,
    ProgramStep,
    ScenarioKind,
    ScenarioObservation,
    ScenarioProgram,
    SceneStamp,
    StepExecutionResult,
    StepStatus,
    ensure_dependency_order,
)
from rm75_app.scenarios.program_runner import ScenarioProgramRunner


@dataclass
class FakeRuntime:
    supports_concurrent_planning: bool = False
    diverge_after_first_execution: bool = False

    def __post_init__(self) -> None:
        self.revision = 0
        self.plan_calls: list[tuple[str, int]] = []
        self.execute_calls: list[tuple[str, int]] = []

    def observe(self) -> ScenarioObservation:
        return ScenarioObservation(
            SceneStamp(self.revision, fingerprint=f"scene-{self.revision}"),
            {"revision": self.revision},
        )

    def plan_step(
        self,
        step: ProgramStep,
        observation: ScenarioObservation,
    ) -> PreparedStep:
        self.plan_calls.append((step.step_id, observation.stamp.revision))
        predicted_revision = observation.stamp.revision + 1
        return PreparedStep(
            step,
            observation.stamp,
            artifact={"source_revision": observation.stamp.revision},
            predicted_observation=ScenarioObservation(
                SceneStamp(
                    predicted_revision,
                    fingerprint=f"scene-{predicted_revision}",
                ),
                {"revision": predicted_revision},
            ),
        )

    def execute_step(self, prepared: PreparedStep) -> StepExecutionResult:
        source = int(prepared.artifact["source_revision"])
        self.execute_calls.append((prepared.step.step_id, source))
        assert source == self.revision
        if self.diverge_after_first_execution and not self.execute_calls[:-1]:
            self.revision += 2
        else:
            self.revision += 1
        return StepExecutionResult(
            True,
            prepared.step.step_id,
            StepStatus.SUCCEEDED,
            observation=self.observe(),
        )

    def predict_observation(
        self,
        prepared: PreparedStep,
        observation: ScenarioObservation,
    ) -> ScenarioObservation:
        del observation
        assert prepared.predicted_observation is not None
        return prepared.predicted_observation

    def plan_is_compatible(
        self,
        prepared: PreparedStep,
        observation: ScenarioObservation,
    ) -> bool:
        return prepared.source_stamp.compatible_with(
            observation.stamp,
            require_fingerprint=True,
        )


def _program(mode: ExecutionMode) -> ScenarioProgram:
    first = ProgramStep("s1", "move", {"value": 1})
    second = ProgramStep("s2", "move", {"value": 2}, depends_on=("s1",))
    third = ProgramStep("s3", "move", {"value": 3}, depends_on=("s2",))
    return ScenarioProgram(
        "program",
        ScenarioKind.SORTING,
        (first, second, third),
        mode,
    )


def test_full_plan_replays_predicted_suffix_without_replanning() -> None:
    runtime = FakeRuntime()
    report = ScenarioProgramRunner().run(
        _program(ExecutionMode.FULL_PLAN),
        runtime,
    )

    assert report.success
    assert report.replans == 0
    assert report.invalidated_plans == 0
    assert runtime.plan_calls == [("s1", 0), ("s2", 1), ("s3", 2)]
    assert runtime.execute_calls == [("s1", 0), ("s2", 1), ("s3", 2)]


def test_full_plan_invalidates_and_rebuilds_suffix_after_scene_divergence() -> None:
    runtime = FakeRuntime(diverge_after_first_execution=True)
    report = ScenarioProgramRunner().run(
        _program(ExecutionMode.FULL_PLAN),
        runtime,
    )

    assert report.success
    assert report.replans == 1
    assert report.invalidated_plans == 2
    assert runtime.execute_calls == [("s1", 0), ("s2", 2), ("s3", 3)]
    assert runtime.plan_calls == [
        ("s1", 0),
        ("s2", 1),
        ("s3", 2),
        ("s2", 2),
        ("s3", 3),
    ]


def test_lookahead_request_falls_back_when_runtime_is_not_thread_safe() -> None:
    runtime = FakeRuntime(supports_concurrent_planning=False)
    report = ScenarioProgramRunner().run(
        _program(ExecutionMode.LOOKAHEAD),
        runtime,
    )

    assert report.success
    assert report.metadata["requested_mode"] == "lookahead"
    assert report.metadata["effective_mode"] == "closed_loop"
    assert runtime.plan_calls == [("s1", 0), ("s2", 1), ("s3", 2)]


def test_dependency_sort_rejects_cycle() -> None:
    first = ProgramStep("a", "move", {}, depends_on=("b",))
    second = ProgramStep("b", "move", {}, depends_on=("a",))

    try:
        ensure_dependency_order((first, second))
    except ValueError as exc:
        assert "cyclic" in str(exc)
    else:
        raise AssertionError("cycle was not rejected")
