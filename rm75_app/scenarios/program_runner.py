"""Safe execution policies for backend-neutral scenario programs."""

from __future__ import annotations

import time
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import replace

from .contracts import (
    ExecutionMode,
    PreparedStep,
    ProgramStep,
    ScenarioObservation,
    ScenarioProgram,
    ScenarioRunReport,
    ScenarioRuntime,
    StepExecutionResult,
    StepStatus,
    ensure_dependency_order,
)


class ScenarioProgramRunner:
    """Execute explicit programs without consuming stale plans.

    ``FULL_PLAN`` builds every step against predicted post-step observations and
    then replays them. If an actual observation diverges, the stale suffix is
    discarded and rebuilt. ``LOOKAHEAD`` overlaps the next planning call with
    execution only when the runtime explicitly declares itself concurrency-safe.
    ``CLOSED_LOOP`` observes, plans, and executes one step at a time.
    """

    def __init__(
        self,
        *,
        stop_on_failure: bool = True,
        replan_on_invalidation: bool = True,
    ) -> None:
        self.stop_on_failure = bool(stop_on_failure)
        self.replan_on_invalidation = bool(replan_on_invalidation)

    @staticmethod
    def _successful(result: StepExecutionResult) -> bool:
        return bool(result.success and result.status is StepStatus.SUCCEEDED)

    def run(
        self,
        program: ScenarioProgram,
        runtime: ScenarioRuntime,
    ) -> ScenarioRunReport:
        steps = ensure_dependency_order(program.steps)
        mode = ExecutionMode(program.execution_mode)
        if mode is ExecutionMode.FULL_PLAN:
            return self._run_full_plan(program, steps, runtime)
        if mode is ExecutionMode.LOOKAHEAD:
            if not bool(getattr(runtime, "supports_concurrent_planning", False)):
                report = self._run_closed_loop(program, steps, runtime)
                return replace(
                    report,
                    metadata={
                        **dict(report.metadata),
                        "requested_mode": ExecutionMode.LOOKAHEAD.value,
                        "effective_mode": ExecutionMode.CLOSED_LOOP.value,
                        "lookahead_disabled_reason": "runtime_not_concurrency_safe",
                    },
                )
            return self._run_lookahead(program, steps, runtime)
        return self._run_closed_loop(program, steps, runtime)

    def _run_closed_loop(
        self,
        program: ScenarioProgram,
        steps: tuple[ProgramStep, ...],
        runtime: ScenarioRuntime,
    ) -> ScenarioRunReport:
        planning_time = 0.0
        execution_time = 0.0
        results: list[StepExecutionResult] = []
        for step in steps:
            observation = runtime.observe()
            planning_started = time.perf_counter()
            prepared = runtime.plan_step(step, observation)
            planning_time += max(
                float(prepared.planning_time_s),
                time.perf_counter() - planning_started,
            )
            execution_started = time.perf_counter()
            result = runtime.execute_step(prepared)
            execution_time += max(
                float(result.execution_time_s),
                time.perf_counter() - execution_started,
            )
            results.append(result)
            if not self._successful(result) and self.stop_on_failure:
                break
        success = len(results) == len(steps) and all(
            self._successful(item) for item in results
        )
        return ScenarioRunReport(
            program.program_id,
            program.scenario,
            success,
            tuple(results),
            planning_time,
            execution_time,
            metadata={"effective_mode": ExecutionMode.CLOSED_LOOP.value},
        )

    def _build_suffix(
        self,
        steps: tuple[ProgramStep, ...],
        start_index: int,
        runtime: ScenarioRuntime,
        observation: ScenarioObservation,
    ) -> tuple[list[PreparedStep], float]:
        prepared_steps: list[PreparedStep] = []
        planning_time = 0.0
        predicted = observation
        for step in steps[start_index:]:
            started = time.perf_counter()
            prepared = runtime.plan_step(step, predicted)
            planning_time += max(
                float(prepared.planning_time_s),
                time.perf_counter() - started,
            )
            prepared_steps.append(prepared)
            predicted = (
                prepared.predicted_observation
                if prepared.predicted_observation is not None
                else runtime.predict_observation(prepared, predicted)
            )
        return prepared_steps, planning_time

    def _run_full_plan(
        self,
        program: ScenarioProgram,
        steps: tuple[ProgramStep, ...],
        runtime: ScenarioRuntime,
    ) -> ScenarioRunReport:
        initial = runtime.observe()
        prepared_steps, planning_time = self._build_suffix(
            steps, 0, runtime, initial
        )
        execution_time = 0.0
        replans = 0
        invalidated = 0
        results: list[StepExecutionResult] = []
        index = 0
        current = initial
        while index < len(steps):
            prepared = prepared_steps[index]
            if not runtime.plan_is_compatible(prepared, current):
                invalidated += len(steps) - index
                if not self.replan_on_invalidation:
                    results.append(
                        StepExecutionResult(
                            False,
                            steps[index].step_id,
                            StepStatus.INVALIDATED,
                            observation=current,
                            message="preplanned suffix no longer matches the observed scene",
                        )
                    )
                    break
                rebuilt, elapsed = self._build_suffix(
                    steps, index, runtime, current
                )
                planning_time += elapsed
                replans += 1
                prepared_steps[index:] = rebuilt
                prepared = prepared_steps[index]
            started = time.perf_counter()
            result = runtime.execute_step(prepared)
            execution_time += max(
                float(result.execution_time_s),
                time.perf_counter() - started,
            )
            results.append(result)
            current = (
                result.observation
                if result.observation is not None
                else runtime.observe()
            )
            if not self._successful(result) and self.stop_on_failure:
                break
            index += 1
        success = len(results) == len(steps) and all(
            self._successful(item) for item in results
        )
        return ScenarioRunReport(
            program.program_id,
            program.scenario,
            success,
            tuple(results),
            planning_time,
            execution_time,
            replans,
            invalidated,
            metadata={"effective_mode": ExecutionMode.FULL_PLAN.value},
        )

    def _run_lookahead(
        self,
        program: ScenarioProgram,
        steps: tuple[ProgramStep, ...],
        runtime: ScenarioRuntime,
    ) -> ScenarioRunReport:
        if not steps:
            return ScenarioRunReport(
                program.program_id,
                program.scenario,
                True,
                (),
                0.0,
                0.0,
                metadata={"effective_mode": ExecutionMode.LOOKAHEAD.value},
            )
        planning_time = 0.0
        execution_time = 0.0
        replans = 0
        invalidated = 0
        results: list[StepExecutionResult] = []
        current = runtime.observe()

        def plan(step: ProgramStep, observation: ScenarioObservation) -> PreparedStep:
            return runtime.plan_step(step, observation)

        with ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="scenario-lookahead",
        ) as pool:
            started = time.perf_counter()
            prepared = plan(steps[0], current)
            planning_time += max(
                float(prepared.planning_time_s),
                time.perf_counter() - started,
            )
            for index, _step in enumerate(steps):
                predicted = (
                    prepared.predicted_observation
                    if prepared.predicted_observation is not None
                    else runtime.predict_observation(prepared, current)
                )
                future: Future[PreparedStep] | None = None
                future_started = 0.0
                if index + 1 < len(steps):
                    future_started = time.perf_counter()
                    future = pool.submit(plan, steps[index + 1], predicted)
                execute_started = time.perf_counter()
                result = runtime.execute_step(prepared)
                execution_time += max(
                    float(result.execution_time_s),
                    time.perf_counter() - execute_started,
                )
                results.append(result)
                current = (
                    result.observation
                    if result.observation is not None
                    else runtime.observe()
                )
                if not self._successful(result):
                    if future is not None:
                        future.cancel()
                    if self.stop_on_failure:
                        break
                if future is None:
                    continue
                candidate = future.result()
                planning_time += max(
                    float(candidate.planning_time_s),
                    time.perf_counter() - future_started,
                )
                if runtime.plan_is_compatible(candidate, current):
                    prepared = candidate
                    continue
                invalidated += 1
                if not self.replan_on_invalidation:
                    results.append(
                        StepExecutionResult(
                            False,
                            steps[index + 1].step_id,
                            StepStatus.INVALIDATED,
                            observation=current,
                            message="lookahead plan invalidated by the observed transition",
                        )
                    )
                    break
                replan_started = time.perf_counter()
                prepared = plan(steps[index + 1], current)
                planning_time += max(
                    float(prepared.planning_time_s),
                    time.perf_counter() - replan_started,
                )
                replans += 1
        success = len(results) == len(steps) and all(
            self._successful(item) for item in results
        )
        return ScenarioRunReport(
            program.program_id,
            program.scenario,
            success,
            tuple(results),
            planning_time,
            execution_time,
            replans,
            invalidated,
            metadata={"effective_mode": ExecutionMode.LOOKAHEAD.value},
        )
