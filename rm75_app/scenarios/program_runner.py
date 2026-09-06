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
        max_replans_per_step: int = 1,
    ) -> None:
        self.stop_on_failure = bool(stop_on_failure)
        self.replan_on_invalidation = bool(replan_on_invalidation)
        if isinstance(max_replans_per_step, bool) or int(max_replans_per_step) != max_replans_per_step or max_replans_per_step < 0:
            raise ValueError("max_replans_per_step must be a non-negative integer")
        self.max_replans_per_step = int(max_replans_per_step)

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
        # After a failed action, a predicted suffix is not authoritative.  In
        # continue-on-failure mode only independent steps may proceed, planned
        # from a fresh observation (never a failed parent's predicted effects).
        if not self.stop_on_failure and mode is not ExecutionMode.CLOSED_LOOP:
            report = self._run_closed_loop(program, steps, runtime)
            return replace(report, metadata={
                **dict(report.metadata), "requested_mode": mode.value,
                "mode_downgrade_reason": "continue_on_failure_requires_observed_dependencies",
            })
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
        completed: set[str] = set()
        replans = invalidated = 0
        for step in steps:
            missing = set(step.depends_on) - completed
            if missing:
                results.append(StepExecutionResult(
                    False, step.step_id, StepStatus.SKIPPED,
                    message="dependency_not_succeeded",
                    diagnostics={"unsatisfied_dependencies": sorted(missing)},
                ))
                continue
            observation = runtime.observe()
            planning_started = time.perf_counter()
            prepared = runtime.plan_step(step, observation)
            planning_time += max(
                float(prepared.planning_time_s),
                time.perf_counter() - planning_started,
            )
            prepared, extra_time, retries, rejected, observation = self._refresh_step(
                step, prepared, runtime
            )
            planning_time += extra_time
            replans += retries
            invalidated += rejected
            if prepared is None:
                results.append(self._invalidated(step, observation))
                if self.stop_on_failure:
                    break
                continue
            execution_started = time.perf_counter()
            result = runtime.execute_step(prepared)
            execution_time += max(
                float(result.execution_time_s),
                time.perf_counter() - execution_started,
            )
            results.append(result)
            if self._successful(result):
                completed.add(step.step_id)
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
            replans,
            invalidated,
            metadata={"effective_mode": ExecutionMode.CLOSED_LOOP.value},
        )

    @staticmethod
    def _invalidated(step: ProgramStep, observation: ScenarioObservation) -> StepExecutionResult:
        return StepExecutionResult(
            False, step.step_id, StepStatus.INVALIDATED, observation=observation,
            message="plan does not match a fresh observation after bounded replanning",
        )

    def _refresh_step(
        self, step: ProgramStep, prepared: PreparedStep, runtime: ScenarioRuntime,
    ) -> tuple[PreparedStep | None, float, int, int, ScenarioObservation]:
        """Reobserve AFTER planning; planning itself can take several seconds.

        Compatibility must be checked again after a replan.  A moving scene is
        not a reason to execute a known-stale command or to replan indefinitely.
        """
        elapsed = 0.0
        retries = rejected = 0
        while True:
            current = runtime.observe()
            if runtime.plan_is_compatible(prepared, current):
                return prepared, elapsed, retries, rejected, current
            rejected += 1
            if not self.replan_on_invalidation or retries >= self.max_replans_per_step:
                return None, elapsed, retries, rejected, current
            started = time.perf_counter()
            prepared = runtime.plan_step(step, current)
            elapsed += max(float(prepared.planning_time_s), time.perf_counter() - started)
            retries += 1

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
            # Do not compare with the pre-compilation snapshot or an old
            # execution result while a long suffix has been prepared.
            current = runtime.observe()
            retries = 0
            while not runtime.plan_is_compatible(prepared, current):
                invalidated += len(steps) - index
                if not self.replan_on_invalidation or retries >= self.max_replans_per_step:
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
                retries += 1
                current = runtime.observe()
            if results and results[-1].status is StepStatus.INVALIDATED:
                break
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
            started = time.perf_counter()
            result = runtime.plan_step(step, observation)
            return replace(result, planning_time_s=max(
                float(result.planning_time_s), time.perf_counter() - started,
            ))

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
                prepared, extra_time, retries, rejected, current = self._refresh_step(
                    steps[index], prepared, runtime
                )
                planning_time += extra_time
                replans += retries
                invalidated += rejected
                if prepared is None:
                    results.append(self._invalidated(steps[index], current))
                    break
                predicted = (
                    prepared.predicted_observation
                    if prepared.predicted_observation is not None
                    else runtime.predict_observation(prepared, current)
                )
                future: Future[PreparedStep] | None = None
                if index + 1 < len(steps):
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
                # Future wait time includes execution; do not bill it as
                # planning latency. The worker above measured actual planning.
                planning_time += float(candidate.planning_time_s)
                prepared = candidate
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
