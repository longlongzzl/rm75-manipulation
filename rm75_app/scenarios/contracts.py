"""Common contracts for the three unified manipulation scenarios.

The package intentionally keeps scenario orchestration independent from cuRobo,
ManiSkill, and the physical robot. Scenario frontends compile user intent into
explicit programs; scenario runtimes plan and execute one program step against
a versioned observation.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping, Protocol, Sequence


class ScenarioKind(str, Enum):
    SORTING = "sorting"
    MAGNETIC_ASSEMBLY = "magnetic_assembly"
    PUSH_T = "push_t"


class ExecutionMode(str, Enum):
    """How a program is turned into robot actions."""

    FULL_PLAN = "full_plan"
    LOOKAHEAD = "lookahead"
    CLOSED_LOOP = "closed_loop"


class StepStatus(str, Enum):
    PENDING = "pending"
    PLANNED = "planned"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    INVALIDATED = "invalidated"
    SKIPPED = "skipped"


def _jsonable(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    if hasattr(value, "tolist"):
        return value.tolist()
    if isinstance(value, Enum):
        return value.value
    return value


def stable_fingerprint(value: Mapping[str, Any]) -> str:
    """Return a deterministic content hash for a planning-relevant snapshot."""

    encoded = json.dumps(
        _jsonable(value),
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


@dataclass(frozen=True)
class SceneStamp:
    """Identity of the scene against which a plan was generated."""

    revision: int
    backend_revision: str | None = None
    fingerprint: str | None = None

    def compatible_with(
        self,
        other: "SceneStamp",
        *,
        require_fingerprint: bool = False,
    ) -> bool:
        if int(self.revision) != int(other.revision):
            return False
        if (
            self.backend_revision is not None
            and other.backend_revision is not None
            and self.backend_revision != other.backend_revision
        ):
            return False
        if require_fingerprint and (
            self.fingerprint is None
            or other.fingerprint is None
            or self.fingerprint != other.fingerprint
        ):
            return False
        if (
            self.fingerprint is not None
            and other.fingerprint is not None
            and self.fingerprint != other.fingerprint
        ):
            return False
        return True


@dataclass(frozen=True)
class ScenarioObservation:
    """Versioned observation passed between a frontend/runtime and the runner."""

    stamp: SceneStamp
    state: Any
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "metadata", dict(self.metadata))


@dataclass(frozen=True)
class ProgramStep:
    """Backend-neutral unit in a scenario program."""

    step_id: str
    operator: str
    payload: Mapping[str, Any]
    depends_on: tuple[str, ...] = ()
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.step_id or not self.operator:
            raise ValueError("step_id and operator must not be empty")
        object.__setattr__(self, "payload", dict(self.payload))
        object.__setattr__(
            self,
            "depends_on",
            tuple(str(item) for item in self.depends_on),
        )
        object.__setattr__(self, "metadata", dict(self.metadata))


@dataclass(frozen=True)
class ScenarioProgram:
    program_id: str
    scenario: ScenarioKind
    steps: tuple[ProgramStep, ...]
    execution_mode: ExecutionMode
    user_command: str = ""
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.program_id:
            raise ValueError("program_id must not be empty")
        object.__setattr__(self, "scenario", ScenarioKind(self.scenario))
        object.__setattr__(self, "execution_mode", ExecutionMode(self.execution_mode))
        steps = tuple(self.steps)
        identifiers = [item.step_id for item in steps]
        if len(set(identifiers)) != len(identifiers):
            raise ValueError("program step ids must be unique")
        known: set[str] = set()
        for step in steps:
            missing = set(step.depends_on) - known
            if missing:
                raise ValueError(
                    f"step {step.step_id!r} has forward or missing dependencies: "
                    f"{sorted(missing)}"
                )
            known.add(step.step_id)
        object.__setattr__(self, "steps", steps)
        object.__setattr__(self, "metadata", dict(self.metadata))

    def as_dict(self) -> dict[str, Any]:
        return {
            "program_id": self.program_id,
            "scenario": self.scenario.value,
            "execution_mode": self.execution_mode.value,
            "user_command": self.user_command,
            "steps": [
                {
                    "step_id": step.step_id,
                    "operator": step.operator,
                    "payload": _jsonable(step.payload),
                    "depends_on": list(step.depends_on),
                    "metadata": _jsonable(step.metadata),
                }
                for step in self.steps
            ],
            "metadata": _jsonable(self.metadata),
        }


@dataclass(frozen=True)
class PreparedStep:
    step: ProgramStep
    source_stamp: SceneStamp
    artifact: Any
    predicted_observation: ScenarioObservation | None = None
    planning_time_s: float = 0.0
    diagnostics: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "diagnostics", dict(self.diagnostics))


@dataclass(frozen=True)
class StepExecutionResult:
    success: bool
    step_id: str
    status: StepStatus
    observation: ScenarioObservation | None = None
    message: str = ""
    execution_time_s: float = 0.0
    diagnostics: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "status", StepStatus(self.status))
        object.__setattr__(self, "diagnostics", dict(self.diagnostics))


@dataclass(frozen=True)
class ScenarioRunReport:
    program_id: str
    scenario: ScenarioKind
    success: bool
    results: tuple[StepExecutionResult, ...]
    total_planning_time_s: float
    total_execution_time_s: float
    replans: int = 0
    invalidated_plans: int = 0
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "scenario", ScenarioKind(self.scenario))
        object.__setattr__(self, "results", tuple(self.results))
        object.__setattr__(self, "metadata", dict(self.metadata))


class ScenarioRuntime(Protocol):
    """Runtime contract consumed by :class:`ScenarioProgramRunner`."""

    supports_concurrent_planning: bool

    def observe(self) -> ScenarioObservation: ...

    def plan_step(
        self,
        step: ProgramStep,
        observation: ScenarioObservation,
    ) -> PreparedStep: ...

    def execute_step(self, prepared: PreparedStep) -> StepExecutionResult: ...

    def predict_observation(
        self,
        prepared: PreparedStep,
        observation: ScenarioObservation,
    ) -> ScenarioObservation: ...

    def plan_is_compatible(
        self,
        prepared: PreparedStep,
        observation: ScenarioObservation,
    ) -> bool: ...


def ensure_dependency_order(steps: Sequence[ProgramStep]) -> tuple[ProgramStep, ...]:
    """Return a stable topological order or raise on a cyclic dependency graph."""

    by_id = {item.step_id: item for item in steps}
    if len(by_id) != len(steps):
        raise ValueError("program step ids must be unique")
    unknown = {
        dependency
        for step in steps
        for dependency in step.depends_on
        if dependency not in by_id
    }
    if unknown:
        raise ValueError(f"unknown dependencies: {sorted(unknown)}")
    remaining = list(steps)
    emitted: list[ProgramStep] = []
    done: set[str] = set()
    while remaining:
        ready = [item for item in remaining if set(item.depends_on) <= done]
        if not ready:
            cycle = sorted(item.step_id for item in remaining)
            raise ValueError(f"cyclic program dependencies: {cycle}")
        ready_ids = {item.step_id for item in ready}
        emitted.extend(ready)
        done.update(ready_ids)
        remaining = [item for item in remaining if item.step_id not in ready_ids]
    return tuple(emitted)
