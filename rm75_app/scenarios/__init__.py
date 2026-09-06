"""Unified scenario layer for sorting, magnetic assembly, and Push-T."""

from .contracts import (
    ExecutionMode,
    PreparedStep,
    ProgramStep,
    ScenarioKind,
    ScenarioObservation,
    ScenarioProgram,
    ScenarioRunReport,
    SceneStamp,
    StepExecutionResult,
    StepStatus,
    stable_fingerprint,
)
from .pickplace_program import (
    AtomBoundaryCommand,
    CompiledPickPlaceAtom,
    CompiledPickPlaceProgram,
    GripperCommand,
    PickPlaceCompilationResult,
    PickPlaceExecutionReport,
    PickPlaceProgramCompiler,
    PickPlaceProgramExecutor,
    TrajectoryCommand,
)
from .program_runner import ScenarioProgramRunner
from .rrtrack_bridge import (
    RRTrackInstanceSample,
    RRTrackPushTTracker,
    RRTrackSceneAdapter,
    RRTrackSceneUpdate,
)
from .sorting import (
    PreparedSortingProgram,
    SortingAssignment,
    SortingPlanCompiler,
    SortingRequest,
    SortingSystem,
    SortingTarget,
)
from .sorting_io import (
    load_sorting_request,
    save_sorting_request,
    sorting_request_as_dict,
    sorting_request_from_dict,
)
from .system import UnifiedManipulationSystem, UnifiedSystemConfig

__all__ = [
    "AtomBoundaryCommand",
    "CompiledPickPlaceAtom",
    "CompiledPickPlaceProgram",
    "ExecutionMode",
    "GripperCommand",
    "PickPlaceCompilationResult",
    "PickPlaceExecutionReport",
    "PickPlaceProgramCompiler",
    "PickPlaceProgramExecutor",
    "PreparedSortingProgram",
    "PreparedStep",
    "ProgramStep",
    "RRTrackInstanceSample",
    "RRTrackPushTTracker",
    "RRTrackSceneAdapter",
    "RRTrackSceneUpdate",
    "ScenarioKind",
    "ScenarioObservation",
    "ScenarioProgram",
    "ScenarioProgramRunner",
    "ScenarioRunReport",
    "SceneStamp",
    "SortingAssignment",
    "SortingPlanCompiler",
    "SortingRequest",
    "SortingSystem",
    "SortingTarget",
    "StepExecutionResult",
    "StepStatus",
    "TrajectoryCommand",
    "UnifiedManipulationSystem",
    "UnifiedSystemConfig",
    "load_sorting_request",
    "save_sorting_request",
    "sorting_request_as_dict",
    "sorting_request_from_dict",
    "stable_fingerprint",
]
