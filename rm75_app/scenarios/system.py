"""One application facade for all three manipulation scenarios."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Mapping, Sequence

from rm75_app.orchestration.multi_object_executor import TaskSceneState
from rm75_app.pickplace.atom_task_builder import FixedSceneAtomTaskBuilder

from .magnetic import (
    MagneticAssemblyFrontend,
    MagneticAssemblySpec,
    MagneticAssemblySystem,
    MagneticContactPlanningBackend,
    MagneticInventoryItem,
    MagneticPanelSpec,
    MagneticPickPlaceTaskBuilder,
    PreparedMagneticProgram,
    StrictMagneticAssemblyPlanner,
    StructureLLM,
)
from .pickplace_program import (
    CompiledPickPlaceAtom,
    PickPlaceExecutionReport,
    PickPlaceProgramCompiler,
    PickPlaceProgramExecutor,
    TrajectorySink,
)
from .contracts import SceneStamp
from .pusht import (
    ObjectFramePushExecutor,
    PushTClosedLoopController,
    PushTControllerConfig,
    PushTGoal,
    PushTMPC,
    PushTMPCConfig,
    PushTModelParameters,
    PushTParameterEnsemble,
    PushTRunReport,
    QuasiStaticPushTModel,
)
from .sorting import (
    PreparedSortingProgram,
    SortingPlanCompiler,
    SortingRequest,
    SortingSystem,
)


@dataclass(frozen=True)
class UnifiedSystemConfig:
    relation_screen_mode: str = "lazy_place"
    grasp_fallback_mode: str = "primary_only"
    max_stage_start_gap_rad: float = 0.10
    max_initial_joint_gap_rad: float = 0.12
    magnetic_max_pieces: int = 12
    # Opt in for physical deployments; offline recording remains compatible.
    require_execution_feedback: bool = False

    def __post_init__(self) -> None:
        if isinstance(self.magnetic_max_pieces, bool) or not isinstance(self.magnetic_max_pieces, int) or not 1 <= self.magnetic_max_pieces <= 12:
            raise ValueError("magnetic_max_pieces must be in [1, 12]")


class UnifiedManipulationSystem:
    """Share one PickPlace foundation across sorting and magnetic assembly.

    Magnetic placement wraps the same planner/task builder with a scoped
    multi-support contact adapter. Push-T deliberately remains closed loop
    because object state changes continuously during contact, but still follows
    observe -> simulate/plan -> explicit action -> observe.
    """

    def __init__(
        self,
        planner: Any,
        task_builder: FixedSceneAtomTaskBuilder,
        *,
        trajectory_sink: TrajectorySink | None = None,
        joint_state_provider: Any | None = None,
        config: UnifiedSystemConfig | None = None,
        scene_stamp_provider: Callable[[], SceneStamp] | None = None,
        atom_validator: Callable[[CompiledPickPlaceAtom], bool] | None = None,
        pre_release_gate: Callable[[str], bool] | None = None,
        cancellation_requested: Callable[[], bool] | None = None,
        stop_callback: Callable[[], None] | None = None,
    ) -> None:
        self.planner = planner
        self.task_builder = task_builder
        self.config = config or UnifiedSystemConfig()
        self.motion_compiler = PickPlaceProgramCompiler(
            planner,
            task_builder,
            relation_screen_mode=self.config.relation_screen_mode,
            grasp_fallback_mode=self.config.grasp_fallback_mode,
            max_stage_start_gap_rad=self.config.max_stage_start_gap_rad,
        )
        self.program_executor = (
            None
            if trajectory_sink is None
            else PickPlaceProgramExecutor(
                trajectory_sink,
                joint_state_provider=joint_state_provider,
                scene_stamp_provider=scene_stamp_provider,
                require_state_feedback=self.config.require_execution_feedback,
                atom_validator=atom_validator,
                pre_release_gate=pre_release_gate,
                cancellation_requested=cancellation_requested,
                stop_callback=stop_callback,
                max_initial_joint_gap_rad=(
                    self.config.max_initial_joint_gap_rad
                ),
            )
        )
        self.sorting = SortingSystem(
            SortingPlanCompiler(),
            self.motion_compiler,
            self.program_executor,
        )

    def _magnetic_motion_compiler(self) -> PickPlaceProgramCompiler:
        return PickPlaceProgramCompiler(
            MagneticContactPlanningBackend(self.planner),
            MagneticPickPlaceTaskBuilder(self.task_builder),
            relation_screen_mode=self.config.relation_screen_mode,
            grasp_fallback_mode=self.config.grasp_fallback_mode,
            max_stage_start_gap_rad=self.config.max_stage_start_gap_rad,
        )

    def prepare_sorting(
        self,
        request: SortingRequest,
        scene: TaskSceneState,
    ) -> PreparedSortingProgram:
        return self.sorting.prepare(request, scene)

    def execute_sorting(
        self,
        prepared: PreparedSortingProgram,
    ) -> PickPlaceExecutionReport:
        return self.sorting.execute(prepared)

    def magnetic_system(
        self,
        catalog: Mapping[str, MagneticPanelSpec],
    ) -> MagneticAssemblySystem:
        structure_planner = StrictMagneticAssemblyPlanner(catalog)
        return MagneticAssemblySystem(
            structure_planner,
            self._magnetic_motion_compiler(),
            self.program_executor,
        )

    def generate_magnetic_structure(
        self,
        description: str,
        catalog: Mapping[str, MagneticPanelSpec],
        inventory: Sequence[MagneticInventoryItem],
        *,
        anchor_pose: Sequence[Sequence[float]],
        llm: StructureLLM | None = None,
        allow_template_fallback: bool = True,
    ) -> MagneticAssemblySpec:
        frontend = MagneticAssemblyFrontend(
            catalog,
            inventory,
            max_pieces=self.config.magnetic_max_pieces,
        )
        return frontend.generate(
            description,
            anchor_pose=anchor_pose,
            llm=llm,
            allow_template_fallback=allow_template_fallback,
        )

    def prepare_magnetic_structure(
        self,
        structure: MagneticAssemblySpec,
        catalog: Mapping[str, MagneticPanelSpec],
        inventory: Sequence[MagneticInventoryItem],
        scene: TaskSceneState,
        *,
        scene_file: str,
    ) -> PreparedMagneticProgram:
        return self.magnetic_system(catalog).prepare(
            structure,
            inventory,
            scene,
            scene_file=scene_file,
        )

    def execute_magnetic_structure(
        self,
        prepared: PreparedMagneticProgram,
        catalog: Mapping[str, MagneticPanelSpec],
    ) -> PickPlaceExecutionReport:
        return self.magnetic_system(catalog).execute(prepared)

    @staticmethod
    def build_push_t_controller(
        tracker: Any,
        cartesian_push_backend: Any,
        *,
        model: QuasiStaticPushTModel | None = None,
        mpc_config: PushTMPCConfig | None = None,
        parameters: PushTModelParameters | None = None,
        estimator: PushTParameterEnsemble | None = None,
        controller_config: PushTControllerConfig | None = None,
    ) -> PushTClosedLoopController:
        model = model or QuasiStaticPushTModel()
        mpc = PushTMPC(model, mpc_config)
        return PushTClosedLoopController(
            tracker,
            ObjectFramePushExecutor(cartesian_push_backend),
            model,
            mpc,
            parameters=parameters,
            estimator=estimator,
            config=controller_config,
        )

    @classmethod
    def run_push_t(
        cls,
        goal: PushTGoal,
        tracker: Any,
        cartesian_push_backend: Any,
        *,
        model: QuasiStaticPushTModel | None = None,
        mpc_config: PushTMPCConfig | None = None,
        parameters: PushTModelParameters | None = None,
        estimator: PushTParameterEnsemble | None = None,
        controller_config: PushTControllerConfig | None = None,
    ) -> PushTRunReport:
        controller = cls.build_push_t_controller(
            tracker,
            cartesian_push_backend,
            model=model,
            mpc_config=mpc_config,
            parameters=parameters,
            estimator=estimator,
            controller_config=controller_config,
        )
        return controller.run(goal)
