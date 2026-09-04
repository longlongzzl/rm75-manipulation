"""One application facade for all three manipulation scenarios."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Sequence

from rm75_app.orchestration.multi_object_executor import TaskSceneState
from rm75_app.pickplace.atom_task_builder import FixedSceneAtomTaskBuilder
from rm75_app.planning.contracts import JointConfiguration

from .magnetic import (
    MagneticAssemblyFrontend,
    MagneticAssemblyPlanner,
    MagneticAssemblySpec,
    MagneticAssemblySystem,
    MagneticInventoryItem,
    MagneticPanelSpec,
    OpenAICompatibleStructureClient,
    PreparedMagneticProgram,
    StructureLLM,
)
from .pickplace_program import (
    PickPlaceExecutionReport,
    PickPlaceProgramCompiler,
    PickPlaceProgramExecutor,
    TrajectorySink,
)
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


class UnifiedManipulationSystem:
    """Share one PickPlace foundation across sorting and magnetic assembly.

    Push-T deliberately remains a closed-loop scenario because the object state
    changes continuously during contact.  It still follows the same system
    pattern: observe reality, imagine/plan in simulation, execute one explicit
    action program, then observe again.
    """

    def __init__(
        self,
        planner: Any,
        task_builder: FixedSceneAtomTaskBuilder,
        *,
        trajectory_sink: TrajectorySink | None = None,
        joint_state_provider: Any | None = None,
        config: UnifiedSystemConfig | None = None,
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
        planner = MagneticAssemblyPlanner(catalog)
        return MagneticAssemblySystem(
            planner,
            self.motion_compiler,
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
