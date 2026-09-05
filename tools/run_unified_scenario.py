#!/usr/bin/env python3
"""Dry-run frontend and simulation tools for the unified scenario layer.

This utility never commands a physical robot.  It compiles sorting plans,
generates/validates magnetic structures, and runs the pure-Python Push-T loop.
The physical cuRobo/ManiSkill/RealMan checks are intentionally left to the
validation protocol in ``docs/CODEX_VALIDATION_UNIFIED_SCENARIOS.md``.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import asdict, is_dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Mapping

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rm75_app.orchestration.multi_object_executor import load_task_scene
from rm75_app.scenarios.magnetic import (
    MagneticAssemblyFrontend,
    OpenAICompatibleStructureClient,
    StrictMagneticAssemblyPlanner,
    load_magnetic_catalog,
)
from rm75_app.scenarios.magnetic.io import (
    load_magnetic_assembly,
    load_magnetic_inventory,
    magnetic_assembly_as_dict,
    save_magnetic_assembly,
)
from rm75_app.scenarios.pusht import (
    PushTClosedLoopController,
    PushTControllerConfig,
    PushTGoal,
    PushTMPC,
    PushTMPCConfig,
    PushTModelParameters,
    PushTParameterEnsemble,
    PushTPose,
    PushTState,
    QuasiStaticPushTModel,
    SimulatedPushTWorld,
)
from rm75_app.scenarios.sorting import SortingPlanCompiler
from rm75_app.scenarios.sorting_io import load_sorting_request


def _jsonable(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if is_dataclass(value):
        return _jsonable(asdict(value))
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Enum):
        return value.value
    return value


def _write_json(path: str | Path | None, value: Any) -> None:
    text = json.dumps(_jsonable(value), ensure_ascii=False, indent=2)
    if path is None:
        print(text)
        return
    target = Path(path).expanduser().resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_text(text + "\n", encoding="utf-8")
    temporary.replace(target)
    print(target)


def _anchor_pose(path: str | None) -> np.ndarray:
    if path is None:
        return np.eye(4, dtype=np.float64)
    raw = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    value = raw.get("anchor_pose", raw) if isinstance(raw, Mapping) else raw
    pose = np.asarray(value, dtype=np.float64)
    if pose.shape != (4, 4):
        raise ValueError("anchor pose file must contain one 4x4 matrix")
    return pose


def command_sorting_compile(args: argparse.Namespace) -> int:
    request = load_sorting_request(args.request)
    scene = load_task_scene(args.scene or request.scene_file)
    plan = SortingPlanCompiler().compile(request, scene)
    _write_json(args.output, plan.as_dict())
    return 0


def _llm_client(args: argparse.Namespace):
    if args.llm_endpoint is None and args.llm_model is None:
        return None
    if not args.llm_endpoint or not args.llm_model:
        raise ValueError("--llm-endpoint and --llm-model must be supplied together")
    api_key = None
    if args.api_key_env:
        api_key = os.environ.get(args.api_key_env)
    return OpenAICompatibleStructureClient(
        args.llm_endpoint,
        args.llm_model,
        api_key=api_key,
        timeout_s=args.llm_timeout,
    )


def command_magnetic_generate(args: argparse.Namespace) -> int:
    catalog = load_magnetic_catalog(args.catalog)
    inventory = load_magnetic_inventory(args.inventory)
    frontend = MagneticAssemblyFrontend(
        catalog,
        inventory,
        max_pieces=args.max_pieces,
    )
    structure = frontend.generate(
        args.description,
        anchor_pose=_anchor_pose(args.anchor_pose),
        llm=_llm_client(args),
        allow_template_fallback=not args.no_template_fallback,
    )
    save_magnetic_assembly(structure, args.output)
    print(Path(args.output).expanduser().resolve())
    return 0


def command_magnetic_validate(args: argparse.Namespace) -> int:
    catalog = load_magnetic_catalog(args.catalog)
    inventory = load_magnetic_inventory(args.inventory)
    structure = load_magnetic_assembly(args.assembly)
    planner = StrictMagneticAssemblyPlanner(catalog)
    symbolic = planner.validate(structure, inventory)
    output: dict[str, Any] = {
        "symbolic_validation": symbolic,
        "structure": magnetic_assembly_as_dict(structure),
    }
    if symbolic.valid:
        placements = planner.resolve(structure, inventory)
        output["resolved_placements"] = [
            {
                "piece_id": item.piece.piece_id,
                "object_id": item.piece.object_id,
                "asset_name": item.piece.asset_name,
                "target_pose": item.target_pose,
                "support_ids": item.support_ids,
                "connection_id": item.connection_id,
                "clearance_m": item.clearance_m,
                "depends_on_piece_ids": item.depends_on_piece_ids,
                "diagnostics": item.diagnostics,
            }
            for item in placements
        ]
        output["geometry_validation"] = planner.last_geometry_report
    _write_json(args.output, output)
    return 0 if symbolic.valid else 2


def command_pusht_sim(args: argparse.Namespace) -> int:
    model = QuasiStaticPushTModel(
        workspace_bounds_xy=(
            args.xmin,
            args.xmax,
            args.ymin,
            args.ymax,
        )
    )
    true_parameters = PushTModelParameters(
        friction=args.true_friction,
        translation_gain=args.true_translation_gain,
        rotation_gain=args.true_rotation_gain,
        contact_efficiency=args.true_contact_efficiency,
        anisotropy=args.true_anisotropy,
    )
    world = SimulatedPushTWorld(
        model,
        PushTState(PushTPose(args.x, args.y, args.yaw)),
        true_parameters=true_parameters,
        observation_noise_std_m=args.position_noise,
        observation_noise_std_rad=args.yaw_noise,
        seed=args.seed,
    )
    mpc = PushTMPC(
        model,
        PushTMPCConfig(
            horizon=args.horizon,
            candidate_sequences=args.candidates,
            minimum_push_m=args.min_push,
            maximum_push_m=args.max_push,
            direction_noise_rad=np.deg2rad(args.direction_noise_deg),
            seed=args.seed,
        ),
    )
    estimator = (
        PushTParameterEnsemble.default_grid()
        if args.system_identification
        else None
    )
    controller = PushTClosedLoopController(
        world,
        world,
        model,
        mpc,
        parameters=PushTModelParameters(),
        estimator=estimator,
        config=PushTControllerConfig(
            max_steps=args.max_steps,
            settle_time_s=0.0,
        ),
        sleep=lambda _seconds: None,
    )
    goal = PushTGoal(
        PushTPose(args.goal_x, args.goal_y, args.goal_yaw),
        args.position_tolerance,
        np.deg2rad(args.yaw_tolerance_deg),
    )
    report = controller.run(goal)
    _write_json(args.output, report)
    return 0 if report.success else 3


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)

    sorting = commands.add_parser(
        "sorting-compile",
        help="compile a JSON sorting request into a ManipulationPlan",
    )
    sorting.add_argument("--request", required=True)
    sorting.add_argument("--scene")
    sorting.add_argument("--output")
    sorting.set_defaults(handler=command_sorting_compile)

    generate = commands.add_parser(
        "magnetic-generate",
        help="generate a validated <=12-piece symbolic magnetic structure",
    )
    generate.add_argument("--catalog", required=True)
    generate.add_argument("--inventory", required=True)
    generate.add_argument("--description", required=True)
    generate.add_argument("--anchor-pose")
    generate.add_argument("--output", required=True)
    generate.add_argument("--max-pieces", type=int, default=12)
    generate.add_argument("--llm-endpoint")
    generate.add_argument("--llm-model")
    generate.add_argument("--api-key-env", default="OPENAI_API_KEY")
    generate.add_argument("--llm-timeout", type=float, default=60.0)
    generate.add_argument("--no-template-fallback", action="store_true")
    generate.set_defaults(handler=command_magnetic_generate)

    validate = commands.add_parser(
        "magnetic-validate",
        help="validate symbolic rules and resolved magnetic geometry",
    )
    validate.add_argument("--catalog", required=True)
    validate.add_argument("--inventory", required=True)
    validate.add_argument("--assembly", required=True)
    validate.add_argument("--output")
    validate.set_defaults(handler=command_magnetic_validate)

    pusht = commands.add_parser(
        "pusht-sim",
        help="run the pure-Python closed-loop many-future Push-T demo",
    )
    pusht.add_argument("--x", type=float, default=0.0)
    pusht.add_argument("--y", type=float, default=0.0)
    pusht.add_argument("--yaw", type=float, default=0.0)
    pusht.add_argument("--goal-x", type=float, required=True)
    pusht.add_argument("--goal-y", type=float, required=True)
    pusht.add_argument("--goal-yaw", type=float, default=0.0)
    pusht.add_argument("--position-tolerance", type=float, default=0.015)
    pusht.add_argument("--yaw-tolerance-deg", type=float, default=8.0)
    pusht.add_argument("--horizon", type=int, default=3)
    pusht.add_argument("--candidates", type=int, default=384)
    pusht.add_argument("--max-steps", type=int, default=20)
    pusht.add_argument("--min-push", type=float, default=0.012)
    pusht.add_argument("--max-push", type=float, default=0.045)
    pusht.add_argument("--direction-noise-deg", type=float, default=35.0)
    pusht.add_argument("--system-identification", action="store_true")
    pusht.add_argument("--true-friction", type=float, default=0.45)
    pusht.add_argument("--true-translation-gain", type=float, default=0.82)
    pusht.add_argument("--true-rotation-gain", type=float, default=3.0)
    pusht.add_argument("--true-contact-efficiency", type=float, default=0.9)
    pusht.add_argument("--true-anisotropy", type=float, default=0.0)
    pusht.add_argument("--position-noise", type=float, default=0.0)
    pusht.add_argument("--yaw-noise", type=float, default=0.0)
    pusht.add_argument("--xmin", type=float, default=-0.35)
    pusht.add_argument("--xmax", type=float, default=0.35)
    pusht.add_argument("--ymin", type=float, default=-0.35)
    pusht.add_argument("--ymax", type=float, default=0.35)
    pusht.add_argument("--seed", type=int, default=0)
    pusht.add_argument("--output")
    pusht.set_defaults(handler=command_pusht_sim)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return int(args.handler(args))


if __name__ == "__main__":
    raise SystemExit(main())
