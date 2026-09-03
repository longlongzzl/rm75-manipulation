from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

from rm75_app.tasks.manipulation_plan import load_plan


def _ensure_desktop_display_env() -> None:
    """Recover the local X11 session for workers launched by a headless parent."""
    uid = os.getuid()
    runtime_dir = Path(f"/run/user/{uid}")
    if "XDG_RUNTIME_DIR" not in os.environ and runtime_dir.is_dir():
        os.environ["XDG_RUNTIME_DIR"] = str(runtime_dir)
    if "DISPLAY" not in os.environ:
        sockets = sorted(Path("/tmp/.X11-unix").glob("X*"))
        if sockets:
            os.environ["DISPLAY"] = f":{sockets[0].name[1:]}"
    if "XAUTHORITY" not in os.environ:
        candidates = (runtime_dir / "gdm" / "Xauthority", Path.home() / ".Xauthority")
        authority = next((path for path in candidates if path.is_file()), None)
        if authority is not None:
            os.environ["XAUTHORITY"] = str(authority)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--gate", choices=["curobo2", "maniskill"], required=True)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--execution-file", type=Path, default=None)
    parser.add_argument("--render-mode", choices=["rgb_array", "human"], default="rgb_array")
    parser.add_argument("--debug-viewer", action="store_true")
    parser.add_argument(
        "--continue-after-validation-failure",
        action="store_true",
        help="debug replay: keep executing later atoms after a pose check fails",
    )
    parser.add_argument(
        "--disable-self-collision",
        action="store_true",
        help="diagnostic only: run the Curobo2 gate without robot self-collision checks",
    )
    args = parser.parse_args(argv)
    plan = load_plan(args.plan)
    if args.gate == "curobo2":
        from rm75_app.planning.backends.curobo2 import Curobo2BackendConfig
        from rm75_app.validation.curobo_gate import run_curobo_gate

        backend_config = None
        if args.disable_self_collision:
            backend_config = Curobo2BackendConfig(
                max_batch_size=4,
                max_goalset=4,
                num_ik_seeds=64,
                num_trajopt_seeds=4,
                attachment_num_spheres=64,
                self_collision_check=False,
            )
        result = run_curobo_gate(plan, args.output_dir, backend_config=backend_config)
    else:
        if args.execution_file is None:
            raise ValueError("--execution-file is required for the ManiSkill gate")
        viewer = None
        if args.debug_viewer:
            # SAPIEN 3 must create its present-capable window before importing
            # ManiSkill environments, which initialize an offscreen renderer.
            _ensure_desktop_display_env()
            from sapien.utils import Viewer

            viewer = Viewer(resolutions=(960, 720))
        from rm75_app.execution.maniskill_scene import ensure_pick_env_registered
        from rm75_app.runtime.curobo2_sim_replay import DEFAULT_EXTRA_MANISKILL_ROOT

        ensure_pick_env_registered("Two_finger_PickJiaobang-v1", DEFAULT_EXTRA_MANISKILL_ROOT)
        from rm75_app.validation.maniskill_gate import run_maniskill_gate

        result = run_maniskill_gate(
            plan,
            args.execution_file,
            args.output_dir,
            render_mode=args.render_mode,
            debug_viewer=args.debug_viewer,
            viewer=viewer,
            stop_on_validation_failure=not args.continue_after_validation_failure,
        )
    payload = result.as_dict()
    result_file = args.output_dir.expanduser().resolve() / "gate_result.json"
    result_file.write_text(json.dumps(payload, ensure_ascii=False, indent=2, default=str), encoding="utf-8")
    print(json.dumps(payload, ensure_ascii=False, indent=2, default=str))
    return 0 if result.passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
