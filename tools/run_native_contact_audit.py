#!/usr/bin/env python3
"""Known Jimu scenes, strict contact audit, no robot motion; no arbitrary argv."""
from __future__ import annotations
import argparse
import importlib.util
import json
import os
from pathlib import Path
import sys
import threading

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rm75_app.workcell.contact_audit import install_contact_audit, StrictContactNotSupported
from rm75_app.workcell.io import atomic_json
from rm75_app.workcell.migration import verify_snapshot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", choices=("four-wall", "triangle-roof"), required=True)
    parser.add_argument("--extensions", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--compatibility-audit", action="store_true")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    extensions = args.extensions.resolve()
    root = ROOT / "rm75_app/_vendor/working_snapshot"
    provenance = verify_snapshot(root)
    name = "rm75_jimu_four_wall_portable.py" if args.scene == "four-wall" else "rm75_jimu_triangle_roof_apriltag_portable.py"
    path = root / "Beta_demo-codex-v0.9" / name
    argv = [str(path), "--render-mode", "none", "--jimu-build-layers",
            "first" if args.scene == "four-wall" else "two", "--auto-execute",
            "--curobo-torch-extensions-dir", str(extensions),
            "--camera-extrinsic-opencv-path", str(ROOT / "assets/calibration/camera_extrinsic_opencv.npy")]
    if args.scene == "triangle-roof":
        argv += ["--jimu-second-layer-triangle-profile", "--no-jimu-demo-triangle-apriltag",
                 "--sam6d-fixed-scene-result-file", "Beta_demo-codex-v0.9/jimu_portable_repro/scenes/jimu_assembly_anchors_default_sam6d.json"]
    os.environ["LEROBOT_ROOT"] = str(root)
    sys.argv = argv
    os.chdir(root)
    sys.path.insert(0, str(root))
    sys.path.insert(0, str(path.parent))
    spec = importlib.util.spec_from_file_location("_contact_audit_native", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    direct = module.direct if hasattr(module, "direct") else module.portable.direct
    lock = threading.Lock()
    def emit(row):
        with lock, (output / "contact.jsonl").open("a") as stream:
            stream.write(json.dumps(row, sort_keys=True) + "\n")
    install_contact_audit(direct, emit, strict=not args.compatibility_audit)
    report = {"scene": args.scene, "argv": argv, "source_commit": provenance["source_commit"],
              "strict": not args.compatibility_audit, "execute_real": False,
              "command_success": False, "verified_task_success": None}
    try:
        result = module.main()
        report.update(status="native_return_unverified", native_return=result,
                      command_success=result in (None, 0))
    except StrictContactNotSupported as exc:
        report.update(status=exc.code, evidence=exc.evidence)
    finally:
        atomic_json(output / "result.json", report)
    print(json.dumps(report, sort_keys=True))
    return 0 if report["command_success"] else 42


if __name__ == "__main__":
    raise SystemExit(main())
