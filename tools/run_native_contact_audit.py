#!/usr/bin/env python3
"""Known Jimu scenes, strict contact audit, no robot motion; no arbitrary argv."""
from __future__ import annotations
import argparse
import contextlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import threading
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rm75_app.workcell.contact_audit import install_contact_audit, StrictContactNotSupported
from rm75_app.workcell.io import atomic_json
from rm75_app.workcell.migration import verify_snapshot
from rm75_app.workcell.native_outcome import NativeOutcomeCapture
from rm75_app.workcell.pickplace_curobo_only import source_adapter,install,install_jimu_binding


def build_native_argv(scene,root,extensions,*,transport_world_checked=False):
    if scene not in ('four-wall','triangle-roof'):raise ValueError('Unknown original Jimu scene')
    name='rm75_jimu_four_wall_portable.py' if scene=='four-wall' else 'rm75_jimu_triangle_roof_apriltag_portable.py'
    path=root/'Beta_demo-codex-v0.9'/name
    fixed=root/'Beta_demo-codex-v0.9/jimu_portable_repro/scenes/jimu_assembly_anchors_default_sam6d.json'
    if not fixed.is_file():raise FileNotFoundError('Original fixed Jimu anchors missing')
    argv=[str(path),'--render-mode','none','--jimu-build-layers','first' if scene=='four-wall' else 'two',
          '--auto-execute','--curobo-torch-extensions-dir',str(extensions),
          '--camera-extrinsic-opencv-path',str(ROOT/'assets/calibration/camera_extrinsic_opencv.npy'),
          '--sam6d-fixed-scene-result-file',str(fixed)]
    if scene=='triangle-roof':argv+=['--jimu-second-layer-triangle-profile','--no-jimu-demo-triangle-apriltag']
    if transport_world_checked:argv+=['--no-fast-chain-cuda-graph-ik']
    return path,argv


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", choices=("four-wall", "triangle-roof"), required=True)
    parser.add_argument("--extensions", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--compatibility-audit", action="store_true")
    modes.add_argument('--tray-final-descent-compatibility', action='store_true',
                       help='Simulation only; left/right fingers vs world ONLY in final vertical tray descent')
    modes.add_argument('--transport-world-checked-compatibility', action='store_true',
                       help='Keep legacy contact stages world-only; fully check loaded transport')
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    extensions = args.extensions.resolve()
    root = ROOT / "rm75_app/_vendor/working_snapshot"
    provenance = verify_snapshot(root)
    path,argv=build_native_argv(args.scene,root,extensions,
        transport_world_checked=args.transport_world_checked_compatibility)
    os.environ["LEROBOT_ROOT"] = str(root)
    sys.argv = argv
    os.chdir(root)
    sys.path.insert(0, str(root))
    sys.path.insert(0, str(path.parent))
    lock = threading.Lock()
    rows=[]
    def emit(row):
        with lock, (output / "contact.jsonl").open("a") as stream:
            rows.append(row)
            stream.write(json.dumps(row, sort_keys=True) + "\n")
    report = {"scene": args.scene, "argv": argv, "source_commit": provenance["source_commit"],
              "strict": not (args.compatibility_audit or args.tray_final_descent_compatibility or args.transport_world_checked_compatibility),
              'transport_world_checked_compatibility': args.transport_world_checked_compatibility,
              'tray_final_descent_compatibility': args.tray_final_descent_compatibility, "execute_real": False,
              "command_success": False, "verified_task_success": None,
              'hardware_connected':False,'camera_captured':False,'planner':'curobo_only',
              'expected_cycles':4 if args.scene=='four-wall' else 12}
    captured=NativeOutcomeCapture(sys.stdout);started=time.monotonic();cleanup=lambda:None
    direct=None
    with source_adapter(root):
        try:
            spec=importlib.util.spec_from_file_location('_contact_audit_native',path)
            module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module
            spec.loader.exec_module(module)
            portable=module if args.scene=='four-wall' else module.portable
            direct=portable.direct
            install_jimu_binding(portable)
            report['clearance_path_audits']=install(direct)
            if args.transport_world_checked_compatibility:
                from curobo_rm75_planner import RM75CuRoboPlanner
                from rm75_app.workcell.transport_contact import (install_transport_contact,
                    install_read_only_jimu_diagnostics,guard_jimu_near_ik,install_jimu_grasp_ik_contact)
                cleanup=install_transport_contact(direct,RM75CuRoboPlanner,emit)
                install_read_only_jimu_diagnostics(portable,emit)
                install_jimu_grasp_ik_contact(direct,emit)
                guard_jimu_near_ik(portable,emit)
                from rm75_app.workcell.jimu_return_diagnostics import install_return_diagnostics,install_release_execution_observer
                install_return_diagnostics(portable,emit)
                from rm75_app.workcell.jimu_release_execution import install_release_execution_guard
                install_release_execution_guard(portable,emit)
                install_release_execution_observer(portable,emit,synchronize=True)
            else:
                install_contact_audit(direct,emit,strict=not args.compatibility_audit,
                    tray_final_descent=args.tray_final_descent_compatibility)
            with contextlib.redirect_stdout(captured):result=module.main()
            report.update(status='native_return_unverified',native_return=result,
                          command_success=result in (None,0) and
                          captured.report(report['expected_cycles'])['native_full_chain_passed'])
        except StrictContactNotSupported as exc:
            report.update(status=exc.code,evidence=exc.evidence)
        except BaseException as exc:
            report.update(status='failed',error=f'{type(exc).__name__}: {exc}',traceback=traceback.format_exc())
        finally:
            try:cleanup()
            except BaseException as exc:
                report.update(command_success=False,status='failed',cleanup_error=f'{type(exc).__name__}: {exc}')
            report.update(captured.report(report['expected_cycles']))
            report['elapsed_s']=time.monotonic()-started
            report['loaded_mplib_modules']=[n for n in sys.modules if n=='mplib' or n.startswith('mplib.')]
            report['clearance_selection_audits']=getattr(direct,'_clearance_selection_audits',[])
            # Portable Jimu replaces the direct dry-run installer. Native
            # release/return markers are NOT an independent execution audit.
            report['independent_clearance_execution_audit_observed']=bool(report.get('clearance_path_audits'))
            report['jimu_release_execution_audits']=getattr(direct,'_jimu_release_execution_audits',[])
            report['transport_path_audits']=[{k:row.get(k) for k in
                ('step_id','samples','payload_spheres','world_exempt_links','scene_fingerprint')}
                for row in rows if row.get('event')=='transport_full_world_audit']
            atomic_json(output/'result.json',report)
    print(json.dumps(report, sort_keys=True))
    return 0 if report["command_success"] else 42


if __name__ == "__main__":
    raise SystemExit(main())
