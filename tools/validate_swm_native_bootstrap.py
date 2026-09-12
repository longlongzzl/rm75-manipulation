#!/usr/bin/env python3
"""Bounded native initialization/readback evidence, NOT atomic task acceptance.

Run only through tools/run_network_isolated.py. No episode, camera, gripper or
arm command is invoked. Uses the original frozen desktop, not a new fixture.
"""
from __future__ import annotations

import argparse
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--probe-planner', action='store_true',
        help='Initialize the existing shared cuRobo2 backend in the SAME process; never plan or execute')
    options = parser.parse_args()
    # Enforce the SAME network guard again before native imports. A generic
    # Seccomp=2 flag alone does not prove non-Unix sockets are denied.
    from tools.run_network_isolated import block_network
    block_network()
    from rm75_app.workcell.events import EventLog, StopToken
    from rm75_app.workcell.legacy import (snapshot_root, import_working_entry, build_native_argv,
                                         PICKPLACE_WORLD_ENTRY)
    from rm75_app.workcell.native_frozen_world import read_contract
    from rm75_app.workcell.pickplace_curobo_only import source_adapter
    from rm75_app.swm.native_bootstrap import initialize_frozen_primary

    output = options.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    profile = json.loads(options.profile.read_text())
    section = profile['pickplace']
    if section.get('fixed_scene_format') != 'native_world' or not section.get('fixed_scene'):
        raise ValueError('Existing original frozen-world profile required')
    section['render_mode'] = 'none'
    root = snapshot_root(ROOT)
    fixed = Path(section['fixed_scene'])
    if not fixed.is_absolute():
        fixed = root / fixed
    contract = read_contract(fixed, 'bi')
    spec = dict(task='pickplace', mode='sim', parameters=dict(object_name='bi'))
    manifest = dict(schema='rm75.swm_native_bootstrap_input_v1', specification=spec,
        input_sha256=contract['sha256'], input_path=str(fixed), object_ids=list(contract['names']),
        profile_sha256=hashlib.sha256(options.profile.read_bytes()).hexdigest(),
        probe_planner=options.probe_planner, actual_python=sys.executable,
        effective_profile=profile, hardware_connected=False, atomic_worker_qualified=False)
    (output / 'input.json').write_text(json.dumps(manifest, indent=2) + '\n')
    events, stop = EventLog(output), StopToken(output / 'STOP')
    report = dict(status='failed', hardware_connected=False, atomic_worker_qualified=False,
                  planner_executed=False, motion_executed=False, captures=[], planner_initialized=False)
    world = None
    try:
        with ExitStack() as resources:
            resources.enter_context(source_adapter(root))
            old_argv, old_cwd, old_path = sys.argv[:], Path.cwd(), sys.path[:]
            resources.callback(lambda: setattr(sys, 'argv', old_argv))
            resources.callback(os.chdir, old_cwd)
            resources.callback(lambda: setattr(sys, 'path', old_path))
            os.chdir(root)
            direct = import_working_entry(root, 'pickplace', entrypoint=PICKPLACE_WORLD_ENTRY)
            argv = build_native_argv(direct, spec, profile, output, root, frozen_contract=contract)
            sys.argv = [str(root / PICKPLACE_WORLD_ENTRY), *argv]
            args = direct.parse_args()
            args.foundationpose_refine_after_render = False
            world = initialize_frozen_primary(resources, direct, args, contract, stop=stop, events=events,
                artifact_directory=output / "initialization_artifacts")
            for _ in range(2):
                report['captures'].append(world.read_state())
            if options.probe_planner:
                from rm75_app.planning.backends.curobo2 import Curobo2Backend, Curobo2BackendConfig
                from rm75_app.planning.contracts import PlanningScene
                backend = resources.enter_context(Curobo2Backend(Curobo2BackendConfig()))
                # Initialization readiness only: an empty initialization scene
                # is NEVER an audited task scene or permission to execute.
                backend.update_scene(PlanningScene())
                planner = backend._ensure_planner()
                report['planner_initialized'] = True
                report['planner_joint_names'] = list(planner.joint_names)
                report['planner_scene_qualified'] = False
                events.emit('swm_shared_planner_initialized', hardware_connected=False,
                            task_scene_qualified=False)
            report['status'] = 'initialized_and_read' 
        report['primary_closed'] = world.closed
    except BaseException as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
        (output / 'traceback.txt').write_text(traceback.format_exc())
    finally:
        report['primary_closed'] = None if world is None else world.closed
        (output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report), flush=True)
    return 0 if report['status'] == 'initialized_and_read' else 1


if __name__ == '__main__':
    raise SystemExit(main())
