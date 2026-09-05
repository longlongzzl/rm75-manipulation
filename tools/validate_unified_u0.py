#!/usr/bin/env python3
"""Reproduce U0 offline gates and retain subprocess logs and JSON evidence."""
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    logs = Path('/tmp/rm75_unified_u0')
    logs.mkdir(exist_ok=True)
    env = {**os.environ, 'PYTHONPATH': str(ROOT)}
    cli = [sys.executable, 'tools/run_unified_scenario.py']
    catalog = ['--catalog', 'configs/scenarios/magnetic_panel_catalog.example.json',
               '--inventory', 'configs/scenarios/magnetic_inventory.example.json']
    checks = [
        ('compileall', [sys.executable, '-m', 'compileall', '-q', 'rm75_app/scenarios']),
        ('lightweight', [sys.executable, '-m', 'pytest', '-q', *[
            'tests/' + name for name in ('test_scenario_program_runner.py',
            'test_sorting_scenario.py', 'test_magnetic_assembly.py',
            'test_magnetic_pickplace_adapter.py', 'test_pusht_scenario.py',
            'test_pickplace_program.py')]]),
        ('full', [sys.executable, '-m', 'pytest', 'tests', '-q']),
        ('help', cli + ['--help']),
        ('sorting', cli + ['sorting-compile', '--request',
            'configs/scenarios/sorting.example.json', '--output', str(logs / 'sorting.json')]),
        ('magnetic_generate', cli + ['magnetic-generate', *catalog,
            '--description', '搭一面能稳定站立的磁吸墙', '--output', str(logs / 'wall.json')]),
        ('magnetic_validate', cli + ['magnetic-validate', *catalog,
            '--assembly', str(logs / 'wall.json'), '--output', str(logs / 'wall_validation.json')]),
        ('pusht', cli + ['pusht-sim', '--goal-x', '0.08', '--goal-y', '0.02',
            '--goal-yaw', '0.0', '--system-identification', '--output', str(logs / 'pusht.json')]),
    ]
    results = []
    for name, command in checks:
        started = time.perf_counter()
        result = subprocess.run(command, cwd=ROOT, env=env, text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        path = logs / (name + '.log')
        path.write_text(result.stdout)
        results.append(dict(name=name, command=command, returncode=result.returncode,
                            elapsed_s=time.perf_counter()-started, log=str(path)))
        print(name, result.returncode, flush=True)
    report = dict(task='U0', tested_commit=subprocess.check_output(
        ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        python=platform.python_version(), executable=sys.executable,
        libraries={name: importlib.metadata.version(name) for name in ('numpy', 'scipy', 'pytest')},
        checks=results, failures=[r['name'] for r in results if r['returncode']],
        calibration='Example assets only; no physical or cuRobo validation claimed.')
    for name, filename in [('magnetic', 'wall_validation.json'), ('pusht', 'pusht.json')]:
        path = logs / filename
        if path.exists():
            report[name] = json.loads(path.read_text())
    target = ROOT / 'benchmarks/unified_scenarios/u0_offline_summary.json'
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    return int(bool(report['failures']))


if __name__ == '__main__':
    raise SystemExit(main())
