#!/usr/bin/env python3
"""U4 synthetic held-out prediction audit; no robot or physics simulator."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rm75_app.scenarios.pusht import (
    PushAction, PushTModelParameters, PushTParameterEnsemble,
    PushTPose, PushTState, QuasiStaticPushTModel,
)


def transitions(model, parameters, seed, count):
    rng = np.random.default_rng(seed)
    contacts = model.geometry.candidate_contact_points()
    for _ in range(count):
        state = PushTState(PushTPose(*rng.uniform([-.18, -.18, -np.pi], [.18, .18, np.pi])))
        contact = contacts[int(rng.integers(len(contacts)))]
        inward = -(model._rotation(state.pose.yaw) @ contact)
        inward /= np.linalg.norm(inward)
        angle = rng.uniform(-.6, .6)
        direction = model._rotation(angle) @ inward
        action = PushAction(contact, direction, float(rng.uniform(.012, .045)))
        yield state, action, model.step(state, action, parameters)


def stats(values):
    return dict(mean=float(np.mean(values)), p50=float(np.median(values)),
                p95=float(np.percentile(values, 95)), max=float(np.max(values)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', type=int, default=200)
    parser.add_argument('--output', type=Path, default=Path('/tmp/rm75_u4_sysid'))
    args = parser.parse_args()
    if args.cases < 1:
        parser.error('--cases must be positive')
    args.output.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(20260905)
    suite = []
    for index in range(args.cases):
        parameters = PushTModelParameters(
            friction=float(rng.uniform(.2, .8)), translation_gain=float(rng.uniform(.6, 1.05)),
            rotation_gain=float(rng.uniform(1.8, 4.5)),
            contact_efficiency=float(rng.uniform(.7, 1)), anisotropy=float(rng.uniform(-.18, .18)))
        suite.append(dict(case=index, parameters=asdict(parameters),
                          fit_seed=100000+index, heldout_seed=200000+index))
    manifest = json.dumps(suite, sort_keys=True, indent=2)
    (args.output / 'suite.json').write_text(manifest + '\n')
    model = QuasiStaticPushTModel(workspace_bounds_xy=(-.3, .3, -.3, .3))
    nominal = PushTModelParameters()
    rows, timings = [], []
    with (args.output / 'raw.jsonl').open('w') as stream:
        for case in suite:
            truth = PushTModelParameters(**case['parameters'])
            estimator = PushTParameterEnsemble.default_grid()
            for before, action, after in transitions(model, truth, case['fit_seed'], 20):
                started = time.perf_counter()
                estimator.update(before, action, after, model)
                timings.append(time.perf_counter()-started)
            estimate = estimator.estimate()
            nominal_errors, fitted_errors = [], []
            for before, action, after in transitions(model, truth, case['heldout_seed'], 20):
                nominal_errors.append(model.pose_error(model.step(before, action, nominal), after))
                fitted_errors.append(model.pose_error(model.step(before, action, estimate), after))
            row = dict(**case, estimate=asdict(estimate), nominal_errors_m=nominal_errors,
                       fitted_errors_m=fitted_errors, ess=estimator.effective_sample_size)
            rows.append(row)
            stream.write(json.dumps(row) + '\n')
            stream.flush()
            if (case['case']+1) % 20 == 0:
                print(f"completed {case['case']+1}/{len(suite)}", flush=True)
    baseline = [v for row in rows for v in row['nominal_errors_m']]
    fitted = [v for row in rows for v in row['fitted_errors_m']]
    report = dict(task='U4.2 held-out system identification sub-gate', state='OFFLINE_VERIFIED',
        tested_commit=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        cases=args.cases, fit_transitions_per_case=20, heldout_transitions_per_case=20,
        suite_seed=20260905, suite_sha256=hashlib.sha256(manifest.encode()).hexdigest(),
        nominal_prediction_error_m=stats(baseline), fitted_prediction_error_m=stats(fitted),
        relative_mean_error_reduction=1-float(np.mean(fitted)/np.mean(baseline)),
        improved_cases=sum(bool(np.mean(r['fitted_errors_m']) < np.mean(r['nominal_errors_m'])) for r in rows),
        update_latency_s=stats(timings), effective_sample_size=stats([r['ess'] for r in rows]),
        raw_log=str(args.output / 'raw.jsonl'), manifest=str(args.output / 'suite.json'),
        limitations=['Synthetic noiseless transitions from the same quasi-static model family.',
                    'Held-out action seeds are disjoint from fitting; no tuning performed.',
                    'Prediction improvement is not control improvement or physical validation.',
                    'Four-controller 200-goal benchmark and recorded tracker replay remain unrun.',
                    'Friction, translation gain and efficiency are not individually identifiable from their product in this model.'])
    target = Path('benchmarks/unified_scenarios/u4_pusht_sysid_summary.json')
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
