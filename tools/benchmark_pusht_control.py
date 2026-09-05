#!/usr/bin/env python3
"""Frozen 200-goal U4 quasi-static comparison, never a physical robot test."""
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rm75_app.scenarios.pusht import (
    PushAction, PushTGoal, PushTModelParameters, PushTMPC, PushTMPCConfig,
    PushTParameterEnsemble, PushTPose, PushTState, QuasiStaticPushTModel,
)
from rm75_app.scenarios.pusht.contracts import wrap_angle
from benchmark_pusht_sysid import stats


def run(item):
    case, policy = item
    model = QuasiStaticPushTModel(workspace_bounds_xy=(-.3, .3, -.3, .3))
    state = PushTState(PushTPose(*case['start']))
    goal = PushTGoal(PushTPose(*case['goal']))
    truth = PushTModelParameters(**case['parameters'])
    nominal = PushTModelParameters()
    ensemble = PushTParameterEnsemble.default_grid() if policy == 'mpc_fit' else None
    config = PushTMPCConfig(horizon=1 if policy == 'greedy' else 3, seed=300000+case['id'])
    planner = PushTMPC(model, config)
    steps = []
    for index in range(30):
        if goal.reached(state):
            break
        params = ensemble.estimate() if ensemble is not None else nominal
        started = time.perf_counter()
        best_cost = median_cost = None
        if policy == 'direct':
            direction = goal.pose.xy - state.pose.xy
            norm = np.linalg.norm(direction)
            direction = direction / norm if norm > 1e-9 else np.array([1., 0.])
            # Select a real boundary contact closest to a line through the COM.
            contact = min(model.geometry.candidate_contact_points(),
                          key=lambda p: np.dot(model._rotation(state.pose.yaw) @ p, direction)
                          / np.linalg.norm(p))
            action = PushAction(contact, direction, float(np.clip(norm, .012, .045)))
        else:
            plan = planner.plan(state, goal, params)
            action = plan.action
            best_cost = plan.cost
            median_cost = plan.diagnostics['cost_median']
        latency = time.perf_counter()-started
        predicted = model.step(state, action, params)
        after = model.step(state, action, truth)
        before_error = model.pose_error(state, PushTState(goal.pose))
        after_error = model.pose_error(after, PushTState(goal.pose))
        update_time = 0.
        if ensemble is not None:
            started = time.perf_counter()
            ensemble.update(state, action, after, model)
            update_time = time.perf_counter()-started
        steps.append(dict(index=index, planning_s=latency, update_s=update_time,
                          best_cost=best_cost, median_cost=median_cost,
                          prediction_error_m=model.pose_error(predicted, after),
                          stall=abs(after_error-before_error)<1e-4,
                          regression=after_error>before_error+1e-4,
                          pose=after.pose.vector().tolist(),
                          action=dict(contact=action.contact_local_xy.tolist(),
                                      direction=action.direction_world_xy.tolist(), distance=action.distance_m)))
        state = after
    return dict(case=case['id'], group=case['group'], policy=policy, success=bool(goal.reached(state)),
                pushes=len(steps), position_error_m=float(np.linalg.norm(state.pose.xy-goal.pose.xy)),
                yaw_error_rad=abs(wrap_angle(state.pose.yaw-goal.pose.yaw)), steps=steps,
                ess=None if ensemble is None else ensemble.effective_sample_size)


def main():
    out = Path('/tmp/rm75_u4_control')
    out.mkdir(exist_ok=True)
    rng = np.random.default_rng(20260906)
    groups = ['translation', 'rotation', 'combined', 'boundary', 'mismatch']
    cases = []
    for i in range(200):
        group = groups[i//40]
        start = rng.uniform([-.14, -.14, -np.pi], [.14, .14, np.pi])
        goal = rng.uniform([-.14, -.14, -np.pi], [.14, .14, np.pi])
        if group == 'translation':
            goal[2] = start[2]
        if group == 'rotation':
            goal[:2] = start[:2]
        if group == 'boundary':
            start[0], goal[0] = rng.choice([-.18, .18], size=2)
        params = PushTModelParameters()
        if group == 'mismatch':
            params = PushTModelParameters(friction=float(rng.uniform(.2,.8)),
                translation_gain=float(rng.uniform(.6,1.05)), rotation_gain=float(rng.uniform(1.8,4.5)),
                contact_efficiency=float(rng.uniform(.7,1.)), anisotropy=float(rng.uniform(-.18,.18)))
        cases.append(dict(id=i, group=group, start=start.tolist(), goal=goal.tolist(), parameters=asdict(params)))
    (out/'suite.json').write_text(json.dumps(cases, indent=2)+'\n')
    policies = ['direct', 'greedy', 'mpc', 'mpc_fit']
    rows = []
    with ProcessPoolExecutor(max_workers=4) as pool, (out/'raw.jsonl').open('w') as log:
        for row in pool.map(run, [(c,p) for c in cases for p in policies], chunksize=1):
            rows.append(row)
            log.write(json.dumps(row)+'\n')
            log.flush()
            if len(rows)%40 == 0:
                print(f'completed {len(rows)}/800', flush=True)
    metrics = {}
    for policy in policies:
        metrics[policy] = {}
        for group in ['all', *groups]:
            selected = [r for r in rows if r['policy']==policy and (group=='all' or r['group']==group)]
            steps = [s for r in selected for s in r['steps']]
            metrics[policy][group] = dict(cases=len(selected), success_count=sum(r['success'] for r in selected),
                pushes=stats([r['pushes'] for r in selected]), position_error_m=stats([r['position_error_m'] for r in selected]),
                yaw_error_rad=stats([r['yaw_error_rad'] for r in selected]),
                planning_latency_s=stats([s['planning_s'] for s in steps]),
                prediction_error_m=stats([s['prediction_error_m'] for s in steps]),
                stalls=sum(s['stall'] for s in steps), regressions=sum(s['regression'] for s in steps))
    report = dict(task='U4.2 synthetic four-policy comparison', cases=200, max_pushes=30,
        seed=20260906, tuning_performed=False, mpc_candidates=384, mpc_horizon=3,
        position_tolerance_m=.015, yaw_tolerance_deg=8, workers=4, metrics=metrics,
        raw_log=str(out/'raw.jsonl'), suite=str(out/'suite.json'),
        limitations=['Same quasi-static model family; not ManiSkill or real dynamics.',
            'MPC+fit uses the existing weighted parameter estimate, not robust ensemble rollout.',
            'CPU latency measured with four concurrent workers; no real-time acceptance threshold specified.',
            'Boundary starts/goals retain object-radius margin; model clips object center only.',
            'Tracker replay, real coordinate calibration and U5 remain unverified.'])
    Path('benchmarks/unified_scenarios/u4_pusht_sim_summary.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({p:{g:metrics[p][g]['success_count'] for g in ['all',*groups]} for p in policies}))


if __name__ == '__main__':
    main()
