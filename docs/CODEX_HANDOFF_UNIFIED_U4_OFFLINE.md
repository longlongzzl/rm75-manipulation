# U4 pure-model benchmark handoff

Task: U4.2 four-policy control and held-out parameter-estimation experiments
State: OFFLINE_VERIFIED
Commit: containing commit, resolvable with `git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U4_OFFLINE.md`
Parent commit: 4fab39519d042327c0e78366d8e0f474bf03d013

Changed files and reasons:
- tools/benchmark_pusht_control.py: reproducible 200-goal suite and four policies, retaining every action and failure.
- tools/benchmark_pusht_sysid.py: 200 random parameter conditions, 20 fitting plus 20 held-out transitions each; separate action seeds.
- benchmarks/unified_scenarios/u4_pusht_sim_summary.json: control results and limitations.
- benchmarks/unified_scenarios/u4_pusht_sysid_summary.json: prediction, update latency, and effective sample size results.
- this handoff: evidence interpretation and remaining gates.

Commands run:
```
python tools/benchmark_pusht_sysid.py
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python tools/benchmark_pusht_control.py
PYTHONPATH=. python -m pytest tests/test_pusht_scenario.py tests/test_pusht_tracking.py -q
PYTHONPATH=. python -m pytest tests -q
```

Environment: same Python 3.12.7 / NumPy 1.26.4 environment as U0. Four independent CPU processes for control benchmarking; timings include CPU contention. No GPU, robot, external API, or ManiSkill call. Existing planning/model production defaults were not changed.

Before metrics: no committed 200-case control or held-out sysid evidence existed. Strategies were compared on exactly the same frozen starts/goals; no tuning or case removal occurred.

After metrics:

| Policy | Success / 200 | Mean pushes (all cases) | Planning P95 |
|---|---:|---:|---:|
| Direct boundary contact aiming through center toward goal | 38 | 25.50 | 0.071 ms |
| One-step greedy, 384 candidates | 200 | 9.205 | 24.53 ms |
| H=3 MPC, 384 sequences | 199 | 11.155 | 70.10 ms |
| H=3 MPC + weighted parameter estimate | 199 | 11.075 | 70.60 ms |

Control cases: 40 each translation, rotation, combined, boundary, parameter mismatch; 30-push limit; unchanged 15 mm / 8 degree goal tolerances. Combined successes: direct 2/40, all search policies 40/40. Both MPC failures occurred in the mismatch group. No failed samples were discarded.

Held-out prediction: 4,000 unseen transitions; nominal mean composite pose error 3.305 mm versus fitted 1.573 mm (52.4% decrease); 188/200 parameter cases improved, 12 did not. Error uses the existing model's yaw-to-distance scale of 0.06 m/rad and is not pure translation error. Fitting update P95 4.47 ms. Effective sample size is included in the committed summary.

Correctness checks: fit/action-test seeds are disjoint, random truth parameters include values outside the estimator grid, every control policy sees the same case and true model parameters, initial parameter estimator resets between cases. Raw step records contain best/median candidate costs for search policies, stalls/regressions, prediction errors, actions, and poses. Controller fitting uses the existing weighted parameter estimate, not a new robust multi-hypothesis rollout algorithm.

Failures and raw statuses: initial benchmark-only JSON serialization failures for NumPy int64/bool_ were fixed and both experiments rerun successfully. Existing Push-T/tracking tests: 7 passed. Full repository regression: 163 passed in 19.10s.

Raw log paths:
- /tmp/rm75_u4_control/suite.json
- /tmp/rm75_u4_control/raw.jsonl (800 complete episodes)
- /tmp/rm75_u4_sysid/suite.json
- /tmp/rm75_u4_sysid/raw.jsonl (200 complete parameter cases)

Committed summary paths: benchmarks/unified_scenarios/u4_pusht_sim_summary.json and u4_pusht_sysid_summary.json.

Unexpected observations / optimization decision: one-step search matches or exceeds H=3 success while using fewer pushes and less planning time. The existing H=3 policy does not establish a many-step advantage on this distribution. Fitting helps synthetic prediction but not observed control success; it remains optional. No production strategy was switched based on a single synthetic suite.

Limits: this is noise-free quasi-static simulation within the same model family, not contact physics validation. The model clips only the object center at workspace bounds; boundary goals retain radius margin but no claim of full-body workspace compliance is made. Friction, translation gain and contact efficiency are confounded in the model, so improved prediction does not imply recovery of individual physical values.

Remaining tasks: U4.1 real coordinate calibration and U4.3 recorded tracker replay are not verified. U4 as a whole remains PROPOSED. U1 real snapshots/joints and U2 panel calibration are still missing; no downstream U3/U5 physical or simulation gates are claimed complete. A robust ensemble-rollout policy, if desired instead of the existing weighted estimate, needs its own implementation and evidence.

Open questions for ChatGPT: accept this as the initial U4.2 synthetic baseline? Evaluate greedy H=1 as a candidate operating point on a separate held-out goal seed before changing runtime defaults. Supply tracker replay and calibrated scene/pusher inputs for the remaining gates.
