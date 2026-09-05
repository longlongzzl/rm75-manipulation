# Release arm-hold policy counterexamples

Task: isolate whether continuing to track the planned arm endpoint during gripper opening causes release disturbance; test generalization before changing production behavior.
State: NEEDS_REVIEW.
Commit: containing commit (`git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U1_RELEASE_HOLD.md`).
Parent commit: 7f769c4.

Changed files / reasons:

- `tools/diagnose_release_hold.py`: explicit opt-in standalone simulation intervention at numbered opening events. Temporarily holds measured arm joints for the selected dwell, then restores the planned endpoint reference in finally. Does not alter production source/defaults, trajectory points, other events or dwell duration. Multiple indices supported with nested scoped wrappers; report identifies modified commands and refuses to claim all interventions ran when a failed atom stops execution first.
- `tests/test_release_hold_diagnostic.py`: three tests cover event selectivity, restoration after exceptions, and nested selected openings without changing close commands or leaking methods/targets.
- Summary JSON and this handoff: paired outcome and negative controls.

Commands run (all use the same frozen full three-atom program):
```bash
PYTHONPATH=. /home/zhangzhao/anaconda3/envs/realman/bin/python tools/diagnose_release_hold.py --compiled-dir /tmp/rm75_u1_cache_three_generated00_001 --output-dir /tmp/rm75_u1_release_hold_observed_001 --open-index 3 > /tmp/rm75_u1_release_hold_observed_001.log 2>&1
PYTHONPATH=. /home/zhangzhao/anaconda3/envs/realman/bin/python tools/diagnose_release_hold.py --compiled-dir /tmp/rm75_u1_cache_three_generated00_001 --output-dir /tmp/rm75_u1_release_hold_all_open_001 --open-index 1 --open-index 2 --open-index 3 > /tmp/rm75_u1_release_hold_all_open_001.log 2>&1
PYTHONPATH=. /home/zhangzhao/anaconda3/envs/realman/bin/python tools/diagnose_release_hold.py --compiled-dir /tmp/rm75_u1_cache_three_generated00_001 --output-dir /tmp/rm75_u1_release_hold_surface_open_001 --open-index 2 --open-index 3 > /tmp/rm75_u1_release_hold_surface_open_001.log 2>&1
PYTHONPATH=. python -m pytest tests -q > /tmp/rm75_u1_release_hold_final_tests.log 2>&1
git diff --check
```

Test environment: unchanged CPU Python 3.12.7; realman Python 3.11.14, ManiSkill/SAPIEN, RM75-MultiObjectTask-v1, pd_joint_pos, 20 Hz, seed 0. No planner invoked during replay, no real robot sink.

Before metrics: 223 tests; production replay passes tennis/glue, fails carrot at 138.048 mm / 33.146 degrees after 55.7217 mm opening displacement.
After metrics: 226 tests passed, no failures/skips, 18.93 s. Only third opening overridden: three task checks pass, carrot error 1.45056 mm / 2.54737 degrees; opening displacement 2.04487 mm. Held command differs from original endpoint by max 0.0757096 rad. Existing gripper limit excess 0.00133544 rad persists, so not full SIM_VERIFIED.

Counterexamples / failures: requesting all three opening overrides fails tennis at 147.584 mm after six stages, so only the first intervention ran. Requesting surface openings 2 and 3 retains tennis but fails glue at 75.8187 mm / 2.41787 degrees after twelve stages, so only opening 2 was overridden. Both tools exit 2 and retain the task failure plus an explicit not-all-selected-openings-reached error. These errors do not mean a crash before the logged physical task failure. Original unchanged replay still fails carrot.

Correctness/safety: no production mode enabled, object-specific exception added, collision or tolerance weakened, geometry changed, or failed object removed. All seven arm joints show zero sampled limit excess in the three runs, but gripper exceptions persist and sampling is control-boundary only. Modifying dwell arm commands makes these diagnostic interventions, not strict unmodified portable-program validation, even though timing checks and no-initial-teleport validation still run internally.

Raw logs: each `/tmp/rm75_u1_release_hold_{observed,all_open,surface_open}_001` contains `diagnostic_summary.json`, `maniskill_gate_result.json`, gripper boundary trace and contact/limit samples; matching adjacent `.log` files retain environment output. Exact paths and metrics are committed in `benchmarks/unified_scenarios/u1_release_hold_summary.json`.

Unexpected observations: a highly effective local carrot intervention is harmful to the other objects when generalized globally or even by surface placement mode. Do not hardcode the successful case into production.

Open questions / next work: investigate residual velocity/contact settling before release and convergence criteria rather than selecting an arm-hold rule solely from one object. Full measured 20-scene suite, collision/limit gates and explicit physical approval remain required. Push remains authorization-blocked; no retry without new destination/payload approval. No main merge or physics identification.
