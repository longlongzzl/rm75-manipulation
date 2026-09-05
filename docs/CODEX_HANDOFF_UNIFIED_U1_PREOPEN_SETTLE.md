# Pre-open planned hold sensitivity

Task: test closed-gripper settling at the original planned endpoint before opening, without a per-object hold-policy exception.
State: NEEDS_REVIEW.
Commit: containing commit (`git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U1_PREOPEN_SETTLE.md`).
Parent commit: ce84703.

Changed files / reasons:

- `tools/diagnose_release_hold.py`: mutually alternative `--preopen-settle-steps` diagnostic. Adds explicit closed-gripper steps at the unchanged planned arm endpoint before invoking the original opening method. Records extra simulated time and before/after joint error. No production default or trajectory points changed.
- `tests/test_release_hold_diagnostic.py`: verifies exact extra step count, unchanged target identity, closed-gripper command, original opening invocation and scoped method restoration.
- Summary JSON and this handoff: full positive/negative timing matrix and safety caveats.

Commands run:
```bash
PYTHONPATH=. /home/zhangzhao/anaconda3/envs/realman/bin/python tools/diagnose_release_hold.py --compiled-dir /tmp/rm75_u1_cache_three_generated00_001 --output-dir /tmp/rm75_u1_preopen_settle_10_001 --open-index 1 --open-index 2 --open-index 3 --preopen-settle-steps 10 > /tmp/rm75_u1_preopen_settle_10_001.log 2>&1
PYTHONPATH=. /home/zhangzhao/anaconda3/envs/realman/bin/python tools/diagnose_release_hold.py --compiled-dir /tmp/rm75_u1_cache_three_generated00_001 --output-dir /tmp/rm75_u1_preopen_settle_5_001 --open-index 1 --open-index 2 --open-index 3 --preopen-settle-steps 5 > /tmp/rm75_u1_preopen_settle_5_001.log 2>&1
PYTHONPATH=. /home/zhangzhao/anaconda3/envs/realman/bin/python tools/diagnose_release_hold.py --compiled-dir /tmp/rm75_u1_cache_three_generated00_001 --output-dir /tmp/rm75_u1_preopen_settle_20_001 --open-index 1 --open-index 2 --open-index 3 --preopen-settle-steps 20 > /tmp/rm75_u1_preopen_settle_20_001.log 2>&1
PYTHONPATH=. python -m pytest tests -q > /tmp/rm75_u1_preopen_settle_tests.log 2>&1
git diff --check
```

Test environment: unchanged CPU Python 3.12.7; realman Python 3.11.14 / ManiSkill / SAPIEN, RM75-MultiObjectTask-v1, pd_joint_pos, 20 Hz, seed 0. Same frozen three-object portable program in all experiments.

Before metrics: 226 tests; production replay fails carrot, observed-hold intervention works only on carrot and regresses tennis/glue when generalized.
After metrics: 227 tests passed, no failures/skips, 19.72 s. Extra 0.25 s before each opening: all three task checks pass, glue 17.3526 mm / 2.92856 degrees, carrot 1.81475 mm / 3.36044 degrees. Extra 0.5 s: all three pass, glue 14.6038 mm / 4.92497 degrees, carrot 6.39464 mm / 3.53062 degrees. Extra 1 s: tennis passes, glue fails at 50.2781 mm / 17.8730 degrees; carrot is not attempted.

Correctness/safety: extra dwell uses current closed-gripper command and original target, no joint/object teleport, no collision/tolerance/friction edits or new production controller behavior. Extra simulation time is explicitly recorded per override (0.75 s total for completed 0.25 s trial, 1.5 s for completed 0.5 s trial, 2 s actually reached in failed 1 s trial). Native gate `simulated_dwell_s` describes original opening only and must not be used alone as total intervention time. All seven arm joints show zero sampled limit excess, but maximum gripper excess persists at 0.001499/0.001391/0.001403 rad for 0.25/0.5/1 s respectively. These are not complete SIM safety passes.

Failures/raw statuses: 1 s experiment exits 2 with glue task failure and explicit warning that not all selected openings were reached. Both successful shorter waits increased maximum arm error during all three extra holds; in the 0.5 s case, errors change approximately 0.08775->0.10671, 0.08292->0.09355, 0.07584->0.08329 rad. Therefore these successes do not demonstrate tracking convergence.

Raw logs: `/tmp/rm75_u1_preopen_settle_{5,10,20}_001/{diagnostic_summary.json,maniskill_gate_result.json}` and matching adjacent `.log` files. Full exact directories and metrics in `benchmarks/unified_scenarios/u1_preopen_settle_summary.json`. Original baseline failures remain intact.

Unexpected observations: waiting longer is not monotonically beneficial, and successful placement does not imply tracking error decreased. Do not hardcode a successful delay from this same fixture and present it as a robust release policy.

Open questions / next work: inspect loaded tracking/contact behavior and whether physical model/controller mismatch explains persistent errors. Measured scene distribution, complete limit/contact gates and physical approval still outstanding. Production untouched; no default delay enabled, no main merge or physics identification. Push remains permission-blocked without new specific destination/payload authorization.
