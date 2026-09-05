# Goal IK current-state diagnostic

Task: test whether native goal IK conditioning on current joints explains screen/native discrepancy.
State: NEEDS_REVIEW (negative recovery result).
Commit: containing commit, `git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U1_IK_CURRENT.md`.
Parent commit: 89d4778.
Changed files / reasons: tools/diagnose_linear_solver.py adds --unconditioned-ik as a mutually exclusive diagnostic alternative to --terminal-only-ik. Only native goal IK receives current_state=None; actual trajectory planning retains its original request current state. Wrapper restores original method even on errors, rejects unexpected positional current-state API rather than silently overriding it. Summary JSON/handoff retain result. No production file changed.

Commands run:
```
/home/zhangzhao/anaconda3/envs/curobo2/bin/python tools/diagnose_linear_solver.py --unconditioned-ik --plan /tmp/rm75_task001_c1_plans/20260904_154547_llm_pick_place_pid21961_952932160/case_01/manipulation_plan.json --output /tmp/rm75_u1_gluestick_unconditioned_001.json > /tmp/rm75_u1_gluestick_unconditioned_001.log 2>&1
PYTHONPATH=. python -m pytest tests -q > /tmp/rm75_u1_unconditioned_tests.log 2>&1
git diff --check
```
Test environment: unchanged CPU Python 3.12.7 and curobo2 Python 3.11; frozen old single-atom input, default fixture joints. Installed source confirms _solve_endpoint_pose_batch already uses current_state=None; native MotionPlanner supplies current_state to goal IK. This justifies testing the difference, not enabling it in production.
Before metrics: 213 tests; baseline native IK 24 calls, zero successes, 370/384 constraint-feasible, position errors 0.0311235–0.0656287 m.
After metrics: 213 tests passed in 18.37 s, zero failed/skipped. Diagnostic still fails all eight linear candidates with 24 zero-success IK calls. Constraint-feasible entries 376/384; position errors 0.0310103–0.0490486 m, still above 0.005 m configured target tolerance. No trajectory recovery.
Correctness/safety checks: no collision geometry, object exemptions, pose threshold, seed count or trajectory start change. Diagnostic mode is explicit and standalone; cannot combine two hypotheses via CLI flags. No physical or simulation command. Do not interpret removal of a goal-IK state condition as permission to omit dynamic path validation.
Failures/raw statuses: eight linear_planner_failed, full single-atom segmented_chain failure. Changes in error/feasibility show current-state input has an effect, but are not success and do not establish the root cause.
Raw log paths: /tmp/rm75_u1_gluestick_unconditioned_001.json and .log; /tmp/rm75_u1_unconditioned_tests.log.
Committed summary path: benchmarks/unified_scenarios/u1_ik_current_state_diagnostic.json.
Unexpected observations: modest error improvement with no valid IK; position convergence remains bottleneck. No runtime change adopted from negative evidence.
Open questions / next work: continue comparing actual solver/candidate state against endpoint results; no claim that more trajectory optimization solves this IK failure. Measured 20-scene suite, full multi-object SIM and real gates remain incomplete. Push destination approval still outstanding after rejection; no retry. Physics identification remains disabled.
