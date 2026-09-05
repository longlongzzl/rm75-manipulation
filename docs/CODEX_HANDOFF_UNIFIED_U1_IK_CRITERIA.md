# Native IK error and criteria diagnostic

Task: compare native goal IK failure against endpoint feasibility, test one criteria hypothesis without changing production.
State: NEEDS_REVIEW; diagnostic hypothesis did not recover a path.
Commit: containing commit, `git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U1_IK_CRITERIA.md`.
Parent commit: 1bd64e2.
Changed files / reasons: tools/diagnose_linear_solver.py retains native feasible, position/rotation error and raw tolerance fields. Adds explicit --terminal-only-ik diagnostic mode which sets only native IK to default full terminal pose criteria before each solve, leaving linear TrajOpt conditions and scene/collision/tolerance settings unchanged. Wrapper restores solve method in finally; inherited backend restores criteria. Production code unchanged. tests/test_linear_solver_trace.py verifies raw metric fields are preserved, not reinterpreted. Summary JSON/handoff retain both failures.

Commands run:
```
/home/zhangzhao/anaconda3/envs/curobo2/bin/python tools/diagnose_linear_solver.py --plan /tmp/rm75_task001_c1_plans/20260904_154547_llm_pick_place_pid21961_952932160/case_01/manipulation_plan.json --output /tmp/rm75_u1_gluestick_ik_metrics_001.json > /tmp/rm75_u1_gluestick_ik_metrics_001.log 2>&1
/home/zhangzhao/anaconda3/envs/curobo2/bin/python tools/diagnose_linear_solver.py --terminal-only-ik --plan /tmp/rm75_task001_c1_plans/20260904_154547_llm_pick_place_pid21961_952932160/case_01/manipulation_plan.json --output /tmp/rm75_u1_gluestick_terminal_ik_001.json > /tmp/rm75_u1_gluestick_terminal_ik_001.log 2>&1
PYTHONPATH=. python -m pytest tests -q > /tmp/rm75_u1_ik_criteria_final_tests.log 2>&1
git diff --check
```
Test environment: unchanged CPU Python 3.12.7 and curobo2 Python 3.11; same historical fixture/default joints. Installed source confirms MotionPlanner.update_tool_pose_criteria forwards to IK and TrajOpt; diagnostic tests separating them only. Tensor observation synchronizes, so no latency claim.
Before metrics: 212 tests; 24 native IK calls fail before TrajOpt, error reason unmeasured.
After metrics: full 213 passed in 18.73 s, no failures/skips. Baseline 384 returned entries: 370 constraint-feasible, 14 infeasible, all unsuccessful. Position error spans 0.0311235–0.0656287 m, versus configured 0.005 m threshold; rotation error 0.012029–0.078168 rad. Raw result tolerance fields are default zero, not evidence of zero configured tolerance. Terminal-only IK diagnostic gives same feasibility counts/error range and zero successes, no recovery.
Correctness/safety checks: no increased seeds/tolerances, removed obstacles or disabled gripper links. Native IK remains full position/orientation endpoint solving; trajectory linear constraints not removed. Diagnostic switch is opt-in and confined to standalone tool, not runtime. No robot/simulation execution.
Failures/raw statuses: both runs retain eight linear_planner_failed candidates/24 unsuccessful native IK calls. Position convergence alone disqualifies every result; some are also constraint-infeasible. Do not claim all failures are collision-only or a trajectory optimizer issue. No fix is inferred from the negative criteria experiment.
Raw log paths: /tmp/rm75_u1_gluestick_ik_metrics_001.json and .log; /tmp/rm75_u1_gluestick_terminal_ik_001.json and .log; /tmp/rm75_u1_ik_criteria_final_tests.log.
Committed summary path: benchmarks/unified_scenarios/u1_ik_criteria_diagnostic.json.
Unexpected observations: separating endpoint criteria from path criteria did not alter the observed result distribution; this hypothesis does not explain/recover the failure in this fixture.
Open questions / next work: compare native solver current-state/seed/cost configuration against endpoint screen without weakening safety. Multi-object program remains uncompiled; measured 20-scene suite, full SIM and all real stages remain outstanding. Push still awaiting explicit destination approval after rejection; no retry. Physics identification disabled.
