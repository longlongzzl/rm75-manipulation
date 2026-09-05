# Confirmed stale IK success cache

Task: trace the same grasp candidate through screening and linear planning; diagnose false feasibility without changing restricted production screening.
State: NEEDS_REVIEW.
Commit: containing commit, `git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U1_STALE_CACHE.md`.
Parent commit: 000bf93.
Changed files / reasons: tools/diagnose_linear_solver.py records exact screen/linear poses and screening policies, plus cache presence/latest screen status at linear entry. Adds mutually exclusive --reject-stale-cache diagnostic that removes prior IK cache entries on failed rescreen; production backend untouched. tests/test_linear_solver_trace.py adds two cases proving opt-in eviction and unchanged diagnostic default. Summary JSON and handoff provide evidence for production review.

Commands run: same tools/diagnose_linear_solver.py --plan frozen C1 gluestick path as previous handoff, first without flags to /tmp/rm75_u1_gluestick_candidate_trace_001.json, then without flags to /tmp/rm75_u1_gluestick_cache_baseline_001.json with added cache logging; --reject-stale-cache to /tmp/rm75_u1_gluestick_stale_cache_001.json. Known-success control uses frozen C1 tennis path /tmp/rm75_task001_c1_plans/20260904_154546_llm_pick_place_pid21903_112358180/case_01/manipulation_plan.json with --reject-stale-cache to /tmp/rm75_u1_tennis_stale_cache_control_001.json. All use /home/zhangzhao/anaconda3/envs/curobo2/bin/python; each output has a matching .log. Final regression:
```
PYTHONPATH=. python -m pytest tests -q > /tmp/rm75_u1_stale_cache_final_tests.log 2>&1
git diff --check
```
Test environment: unchanged Python 3.12.7 CPU and curobo2 Python 3.11; no robot or physics run.
Before metrics: 213 tests; 16 apparently complete glue relations leading to eight failed linear attempts. Exact candidate trace shows original full-gripper coarse failure, axis_constrained success with four gripper links temporarily disabled and orientation weights [1,0,1], followed by failure of resolved pose under full-gripper coarse screening.
After metrics: 215 tests passed in 18.74 s, zero failures/skips. At linear entry all 8 candidates have cache_present=true and latest_screen_success=false. Diagnostic invalidation makes complete relation count zero, stops at relation_screen with no linear calls. Known-success tennis control still completes, 3 linear calls. Glue remains infeasible under this run; no recovery claimed.
Correctness/safety checks: source confirms _prepare_pose_candidates_with_solver updates failure metrics but only writes cache on success, never evicting older success. resolve_axis_constrained_pose_candidates installs resolved successful cache; feasible_pose_candidate_ids relies on key presence. Diagnostic eviction honors newest failure without changing tolerances, collision geometry or targets. No production lazy_place alteration, no link exemption added. Earlier failures retained.
Failures/raw statuses: baseline segmented_chain/linear_planner_failed; diagnostic relation_screen rejection. This is a correctness/early-rejection improvement, not a new valid grasp. The disabled-link state was part of pre-existing axis resolution, not introduced in this task; it cannot remain valid evidence after stricter rescreen fails.
Raw log paths: /tmp/rm75_u1_gluestick_candidate_trace_001.json; /tmp/rm75_u1_gluestick_cache_baseline_001.json; /tmp/rm75_u1_gluestick_stale_cache_001.json; /tmp/rm75_u1_tennis_stale_cache_control_001.json; all matching .log files; /tmp/rm75_u1_stale_cache_final_tests.log.
Committed summary path: benchmarks/unified_scenarios/u1_stale_cache_diagnostic.json.
Unexpected observations: initial aggregate feasible relation count was stale-cache evidence, not a matched full-geometry success. This resolves the apparent contradiction without weakening any geometric test.
Open questions / requested direction: docs/CODEX_VALIDATION_UNIFIED_SCENARIOS.md prohibits changing production lazy_place screening during validation. Approve a scoped backend correctness fix that invalidates cached IK on failed rescreen, followed by full frozen regression, before applying it to production. Until approval only diagnostic mode changes behavior. Measured 20-scene suite and push destination approval also remain outstanding. No full SIM or real-stage pass; physics identification disabled.
