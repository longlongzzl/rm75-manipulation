# Gluestick native failure localization

Task: distinguish sequential-program regression from existing single-atom failure, locate native solver stage.
State: NEEDS_REVIEW (diagnostic evidence, no recovery).
Commit: containing commit, `git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U1_NATIVE_TRACE.md`.
Parent commit: c056b68.
Changed files / reasons: tools/diagnose_linear_solver.py runs the old frozen single-atom input through production coordinator/backend using a diagnostic subclass. Temporarily wraps native IK, trajectory optimization and graph-seed calls only during linear planning, records returned success counts, passes arguments/results unchanged and restores methods in finally. tests/test_linear_solver_trace.py verifies return identity/arguments/restoration including exceptions. Summary and handoff retain comparison evidence. No production planner changes.

Commands run:
```
/home/zhangzhao/anaconda3/envs/curobo2/bin/python tools/benchmark_grasp_relation_screen.py --plan /tmp/rm75_task001_c1_plans/20260904_154547_llm_pick_place_pid21961_952932160/case_01/manipulation_plan.json --full-chain --repetitions 1 --output-jsonl /tmp/rm75_u1_gluestick_control_001.jsonl --summary-json /tmp/rm75_u1_gluestick_control_001_summary.json > /tmp/rm75_u1_gluestick_control_001.log 2>&1
/home/zhangzhao/anaconda3/envs/curobo2/bin/python tools/diagnose_linear_solver.py --plan /tmp/rm75_task001_c1_plans/20260904_154547_llm_pick_place_pid21961_952932160/case_01/manipulation_plan.json --output /tmp/rm75_u1_gluestick_native_trace_001.json > /tmp/rm75_u1_gluestick_native_trace_001.log 2>&1
PYTHONPATH=. python -m pytest tests -q > /tmp/rm75_u1_native_trace_tests.log 2>&1
git diff --check
```
Test environment: unchanged CPU Python 3.12.7 and curobo2 Python 3.11; production lazy_place/primary_only, planner-default fixture joints. This is a same-checkout old-entrypoint control, not a checkout-of-main comparison. Instrumentation synchronizes result tensors for inspection, so its timings are not optimization evidence.
Before metrics: 210 tests; three-object failure at second atom with coarse feasible relations but no complete grasp path.
After metrics: old single-atom entry also fails segmented_chain, 8 grasp candidates linear_planner_failed, 16 complete endpoint relations, no execution stages. Reverse/tool-axis retries unused. Instrumented run: same 8 failed linear candidates; 3 native IKSolver.solve_pose calls each, 24 total, each has 0 successes out of 16 returned boolean entries. No graph-seed or TrajOpt solve call observed inside these attempts. Full tests 212 passed in 18.82 s, no failures/skips.
Correctness/safety checks: no arguments, collision, tolerance, seed or candidate settings changed; no physical sink, graph/trajectory calls not fabricated. Wrappers restore originals on exception. Counts refer to returned boolean entries, not inferred count of optimizer seeds. No new runtime fallback enabled.
Failures/raw statuses: both control and trace retain failure. Source inspection of local MotionPlanner single-goal code shows IK-zero-success retries occur before graph/trajopt; observation supports that path for this run. Does not establish geometric infeasibility or identify why screening/native IK differ.
Raw log paths: /tmp/rm75_u1_gluestick_control_001.jsonl; /tmp/rm75_u1_gluestick_control_001_summary.json; /tmp/rm75_u1_gluestick_control_001.log; /tmp/rm75_u1_gluestick_native_trace_001.json and .log; /tmp/rm75_u1_native_trace_tests.log.
Committed summary path: benchmarks/unified_scenarios/u1_gluestick_native_trace_summary.json.
Unexpected observations: linear_planner_failed here occurs before trajectory optimization; increasing trajectory effort would not address the observed stage. The new multi-atom layer is not necessary to reproduce failure.
Open questions / next work: compare native goal IK versus successful endpoint screen criteria, geometry/state and seeds without relaxing safety. Keep 20 measured snapshots and push destination approval outstanding. No multi-object SIM or REAL pass, no physics identification.
