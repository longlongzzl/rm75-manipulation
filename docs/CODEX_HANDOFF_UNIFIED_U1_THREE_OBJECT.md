# Three-object full-program diagnostic

Task: step 4/5 broaden beyond one atom using all three historical same-scene goals.
State: NEEDS_REVIEW (complete program compilation failed; no multi-object replay).
Commit: containing commit, `git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U1_THREE_OBJECT.md`.
Parent commit: df9b008.
Changed files / reasons: benchmarks/unified_scenarios/fixtures/sorting_historical_three_objects.json freezes tennis/holder, gluestick/cube and carrot/brush goals copied from TASK001 C1 historical plans before benchmarking. Explicit priorities set tennis, glue, carrot order. Surface goals use existing 0.02 m/20 deg pose criteria; no claim of separate on_top physical validation. tests/test_sorting_three_object_fixture.py checks all assignments/supports/order remain present and synthetic-joint limitation labeled. Summary JSON and handoff preserve failure evidence.

Commands run:
```
/home/zhangzhao/anaconda3/envs/curobo2/bin/python tools/benchmark_sorting_program.py --request benchmarks/unified_scenarios/fixtures/sorting_historical_three_objects.json --initial-joints /tmp/rm75_u1_strict_replay_002/replay_initial_state.json --output-dir /tmp/rm75_u1_three_object_compile_001 > /tmp/rm75_u1_three_object_compile_001.log 2>&1
PYTHONPATH=. python -m pytest tests -q > /tmp/rm75_u1_three_object_tests.log 2>&1
git diff --check
```
Test environment: unchanged Python 3.12.7 CPU tests and curobo2 Python 3.11/GPU production config; actual simulator reset joint observation from prior strict run, not real robot joints.
Before metrics: 209 tests; only one historical atom verified through actual new complete-program pipeline.
After metrics: full 210 passed in 19.04 s, no failures/skips. Three-object compile FAILED in 16.030 s including backend initialization. First atom tennis completed offline planning, revision 0→1. Second atom sort_02_gluestick failed segmented_chain, message `no complete segmented pick-place chain`. Eight attempted grasp candidates each returned linear_planner_failed; segmented planning 2.733 s. Relation screen had 16 complete endpoint relations, insufficient to guarantee an executable chain. Third atom not attempted after failure.
Correctness/safety checks: no failed object removed or target adjusted after benchmark; unrelated scene obstacles remain. No executable full program exported; partial tennis planning was not replayed as success for the three-object task. No robot/simulation actions by compile tool. Production lazy_place/primary_only, collision geometry and tolerances unchanged.
Failures and raw statuses: eight grasp linear failures, start collision diagnostics empty for these attempts; this does not prove collision-free entire paths. Unabridged relation and candidate diagnostics retained. No claim of root cause from endpoint screening alone.
Raw log paths: /tmp/rm75_u1_three_object_compile_001.log; /tmp/rm75_u1_three_object_compile_001/{frozen_inputs,summary}.json; /tmp/rm75_u1_three_object_tests.log.
Committed summary path: benchmarks/unified_scenarios/u1_three_object_compile_summary.json.
Unexpected observations: compiling from the preceding atom's end joints preserves the known gluestick grasp-chain failure despite endpoint feasibility; simple composition of independent goals is not automatically a feasible continuous program.
Open questions / next work: diagnose retained gluestick linear failure against existing frozen baseline before any production planning change. Required 20 measured snapshots with matching targets/current joints are still missing; no synthetic replacement. Multi-object SIM, full contact/limit acceptance and real stages remain incomplete. Push awaits explicit destination authorization after earlier rejection; no retry. Physics identification unstarted.
