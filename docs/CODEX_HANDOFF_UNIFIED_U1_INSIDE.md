# Explicit sorting success relation

Task: restore explicit containment semantics lost by the sorting frontend; independent fixed-scene compile/replay.
State: NEEDS_REVIEW (single physical containment check passed; full SIM acceptance unproven).
Commit: containing commit, `git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U1_INSIDE.md`.
Parent commit: 5be15db.

Changed files / reasons:
- rm75_app/scenarios/sorting.py: SortingTarget.success_relation explicitly supports target_pose (default) and inside. Validate required/existing support instance and reject self-containment. Preserve per-target relation through move/buffer compilation; do not infer containment from all drop_place commands.
- rm75_app/scenarios/sorting_io.py: roundtrip explicit success relation, backwards-compatible default.
- tests/test_sorting_success_relation.py: 6 cases including roundtrip/buffer distinction, unchanged legacy requests, invalid relation, absent/missing/self support.
- rm75_app/validation/maniskill_gate.py: retain validation relation/diagnostics and observed support pose. Existing bridge already evaluates containment against the observed support after settling, not stale planned pose.
- benchmarks/unified_scenarios/fixtures/sorting_historical_tennis_inside.json: separate frozen request restoring original historical inside intent; same pose/tolerance as previous fixture. Previous request and its failed trials unchanged.
- benchmarks/unified_scenarios/u1_inside_semantics_summary.json and this handoff: evidence and limitations.

Commands run:
```
PYTHONPATH=. python -m pytest tests/test_sorting_success_relation.py tests/test_maniskill_task_bridge.py -q
/home/zhangzhao/anaconda3/envs/curobo2/bin/python tools/benchmark_sorting_program.py --request benchmarks/unified_scenarios/fixtures/sorting_historical_tennis_inside.json --initial-joints /tmp/rm75_u1_strict_replay_002/replay_initial_state.json --output-dir /tmp/rm75_u1_inside_compile_001 > /tmp/rm75_u1_inside_compile_001.log 2>&1
/home/zhangzhao/anaconda3/envs/realman/bin/python tools/replay_sorting_program.py --compiled-dir /tmp/rm75_u1_inside_compile_001 --output-dir /tmp/rm75_u1_inside_replay_001 > /tmp/rm75_u1_inside_replay_001.log 2>&1
PYTHONPATH=. python -m pytest tests -q > /tmp/rm75_u1_inside_final_tests.log 2>&1
git diff --check
```
Test environment: unchanged CPU Python 3.12.7, curobo2 planner and realman ManiSkill Python 3.11 environments; strict no-teleport replay at actual 20 Hz control rate.
Before metrics: 192 tests; sorting always emitted target_pose although historical source specified inside. Prior pose-based fixture still failed at 0.0609967 m release-pose distance.
After metrics: 198 tests passed in 18.87 s, zero failures/skips. Separate compile succeeds in 12.514 s including initialization, initial gap zero, stage gap 5.960e-8 rad. Separate replay existing containment check PASSED: horizontal excess 0, lower excess 0, vertical overlap 0.0693952 m. Existing 0.003 m geometry slack unchanged. Release-pose distance remains 0.0609967 m and is explicitly logged, not hidden.
Correctness/safety checks: no new collision exemption, tolerance loosening, motion target edit, teleport or artificial object attach. Explicit relation frozen before rerun, not inferred after seeing an outcome. Legacy requests retain previous behavior; prior failures are not overwritten. Missing/self support rejected before motion compilation. Post-settle support pose is observed from simulation.
Failures and raw statuses: no new test failures; one physical fixture passed its existing bounds-based containment criterion. This validator is not a proof of exact hollow-container collision geometry; broader limits/contact/idle and multi-scene acceptance are outstanding. Additional support-presence checks were added after the diagnostic run; final tests cover them, and fixture has valid distinct support.
Raw log paths: /tmp/rm75_u1_inside_compile_001.log and directory containing frozen_inputs.json, summary.json, program; /tmp/rm75_u1_inside_replay_001.log and directory containing strict_replay_summary.json, maniskill_gate_result.json, observed start/support information and video/0.mp4; /tmp/rm75_u1_inside_final_tests.log.
Committed summary path: benchmarks/unified_scenarios/u1_inside_semantics_summary.json.
Unexpected observations: the physical outcome matches the previous endpoint-hold run, but explicit intended containment and release-pose distance represent different questions. Both values are retained to avoid a misleading zero-error claim.
Open questions / next work: review explicit success schema; finish contact/limit and idle-gap checks plus the required measured 20-scene sorting suite. Do not promote one fixture to full SIM_VERIFIED or start real trials. Push remains pending explicit destination confirmation after safety-review rejection; no retry.
