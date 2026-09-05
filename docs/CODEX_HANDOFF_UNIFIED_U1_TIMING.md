# Sorting replay timing prerequisite

Task: ordered step 5 preparation, preserve planner time before physics replay.
State: OFFLINE_VERIFIED; historical cuRobo compile passed, no SIM pass.
Commit: containing commit, resolve with `git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U1_TIMING.md`.
Parent commit: 1ca0621.

Changed files / reasons:
- rm75_app/planning/backends/curobo2.py: all trajectory extraction callers pass actual solver interpolation_dt for interpolated states; raw knot paths keep unknown timing. Preserve time through joint reordering/trimming, reject invalid known periods.
- rm75_app/execution/trajectory_executor.py: opt-in control_dt mode resamples by time rather than point count. Accept scalar dt or N-1 positive intervals, reject invalid timing before actions, avoid extra t=0 hold, preserve endpoint with at most one tick rounding. Legacy behavior unchanged; this is command resampling, not physical tracking verification.
- tests/test_curobo_trajectory_timing.py: 6 cases for timing, raw fallback, reorder/trim and invalid periods.
- tests/test_timed_trajectory_executor.py: 9 cases for duration, variable intervals, terminal tick, invalid timing and independence from point cap.
- benchmarks/unified_scenarios/u1_trajectory_timing_summary.json and this handoff: evidence and remaining gates.

Commands run:
```
PYTHONPATH=. python -m pytest tests/test_curobo_trajectory_timing.py tests/test_pickplace_coordinator.py -q
/home/zhangzhao/anaconda3/envs/curobo2/bin/python tools/benchmark_sorting_program.py --request benchmarks/unified_scenarios/fixtures/sorting_historical_tennis.json --fixture-default-joints --output-dir /tmp/rm75_u1_sorting_timed_001 > /tmp/rm75_u1_sorting_timed_001.log 2>&1
PYTHONPATH=. python -m pytest tests/test_timed_trajectory_executor.py tests/test_trajectory_executor.py -q
PYTHONPATH=. python -m pytest tests -q > /tmp/rm75_u1_timing_resample_full_tests.log 2>&1
git diff --check
```
Also loaded the GPU-exported execution.json via load_replay_events to inspect all six NPZ dt values/durations.

Test environment: unchanged Python 3.12.7 CPU tests and curobo2 Python 3.11/GPU config. Timing checked against installed cuRobo getting_started/motion_planning.py and solver_trajopt_cfg.py; not guessed from point count.
Before metrics: 170 tests; six dt=None paths; legacy simulator executor stepped once per selected path point.
After metrics: focused backend suite 33 passed; focused execution suite 14 passed; final full suite 185 passed in 21.69 s, no failures/skips. Intermediate backend-only full suite 176 passed in 21.57 s. Historical compile succeeds, six point counts unchanged (61/41/21/41/21/21), all dt=0.025 s, total motion 5.0 s excluding dwell/checkpoints. Compilation wall 10.072 s including initialization, not a latency claim. Initial gap 1.192e-7 rad; max inter-stage gap 2.384e-7 rad.
Correctness/safety checks: no pose/IK/collision changes; unknown raw timing is not fabricated; timed mode requires explicit actual control period; no physical commands or physics identification.
Failures and raw statuses: tests and GPU fixture passed. Physics replay not run. Existing run_maniskill_gate calls set_arm_qpos(first point), violating this task's no-teleport requirement; it has not been used to claim SIM success.
Raw log paths: /tmp/rm75_u1_sorting_timed_001.log; /tmp/rm75_u1_sorting_timed_001/{frozen_inputs,summary}.json; /tmp/rm75_u1_sorting_timed_001/program/execution.json and NPZ; /tmp/rm75_u1_timing_resample_full_tests.log; /tmp/rm75_u1_timing_full_tests.log.
Committed summary path: benchmarks/unified_scenarios/u1_trajectory_timing_summary.json.
Unexpected observations: preserving dt alone is insufficient; strict replay also needs initial-state validation instead of teleport.
Open questions / next work: connect strict timed multi-object replay to actual control rate, validate joint order/initial gap, measure outcome/contact/idle gaps. The 20 measured snapshots remain required; one historical fixture does not satisfy U1. No SIM or REAL claim.
Push status at handoff creation: auto-review rejected the combined commit/push command before execution because the destination requires explicit user confirmation. Local evidence/commit can proceed; no retry of the push without resolution. Intended destination is https://github.com/longlongzzl/rm75-manipulation.git, branch chatgpt/unified-three-scenarios.
