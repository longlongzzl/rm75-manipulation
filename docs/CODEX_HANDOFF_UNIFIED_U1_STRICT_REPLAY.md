# Strict timed multi-object physics replay

Task: ordered step 5 actual ManiSkill replay of the historical sorting fixture.
State: NEEDS_REVIEW (physics attempted and failed, not SIM_VERIFIED).
Commit: containing commit, `git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U1_STRICT_REPLAY.md`.
Parent commit: 4a92cf0.

Changed files / reasons:
- rm75_app/validation/maniskill_gate.py: opt-in strict timed path validates every trajectory's joints, timing and stage gaps before action; checks actual environment reset joints within 0.12 rad instead of calling set_arm_qpos. Uses actual env control_freq. Records observed/planned start state and time policy. Legacy path unchanged.
- tests/test_strict_timed_replay.py: 6 offline cases for matching state, initial mismatch, later missing dt, stage mismatch, wrong joints and empty package.
- tools/replay_sorting_program.py: local no-robot physics diagnostic using frozen compiled plan and new manifest; preserve exceptions and gate failures, nonzero exit on failure. Summary deliberately never promotes the broad SIM gate automatically.
- tools/benchmark_sorting_program.py: mutually exclusive explicit initial-joints input versus planner defaults; freeze input hash/provenance. Correct fixture-only label for historical fixtures.
- benchmarks/unified_scenarios/u1_strict_replay_summary.json and this handoff: retain all attempts and failures.

Commands run:
```
PYTHONPATH=. python -m pytest tests/test_strict_timed_replay.py -q
/home/zhangzhao/anaconda3/envs/realman/bin/python tools/replay_sorting_program.py --compiled-dir /tmp/rm75_u1_sorting_timed_001 --output-dir /tmp/rm75_u1_strict_replay_002 > /tmp/rm75_u1_strict_replay_002.log 2>&1
/home/zhangzhao/anaconda3/envs/curobo2/bin/python tools/benchmark_sorting_program.py --request benchmarks/unified_scenarios/fixtures/sorting_historical_tennis.json --initial-joints /tmp/rm75_u1_strict_replay_002/replay_initial_state.json --output-dir /tmp/rm75_u1_sim_start_compile_001 > /tmp/rm75_u1_sim_start_compile_001.log 2>&1
/home/zhangzhao/anaconda3/envs/realman/bin/python tools/replay_sorting_program.py --compiled-dir /tmp/rm75_u1_sim_start_compile_001 --output-dir /tmp/rm75_u1_strict_replay_003 > /tmp/rm75_u1_strict_replay_003.log 2>&1
PYTHONPATH=. python -m pytest tests -q > /tmp/rm75_u1_strict_final_tests.log 2>&1
git diff --check
```
Attempt 001 used the same timed compilation and failed before initial-state artifact logging was added; retained separately.

Test environment: CPU Python 3.12.7; curobo2 Python 3.11 unchanged GPU config; realman Python 3.11.14 ManiSkill/SAPIEN, RM75-MultiObjectTask-v1, pd_joint_pos, actual control frequency 20 Hz. No robot API.
Before metrics: 185 tests; no strict physical replay; legacy gate teleported arm to trajectory start and ignored time.
After metrics: 6 new tests passed; full 191 passed in 23.30 s, zero failed/skipped. First two physics attempts correctly rejected 1.570796 rad initial mismatch before playback. Environment reset joints are seven zeros. Recompile from this observed state succeeded in 11.219 s, initial gap zero and stage max gap 5.960e-8 rad. Third attempt actually executed all six stages without arm teleport at 0.05 s control period.

Correctness/safety checks: unchanged scene/targets/collision/tolerances; only initial joints updated from actual simulator observation, explicitly not measured robot state. All programs compile before replay. Whole-package timing/joint/continuity preflight occurs before simulated motion. Unknown dt fails closed in strict mode. Legacy behavior remains accessible and must not be confused with strict validation.
Failures and raw statuses: attempt 003 gate FAILED: `target pose outside tolerance`, position error 0.1241348162 m. Stage-end maximum joint errors: approach 0.02647, grasp 0.00994, lift 0.05790, preplace 0.06412, place 0.06315, retreat 0.01288 rad. These are endpoint tracking measurements, not continuous-time maxima. Object lifted and was transported according to pose trace, then ended near (-0.20739, -0.33416, 0.03493). Root cause not yet established; no tolerance change or invented attachment to force success.
Raw log paths: /tmp/rm75_u1_strict_replay_001.log, /tmp/rm75_u1_strict_replay_002.log, /tmp/rm75_u1_strict_replay_003.log; corresponding directories contain strict_replay_summary.json, scene spec, start-state artifact (002/003), and video. Attempt 003 also contains full maniskill_gate_result.json with contacts and stage traces. Compile evidence: /tmp/rm75_u1_sim_start_compile_001/{frozen_inputs,summary}.json and program NPZ. Tests: /tmp/rm75_u1_strict_final_tests.log.
Committed summary path: benchmarks/unified_scenarios/u1_strict_replay_summary.json.
Unexpected observations: real simulator reset differs substantially from planner default. Strict start validation exposed this before playback. A geometrically compiled, continuous path still failed the physical task after correcting the start.
Open questions / next work: inspect physical video/contact traces to separate tracking, grasp/release geometry and scene settling; add missing idle-gap/full U1 acceptance checks. One historical scene remains insufficient for the measured 20-scene suite. Steps 6–14 remain unadvanced; no real approval requested while SIM fails. Push pending explicit destination approval after prior auto-review refusal; no network retry was made.
