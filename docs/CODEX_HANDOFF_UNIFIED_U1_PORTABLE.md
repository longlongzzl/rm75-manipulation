# Historical sorting compile and portable reader

Task: ordered step 4 fixed-scene regression and step 5 replay preparation.
State: CUROBO_VERIFIED (one historical fixture only; U1 and SIM remain incomplete).
Commit: containing commit, `git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U1_PORTABLE.md`.
Parent commit: 206bd07.

Changed files / reasons:
- benchmarks/unified_scenarios/fixtures/sorting_historical_tennis.json: freeze a separate historical-target regression. Pose and 0.04 m tolerance copied from the old frozen plan; no optimization/tuning after failure. No claim of equivalent frontend semantics or measured current joints. The original failed sorting example remains unchanged.
- rm75_app/runtime/curobo2_sim_replay.py: reader now accepts the exact new compiled-program/v1 schema as well as legacy lists; retains path containment and non-pickle NPZ loading.
- tests/test_curobo2_sim_replay.py: malformed schema/command collection and path traversal rejection.
- tests/test_sorting_full_program.py: real compiler export can be read by the actual replay loader, preserving atom boundaries.
- benchmarks/unified_scenarios/u1_historical_portable_summary.json and this handoff: evidence and limitations.

Commands run:
```
/home/zhangzhao/anaconda3/envs/curobo2/bin/python tools/benchmark_sorting_program.py --request benchmarks/unified_scenarios/fixtures/sorting_historical_tennis.json --fixture-default-joints --output-dir /tmp/rm75_u1_sorting_historical_001 > /tmp/rm75_u1_sorting_historical_001.log 2>&1
PYTHONPATH=. python -m pytest tests/test_curobo2_sim_replay.py tests/test_sorting_full_program.py -q
PYTHONPATH=. python -m pytest tests -q > /tmp/rm75_u1_portable_full_tests.log 2>&1
```
Also invoked load_replay_events directly on the GPU-generated execution.json and imported mani_skill/sapien in the realman environment.

Test environment: unchanged curobo2 Python 3.11/GPU configuration and default fixture joints; CPU pytest Python 3.12.7; realman Python 3.11.14 imports ManiSkill/SAPIEN successfully (pkg_resources deprecation warning only).
Before metrics: 167 tests; example full-program compilation failed at relation_screen; replay reader rejected new manifest object shape.
After metrics: independent historical target compiles 1 atom, 6 stages (61/41/21/41/21/21 points), 9.822 s including backend construction; initial gap 1.192e-7 rad, maximum inter-stage gap 2.384e-7 rad. Actual exported package loads 10 events. Focused tests 7 passed in 1.74 s; full regression 170 passed in 19.16 s, no failures/skips.

Correctness/safety checks: no physical sink; all planning precedes any replay; original example failure retained separately; no collision/IK/grasp-screen changes; fixture request frozen before its benchmark; serialized trajectory values remain unchanged.
Failures and raw statuses: historical compile success; all six exported dt values are None. Existing Curobo2Backend._trajectory constructs JointTrajectory without dt, and ManiSkillTrajectoryExecutor currently steps/downsamples positions without consulting dt. Therefore no timing-faithful physics replay has been run or claimed.
Raw log paths: /tmp/rm75_u1_sorting_historical_001.log; /tmp/rm75_u1_sorting_historical_001/{frozen_inputs,summary}.json; /tmp/rm75_u1_sorting_historical_001/program/execution.json and NPZ files; /tmp/rm75_u1_portable_full_tests.log.
Committed summary: benchmarks/unified_scenarios/u1_historical_portable_summary.json.

Unexpected observations: the historical goal compiles while the unrelated example target does not. This supports continued pipeline validation, not the full U1 acceptance rate. Installing a new simulator is unnecessary: the local realman environment imports successfully.
Next work / open questions for ChatGPT: preserve actual cuRobo interpolation timing and resample to the simulator control rate with tests; then replay the portable program in the correct multi-object scene and measure physical outcome. Do not use the legacy single-object CLI as evidence of multi-object replay. The 20 real snapshots with matching assignments/current joints remain required. No real robot trial and no physics identification activation.
