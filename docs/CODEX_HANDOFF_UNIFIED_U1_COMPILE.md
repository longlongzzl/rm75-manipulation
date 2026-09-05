# Ordered validation: sorting compile progress

Task: ordered steps 1–4; no-robot Organization/sorting full-program integration.
State: NEEDS_REVIEW (offline integration verified; cuRobo fixture failed).
Commit: containing commit; resolve with `git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U1_COMPILE.md`.
Parent commit: 76fc0ae00f2e42a97dfc51212b1ad7c825b84eff.

Changed files and reasons:
- tests/test_sorting_full_program.py: retain the real sorting frontend, program compiler, FixedSceneAtomTaskBuilder and coordinator; replace only the planner with the existing fake backend. Verify 3-atom buffer swap, propagated joints/object poses/revisions, unchanged input scene, trajectory continuity, gripper ordering, portable NPZ roundtrip and no planning during replay. Inject second-atom failure and prove no partial execution; reject no-buffer cycle before motion builder.
- tools/benchmark_sorting_program.py: real cuRobo full-program diagnostic with no robot sink; exclusive output directory, frozen input/config hashes, explicit fixture-default-joints flag, per-atom diagnostics and portable export on success. Legacy scene loader only loads objects, so adapter explicitly reads snapshot joint_names/joint_positions. It is a diagnostic adapter, not yet the complete U1 benchmark matrix.
- tools/run_unified_scenario.py and tests/test_unified_benchmark_serialization.py: fix and test Path values in backend configuration JSON.
- benchmarks/unified_scenarios/u1_sorting_compile_progress.json: measured progress and failure evidence, not an overall U1 pass.
- this handoff: reviewable evidence and next gate.

Commands run:
```
PYTHONPATH=. python -m pytest tests -q
PYTHONPATH=. python -m pytest tests/test_scenario_program_runner.py tests/test_sorting_scenario.py tests/test_magnetic_assembly.py tests/test_magnetic_pickplace_adapter.py tests/test_pusht_scenario.py tests/test_pickplace_program.py tests/test_magnetic_plan_compilation.py tests/test_pusht_tracking.py tests/test_sorting_full_program.py -q
PYTHONPATH=. python -m pytest tests/test_unified_benchmark_serialization.py tests/test_sorting_full_program.py -q
/home/zhangzhao/anaconda3/envs/curobo2/bin/python tools/benchmark_sorting_program.py --request configs/scenarios/sorting.example.json --fixture-default-joints --output-dir /tmp/rm75_u1_sorting_fixture_002 > /tmp/rm75_u1_sorting_fixture_002.log 2>&1
PYTHONPATH=. python -m pytest tests -q > /tmp/rm75_u1_full_tests.log 2>&1
git diff --check
```

Test environment: CPU tests use existing Python 3.12.7 environment; GPU diagnostic uses curobo2 Python 3.11, CUDA device 0 and existing production Curobo2BackendConfig (32 IK seeds, interpolation buffer 640). Full config is in raw summary. No planner/collision/tolerance changes. No robot or ManiSkill execution.

Before metrics: full regression 163 passed in 20.82 s; previous program test replaced the coordinator.
After metrics: new-module set 32 passed in 2.12 s; new integration + serialization tests 4 passed in 1.68 s; final full regression 167 passed in 18.75 s, zero failed/skipped. CPU fake-planner success is software-contract evidence only.

Correctness/safety checks: no physical sink exists in benchmark; failed compilation exposes no executable program; input scene is copied; real coordinator is not patched in new integration tests; production lazy_place/primary_only unchanged; example/default joints explicitly labeled non-measured. The script preserves the tested inputs rather than modifying targets after failure.

Failures and raw statuses: first GPU attempt could not serialize PosixPath in config (no valid summary claimed). Fixed writer, added regression test, reran same inputs in a new directory. Attempt 002: sort_01_tennis / relation_screen / `no grasp relation has feasible preplace and place endpoints`; 608 grasp candidates, 535 pregrasp-feasible, zero complete relations, zero completed atoms. Initialization 5.718 s, total 8.877 s. No program available for SIM replay.

Raw log paths: /tmp/rm75_u1_full_tests.log; /tmp/rm75_u1_sorting_fixture_002.log; /tmp/rm75_u1_sorting_fixture_002/{frozen_inputs,summary}.json. Initial attempt's frozen inputs remain under /tmp/rm75_u1_sorting_fixture_001. Run 002 hashes preserve the exact adapter before subsequent snapshot-joint-loading hardening; that later path is not exercised by the explicit default-joint flag.
Committed summary: benchmarks/unified_scenarios/u1_sorting_compile_progress.json.

Unexpected observations: example inside-holder target is world (0.42, 0.08, 0.10), while the actual holder is near (-0.298, -0.280, 0.049). Fallback base x=-0.615 makes the example target base x=1.035 m. Historical frozen tennis target is near (-0.297, -0.280, 0.103). This demonstrates mismatched example data; it does not establish a regression or justify changing solver thresholds. Example config already says its poses require verification.

Next gate / open questions for ChatGPT: supply or identify the frozen real-scene sorting requests with matching targets and measured initial joints (20 snapshots per U1.1). Keep the example failure as-is. A separately labeled historical-fixture integration can diagnose the compiler further, but cannot substitute for the real-snapshot suite. Step 4 remains NEEDS_REVIEW; step 5 SIM and steps 6–14 are not advanced by this commit. Prior U4 synthetic experiments are historical evidence only; online physics identification has not been enabled. No merge into main.
