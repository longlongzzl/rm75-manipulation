# U1 — Failed-rescreen IK cache invalidation

Task: apply the scoped production cache correctness fix explicitly approved by the user on 2026-09-06, then unit and frozen full-chain regression.
State: NEEDS_REVIEW.
Commit: containing commit (`git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U1_IK_CACHE_FIX.md`).
Parent commit: afecd6a.

Changed files / why:
- `rm75_app/planning/backends/curobo2.py`: four-line failure branch removes the obsolete successful IK entry for the same pose/gripper-state key. This is the approved exception to the task's production-screening modification restriction, not permission to change constraints or screening policy.
- `tests/test_curobo_ik_cache.py`: four CPU tensor tests exercise the actual production screening function with controlled solver results: success/failure/recovery, mixed and uncached failures, identical pose with different ids, and gripper-state key isolation.
- `benchmarks/unified_scenarios/u1_ik_cache_fix_summary.json`: results and exact raw evidence locations.
- This handoff: review record and remaining gates.

Commands run:
```bash
PYTHONPATH=. python -m pytest tests/test_curobo_ik_cache.py -q
PYTHONPATH=. python -m pytest tests -q > /tmp/rm75_u1_cache_fix_full_tests.log 2>&1
git diff --check
```
The first new-test invocation failed because the test backend fixture omitted `_gripper_collision_state`; the fixture was corrected, not the production cache key.

Frozen benchmark invocation: `/home/zhangzhao/anaconda3/envs/curobo2/bin/python tools/benchmark_grasp_relation_screen.py --full-chain --repetitions 1 --output-jsonl /tmp/rm75_u1_cache_fix_frozen.jsonl --summary-json /tmp/rm75_u1_cache_fix_frozen_summary.json`, with one `--plan ROW.plan --scene ROW.scene` pair for each of the 16 rows, in original order, in `/tmp/rm75_relation_screen_benchmark/d2_full_chain_lazy.jsonl`. All 32 referenced plan/scene paths existed. No snapshots were edited. Output redirected to `/tmp/rm75_u1_cache_fix_frozen.log`.

Same-environment parent control: a fresh planner process ran the identical benchmark and input order, replacing only `Curobo2Backend._prepare_pose_candidates_with_solver` in memory with the original method extracted by AST from `git show afecd6a:rm75_app/planning/backends/curobo2.py`. Compiled the original method in a copy of its module globals; no worktree file replacement. Outputs use `/tmp/rm75_u1_cache_fix_parent_frozen` with `.jsonl`, `_summary.json`, `.log`. This isolates the sole production change and retains the same current benchmark instrumentation. The same in-memory parent-method control ran the four new unit tests, logging to `/tmp/rm75_u1_cache_fix_red_tests.log`.

Test environment: CPU Python 3.12.7, torch 2.9.0+cu128; curobo2 Python 3.11, torch 2.11.0+cu128, RTX 5060 Ti. Same production lazy_place / primary_only, num_ik_seeds 32 and unchanged solver tolerances/collision geometry. Benchmark summaries preserve full backend configuration.

Before metrics: parent baseline 10/16 full-chain successes, matching the historical D2 total; four new tests on parent yield two expected failures and two passes.
After metrics: 219 tests passed, zero failed/skipped, 22.55 s. Fixed full-chain 14/16, zero parent-success/fixed-failure cases. All ten historical successes retained. Gluestick generated scenes 00, 02, 03 and 06 change from failure to success in this paired run.

Correctness/safety checks: failed screens invalidate only the matching key; unrelated pose and gripper-state entries survive. Successful rescreens repopulate the cache. No tolerance loosening, target edits, geometry shrinkage, extra ignored objects/links, reverse fallback, or changed production search strategy. All planning uses the existing no-op executor; no robot commands and no physics replay. Full success requires the benchmark's complete segmented chain, not only IK. Test/benchmark results are a scoped regression gate, not full sorting SIM/REAL approval.

Failures and raw statuses: current-table gluestick remains unsuccessful, but now rejects at relation_screen with zero complete relations instead of sixteen stale relations followed by segmented_chain failure. Generated scene 05 still fails in lift/segmented_chain; raw candidate failures report start collision with the table, including gripper_Right_Support_Link (~0.345 mm) and attached_object (~3.595 mm). Constraints were not changed to conceal these failures. Full candidate failure arrays are retained in JSONL.

Raw log paths and committed summary: see every exact path in `benchmarks/unified_scenarios/u1_ik_cache_fix_summary.json`. Logs are local `/tmp` artifacts, not remotely available just because their paths are committed.

Unexpected observations: removing obsolete candidates also changed subsequent candidate search sufficiently for four previously failing frozen tasks to complete. One paired run does not establish general success-rate improvement or benchmark latency gains.

Open questions for ChatGPT: review the scoped production cache invalidation and remaining geometric failures. U1 still needs the user-provided measured 20-scene suite with actual joints; historical fixtures cannot satisfy that requirement. No full multi-object SIM pass, no physical trial, and no physics identification enabled. Stop for review before merging into main.
