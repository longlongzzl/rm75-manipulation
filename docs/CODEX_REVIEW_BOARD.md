# CODEX_REVIEW_BOARD

> Repo target path: `rm75_pick_place_app/docs/CODEX_REVIEW_BOARD.md`  
> Baseline branch audited by ChatGPT: `1.0.25`  
> Purpose: ChatGPT performs architecture/code review and writes bounded optimization tasks; local Codex implements, tests, benchmarks, and records evidence; ChatGPT reviews the resulting commit/logs and issues the next iteration.

---

## 0. Roles and collaboration protocol

### Roles

**ChatGPT — reviewer / architect**
- Reads the current GitHub code and this review board.
- Defines one bounded task at a time.
- Explains the suspected bottleneck, desired behavior, acceptance criteria, and regression risks.
- Reviews the Codex diff, tests, benchmark results, and failure logs.
- Does **not** treat an untested code change as a verified improvement.

**Local Codex — implementer / validator**
- Works on the user's local checkout.
- Makes only changes required by the current task unless a blocking issue is discovered.
- Runs unit tests and performance benchmarks locally.
- Records the exact commit, changed files, commands, hardware/environment, before/after metrics, failures, and unresolved questions in this document.
- Must not silently change candidate semantics, tolerances, collision rules, task success criteria, or robot geometry to make numbers look better.

**User — experiment owner**
- Decides when a change is allowed to reach the real robot.
- Provides ChatGPT with the updated GitHub commit/branch or the updated review-board content after each Codex iteration.
- Supervises all physical robot tests.

### Iteration states

Use one of these exact states for every task:

- `PROPOSED`
- `CODEX_IMPLEMENTED`
- `OFFLINE_VERIFIED`
- `SIM_VERIFIED`
- `REAL_VERIFIED`
- `NEEDS_REVIEW`
- `REJECTED`

### Required handoff after every Codex iteration

Codex must append:

```text
Commit:
Parent commit:
Changed files:
Why each file changed:
Commands run:
Unit-test result:
Benchmark hardware:
Benchmark environment:
Warm/cold condition:
Before metrics:
After metrics:
Correctness/regression metrics:
Failures:
Unexpected observations:
Open questions for ChatGPT:
```

Do not summarize benchmark results as only “faster” or “works”. Keep raw JSON/CSV logs and provide their paths.

### Scope discipline

1. One optimization hypothesis per commit whenever practical.
2. Do not combine candidate-speed work with grasp execution, held-object refinement, retreat logic, perception, or real-robot control changes.
3. Do not reduce candidate grids, loosen collision checks, loosen IK tolerances, or lower seed counts in Task 001 unless explicitly moved to a later experimental branch.
4. A performance optimization is accepted only when it passes the correctness gate below.
5. Run offline/unit tests before simulation; run simulation before physical-robot tests.
6. Keep a feature flag or a clean revert path until the new path is verified.

---

# TASK 001 — Grasp/relation candidate screening under 5 seconds

**State:** `PROPOSED`

## 1. Goal

Optimize the **candidate-generation + relation-screening stage** of the PickPlace pipeline while preserving the current feasible-relation behavior.

For this task, timing starts immediately before:

```python
FixedSceneAtomTaskBuilder.__call__(...)
```

and ends when `PickPlaceCoordinator.run(...)` has completed the **relation-screen** and produced a non-empty `relation_grasp_candidates` shortlist, immediately before `_run_segmented_chain(...)` begins full MotionGen planning.

The 5-second target therefore **does not include**:
- final `pregrasp` MotionGen,
- grasp linear trajectory generation,
- attach/lift planning,
- transport/preplace MotionGen,
- place trajectory generation,
- robot execution.

This isolates “grasp/place candidate computation” from full manipulation-program planning.

If the project later wants the **entire PickPlace plan** under 5 s, create a separate task after Task 001.

## 2. Acceptance criteria

### Performance

Primary online target, after one-time planner/IK warmup:

- `P95 candidate+relation screening <= 5.0 s`
- desired `P50 <= 3.0 s`
- no single normal ID scene should exceed `8.0 s` without a logged fallback reason.

Cold-start planner construction/warmup must be reported separately and must **not** be hidden inside the warm number.

### Correctness

Use the current `1.0.25` screening path as the reference implementation.

On the benchmark suite:

1. **Task-level feasibility recall**
   - If reference screening finds at least one complete grasp-place relation, optimized screening must also find at least one.
   - Target recall: `>= 99%`.
   - For the initial merge, preferred target is `100%` on the curated regression suite.

2. **Downstream planning non-regression**
   - On every scene where both screeners produce a relation, run the existing segmented-chain planner.
   - Optimized path must not reduce full-chain planning success by more than 1 percentage point on the benchmark set.

3. **No semantic shortcuts**
   - same IK position/orientation tolerances,
   - same collision scene,
   - same ignored-object semantics,
   - same candidate grids,
   - same `num_ik_seeds`,
   - same object/task success definition.

A different selected candidate ID is acceptable if the resulting relation is valid and downstream success is not worse.

## 3. Current-code observations

### 3.1 Candidate grids are already tiered

`atom_task_builder.py` already assigns `search_tier` and progressively expands difficult orientations/shifts. Keep that mechanism in Task 001.

Examples in current code:
- generic grasps: yaw × tilt grid;
- spherical grasps: fixed `8 × 8` relation lattice;
- pen/axial modes: additional shifts and refinement candidates.

**Do not shrink these grids in the first optimization.**

### 3.2 Current relation screening performs up to four fixed-shape coarse IK calls per tier

In `PickPlaceCoordinator.run(...)`, the current order is approximately:

```text
preplace candidates -> prepare_coarse(...)
place candidates    -> prepare_coarse(...)

then

pregrasp candidates -> prepare_coarse(...)
grasp candidates    -> prepare_coarse(...)
```

The coarse cuRobo2 solver has a fixed `coarse_ik_batch_size = 64`.  
`_prepare_pose_candidates_with_solver(...)` pads every smaller chunk to that batch size.

Therefore a common tier containing far fewer than 64 endpoints can still execute four 64-row coarse IK solves.

This is the first optimization target.

### 3.3 Place endpoints are screened before grasp feasibility is known

For grasp-dependent place candidates, the current path can spend GPU work screening place/preplace endpoints for grasp parents that will later fail grasp/pregrasp IK.

Any complete relation requires both grasp-side endpoints to be feasible, so place work for an infeasible grasp parent cannot contribute to a valid relation.

### 3.4 Current cache should be treated carefully

`Curobo2Backend` caches accepted IK solutions by rounded pose. During Task 001:
- do not assume a cached pose result is automatically valid under a different collision-ignore context;
- do not let a previous success survive a later authoritative failure accidentally;
- any cache optimization must include tests for collision-ignore semantics.

Do not redesign the cache in the first commit unless required by the merged-batch change.

---

## 4. Proposed optimization sequence

### Phase A — instrumentation first

Before algorithm changes, add timings/counters so every run records at least:

```json
{
  "candidate_build_time_s": 0.0,
  "screen_total_time_s": 0.0,
  "grasp_family_screen_time_s": 0.0,
  "place_family_screen_time_s": 0.0,
  "coarse_ik_call_count": 0,
  "coarse_ik_rows_requested": 0,
  "coarse_ik_rows_padded": 0,
  "stable_ik_call_count": 0,
  "search_tier": 0,
  "grasp_candidate_count": 0,
  "place_candidate_count": 0,
  "complete_relation_count": 0
}
```

Preserve the existing `relation_screen` diagnostics and extend them rather than creating an unrelated logging path.

**Acceptance for Phase A:** no behavior change; all current unit tests pass.

---

### Phase B — merge endpoints with identical collision semantics

This should be the first performance change.

Instead of:

```python
prepare_coarse(pregrasps, ..., ignore=grasp_ignores)
prepare_coarse(grasps, ..., ignore=grasp_ignores)
```

build one tuple:

```python
grasp_family_endpoints = pregrasps + grasps
prepare_coarse(
    grasp_family_endpoints,
    ...,
    ignore_object_names=grasp_ignores,
)
```

Likewise merge:

```python
place_family_endpoints = preplaces + places
prepare_coarse(
    place_family_endpoints,
    ...,
    ignore_object_names=contact_ignores,
)
```

This preserves:
- the exact poses,
- the exact solver,
- the exact seed count,
- the exact tolerance,
- the exact collision-ignore set.

For a tier that previously generated four padded batch64 calls, this can reduce it to two.

#### Important

The backend can still chunk if a merged tuple exceeds 64, so do **not** manually truncate.

**Expected benefit:** approximately halve fixed-shape coarse-solver invocations in common tiers.

**Correctness gate:** run old vs merged path on identical scene/task manifests and compare complete-relation recall and downstream full-chain success.

---

### Phase C — grasp-first pruning before place screening

After Phase B is verified, change the screening order inside each search tier:

```text
1. build enabled grasp candidates for tier
2. build corresponding pregrasps
3. batch-screen grasp + pregrasp together
4. keep only grasp parents with both endpoints feasible
5. collect place candidates only for those grasp-feasible parents
6. build their preplaces
7. batch-screen place + preplace together
8. construct complete relations
```

For spherical shared-place candidates, continue to de-duplicate shared place poses.

Do not permanently discard higher-tier candidates. If no complete relation exists, continue the existing tier expansion/refinement logic.

This change should reduce unnecessary place-side IK work while preserving the logical relation condition:

```text
complete relation =
grasp feasible
AND pregrasp feasible
AND place feasible
AND preplace feasible
```

#### Correctness note

Because cuRobo IK can be sensitive to batch composition, benchmark this separately from Phase B. Keep each phase as its own commit.

---

### Phase D — lossless fallback

For the first production version, retain the legacy screening implementation behind a flag/helper:

```python
fast_result = screen_relations_fast(...)

if fast_result.has_complete_relation:
    return fast_result

return screen_relations_legacy(...)
```

The fallback should be clearly counted in diagnostics:

```json
{
  "fast_path_success": false,
  "legacy_fallback_used": true
}
```

This is intentionally conservative:
- common scenes should hit the fast path and satisfy the 5 s target;
- difficult/edge scenes preserve current recall even if they occasionally exceed 5 s.

Do not impose a hard 5 s abort until benchmark evidence shows that doing so does not reduce feasibility recall.

---

### Phase E — targeted stable verification/fallback, only after profiling

The backend already has:
- large-batch coarse IK;
- smaller “stable” IK;
- continuous residual metrics (`normalized_pose_gap`).

After Phases B/C are measured, consider a small stable pass only for:
- the selected/top few complete relations, or
- coarse near-miss parents when fast screening finds no relation.

Do **not** stable-screen every candidate; that defeats the purpose.

Candidate near-miss ranking can use existing metrics:
- `constraint_feasible`;
- position error;
- orientation error;
- normalized pose gap.

This phase is optional for Task 001 and should only be added if benchmark data shows coarse-screen false negatives or unstable candidate selection.

---

## 5. Changes explicitly forbidden in the first performance pass

Do not:
- reduce `sphere_grasp_yaw_offsets_deg`;
- reduce tilt grids;
- reduce pen axis-shift grids;
- reduce `coarse_ik_num_seeds`;
- increase IK tolerances;
- disable self collision;
- remove world obstacles;
- enlarge ignored-object sets;
- reduce attachment geometry;
- skip pregrasp/preplace checks;
- pick Top-1 by heuristic score without a feasibility fallback.

Those may make latency look better while silently changing correctness.

---

## 6. Small safe CPU optimization

`FixedSceneAtomTaskBuilder._grasp_candidates(...)` loads the same asset mesh with `trimesh.load(...)` on each task build.

After GPU-call optimization, profile whether this matters. If measurable, cache immutable asset geometry metadata by:

```text
(mesh_file, mesh_scale)
```

At minimum cache:
- loaded local mesh/vertices needed for bounds;
- extents.

Do not replace the current world-AABB-center logic with an approximation unless equivalence is tested.

This is lower priority than reducing coarse IK calls.

---

## 7. Benchmark suite to create

Add a reproducible benchmark script, suggested path:

```text
rm75_pick_place_app/scripts/benchmark_grasp_relation_screen.py
```

It should:
1. warm the planner and coarse IK solver once;
2. replay identical task/scene manifests for reference and optimized screeners;
3. run multiple repetitions;
4. write raw JSONL;
5. print P50/P90/P95/P99;
6. compare feasible-relation recall;
7. optionally run downstream segmented planning.

### Minimum scene coverage

Include at least:
- generic asymmetric object;
- spherical/symmetric object;
- axial object if used by the paper;
- easy center-workspace scene;
- workspace-boundary scene;
- cluttered scene;
- one scene known to require a higher search tier;
- one previous failure/regression scene if available.

Use exactly the same scene snapshot and current joint state for reference/optimized comparison.

### Required output summary

```text
N scenes:
N repetitions:
Warm P50:
Warm P95:
Warm max:
Cold first-call:
Reference relation-found rate:
Optimized relation-found rate:
Feasibility recall:
Downstream plan success reference:
Downstream plan success optimized:
Fast-path hit rate:
Legacy-fallback rate:
Mean coarse IK calls before:
Mean coarse IK calls after:
```

---

## 8. Unit/regression tests required

At minimum extend `tests/test_pickplace_coordinator.py` with tests that verify:

1. pregrasp+grasp are submitted together when collision semantics are identical;
2. preplace+place are submitted together when collision semantics are identical;
3. grasp-infeasible parents do not trigger unnecessary place screening in the grasp-first implementation;
4. a lower-score complete relation is still retained if the highest-score relation is incomplete;
5. tier expansion still finds a relation when tier 0 fails;
6. refinement-parent behavior still works;
7. fast-path failure invokes legacy fallback;
8. paired-place constraints remain intact.

All existing tests must continue to pass.

---

## 9. Codex implementation order

### Commit 001-A — instrumentation only
Status after implementation: `CODEX_IMPLEMENTED`

### Commit 001-B — merged same-context endpoint batches
No pruning/reordering yet.

### Commit 001-C — grasp-first place pruning
Only after 001-B benchmark is reviewed.

### Commit 001-D — fallback / optional stable near-miss recovery
Only if required by data.

Do not implement B+C+D in one commit.

---

## 10. First Codex instruction

Local Codex should execute only **Commit 001-A and 001-B** now.

Suggested instruction:

```text
Read rm75_pick_place_app/docs/CODEX_REVIEW_BOARD.md.

Implement TASK 001 only through Phase A and Phase B.

Requirements:
1. First add timing/call-count instrumentation without changing semantics.
2. Then merge pregrasp+grasp coarse IK into one call because they use the same grasp ignore context.
3. Merge preplace+place coarse IK into one call because they use the same contact ignore context.
4. Do not change candidate grids, IK seeds, tolerances, collision rules, search tiers, ranking, or segmented MotionGen.
5. Keep the code easy to revert.
6. Update/extend test_pickplace_coordinator.py.
7. Run all relevant unit tests.
8. If a real cuRobo2 benchmark environment is available, run the same fixed scenes before/after after warmup and save raw JSONL.
9. Append the required handoff block below in this file.
10. Stop after Phase B and wait for ChatGPT review before implementing grasp-first pruning.
```

---

# CODEX HANDOFF — TASK 001 / ITERATION 1

**State:** `PROPOSED`

```text
Commit:
Parent commit:
Changed files:
Why each file changed:

Commands run:

Unit-test result:

Benchmark hardware:
Benchmark environment:
Warm/cold condition:

BEFORE
Candidate build P50:
Candidate build P95:
Relation screen P50:
Relation screen P95:
Mean coarse IK calls:
Relation-found rate:
Downstream full-plan success:

AFTER
Candidate build P50:
Candidate build P95:
Relation screen P50:
Relation screen P95:
Mean coarse IK calls:
Relation-found rate:
Downstream full-plan success:

Feasibility recall vs reference:
Fast path anomalies:
Failures:
Unexpected observations:
Raw log paths:

Open questions for ChatGPT:
```

---

# ChatGPT review checkpoint

Do **not** start Phase C until ChatGPT has reviewed:
- the 001-B diff,
- updated unit tests,
- before/after coarse IK call counts,
- warm P50/P95,
- relation-feasibility recall,
- any scene where candidate IDs or downstream planning outcomes differ.

The next review will decide whether:
1. Phase B alone is enough to meet the 5-second goal;
2. grasp-first pruning should be implemented;
3. a stable near-miss fallback is needed;
4. cache semantics need a dedicated correctness fix before further speed work.


---

# CODEX HANDOFF — TASK 001 / ITERATION 1 (本地 Codex 写入)

**State:** `OFFLINE_VERIFIED`

```text
Commit: Pending — see repository-state note below.
Parent commit: 97d4e28bc322e139732dabc32f52372a731972b2
Changed files:
- rm75_pick_place_app/rm75_app/pickplace/coordinator.py
- rm75_pick_place_app/rm75_app/pickplace/atom_task_builder.py
- rm75_pick_place_app/tests/test_pickplace_coordinator.py
- rm75_pick_place_app/docs/CODEX_REVIEW_BOARD.md

Why each file changed:
- coordinator.py: Local Codex added relation-screen diagnostics and a single same-context coarse-IK wrapper. It submits `pregrasp + grasp` together under `(object_name,)`, and `preplace + place` together under the existing contact-ignore tuple. Candidate grids, seeds, tolerances, collision rules, tiers, ranking, and segmented MotionGen were not changed.
- atom_task_builder.py: Local Codex measures `FixedSceneAtomTaskBuilder.__call__` and carries the elapsed value in `PickPlaceTask` for the relation-screen diagnostic.
- test_pickplace_coordinator.py: Local Codex verifies endpoint family merging, ignore-context preservation, and fixed-batch requested/padded-row accounting.
- CODEX_REVIEW_BOARD.md: Local Codex recorded this bounded implementation and evidence.

Commands run:
- `git diff --check -- rm75_pick_place_app/rm75_app/pickplace/coordinator.py rm75_pick_place_app/rm75_app/pickplace/atom_task_builder.py rm75_pick_place_app/tests/test_pickplace_coordinator.py`
- `python -m py_compile ../rm75_app/pickplace/coordinator.py ../rm75_app/pickplace/atom_task_builder.py ../tests/test_pickplace_coordinator.py`
- `PYTHONPATH=.. python -m pytest ../tests/test_pickplace_coordinator.py -q`
- `(cd rm75_pick_place_app && PYTHONPATH=. python -m pytest tests -q)`

Unit-test result:
- Coordinator focused test: 19 passed in 1.12 s.
- Full project suite from project root: 125 passed in 18.02 s.
- An earlier full-suite invocation from `docs/` had 5 unrelated path failures because `tests/test_llm_desk_support_surface.py` resolves `assets/test_scenes/...` from the process CWD; rerunning from `rm75_pick_place_app/` passed all 125 tests.

Benchmark hardware:
- NVIDIA GeForce RTX 5060 Ti; 8151 MiB; driver 580.173.02.

Benchmark environment:
- Python 3.12.7 (Anaconda), torch 2.9.0+cu128, `torch.cuda.is_available() == True`, cuRobo import version `0.0.0`.

Warm/cold condition:
- No real cuRobo fixed-scene before/after timing was run. No existing benchmark entrypoint was found that replays the same manifests through both the legacy and merged relation-screen implementations, so reporting P50/P95 would be misleading.

BEFORE
Candidate build P50: Not measured (no fixed-scene replay harness).
Candidate build P95: Not measured (no fixed-scene replay harness).
Relation screen P50: Not measured (no fixed-scene replay harness).
Relation screen P95: Not measured (no fixed-scene replay harness).
Mean coarse IK calls: 4 in the deterministic one-grasp/one-place structural fixture: separate pregrasp, grasp, preplace (8 endpoints), and place submissions; each is padded to batch64.
Relation-found rate: 100% (1/1 deterministic fixture; not a scene-suite metric).
Downstream full-plan success: 100% (1/1 deterministic fixture; not a scene-suite metric).

AFTER
Candidate build P50: Not measured (no fixed-scene replay harness).
Candidate build P95: Not measured (no fixed-scene replay harness).
Relation screen P50: Not measured (no fixed-scene replay harness).
Relation screen P95: Not measured (no fixed-scene replay harness).
Mean coarse IK calls: 2 in the same deterministic fixture: merged `pregrasp + grasp` and merged `preplace + place` submissions.
Relation-found rate: 100% (same 1/1 deterministic fixture).
Downstream full-plan success: 100% (same 1/1 deterministic fixture).

Feasibility recall vs reference:
- 100% in the deterministic lower-score-complete-relation regression test. This is a semantic unit check, not the required curated-scene recall result.

Fast path anomalies:
- Not applicable: Phase D fallback was intentionally not implemented.

Failures:
- No failures in the focused or root-CWD full test suites.
- Real GPU timing and curated-scene feasibility recall remain unmeasured because the specified legacy-vs-merged replay harness/raw JSONL output does not yet exist.

Unexpected observations:
- Local Codex structural experiment: with 1 grasp and 1 place, the merged path requests 11 endpoint rows and pads 128 rows across 2 batch64 solves. The prior split sequence requests the same 11 rows but pads 256 rows across 4 batch64 solves. This verifies the expected 50% reduction in fixed-shape coarse-solver invocations without changing endpoint poses or ignore contexts; it is not evidence of a 50% wall-clock reduction.
- `candidate_build_time_s` is available only for tasks built by `FixedSceneAtomTaskBuilder`; manually constructed `PickPlaceTask` values report `None`, avoiding invented timing.

Raw log paths:
- No GPU benchmark JSONL was produced; no valid legacy-vs-merged benchmark harness is present.
- Test evidence is the command output recorded in this handoff; the new regression test is `tests/test_pickplace_coordinator.py::test_relation_screen_reports_merged_coarse_ik_batch_metrics`.

Repository-state note:
- Before Local Codex began, the repository already had extensive unrelated modified/untracked work, including pre-existing changes in all three code files above. Local Codex has deliberately not staged or committed those user-owned changes. A safe push requires either a clean worktree/baseline commit containing that prior work, or the user's explicit authorization to include it in this branch.

Open questions for ChatGPT:
- Please review Phase B before Phase C, as required.
- Should the next iteration add the proposed replay benchmark script plus frozen scene-manifest list, so warm P50/P95, feasibility recall, and downstream success can be measured against the legacy implementation?
- The coordinator currently reports padded rows from `planner.config.coarse_ik_batch_size` when available; should backend-native counters also be added in a later instrumentation-only commit to cover nonstandard chunking paths?
```

---

# CODEX FOLLOW-UP — runnable relation-screen replay

**State:** `NEEDS_REVIEW`

Added `tools/benchmark_grasp_relation_screen.py`. It replays frozen
`manipulation_plan.json` files through the production task builder and
coordinator, but stops at the segmented-MotionGen handoff. CUDA is synchronized;
no robot, trajectory execution, or ManiSkill runs. It writes raw JSONL plus a
summary. Run it with:

```text
/home/zhangzhao/anaconda3/envs/curobo2/bin/python \
  tools/benchmark_grasp_relation_screen.py \
  --plan <frozen manipulation_plan.json> --repetitions 10 \
  --output-jsonl <output>.jsonl
```

2026-09-03 smoke replay: three mock-frozen `current_table` tasks; Curobo2
Python 3.11, torch 2.11.0+cu128, RTX 5060 Ti, batch64 coarse IK, 32 coarse IK
seeds, and 32 normal IK seeds.

```text
Planner construction/warmup: 5.482 s
Warm suite relation-screen wall P50/P95: 6.519 / 9.087 s (N=6)
Warm relation-found rate: 100% (6/6)

tennis -> inside bitong: 0.201 / 0.230 s (N=2)
gluestick -> on lvmukuai: 8.960 / 9.129 s (N=2)
carriot -> on shuazi: 6.513 / 6.524 s (N=2)
```

Task 001 therefore does not meet the warm P95 <= 5 second target on this local
suite. Also, the coordinator `screen_total_time_s` does not synchronize CUDA
and materially under-reports real wall time (gluestick reports about 0.34 s,
but the synchronized screen wall time is about 9 s). Use the benchmark
`screen_wall_time_s` until the production diagnostic is corrected.

Host-specific raw evidence, not committed:

```text
/tmp/rm75_relation_screen_benchmark/current_table_three_tasks.jsonl
/tmp/rm75_relation_screen_benchmark/current_table_three_tasks.summary.json
```

This does not establish legacy-vs-current feasibility recall or downstream
planning non-regression. The local Git history has no recoverable `1.0.25`
screener, and the current coordinator already includes grasp-first place
pruning, so a true Phase-B comparison needs a preserved legacy checkout/path.
