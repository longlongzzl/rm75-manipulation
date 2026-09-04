# CODEX TASK 001-D — Validate lazy-place and progressively screen preplace clearances

**Reviewer:** ChatGPT  
**Repository:** `longlongzzl/rm75-manipulation`  
**Reviewed C1:** `862c6eb6b5dfeb8851799e617d9daea0a84c3cfa`  
**Reviewed C2:** `1ecb5c0f335d6fe8a516029cf5470832fabc9656`  
**Status:** `PROPOSED`

---

# 0. Reviewer verdict on TASK 001-C

## Performance verdict: PASS on current smoke suite

The synchronized wall-clock benchmark now meets the original screen-only target on the current three-task smoke suite:

| Task | Eager P95 | Lazy-place P95 |
|---|---:|---:|
| tennis | ~0.208 s | ~0.209 s |
| gluestick | ~8.994 s | ~1.694 s |
| carrot | ~6.379 s | ~4.494 s |
| suite | ~8.946 s | ~4.479 s |

The main hypothesis was confirmed:

- gluestick `pose_tolerance`: 47 solver calls / 3008 rows -> 1 call / 16 rows;
- carrot `pose_tolerance`: 30 calls / 1872 rows -> 17 calls / 1053 rows.

No seeds, tolerances, collision rules, object symmetries, or candidate grids were reduced.

## Correctness verdict: PROVISIONAL, not yet enough to switch production default

Current evidence is promising but insufficient for a production-default change:

- eager relation-found: 100% on three smoke tasks;
- lazy relation-found: 100% on three smoke tasks;
- selected search tier matched on those tasks;
- no differing selected relation observed;
- focused tests passed.

Missing evidence:

1. expanded curated/random scene comparison;
2. higher-tier-required cases;
3. downstream segmented MotionGen non-regression;
4. full repository test suite after C2;
5. real production callsite verification.

**Do not change the default from `eager` to `lazy_place` until D0 below passes.**

---

# 1. New bottleneck after C2

The remaining slow smoke task is carrot, with warm P95 ~= 4.49 s, too close to the 5 s ceiling for robust deployment.

Backend-native accounting shows the dominant remaining work is now strict `coarse` endpoint screening:

```text
carrot lazy-place:
coarse:           123 solver calls / 7705 requested / 7872 padded rows
axis_constrained:   5 solver calls /   70 requested /  320 padded rows
pose_tolerance:    17 solver calls / 1053 requested / 1088 padded rows
```

The next optimization should therefore **not** reduce grasp seeds first.

The coordinator's `_place_approach_candidates()` currently produces up to four clearances, each in two approach frames:

```text
nominal            -> world-Z + tool-Z
0.75 * nominal     -> world-Z + tool-Z
0.50 * nominal     -> world-Z + tool-Z
0.30 * nominal     -> world-Z + tool-Z
```

So one resolved place can create up to **8 preplace endpoints**, and relation screening coarse-solves all of them even though it only needs **one feasible preplace** to prove that relation viable.

This is now a larger lossless optimization opportunity than lazy grasp-tier screening.

---

# 2. Execution order

Implement in this order and keep commits separate:

1. **001-D0 — correctness/coverage validation of current C2**
2. **001-D1 — progressive preplace-clearance screening (opt-in)**
3. **001-D2 — production-default switch only after validation**

Do not implement lazy grasp-tier screening in TASK 001-D.

---

# 3. 001-D0 — Validate current lazy-place implementation

## 3.1 Full tests

Run from repository root with the correct project `PYTHONPATH`:

```bash
PYTHONPATH=. python -m pytest tests -q
```

Record total passed/failed count.

## 3.2 Expanded eager-vs-lazy relation suite

Build a frozen manifest list using available repository scenes/plans. Include at least:

- tennis/spherical;
- gluestick/PEN_TABLE_GRASP;
- carrot/PEN_TABLE_GRASP;
- one generic asymmetric object;
- one cubic/box object;
- `gluestick_desk_regression`;
- at least one case that requires search tier > 0;
- at least 10 valid frozen/random scenes from `assets/test_scenes/generated_random_batch_*` if compatible with the current task compiler.

For correctness, one or two repetitions per frozen scene are sufficient. Do not spend 10 repetitions on every correctness scene.

Compare `eager` and `lazy_place` in the **same C2 checkout**.

Required fields per case:

```text
scene/task id
object
mode
relation_found
selected_search_tier
selected_grasp
selected_place
complete_relation_count
place_manifold attempted/resolved counts
backend solver counts by screen_kind
```

Required acceptance:

- curated regression feasibility recall: 100%;
- expanded suite recall: >=99%;
- no silent crash/exception;
- any selected-relation difference must be logged.

## 3.3 Downstream segmented MotionGen validation

Add a benchmark mode that uses production `_run_segmented_chain()` but a **no-op executor** so it plans the complete chain without sending robot commands.

The no-op executor must:

- accept trajectories;
- not sleep;
- not talk to RealMan;
- not run ManiSkill;
- record stage names only.

This is a planning benchmark, not execution validation.

Compare eager vs lazy on the expanded suite:

```text
full_chain_plan_success
failure_stage
selected_grasp
selected_place
segmented_plan_time_s
```

Acceptance:

- no more than 1 percentage-point downstream planning regression;
- every curated case where eager succeeds should preferably also succeed under lazy;
- if a lazy failure/eager success occurs, freeze that case and diagnose before changing defaults.

## 3.4 Commit evidence

Commit a compact summary to:

```text
benchmarks/task001/task001_d0_lazy_validation_summary.json
```

Large raw JSONL may stay local; record its path.

---

# 4. 001-D1 — Progressive preplace-clearance screening

Proceed only after D0 does not expose a correctness blocker.

## 4.1 Goal

Preserve the complete set of preplace options, but solve them **progressively** instead of all at once.

This must be lossless with respect to the existing search space.

## 4.2 Preserve search-tier priority

The current `search_tier` priority is more important than preplace-clearance rank.

For each search tier:

```text
for search_tier in increasing order:
    resolve/place-screen candidates enabled for this search tier

    for preplace_clearance_rank in increasing order:
        add only newly enabled preplace endpoints of this rank
        coarse-screen them
        update cumulative feasible preplaces
        rebuild complete relations

        if complete relation exists:
            stop and keep this search tier

    # only after all preplace ranks for this tier fail:
    advance to next search tier
```

This preserves the current semantic priority:

> exhaust all preplace variants for tier T before accepting a higher grasp/place search tier T+1.

Do **not** move to a higher search tier just because the nominal preplace clearance failed.

## 4.3 Clearance ranks already exist

`_place_approach_candidates()` writes:

```python
metadata["preplace_clearance_rank"] = index
```

Use that metadata rather than inferring rank from the candidate ID.

Rank 0 contains the nominal clearance pair:

- world-Z nominal;
- tool-Z nominal.

Higher ranks are fallback clearances and should only be screened if lower ranks fail to produce a complete relation for the current search tier.

## 4.4 Cumulative feasibility cache

Maintain cumulative sets:

```python
screened_preplace_ids: set[str]
preplace_feasible_ids: set[str]
```

A preplace endpoint must never be solved twice within one coordinator run.

When a new rank is added:

1. screen only unseen preplace candidates;
2. merge newly feasible IDs into cumulative feasibility;
3. use all cumulative feasible preplaces when testing complete relations.

Place endpoint feasibility should likewise not be recomputed if already screened for the same scene/collision semantics.

## 4.5 Strict collision behavior must remain

Do not remove the current strict place/preplace coarse screen.

Do not expand ignored object sets.

Do not disable pad/support collision links during the normal coarse preplace screen.

The optimization is only **when** fallback preplace endpoints are computed, not **how** they are validated.

## 4.6 Keep an A/B mode

Do not overwrite the verified C2 path immediately.

Add an opt-in mode such as:

```python
relation_screen_mode in {
    "eager",
    "lazy_place",
    "lazy_place_progressive_preplace",
}
```

Naming can differ, but benchmark must compare C2 lazy-place against D1 progressive preplace in the same checkout.

## 4.7 Required unit tests

Add tests proving:

1. rank-0 preplace success prevents ranks 1+ from being submitted;
2. rank-0 failure causes rank 1 to be submitted;
3. if rank 1 succeeds, later ranks are not submitted;
4. all ranks are eventually searched before advancing to a higher `search_tier`;
5. a candidate is never coarse-screened twice;
6. complete relation mapping remains correct;
7. collision-ignore semantics are unchanged;
8. existing lazy-place relation behavior remains available;
9. all project tests pass.

---

# 5. D1 benchmark and acceptance

Use synchronized wall timing.

For the existing three smoke tasks, run at least 10 warm repetitions/task if runtime permits.

Report C2 `lazy_place` vs D1 `progressive`:

```text
per task:
P50 / P90 / P95 / max
relation-found rate
selected search tier
selected relation changes
coarse solver calls / requested rows / padded rows
pose_tolerance solver calls / rows
```

Primary performance goal:

```text
suite warm P95 <= 5.0 s
```

Desired robustness margin:

```text
carrot warm P95 <= 3.0 s
```

The 3 s carrot target is a desired margin, not a correctness reason to alter seeds or tolerances.

Correctness remains primary.

Expected effect if the hypothesis is correct:

- gluestick should stay near its current ~1.7 s;
- tennis should stay near ~0.2 s;
- carrot's ~7705 strict coarse endpoint rows should drop substantially because only the first useful preplace ranks are screened.

If carrot does not improve, do not change seeds. Inspect which coarse endpoints dominate before opening TASK 001-E.

Commit summary:

```text
benchmarks/task001/task001_d1_progressive_preplace_summary.json
```

---

# 6. 001-D2 — Production default decision

Only after D0 and D1 evidence is reviewed.

Do not automatically switch the default in the same D1 commit.

Candidate decision hierarchy:

1. if progressive passes expanded correctness and downstream planning validation, make progressive the default;
2. otherwise, if C2 lazy-place passes expanded validation, make `lazy_place` the default;
3. otherwise keep eager default and use a feature flag while debugging.

A default-switch commit must be tiny and contain no algorithmic changes.

After default switch:

- run full tests again;
- run the three smoke benchmark once more;
- record production mode in diagnostics.

---

# 7. Still explicitly forbidden

Do not yet:

- reduce IK seeds;
- reduce grasp tilts;
- reduce contact-axis shifts;
- reduce axial 16-angle symmetry set;
- loosen terminal pose tolerance;
- loosen collision checks;
- remove preplace endpoint validation;
- implement lazy grasp-tier screening;
- touch MotionGen parameters;
- touch real robot execution code.

We are still pursuing a lossless compute-scheduling optimization.

---

# 8. Codex handoff

Append:

```text
# CODEX HANDOFF — TASK 001-D

State:
D0 commit:
D1 commit:
Parent:

D0 full tests:
D0 scene count:
D0 eager relation success:
D0 lazy relation success:
D0 feasibility recall:
D0 downstream full-chain eager/lazy:
D0 differing cases:
D0 summary path:

D1 smoke performance:
- tennis lazy/progressive P50/P95:
- gluestick lazy/progressive P50/P95:
- carrot lazy/progressive P50/P95:
- suite lazy/progressive P50/P95:

D1 solver work:
- tennis:
- gluestick:
- carrot:

D1 expanded-suite correctness:
D1 downstream full-chain success:
D1 differing selected relations:
D1 summary path:

Failures:
Unexpected observations:
Raw logs:
Open questions for ChatGPT:
```

Stop after D1. Do not switch the production default until ChatGPT reviews the evidence.

# CODEX HANDOFF — TASK 001-D

State:
D0 commit: pending
D1 commit: not started (D0 downstream blocker)
Parent: 1ecb5c0f335d6fe8a516029cf5470832fabc9656

D0 full tests: 126 passed in 16.71s (`PYTHONPATH=. python -m pytest tests -q`)
D0 scene count: screen-only C2 reference has 3 frozen smoke cases; expanded suite not run after downstream blocker
D0 eager relation success: C2 baseline reference available in `benchmarks/task001/task001_c2_lazy_place_summary.json`
D0 lazy relation success: 100% (3/3 C2 smoke reference)
D0 feasibility recall: not expanded
D0 downstream full-chain eager/lazy: blocked on first lazy full-chain plan before a comparison
D0 differing cases: none measured
D0 summary path: `benchmarks/task001/task001_d0_lazy_validation_summary.json`

D1 smoke performance: not run
D1 solver work: not run
D1 expanded-suite correctness: not run
D1 downstream full-chain success: not run
D1 differing selected relations: not run
D1 summary path: not created

Failures:
- `test_cached_scene` was initially missing the migrated carrot perception fixture. Restored from the local legacy lerobot project in commit `3e6522c`; all 126 tests then pass.
- Production `_run_segmented_chain()` with the D0 no-op executor reached MotionGen `place` planning, where cuRobo requested 521 interpolation steps against the configured 500-row buffer and raised `ValueError: Interpolated trajectory buffer was recreated, but cuda graph is not available`.

Unexpected observations:
- The full-chain benchmark does not command a robot: its executor records accepted trajectory stage names only. The failure occurs before a successful trajectory can be handed to it.

Raw logs:
- `/tmp/rm75_relation_screen_benchmark/d0_full_chain_lazy.jsonl` (not written: planner exception interrupted the first sample)

Open questions for ChatGPT:
- Should D0 rerun after an independently reviewed production MotionGen interpolation-buffer configuration fix? TASK 001-D prohibits changing MotionGen parameters as part of this relation-screen optimization, so D1 was deliberately not started.

## D0 rerun after infrastructure review

Infrastructure fix commit: `353c3c1` (`interpolation_buffer_size` 500 -> 640; default-contract update `2712408`)

Full tests: `126 passed in 16.81s`

D0 frozen eager full-chain: tennis 2/2 success; gluestick 0/2 `segmented_chain`; carriot 2/2 success.

D0 frozen lazy full-chain: tennis 2/2 success; gluestick 0/2 `segmented_chain`; carriot 2/2 success. The selected grasp/place matched eager for every paired sample, and no interpolation-capacity exception reoccurred.

Expanded D0 eager/lazy correctness: 10 frozen generated gluestick scenes x 2 reps/mode were 20/20 relation-found in both modes, with zero selected tier/grasp/place differences. The fixed `gluestick_desk_regression` was 2/2 in both modes, also with matching selected relation. 24/40 paired generated samples required `search_tier > 0`.

Any full-chain success differences: none (both modes 4/6 successful smoke samples); gluestick remains an equal baseline failure in `segmented_chain`.

Max observed interpolation waypoint requirement: 521. The current 640 capacity did not reallocate during rerun. Point-sampled CUDA memory was approximately 2.1 GiB / 8.0 GiB; peak was not instrumented.

D1 started: no. The D0 matrix still lacks dedicated generic-asymmetric and cubic/box task atoms, plus full-chain validation for the full expanded set. This is recorded rather than treating the current subset as acceptance.

Raw logs:
- `/tmp/rm75_relation_screen_benchmark/d0_full_chain_eager.jsonl`
- `/tmp/rm75_relation_screen_benchmark/d0_full_chain_lazy_640.jsonl`
- `/tmp/rm75_relation_screen_benchmark/d0_generated10_eager.jsonl`
- `/tmp/rm75_relation_screen_benchmark/d0_generated10_lazy.jsonl`
- `/tmp/rm75_relation_screen_benchmark/d0_desk_eager.jsonl`
- `/tmp/rm75_relation_screen_benchmark/d0_desk_lazy.jsonl`
