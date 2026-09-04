# CODEX TASK 001-C — Lazy tiered place-manifold screening

**Reviewer:** ChatGPT  
**Repository:** `longlongzzl/rm75-manipulation`  
**Reviewed head:** `dbf50c038fcb93cfd0a5f31482d62905bf82979c`  
**Status:** `PROPOSED`

## 0. Review verdict on ITERATION 1

Phase A/B is **structurally approved but TASK 001 is not yet accepted**.

Evidence currently available:

- focused coordinator tests: 19 passed;
- full project tests: 125 passed;
- deterministic fixture reduced fixed-shape coarse IK submissions from 4 to 2;
- requested endpoints stayed unchanged while padded rows fell from 256 to 128 in that fixture;
- synchronized replay on RTX 5060 Ti found all relations but measured warm relation-screen wall time:
  - tennis: ~0.20–0.23 s;
  - carrot: ~6.51–6.52 s;
  - gluestick: ~8.96–9.13 s;
- suite warm P50/P95: 6.519 / 9.087 s.

Therefore the `P95 <= 5.0 s` target is **not met**.

Important measurement finding: production `screen_total_time_s` is not CUDA-synchronized and can under-report wall time by orders of magnitude. For TASK 001 acceptance, use the synchronized benchmark `screen_wall_time_s` as authoritative.

## 1. New code-review finding

The current coordinator already contains grasp-first relation pruning and merged endpoint families, but `search_tier` is not fully a compute-saving tier.

Two expensive operations happen eagerly before the tier loop:

1. all grasp + pregrasp endpoints are coarse-screened up front;
2. if place candidates carry `continuous_place_manifold=True`, `resolve_pose_tolerance_candidates(all_place_candidates, ...)` is called on the entire eligible place set before the tier loop.

For PEN-like / axial objects this is especially expensive:

- `PEN_TABLE_GRASP` uses continuous freedom seeds, bidirectional closing-axis variants and multiple contact-axis shifts;
- axial placement may expand over many spin-equivalent targets;
- every place candidate receives `continuous_place_manifold=True` whenever the task has a non-null orientation tolerance;
- the pose-tolerance resolver itself invokes the same fixed-shape coarse IK solver, but these solver calls are **not included** in the coordinator's current `coarse_ik_call_count` metric.

This means the current diagnostic can say “2 coarse calls” while many additional `pose_tolerance` solver batches have already run.

### Root-cause hypothesis

The 6–9 s carrot/gluestick wall time is likely dominated by eager pose-tolerance solving for high-tier place candidates that are never needed once a low-tier complete relation exists.

TASK 001-C must test this hypothesis directly.

---

## 2. Scope of TASK 001-C

Implement exactly two bounded commits:

### Commit 001-C1 — backend-native solver accounting

Instrumentation only. No selection or solver semantics change.

### Commit 001-C2 — lazy tiered place-manifold resolution

Defer continuous place-manifold solving until a search tier actually needs those place candidates.

**Do not change grasp candidate grids, seeds, tolerances, collision rules, object symmetries, or task success thresholds.**

Do not implement tiered grasp screening yet. That is a possible TASK 001-D only if C2 is insufficient.

---

# 3. Commit 001-C1 — backend-native solver accounting

## 3.1 Required counters

Instrument `Curobo2Backend._prepare_pose_candidates_with_solver(...)` (or an equivalent backend-owned location) so each actual solver batch is accounted for by `screen_kind`.

At minimum expose/reset/read counters equivalent to:

```python
{
    "coarse": {
        "solver_calls": 0,
        "rows_requested": 0,
        "rows_padded": 0,
    },
    "pose_tolerance": {
        "solver_calls": 0,
        "rows_requested": 0,
        "rows_padded": 0,
    },
    "axis_constrained": {
        "solver_calls": 0,
        "rows_requested": 0,
        "rows_padded": 0,
    },
}
```

Count an actual invocation of `solver.solve_pose(...)`, not a coordinator helper call.

If another `screen_kind` exists, preserve it rather than dropping it.

## 3.2 Reset semantics

Add an explicit profiling/metrics reset method, e.g.:

```python
planner.reset_endpoint_screen_metrics()
planner.endpoint_screen_metrics()
```

Do not clear the IK cache when resetting metrics.

## 3.3 Benchmark output

Extend `tools/benchmark_grasp_relation_screen.py` to save these backend-native counters for every replay row.

Also report, per atom/task:

- grasp candidate count;
- declared place candidate count;
- eligible place candidate count;
- pose-manifold input count;
- pose-manifold resolved count;
- selected search tier;
- backend solver calls/rows by `screen_kind`;
- synchronized `screen_wall_time_s`.

The summary must aggregate by task name, not only over the whole suite.

## 3.4 Timing

Do not add unconditional `torch.cuda.synchronize()` inside production planning code.

The benchmark may synchronize before/after the full relation-screen measurement as it already does.

If per-screen-kind GPU timing is desired, make it benchmark/profiling-only. Counters are mandatory; per-kind GPU milliseconds are optional.

## 3.5 C1 acceptance

- no candidate/selection behavior change;
- existing tests pass;
- new counter unit test verifies a fixed-shape padded chunk increments exactly once per actual `solve_pose` call;
- run the three existing smoke tasks with at least 5 warm repetitions each;
- commit the small summary JSON (not huge raw logs) under `benchmarks/task001/` so ChatGPT can read it from GitHub.

Stop and inspect the counts before C2. If gluestick/carrot do **not** show meaningful `pose_tolerance` work, document that result before proceeding.

---

# 4. Commit 001-C2 — lazy tiered place-manifold resolution

Proceed if C1 confirms meaningful eager place-manifold work.

## 4.1 Current behavior to remove from fast path

Do not resolve the complete `all_place_candidates` set before entering the tier loop.

Current conceptual pattern:

```python
all_place_candidates = ... all grasp-ready parents ...
resolved_places = resolve_pose_tolerance_candidates(all_place_candidates, ...)
preplace_by_place_id = build_for_all(resolved_places)
for tier in tiers:
    use_subset_for_tier(...)
```

This performs high-tier GPU work even when tier 0 succeeds.

## 4.2 Required fast-path behavior

Use a lazy cache keyed by place candidate ID.

Conceptually:

```text
for tier in tiers:
    1. identify place candidates newly enabled at this tier
       AND belonging to currently grasp-ready parents;

    2. split them into:
       a) continuous-manifold candidates not yet resolved;
       b) ordinary candidates;

    3. resolve only group (a) for this tier;

    4. cache returned resolved candidates using the same candidate IDs;

    5. create preplace candidates only from resolved/ordinary places that are
       actually active at this tier;

    6. run the existing endpoint feasibility checks;

    7. if a complete relation exists, stop exactly as today;
       otherwise advance to the next tier.
```

Do not generate/solve higher-tier place manifolds after a complete lower-tier relation is found.

## 4.3 Preserve relation mappings

`runtime_places_by_grasp` must continue to preserve physical grasp→place pairing.

Resolved candidates retain the same candidate ID in the current backend; update the runtime mapping by ID rather than flattening away parent relations.

For a place manifold input that fails to resolve, record that candidate as attempted/failed for that tier. Do not repeatedly re-run it at every later tier.

## 4.4 Do NOT skip the existing strict place endpoint screen in C2

Although `resolve_pose_tolerance_candidates()` returns a feasible IK/FK solution, it temporarily disables the contact-endpoint collision links. The existing later place endpoint screen may therefore still catch collisions with unrelated scene geometry under the normal link set.

For correctness, TASK 001-C2 must keep the current strict place/preplace endpoint screen after resolution.

A future optimization may replace duplicate IK with a dedicated collision-only validation, but that is **out of scope** until equivalence is proven.

## 4.5 Preserve tier semantics

The lowest tier that yields a complete relation must remain the selected tier.

Do not reduce or reorder the declared candidate grid in this commit.

The only intended behavioral difference is avoiding computation for candidates belonging to tiers that are never reached.

## 4.6 Lossless safety fallback

Keep an opt-in legacy/eager mode so the benchmark can compare both paths in the same checkout.

Suggested coordinator option:

```python
relation_screen_mode: Literal["eager", "lazy_place"] = "eager"
```

or an equivalent private/testing switch.

During verification, run both on identical frozen plans.

Do **not** make `lazy_place` the only path until the correctness gate passes.

After verification, the production default may be changed in a separate tiny commit.

---

# 5. Correctness gate for C2

Use at least:

- tennis → bitong;
- gluestick → lvmukuai;
- carrot → shuazi;
- gluestick desk regression scene;
- at least one higher-tier-required relation case from tests/assets;
- at least 10 additional frozen/random scenes if already available under `assets/test_scenes/generated_random_batch_*`.

For each identical scene/task, compare `eager` vs `lazy_place`.

Required:

1. **Relation feasibility recall:** 100% on curated regression set; >=99% on expanded suite.
2. **Selected search tier:** must match unless both paths return a valid complete relation and the difference is explained by nondeterministic IK basin selection.
3. **Downstream segmented-chain success:** no >1 percentage-point regression on the tested suite.
4. **No safety relaxation:** same collision objects, same ignored object tuple, same disabled contact-link semantics, same IK seeds/tolerances.
5. Record any differing selected grasp/place IDs.

If lazy fails where eager succeeds, do not loosen tolerances. Record the scene and fall back to eager for that run while debugging.

---

# 6. Performance gate for C2

On RTX 5060 Ti, after planner warmup:

Primary target remains:

```text
P95 candidate build + relation screen <= 5.0 s
```

For the current three-task smoke suite, desired outcome:

- tennis remains < 1 s;
- carrot < 5 s;
- gluestick < 5 s;

Measure at least 10 warm repetitions per task for final C2 review if runtime permits.

Report:

```text
EAGER vs LAZY_PLACE

per task:
- P50/P90/P95/max wall time
- relation-found rate
- selected tier distribution
- backend coarse solver calls
- backend pose_tolerance solver calls
- requested/padded rows by screen kind
- downstream segmented-chain success
```

Commit the summary JSON/CSV under:

```text
benchmarks/task001/
```

Raw large JSONL may remain local; record its path in the handoff.

---

# 7. Tests required

Add/extend tests to prove:

1. high-tier continuous place candidates are **not** submitted to the manifold resolver when tier 0 finds a complete relation;
2. when tier 0 fails, tier 1 candidates are then resolved;
3. failed manifold candidates are not resolved repeatedly at later tiers;
4. grasp→place mapping remains correct after resolved-candidate replacement;
5. eager and lazy paths return a relation on the same deterministic fixture;
6. same contact-ignore tuple and `_CONTACT_ENDPOINT_COLLISION_LINKS` are used by the resolver;
7. normal strict place/preplace screen still runs after manifold resolution;
8. all existing tests pass.

---

# 8. Explicitly out of scope

Do not yet:

- reduce `coarse_ik_num_seeds`;
- reduce `num_ik_seeds`;
- shrink PEN_TABLE_GRASP seeds/shifts;
- shrink axial 16-angle placement symmetry;
- loosen pose tolerance;
- disable collision links in the normal coarse place screen;
- implement tiered grasp screening;
- alter segmented MotionGen;
- alter real robot execution;
- change grasp scores/ranking.

If C2 still misses 5 s, the next candidate is TASK 001-D: **true lazy grasp-tier screening** (currently all grasp/pregrasp candidates are solved before the tier loop).

---

# 9. Codex handoff format

Append results to this file:

```text
# CODEX HANDOFF — TASK 001-C

State:
C1 commit:
C2 commit:
Parent commit:
Changed files:

C1 backend-native counter result:
- tennis:
- carrot:
- gluestick:

C2 correctness:
- eager relation-found:
- lazy relation-found:
- feasibility recall:
- downstream plan success eager/lazy:
- differing selected relations:

C2 performance:
- tennis eager/lazy P50/P95:
- carrot eager/lazy P50/P95:
- gluestick eager/lazy P50/P95:
- suite eager/lazy P50/P95:

Solver accounting before/after:
- coarse calls/rows:
- pose_tolerance calls/rows:
- axis_constrained calls/rows:

Tests:
Failures:
Unexpected observations:
Raw log paths:
Committed summary path:
Open questions for ChatGPT:
```

Stop after C2 and wait for ChatGPT review before implementing tiered grasp screening or reducing any candidate set.
