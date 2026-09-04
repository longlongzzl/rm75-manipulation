# ChatGPT Review — TASK 001-D1 implementation

**Reviewed implementation:** `d93b45ffe6239281501962edd9d3e21c70af3021`  
**D0 status:** accepted  
**Verdict:** `IMPLEMENTATION_ACCEPTABLE_FOR_BENCHMARK; D1_NOT_YET_COMPLETE`

## 1. What is accepted in the current implementation

The implementation direction matches the approved D1 design:

- a third opt-in mode `lazy_place_progressive_preplace` was added;
- existing `eager` and `lazy_place` modes remain available;
- `search_tier` remains the outer priority;
- preplace candidates are grouped by `metadata["preplace_clearance_rank"]`;
- fallback preplace ranks are screened progressively;
- cumulative screened/feasible ID sets are maintained;
- the code stops after a complete relation is found for the current tier;
- no production-default switch is included in this commit;
- no seed/tolerance/collision/MotionGen parameter change is included.

This is suitable for GPU validation. Do not redesign the algorithm before measuring it unless a required test exposes a correctness defect.

## 2. D1 is not complete yet

Commit `d93b45f` contains only implementation/test/benchmark-tool changes. It does **not** contain the required D1 evidence:

- no `benchmarks/task001/task001_d1_progressive_preplace_summary.json`;
- no final D1 handoff appended to `docs/CODEX_TASK_001_D.md`;
- no recorded full repository test result;
- no 10-warm-run timing comparison;
- no solver-work before/after accounting;
- no expanded screen-only correctness comparison;
- no 16-case full-chain lazy-vs-progressive comparison.

Therefore do not start D2 and do not change the production default.

## 3. Unit-test coverage must be completed before benchmarking

The current commit adds a useful test proving the easy path: rank-0 success stops later ranks and avoids duplicate endpoint submission.

Before GPU timing, add focused tests (or explicitly cite existing tests if they already prove the condition) for the remaining semantics:

1. rank-0 preplace failure submits rank 1;
2. rank-1 success prevents ranks 2+;
3. all preplace ranks of search tier T are exhausted before search tier T+1 may be accepted;
4. cumulative feasible preplaces from an earlier rank remain usable after adding a later rank;
5. no preplace endpoint is coarse-screened twice on a fallback path, not only the rank-0-success path;
6. selected grasp/place relation remains identical to ordinary `lazy_place` in a deterministic fixture where both are feasible;
7. collision-ignore / disabled-link semantics are unchanged on the progressive path;
8. `eager` and ordinary `lazy_place` behavior remains available.

Do not add redundant tests merely to reach a count. Existing tests may be named in the handoff if they already cover an item.

Then run:

```bash
PYTHONPATH=. python -m pytest tests -q
```

Record the exact pass count.

## 4. Required D1 timing benchmark

Compare in the same checkout:

```text
lazy_place
lazy_place_progressive_preplace
```

Use the same three frozen smoke plans used for C2/D0:

- tennis;
- gluestick;
- carrot (`carriot`).

Run at least **10 warm repetitions per task per mode** with synchronized wall timing.

Report per task and combined suite:

```text
P50
P90
P95
max
relation_found_rate
selected_search_tier
selected_grasp
selected_place
selected_relation_difference_count
```

Acceptance target:

```text
suite warm P95 <= 5.0 s
```

Desired margin:

```text
carrot warm P95 <= 3.0 s
```

The 3 s target is not a reason to change seeds, tolerances, collision checks, or candidate sets.

## 5. Required solver-work accounting

For each smoke task, compare ordinary lazy-place against progressive by `screen_kind`:

```text
solver_calls
rows_requested
rows_padded
```

Especially record `coarse` work.

Reference carrot C2 lazy-place bottleneck:

```text
coarse ~= 123 solver calls / 7705 requested / 7872 padded rows
```

The main purpose of D1 is to reduce this work by avoiding unnecessary fallback preplace clearances.

Also record, if practical without changing behavior:

```text
preplace ranks attempted
preplace endpoints submitted per rank
unique preplace endpoint count
```

This will make it obvious whether the progressive scheduling is doing what the code intends.

## 6. Expanded correctness comparison

Run `lazy_place` vs `lazy_place_progressive_preplace` on the same D0 screen-only matrix, including:

- tennis;
- gluestick;
- carrot;
- gluestick desk regression;
- `lvmukuai` cube fixture;
- `shuazi` asymmetric fixture;
- the same 10 frozen generated scenes.

Acceptance:

- curated relation recall = 100% relative to lazy-place;
- expanded relation recall >= 99% relative to lazy-place;
- zero lazy-place-success/progressive-failure curated cases;
- list every selected tier/grasp/place difference.

## 7. Full-chain non-regression

Run the same D0 **16-case** full-chain matrix with one repetition per mode:

```text
lazy_place
lazy_place_progressive_preplace
```

Record:

```text
case_id
mode
relation_found
selected_search_tier
selected_grasp
selected_place
full_chain_plan_success
failure_stage
segmented_plan_time_s
```

Acceptance:

- progressive full-chain success regression <= 1 percentage point;
- zero lazy-place-success/progressive-failure curated cases;
- any generated lazy-success/progressive-failure case must be frozen and diagnosed;
- no new interpolation-capacity exception;
- selected-relation differences must be explicit.

The known equal baseline failures from D0 are allowed to remain equal failures. Do not fix those inside D1.

## 8. Required evidence commit

Create:

```text
benchmarks/task001/task001_d1_progressive_preplace_summary.json
```

It should contain at minimum:

```text
implementation_commit
full_tests
smoke_timing_lazy
smoke_timing_progressive
suite_timing_lazy
suite_timing_progressive
solver_work_lazy
solver_work_progressive
expanded_screen_correctness
full_chain_16_lazy
full_chain_16_progressive
selected_relation_differences
lazy_success_progressive_failure_cases
acceptance
raw_log_paths
```

Append to `docs/CODEX_TASK_001_D.md`:

```text
# D1 completion

D1 implementation commit: d93b45ffe6239281501962edd9d3e21c70af3021
D1 evidence commit:
Full tests:

Timing lazy -> progressive:
- tennis P50/P95:
- gluestick P50/P95:
- carrot P50/P95:
- suite P50/P95:

Solver work lazy -> progressive:
- tennis:
- gluestick:
- carrot:

Curated relation recall:
Expanded relation recall:
Selected relation differences:
Downstream full-chain lazy/progressive:
Lazy-success/progressive-failure cases:
Summary path:
Raw logs:
Failures/unexpected observations:
Open questions for ChatGPT:
```

## 9. Stop condition

After the evidence commit is pushed, stop for ChatGPT review.

Do **not** implement D2 or switch the default yet.
