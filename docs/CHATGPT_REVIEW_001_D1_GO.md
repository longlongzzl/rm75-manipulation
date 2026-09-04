# ChatGPT Review — TASK 001-D1 GO

**Reviewed head:** `7e0945d829aef42a6bd90d7ffcf0f606a0119adb`  
**D0 verdict:** `PASS`  
**Next action:** `START_D1_NOW`

## 1. D0 is accepted

The completed D0 evidence satisfies the gate for continuing from C2 `lazy_place` to D1:

- cube/box coverage added with `lvmukuai`;
- generic-asymmetric coverage added with `shuazi`;
- curated eager/lazy relation recall remains 100%;
- generated eager/lazy relation recall remains 20/20 vs 20/20;
- expanded full-chain matrix: eager 10/16, lazy 10/16;
- zero eager-success/lazy-failure cases;
- zero selected-relation differences;
- no recurrence of the interpolation-capacity exception at buffer size 640.

The equal full-chain failures are an important later reliability issue, but they are **not** a blocker for D1 because D1 is a lossless scheduling optimization of relation screening and lazy_place shows no downstream regression relative to eager.

Do not rerun or redesign D0 before starting D1.

## 2. Implement D1 exactly as the progressive-preplace optimization

Add a third opt-in relation-screen mode, for example:

```python
lazy_place_progressive_preplace
```

Keep both existing modes unchanged:

```python
eager
lazy_place
```

Do not change the production default in this task.

### Required semantic order

The current search-tier priority must remain authoritative.

For each `search_tier` in increasing order:

1. resolve/screen the place candidates enabled at that tier using the existing lazy-place behavior;
2. consider preplace candidates grouped by `metadata["preplace_clearance_rank"]`;
3. coarse-screen rank 0 only;
4. merge feasible preplaces into a cumulative feasible set;
5. rebuild complete grasp/place/preplace relations using all cumulative feasible preplaces for this tier;
6. if a complete relation exists, stop at this tier;
7. otherwise screen the next clearance rank;
8. only after all clearance ranks for the current search tier fail may the coordinator advance to the next `search_tier`.

Do **not** allow a higher search tier to win merely because the nominal clearance of the lower tier failed.

## 3. No duplicate endpoint work

Within one coordinator run maintain the equivalent of:

```python
screened_preplace_ids: set[str]
preplace_feasible_ids: set[str]
```

Every preplace endpoint must be coarse-solved at most once.

Use the existing `preplace_clearance_rank` metadata written by `_place_approach_candidates()`; do not parse rank from candidate IDs.

Keep cumulative feasibility across ranks within the current relation-screen run so a feasible lower-rank endpoint remains available when higher ranks are added.

Place endpoint feasibility already computed for the same scene/collision semantics must not be recomputed just because another preplace rank is being added.

## 4. Collision/correctness constraints remain frozen

This task is only compute scheduling.

Do not change:

- IK seeds;
- grasp tilt/contact-shift candidate sets;
- symmetry samples;
- terminal pose tolerances;
- place/preplace collision strictness;
- ignored-object sets;
- contact-link collision semantics;
- MotionGen parameters;
- interpolation settings;
- real-robot execution.

Do not remove any preplace candidate from the search space. D1 may only defer fallback clearance ranks until they are needed.

## 5. Unit tests required before GPU benchmark

Add tests proving at least:

1. rank-0 success prevents ranks 1+ from being submitted;
2. rank-0 failure submits rank 1;
3. rank-1 success prevents later ranks;
4. all ranks of search tier T are exhausted before search tier T+1 is accepted;
5. no preplace candidate is coarse-screened twice;
6. cumulative feasible preplaces are retained across ranks;
7. complete relation mapping is unchanged;
8. existing `eager` and `lazy_place` modes remain available and behavior-compatible;
9. collision-ignore/disable-link semantics are unchanged;
10. full repository tests pass.

## 6. Benchmark protocol

### A. Primary timing benchmark

Compare in the **same D1 checkout**:

```text
lazy_place
vs
lazy_place_progressive_preplace
```

Use the existing three frozen smoke tasks:

- tennis;
- gluestick;
- carrot (`carriot`).

Run at least 10 warm repetitions per task/mode. The benchmark utility now permits one repetition for full-chain validation, but **do not use one repetition for timing percentiles**.

Report per task and suite:

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

### B. Solver-work accounting

Report before/after by `screen_kind`, especially `coarse`:

```text
solver_calls
rows_requested
rows_padded
```

The key reference bottleneck from C2 is carrot lazy-place approximately:

```text
coarse: 123 solver calls / 7705 requested / 7872 padded rows
```

D1 should reduce this materially if fallback preplace clearances are often unnecessary.

### C. Correctness

Run progressive mode on the D0 curated/expanded screen-only matrix and compare to `lazy_place`.

Acceptance:

- curated relation recall: 100% relative to lazy_place;
- expanded relation recall: >=99% relative to lazy_place;
- zero lazy-place-success / progressive-failure curated cases;
- log every selected tier/grasp/place difference.

### D. Downstream full-chain non-regression

Use the same D0 16-case full-chain matrix with one repetition per mode:

```text
lazy_place
vs
lazy_place_progressive_preplace
```

Acceptance:

- progressive success regression <= 1 percentage point;
- zero lazy-place-success / progressive-failure curated cases;
- any generated lazy-success/progressive-failure case must be frozen and diagnosed;
- no new interpolation-capacity exception;
- selected relation differences must be listed explicitly.

Equal failures already present under lazy_place may remain recorded as baseline failures.

## 7. Performance acceptance

Primary target:

```text
suite warm P95 <= 5.0 s
```

Desired robustness target:

```text
carrot warm P95 <= 3.0 s
```

Correctness takes priority. Do not alter candidate sets, tolerances, seeds, or collision rules merely to reach the 3 s desired target.

If carrot does not materially improve, stop after collecting endpoint-work diagnostics; do not open a new optimization inside the same D1 commit.

## 8. Commit structure

Keep implementation/tests and benchmark evidence separable if practical. At minimum, do not mix a production-default switch into D1.

Commit summary evidence to:

```text
benchmarks/task001/task001_d1_progressive_preplace_summary.json
```

Append the final handoff to `docs/CODEX_TASK_001_D.md`:

```text
# D1 completion

D0 accepted by ChatGPT: yes
D1 implementation commit:
D1 benchmark/evidence commit:
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

After D1 implementation, tests, timing benchmark, expanded correctness, and full-chain comparison are committed, **stop and wait for ChatGPT review**.

Do not perform D2 and do not switch the production default yet.

A later task will separately address the current equal full-chain baseline failures (10/16 success on the expanded D0 matrix); do not mix that reliability investigation into D1.
