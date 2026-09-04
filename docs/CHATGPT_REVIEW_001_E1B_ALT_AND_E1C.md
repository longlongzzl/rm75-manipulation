# ChatGPT Review — E1B-alt result and E1C linear-planner diagnosis

**Repository:** `longlongzzl/rm75-manipulation`  
**Reviewed head:** `49f0385a2c638011bcdc6fce15d7a9fd71747da3`  
**E1B-alt implementation:** `ff5b0f20260d79caef2faa41247ca812d70f420a`  
**Verdict:** `NEGATIVE_RESULT_ACCEPTED — STOP ADDING GRASP FALLBACKS`  
**Next action:** `DIAGNOSE_LINEAR_PLANNER_FAILURE_ONLY`

---

## 1. E1B-alt review verdict

The implementation is appropriately isolated:

- production `grasp_fallback_mode` defaults to `primary_only`;
- E1A tool-axis retry is no longer in the production hot path;
- reverse probing is experimental only;
- no candidate, seed, tolerance, or collision semantic was relaxed;
- all tests pass (`134 passed`);
- frozen 16-case success remains `10/16` with zero previous-success regressions and zero new exceptions.

The result is unambiguous:

- Cluster A targeted attempts: 96 reverse probes;
- cached grasp endpoint configuration available: 96/96;
- reverse probe trajectory produced: 0/96;
- continuity rejection: 0, because no reverse trajectory existed;
- Cluster A recovered: 0/4;
- matrix success: `10/16 -> 10/16`.

This rules out the two preceding hypotheses as useful production fixes:

1. switching forward linear projection mode (E1A);
2. solving the exact segment in reverse and reversing the trajectory (E1B-alt).

Do **not** add a fourth grasp fallback.

---

## 2. Important backend observation

Current `plan_linear_candidates()` collapses two qualitatively different failure classes:

### Class L0 — `planner.plan_pose(...)` returns `None`

Reported today as:

```text
linear_planner_failed
```

The diagnostic only audits the start state for collision.

### Class L1 — `planner.plan_pose(...)` returns a result but no successful row

Reported today as:

```text
linear_failed (<raw.status or unknown>)
```

E0/E1B evidence shows both classes occur:

- primary grasp failures are predominantly L0 (`linear_planner_failed`);
- E1B reverse probes return L1 (`linear_failed (unknown)`).

Endpoint screening has already shown that both semantic endpoints have IK solutions. Therefore the next task must determine **what fails between endpoint reachability and constrained trajectory generation**.

---

# TASK 001-E1C — Diagnostic-only linear failure decomposition

**State:** `PROPOSED`

## 3. Scope

E1C must not change production planning behavior.

Implement diagnostics behind an explicit opt-in flag or benchmark-only entrypoint, for example:

```python
linear_failure_diagnostics=False
```

or a dedicated backend method called only by the benchmark.

Production default must remain:

```text
relation_screen_mode = lazy_place
grasp_fallback_mode = primary_only
```

No real robot execution is required or allowed in this task.

---

## 4. Per-failed-grasp probes

For each failed Cluster-A grasp candidate, record the following **without using the probe output as the production trajectory**.

### Probe A — endpoint geometry consistency

From:

- actual planned `pregrasp_end` joint state;
- exact `grasp_candidate.pose`;
- exact `pregrasp_candidate.pose`;

record:

```text
cached_grasp_available
cached_grasp_joint_distance_from_pregrasp
pregrasp_fk_position
pregrasp_fk_quaternion
target_position
target_quaternion
translation_distance_m
rotation_distance_rad
```

Also decompose the pregrasp→grasp displacement against the intended approach direction used by `_approach_offset_candidates()`:

```text
intended_linear_distance_m
actual_translation_along_approach_m
orthogonal_translation_error_m
```

The purpose is to verify that the two poses actually define the line we think they define after the preceding MotionGen approach stage.

Do not infer this from nominal poses only; use FK of the **actual planned pregrasp_end**.

### Probe B — start and cached-goal collision audit

With exactly the same target-object ignore semantics as the normal grasp segment, audit:

```text
pregrasp_end collision pairs
the cached grasp endpoint configuration collision pairs
```

Record self/world collision separately, including penetration depth.

If the cached goal is colliding with an unrelated obstacle/self while coarse screening says success, this is a cache/collision-context bug and should be fixed before any trajectory heuristic.

### Probe C — C-space connectivity to the cached endpoint

Using the exact cached grasp joint configuration selected near `pregrasp_end`, call the existing C-space planner diagnostically:

```python
plan_to_configuration(
    current=pregrasp_end,
    target=cached_grasp_configuration,
    scene=task.scene,
    ...
)
```

Important: apply the **same target-object contact exception** as the grasp segment during this diagnostic. If `plan_to_configuration()` cannot currently accept the ignore context, add a benchmark-only/backend diagnostic helper rather than changing production semantics.

Record:

```text
cspace_success
cspace_status
cspace_time_s
cspace_trajectory_points
```

Interpretation:

- C-space fails too → likely collision/topology/endpoint-state issue;
- C-space succeeds while linear fails → linear constraint / pose-planning issue.

### Probe D — unconstrained pose planning

From the same `pregrasp_end`, to the same exact `grasp_candidate.pose`, same world/collision ignore semantics, call `planner.plan_pose()` with the default `ToolPoseCriteria()` rather than `linear_motion(...)`.

This is diagnostic only.

Record:

```text
unconstrained_pose_raw_is_none
unconstrained_pose_success
unconstrained_pose_raw_status
unconstrained_pose_time_s
```

Interpretation:

- unconstrained succeeds, linear fails → the linear pose criterion / constrained optimization is the main culprit;
- unconstrained also fails while C-space succeeds → pose IK/branch/pose-planning issue;
- C-space and unconstrained both fail → likely collision/graph/topology or current-dependent endpoint issue.

### Probe E — official cuRobo2 `plan_grasp` comparison

The backend already implements `plan_grasps()` and exposes cuRobo2's official goalset → approach → grasp pipeline, including stage-level status and endpoint/graph diagnostics.

Run it **as a diagnostic comparator only** on the same Cluster-A candidates.

Use:

- same scene;
- same current task state;
- same grasp candidate;
- target object name = task object;
- approach offset matching the task pregrasp offset;
- approach axis = `z`;
- approach in tool frame = `True`, matching candidate-local approach semantics;
- `plan_lift=False` for E1C.

Record:

```text
official_plan_grasp_success
official_plan_grasp_status
goalset_success
approach_success
grasp_success
collision_diagnostics
```

Do not switch production coordinator to `plan_grasp` in this task.

This comparison is high leverage:

- official succeeds while custom linear fails → segmented custom grasp primitive is the problem, and a bounded production fallback may be justified next;
- official also fails at grasp → inspect its stage diagnostics before changing anything;
- official fails at goalset/approach → our coarse relation screen and final planner disagree in a more fundamental way.

---

## 5. Raw `plan_pose` status instrumentation

Current `plan_linear_candidates()` should expose more information when `raw` is present:

```text
raw.status
raw.success shape / any-success
position_error if available
rotation_error if available
interpolated trajectory availability
js_solution availability
```

When `raw is None`, report explicitly:

```text
raw_is_none = true
```

Do not fabricate an `unknown` status when the distinction `None` vs unsuccessful raw result is available.

If cuRobo2 exposes optimizer/graph status fields deeper in the result object, log them generically using a small allowlist of scalar/string fields; do not serialize arbitrary huge tensors.

---

## 6. Classification output

For each failed grasp candidate, assign one diagnostic class after probes:

```text
G0 endpoint/collision inconsistency
G1 C-space connectivity failure
G2 C-space success + unconstrained pose failure
G3 unconstrained pose success + linear constraint failure
G4 official plan_grasp success while custom linear fails
G5 official plan_grasp failure with identifiable official stage
G6 unresolved
```

A candidate can carry multiple evidence tags, but choose one primary class.

The benchmark summary must count candidates and cases per class.

---

## 7. Validation set

Run E1C on:

### Cluster A

- `current_table_gluestick`
- `generated_00`
- `generated_02`
- `generated_03`

One full diagnostic repetition per case is sufficient initially because E0/E1A/E1B already showed deterministic 3/3 failure. If classification differs between candidates unexpectedly, run 3 repetitions.

### Positive controls

Also run at least two previously successful full-chain cases and collect the same geometry/collision probes for their selected grasp segments.

This is critical: diagnostic thresholds such as orthogonal error or cached joint distance are meaningful only relative to successful examples.

### Cluster B

Do not diagnose lift in E1C. Leave `generated_05/06` unchanged.

---

## 8. Acceptance / stop condition

E1C is complete when:

1. full tests pass;
2. production behavior is unchanged;
3. 16-case production matrix remains `10/16` if rerun;
4. every Cluster-A case receives a concrete failure decomposition with the probes above;
5. no new grasp fallback is added.

Stop for ChatGPT review after the diagnostic summary is committed.

The next code fix must be selected from evidence:

- if G3 dominates: focus on linear criterion/trajectory formulation;
- if G4 dominates: compare/adopt official `plan_grasp` only after a separate non-regression task;
- if G1 dominates: investigate collision/topology or endpoint configuration selection;
- if G2 dominates: investigate final-state IK branch and pose-planner seeding;
- if G0 appears: fix scene/cache/collision correctness first.

---

## 9. Required committed evidence

Add a compact summary under:

```text
benchmarks/task001/task001_e1c_linear_failure_diagnosis_summary.json
```

Do not commit giant tensor dumps.

Raw detailed JSONL can stay under `/tmp/...`; record its path.

Summary should contain:

```text
full_tests
production_mode
cluster_a_cases
positive_controls
per_case_primary_class
class_counts
cspace_success_count
unconstrained_pose_success_count
official_plan_grasp_success_count
raw_none_count
raw_unsuccessful_count
endpoint_collision_inconsistency_count
production_matrix_if_rerun
raw_log_paths
```

Append a Codex handoff to `docs/CODEX_TASK_001_D.md` or a new `docs/CODEX_HANDOFF_001_E1C.md`.

---

## 10. Collaboration ownership rule

Files named `CHATGPT_REVIEW_*` are authored only by ChatGPT.

Local Codex should write only:

- `CODEX_TASK_*` when explicitly asked to copy an existing task;
- `CODEX_HANDOFF_*`;
- benchmark summaries;
- implementation code/tests.

Do not create a new file that claims `Reviewed by ChatGPT`, `ChatGPT verdict`, or `ChatGPT approved` unless that text was actually committed by ChatGPT through this review loop.
