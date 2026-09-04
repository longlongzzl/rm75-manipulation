# ChatGPT Review — TASK 001-E1A Grasp Tool-Axis Retry

**Reviewed head:** `201496e482c9c20c731b19dd4d11632a2157d32b`  
**E0 verdict:** `PASS`  
**Next action:** `FIX_GRASP_CLUSTER_FIRST`

## 1. E0 diagnosis is accepted

The six baseline full-chain failures reproduce deterministically at 3/3 per case and split into two stable clusters:

- **Cluster A — grasp contact-line failure (4 cases):**
  - `current_table_gluestick`
  - `generated_00`
  - `generated_02`
  - `generated_03`
  - deepest stage: `grasp`
  - common status: `linear_planner_failed`
  - start-state collision diagnostics: empty

- **Cluster B — post-grasp lift failure (2 cases):**
  - `generated_05`
  - `generated_06`
  - some grasps pass; deepest stage is `lift`
  - statuses include `linear_planner_failed` / `linear_failed (unknown)`

Do **not** mix the two clusters in one fix. E1A addresses only Cluster A.

## 2. Why the first E1A hypothesis is tool-axis geometry, not relaxed collision

The coordinator constructs each pregrasp with `_approach_offset_candidates()`, which moves away from the contact pose along the candidate TCP's **local -Z approach axis**.

The current segmented grasp stage then calls the linear planner with its existing default `project_distance_to_goal=False`.

This mismatch is a plausible explanation for the stable Cluster-A pattern:

- both pregrasp and grasp endpoints passed broad endpoint screening;
- the target object is already scoped out for intended grasp contact;
- E0 reports no collision at the linear start state;
- nevertheless the linear planner returns no trajectory.

Before introducing reverse planning, seed changes, collision exemptions, or candidate changes, test the smallest geometry-consistent fallback: retry the **same pregrasp -> grasp segment** using the tool-axis projection mode already used elsewhere for tool-frame linear motion.

## 3. Approved E1A implementation

Modify only the segmented-chain grasp failure path in `rm75_app/pickplace/coordinator.py`.

Keep the existing primary attempt unchanged:

```python
grasp = self._plan_linear_stage(
    stage="grasp",
    current=pregrasp_end,
    candidate=grasp_candidate,
    task=task,
    ignore_object_name=task.object_name,
)
```

If and only if that primary attempt fails, make one fallback attempt on the **same endpoints**:

```python
grasp = self._plan_linear_stage(
    stage="grasp_tool_axis_retry",
    current=pregrasp_end,
    candidate=grasp_candidate,
    task=task,
    axis="z",
    project_distance_to_goal=True,
    ignore_object_name=task.object_name,
)
```

Requirements:

- primary success path must remain byte-for-byte behavior-equivalent where practical;
- retry only after primary grasp failure;
- same `pregrasp_end` and same `grasp_candidate`;
- same object-contact ignore (`task.object_name`);
- no extra disabled collision links;
- `allow_start_contact_escape=False` (the retry starts at pregrasp, not at contact);
- no new grasp candidates;
- no seed/tolerance/MotionGen configuration changes;
- no changes to relation screening;
- no changes to lift yet.

If the retry succeeds, use its trajectory as the grasp trajectory and continue the existing chain normally.

Preserve both primary and retry failure diagnostics when both fail.

## 4. Diagnostics

Add a successful-run diagnostic field such as:

```text
grasp_tool_axis_retry_used: true/false
```

For failure logs, preserve two distinct stages:

```text
grasp
grasp_tool_axis_retry
```

Do not collapse them into one generic `linear_planner_failed` record.

## 5. Unit tests

Add focused tests proving:

1. primary grasp success does not invoke tool-axis retry;
2. primary grasp failure + tool-axis retry success continues the chain;
3. primary + retry failure preserves both failure records and proceeds to the next grasp candidate;
4. retry uses `project_distance_to_goal=True`;
5. retry still ignores only the grasped object and does not add collision-link exemptions;
6. successful retry does not change later attach/lift/place sequencing;
7. full repository tests pass.

Do not add a reverse-grasp fallback in the same commit.

## 6. GPU/full-chain validation

### A. Targeted six-case rerun

Run the exact E0 six cases with the production default `lazy_place`, at least **3 repetitions per case**.

Report per case:

```text
success_count / repetitions
deepest_failure_stage
grasp_tool_axis_retry_used_count
primary grasp status
retry grasp status
selected grasp
selected place
```

Primary E1A question:

> Do the four Cluster-A cases become full-chain plannable without changing endpoint/collision semantics?

Do not require Cluster B (`generated_05/06`) to improve in E1A.

### B. 16-case non-regression

Run the same frozen 16-case full-chain matrix once with the E1A checkout.

Record:

```text
baseline D2 success: 10/16
E1A success: X/16
previous-success -> new-failure cases
newly-recovered cases
selected-relation differences
new exceptions
```

Acceptance:

- **zero previous-success -> new-failure cases**;
- zero new exceptions;
- relation-screen selections remain unchanged (E1A is downstream only);
- all four Cluster-A cases should preferably recover; partial recovery is still useful evidence but must be reported honestly.

If all four Cluster-A cases recover and the two lift cases remain failures, the expected matrix is approximately **14/16**. This is a target implied by E0 clustering, not a pass condition to be achieved by relaxing constraints.

## 7. Stop condition

After implementation, tests, six-case x3 validation, and 16-case validation are committed, **stop for ChatGPT review**.

Do not start the lift fix in the same pass.

If tool-axis retry recovers none or only a minority of Cluster A, stop and report the exact retry statuses. The next hypothesis will be a reverse-validated grasp segment, but it must not be added automatically in E1A.

## 8. Required handoff

Append to `docs/CODEX_TASK_001_D.md`:

```text
# E1A completion

E0 accepted by ChatGPT: yes
E1A implementation commit:
Full tests:

Cluster A targeted x3:
- current_table_gluestick:
- generated_00:
- generated_02:
- generated_03:
Cluster B observational x3:
- generated_05:
- generated_06:

Tool-axis retry usage:
Primary/retry statuses:
16-case baseline -> E1A success: 10/16 -> X/16
Recovered cases:
Previous-success/new-failure cases:
Selected relation differences:
New exceptions:
Summary path:
Raw logs:
Open questions for ChatGPT:
```
