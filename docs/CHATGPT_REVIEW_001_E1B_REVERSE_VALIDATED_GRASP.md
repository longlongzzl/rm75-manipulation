# ChatGPT Review — TASK 001-E1B Reverse-Validated Grasp Fallback

**Reviewed head:** `04cc994d4e7f414bcda4bf4fe184ade1a06f4148`  
**E1A implementation:** `39f2ef49260f179563130c3d58ba2e5bb08d1e9d`  
**E1A verdict:** `SAFE_BUT_INEFFECTIVE_FOR_CLUSTER_A`  
**Next action:** `IMPLEMENT_E1B_GRASP_REVERSE_PROBE_ONLY`

## 1. What E1A established

E1A was a useful negative result:

- all tests pass: `131 passed`;
- the 4 Cluster-A cases remain 0/3 each;
- both the primary forward grasp and tool-axis retry return `linear_planner_failed`;
- no previously successful frozen case regressed;
- the 16-case matrix remains 10/16.

Therefore do not spend another pass changing `project_distance_to_goal`, the approach axis, IK seeds, tolerances, collision rules, or grasp candidate generation.

The next hypothesis is narrower:

> The same Cartesian contact segment may be plan-able when solved from the known grasp endpoint outward to the already planned pregrasp pose, even when MotionGen fails to solve it in the forward direction.

This is analogous to the already-used reverse-validated place fallback, but it must be implemented conservatively and only on the failed grasp path.

## 2. Scope

Implement a third fallback after both existing grasp attempts fail:

```text
primary forward grasp
    -> if fail: E1A tool-axis forward retry
        -> if fail: E1B reverse grasp probe
```

Do not touch lift, preplace, place, retreat, relation screening, production `lazy_place`, candidate generation, MotionGen config, seeds, tolerances, or collision semantics.

## 3. Resolve the known grasp endpoint configuration

For the failed `grasp_candidate`, obtain the already-screened grasp joint solution using the existing cache helper:

```python
grasp_configuration = _cached_configuration(
    self.planner,
    grasp_candidate,
    pregrasp_end,
)
```

The reference must be `pregrasp_end` so periodic-joint alternatives stay near the actual incoming branch.

If no cached grasp configuration is available, record that the reverse fallback is unavailable and continue to the next grasp candidate. Do not run a new relaxed IK solve merely to enable E1B.

## 4. Reverse Cartesian probe

From that known grasp joint configuration, ask the existing linear planner to move outward to the exact existing `pregrasp_candidate`:

```text
current: cached grasp configuration
candidate: exact pregrasp_candidate
axis: z
project_distance_to_goal: true
ignore_object_name: task.object_name
allow_start_contact_escape: false
additional disabled collision links: none
```

Suggested stage name:

```text
grasp_reverse_probe
```

Why ignoring only `task.object_name` is correct here:

- the grasp contact segment intentionally terminates at the target object;
- the normal forward grasp already uses the same object-scoped contact exception;
- every other world/self collision must remain active.

Do not disable whole gripper links and do not broaden the ignored-object set.

## 5. Reverse and continuity-check the trajectory

If the reverse probe succeeds, reverse its trajectory with the existing `_reverse_trajectory()` helper.

Before accepting it as the forward grasp trajectory, require continuity with the already planned pregrasp state:

```python
start_gap = _trajectory_start_gap(reversed_trajectory, pregrasp_end)
```

Accept only when:

```text
start_gap <= 0.10 rad
```

Use the same 0.10-rad continuity convention already present in the reverse-place fallback. Do not silently bridge a larger mismatch in E1B.

On acceptance, create a successful `CandidatePlan` for the original grasp candidate with a status such as:

```text
reverse_validated_grasp_approach
```

and diagnostics including:

```text
reverse_start_gap_rad
reverse_probe_status
grasp_configuration_source = cached_endpoint_ik
```

If the reverse probe plans but continuity fails, record a `trajectory_discontinuity` failure and continue to the next grasp candidate.

## 6. Preserve failure evidence

For each candidate preserve, separately:

```text
grasp primary failure
grasp_tool_axis_retry failure
grasp_reverse_probe failure or discontinuity
```

On successful E1B recovery, successful-run diagnostics should expose at least:

```text
grasp_reverse_fallback_used: true/false
grasp_reverse_start_gap_rad
grasp_primary_status
grasp_tool_axis_retry_status
grasp_reverse_probe_status
```

This lets us distinguish an ordinary grasp from a reverse-validated recovery in later real-robot evaluation.

## 7. Required unit tests

Add focused tests proving:

1. primary grasp success does not invoke E1A/E1B;
2. E1A success does not invoke E1B;
3. primary + E1A failure causes one reverse probe when cached grasp configuration exists;
4. successful reverse probe is reversed and used as the forward grasp segment;
5. accepted reversed trajectory begins within <=0.10 rad of the actual `pregrasp_end`;
6. a >0.10-rad reversed-start mismatch is rejected as `trajectory_discontinuity`;
7. missing cached grasp configuration does not invent a new endpoint IK solve;
8. reverse probe uses only `ignore_object_name=task.object_name`, no extra collision-link exemptions;
9. stage execution order remains approach -> grasp -> lift -> preplace -> place -> retreat;
10. all repository tests pass.

## 8. GPU validation

### A. Cluster A targeted

Run 3 repetitions each on:

- current-table gluestick;
- generated 00;
- generated 02;
- generated 03.

Report per case:

```text
success_count / 3
reverse_probe_attempt_count
reverse_probe_success_count
reverse_fallback_used_count
continuity_reject_count
primary/tool-axis/reverse statuses
selected grasp/place on success
```

### B. Cluster B observational

Run generated 05 and 06 once each or 3x if convenient, but do not change lift behavior in this task. The purpose is only to verify E1B does not create a regression before the known lift failure.

### C. Frozen 16-case matrix

Run one repetition per case using production `lazy_place`.

Report:

```text
baseline D2 success = 10/16
E1B success = ?/16
newly recovered cases
previous-success -> new-failure cases
new exceptions
upstream relation selection differences
```

Acceptance:

- zero previous-success -> new-failure cases;
- zero new exceptions;
- no upstream relation-screen changes caused by E1B;
- every claimed recovery must identify whether E1B reverse fallback was actually used.

Do not claim success merely because a different downstream grasp branch happened to win.

## 9. Stop condition

After implementation, tests, targeted Cluster-A runs, and 16-case matrix are committed, stop for ChatGPT review.

Do not implement a lift fix in the same commit.

If E1B recovers all 4 Cluster-A cases, expected matrix success is approximately 14/16 and the next task will isolate generated 05/06 lift failures.

If E1B recovers none, stop anyway and report whether the reverse probe itself failed, or whether it planned but failed the continuity check. That distinction determines the next hypothesis.

## 10. Required handoff

Append to `docs/CODEX_TASK_001_D.md`:

```text
# E1B completion

E1B implementation commit:
Full tests:

Cluster A x3:
- current_table_gluestick:
- generated_00:
- generated_02:
- generated_03:

Reverse probe attempted/succeeded/used:
Continuity rejects:
Primary/tool-axis/reverse statuses:

16-case baseline -> E1B:
Recovered cases:
Previous-success/new-failure cases:
New exceptions:
Upstream relation differences:

Summary path:
Raw logs:
Open questions for ChatGPT:
```
