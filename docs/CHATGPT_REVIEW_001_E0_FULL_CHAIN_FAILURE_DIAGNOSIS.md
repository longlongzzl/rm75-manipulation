# ChatGPT Review — TASK 001-E0: Diagnose the remaining full-chain failures

**Reviewed head:** `3f369dc554cfa72d78200077e4104162141f1b82`  
**D2 verdict:** `PASS`  
**Production relation-screen default:** `lazy_place`  
**Next action:** `DIAGNOSE_ONLY_BEFORE_ANY_RELIABILITY_FIX`

## 1. Close TASK 001-D

TASK 001-D is accepted and complete.

Evidence at the D2 production path:

- the default switch commit changes only `PickPlaceCoordinator.__init__.relation_screen_mode` from `eager` to `lazy_place`;
- full tests: `128 passed in 17.00s`;
- default-path smoke: relation-found `3/3`, warm P50 ~= `1.621 s`, warm P95 ~= `4.391 s`;
- expanded full-chain: `10/16` success;
- no new exceptions/regressions after the default switch.

Do not reopen relation-screen performance work in TASK 001-E.

## 2. Reliability problem to solve next

The remaining issue is downstream segmented MotionGen/full-chain reliability, not relation screening.

The six known equal baseline failures are:

1. current-table `gluestick`;
2. generated scene `00`;
3. generated scene `02`;
4. generated scene `03`;
5. generated scene `05`;
6. generated scene `06`.

These were failures under both eager and lazy-place and therefore are not regressions introduced by the optimization.

Do **not** change seeds, tolerances, collision rules, candidate sets, MotionGen configuration, placement clearances, or robot execution in E0.

E0 is diagnosis only.

## 3. First use the diagnostics that already exist

`PickPlaceCoordinator._run_segmented_chain()` already records unsuccessful planning attempts in:

```python
result.diagnostics["candidate_failures"]
```

Each failure can contain at least:

```text
stage
candidate_id
status
planner diagnostics
```

Relevant stage names already include paths such as:

```text
pregrasp
pregrasp_from_initial
pregrasp_fallback_initial
grasp
lift
preplace
place
place_reverse_probe
```

Therefore, do not add production tracing before first checking whether these existing diagnostics are sufficient.

## 4. Freeze and rerun exactly the six failures

Use the current production default (`lazy_place`) and the same frozen plan/scene pairs used by D2.

Run each failure case at least **3 repetitions** with the no-op full-chain executor.

Do not regenerate LLM plans and do not perturb object poses between repetitions.

For each repetition record:

```text
case_id
object_id
scene_path
relation_screen_mode
relation_found
selected_search_tier
relation_grasp_count
full_chain_plan_success
failure_stage
segmented_plan_time_s
candidate_failure_count
candidate_failure_stage_histogram
candidate_failure_status_histogram
nearest/deepest attempted stage
unique failed grasp candidate ids
unique failed place candidate ids (when recoverable)
unique failed preplace candidate ids
```

Also preserve the raw `candidate_failures` for each run in local JSONL.

## 5. Classify each case by the deepest failure reached

Use the following semantic order for diagnosis:

```text
pregrasp
-> grasp
-> lift
-> preplace
-> place
-> place_reverse_probe
```

For each case, report the deepest stage reached before all candidate chains were exhausted.

Classify the dominant failure into one of:

### A. pregrasp free-space planning

Examples:

- no MotionGen route to pregrasp;
- graph/trajectory optimization failure before contact.

### B. grasp contact-line planning

Examples:

- endpoint is coarse-IK feasible but the linear contact approach fails;
- collision/status evidence localizes the failure to grasp.

### C. post-grasp lift

Examples:

- both world-Z and tool-Z lift variants fail after attachment.

### D. preplace free-space planning with attached object

Examples:

- grasp/lift succeed but MotionGen cannot reach any feasible preplace configuration.

### E. final place line / reverse-probe

Examples:

- preplace succeeds, but direct place line fails;
- reverse probe also fails or produces trajectory discontinuity.

### F. mixed / unstable

Use this only if different repetitions genuinely terminate at different semantic stages or the current diagnostics cannot identify a dominant stage.

Do not call a case `mixed` merely because earlier candidate attempts also failed. The key signal is the **deepest reachable stage across attempted chains**.

## 6. Status-level evidence

For every failed stage, aggregate planner `status` values rather than only reporting `segmented_chain`.

Examples may include graph, trajectory optimization, linear collision, pose/terminal, or trajectory discontinuity statuses depending on the backend result.

If a diagnostics payload includes collision or terminal information, preserve compact fields such as:

```text
world_object
robot_link
penetration_m
position_error_m
orientation_error_rad
start_gap_rad
```

Do not infer a collision cause without actual diagnostics.

## 7. If the existing diagnostics are insufficient

Only if E0 cannot associate failures to a concrete grasp/place/preplace chain, create a **separate diagnostics-only commit** that adds an attempt trace to `_run_segmented_chain()`.

The trace should record, without changing control flow:

```text
grasp_candidate_id
place_candidate_id
preplace_candidate_id
stage
success/failure
status
```

Requirements:

- no planner call order changes;
- no candidate sorting/ranking changes;
- no new retries;
- no tolerance/collision/configuration changes;
- no real-robot changes;
- full tests must remain green.

Do not implement a reliability fix in the same commit as diagnostic instrumentation.

## 8. Required E0 output

Commit a compact summary to:

```text
benchmarks/task001/task001_e0_full_chain_failure_diagnosis.json
```

The summary must contain:

```text
production_default: lazy_place
failure_case_count: 6
repetitions_per_case: >=3
per_case:
  - case_id
  - reproduction_rate
  - dominant/deepest_failure_stage
  - stage_histogram
  - status_histogram
  - relevant candidate ids
  - compact diagnostic evidence
failure_clusters:
  - cluster name
  - case ids
  - shared evidence
next_fix_recommendation_by_cluster
```

`next_fix_recommendation_by_cluster` is a recommendation only. Do not implement it in E0.

Append a handoff to a new file or to the existing task log with:

```text
# E0 handoff

E0 commit:
Full tests:
Six failure cases reproduced:
Per-case deepest stage:
Failure clusters:
Common planner statuses:
Diagnostics instrumentation added: yes/no
E0 summary path:
Raw logs:
Recommended E1 fix order:
Open questions for ChatGPT:
```

## 9. Stop condition

After the six cases are reproduced, classified, clustered, and the summary is committed, **stop and wait for ChatGPT review**.

Do not change MotionGen behavior or candidate generation yet.

The goal of E0 is to turn the current generic `10/16` result into a precise statement such as:

> 4/6 failures are attached-object preplace MotionGen failures, while 2/6 are final place-line failures.

Only after we have that evidence should TASK 001-E1 modify the smallest common failure mechanism.
