# ChatGPT Review — TASK 001 Current State

**Actual review date:** 2026-09-04  
**Reviewed repository:** `longlongzzl/rm75-manipulation`  
**Reviewed head:** `c7f14e72ee59d9de8678b90aa1d7f872549bd9d1`

> Authority note: this file is written by ChatGPT through the connected GitHub integration. Local Codex must not create or edit files named `CHATGPT_REVIEW_*` or claim `ChatGPT reviewed/approved` unless this connected review actually occurred. Codex should only append implementation/benchmark handoffs to `CODEX_*` documents.

## 1. Current verdict

### Relation-screen speed work: accepted

Production `lazy_place` is supported by the available evidence:

- full tests: 128 passed;
- default-path smoke relation success: 3/3;
- warm P50/P95: approximately 1.62 / 4.39 s;
- full-chain frozen matrix: 10/16;
- no new exceptions reported when switching the default.

The experimental progressive-preplace mode reduces suite P95 further to about 3.93 s and carrot P95 to about 3.93 s, but changes some selected downstream relations and does not improve 16-case full-chain success. Keeping `lazy_place` as the conservative production default is reasonable for now.

### Full-chain reliability: not yet acceptable

The 16-case matrix remains 10/16. E0 reproduced all six failures 3/3 and separated them into two stable clusters:

- Cluster A: four cases fail at the grasp contact segment (`linear_planner_failed`);
- Cluster B: two cases reach grasp but fail during attached-object lift.

This clustering is useful and should remain the basis for further work.

## 2. E1A verdict

**Verdict: SAFE_BUT_INEFFECTIVE — do not keep stacking it in the production fallback chain.**

E1A added a second forward grasp attempt using `project_distance_to_goal=True` on the same endpoints.

Evidence:

- full tests: 131 passed;
- frozen 16-case success: 10/16 -> 10/16;
- zero previous-success -> new-failure regressions;
- Cluster A recovered: 0/4;
- every targeted primary grasp failure was followed by the same `linear_planner_failed` result in the tool-axis retry.

Therefore E1A has not demonstrated any value on the available failure set and adds an extra MotionGen attempt on every failed grasp candidate.

### Required cleanup before the next fallback

Do one of these before adding E1B:

1. **Preferred:** remove the E1A retry from the production chain and keep the implementation only behind an experimental/debug option; or
2. replace E1A directly with E1B rather than executing `primary -> E1A -> E1B` for every failure.

Do not leave an empirically dead retry permanently on the hot path.

## 3. Review of the proposed reverse-grasp hypothesis

The reverse-validation hypothesis is reasonable, but it should be tested as an **alternative fallback**, not stacked after E1A.

Rationale:

- both pregrasp and grasp endpoints passed endpoint IK screening;
- forward linear planning fails deterministically in Cluster A;
- the grasp endpoint therefore has a known cached IK solution;
- solving the same short Cartesian segment from contact outward may avoid a direction-sensitive optimizer failure;
- reversing the resulting trajectory is safe only if continuity with the already planned pregrasp branch is explicitly checked.

### Approved bounded experiment: E1B-alt

For a failed primary grasp only:

```text
primary forward grasp
    -> if fail: reverse grasp probe from cached grasp IK to exact pregrasp
        -> reverse trajectory
        -> accept only if start_gap <= 0.10 rad
```

Do **not** execute E1A first in this experiment.

Requirements:

- cached grasp configuration must be chosen relative to `pregrasp_end`;
- no new relaxed IK solve;
- ignore only `task.object_name`, exactly as the intended contact semantics already require;
- no additional disabled collision links;
- same seeds/tolerances/collision world;
- no lift changes in this commit;
- preserve primary and reverse-probe failure evidence separately.

## 4. Important diagnostic addition for E1B-alt

For every reverse attempt, record:

```text
cached_grasp_available
cached_grasp_distance_from_pregrasp
reverse_probe_status
reverse_probe_trajectory_points
reversed_start_gap_rad
reverse_fallback_used
```

Also record the forward grasp failure diagnostics returned by the backend. If all reverse probes fail too, the next step should be **linear-planner failure instrumentation**, not a fourth planning heuristic.

## 5. E1B-alt validation gate

Run:

- Cluster A: current-table gluestick + generated 00/02/03, 3 repetitions each;
- Cluster B: generated 05/06 only observationally; do not fix lift yet;
- frozen 16-case matrix: one full-chain run per case.

Acceptance:

- zero previous-success -> new-failure regressions;
- zero new exceptions;
- relation-screen selections unchanged;
- every recovery must explicitly show `reverse_fallback_used=true`;
- report wall-time impact on failed candidates.

If E1B-alt recovers >=3 of 4 Cluster-A cases, it is worth retaining. If it recovers 0/4, stop adding grasp fallbacks and instrument the linear planner to separate optimization failure, interpolation/collision rejection, and branch discontinuity.

## 6. Cluster B remains separate

Do not touch generated 05/06 lift behavior until the grasp cluster is resolved or explicitly closed.

The lift failures occur after attachment and therefore have different likely causes:

- attached-object collision geometry / start contact;
- chosen lift direction versus actual grasp orientation;
- start-state contact escape constraints;
- branch consistency after attach.

These must be diagnosed as a separate task.

## 7. Next instruction to local Codex

1. Do **not** implement the currently drafted `primary -> E1A -> E1B` chain.
2. Make E1A non-production/experimental or remove it from the hot path.
3. Implement E1B as `primary -> reverse-probe` only, behind an explicit experimental mode first.
4. Run the validation gate above.
5. Append results to a `CODEX_*` handoff document and stop for connected ChatGPT review.
