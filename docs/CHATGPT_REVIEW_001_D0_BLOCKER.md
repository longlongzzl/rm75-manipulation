# ChatGPT Review — TASK 001-D0 MotionGen blocker

**Reviewed commits:**
- `1ecb5c0f335d6fe8a516029cf5470832fabc9656` — lazy-place implementation
- `4755a377c8f8fcb777724a44a228f51446387abe` — D0 full-chain blocker report

**Verdict:** `FIX_INFRASTRUCTURE_THEN_RERUN_D0`

## 1. What is already accepted

The current `lazy_place` implementation is provisionally accepted for performance/correctness on the existing three smoke cases:

- relation found: 100% on the three smoke tasks;
- selected tier unchanged in the smoke comparison;
- warm suite P95: ~4.479 s;
- gluestick: ~8.994 s eager -> ~1.694 s lazy;
- carrot: ~6.379 s eager -> ~4.494 s lazy;
- focused and full repository tests reported passing, with the latest D0 report showing `126 passed`.

Do **not** roll back lazy-place because of the current full-chain failure.

## 2. Blocker diagnosis

The D0 full-chain failure is downstream of relation screening. MotionGen `place` planning requested 521 interpolated waypoints while the production backend preallocates only 500:

```text
configured interpolation_buffer_size = 500
required interpolation steps          = 521
observed buffer shape                  = [16, 500, 7]
```

With CUDA Graph enabled, a shape-changing reallocation is not a valid normal execution path in this setup. This is an infrastructure/configuration-capacity issue, not evidence that the lazy relation mapping is wrong.

The current backend intentionally lowers the interpolation buffer to 500 for the 8 GB workstation. At 25 ms/waypoint, 500 covers 12.5 s. The observed complete-chain trajectory needs about 13.0 s, so the margin is simply too tight.

## 3. Approved fix

Create a **separate tiny commit** before continuing D0.

Change only the production `Curobo2BackendConfig.interpolation_buffer_size` default:

```python
interpolation_buffer_size: int = 640
```

Update the adjacent comment to reflect that 640 waypoints at 25 ms cover 16 s.

Why 640 rather than immediately jumping to a very large value:

- it covers the observed 521-waypoint trajectory with ~23% waypoint headroom;
- capacity increases only 28% over the current 500 setting;
- the repository explicitly targets an 8 GB GPU and already keeps MotionGen batches small for memory reasons;
- this keeps the fix narrowly scoped to the observed capacity blocker.

Do **not** change `interpolation_dt`, velocity/acceleration limits, seeds, tolerances, collision rules, graph settings, or relation-screen logic in this commit.

If any frozen D0 case still exceeds 640, stop and report the required maximum waypoint count before increasing the value again. Do not silently keep growing the buffer.

## 4. Required validation after the tiny fix

Reconstruct/warm the planner with the new fixed buffer, then rerun:

1. full repository tests;
2. D0 full-chain `eager` on the three frozen smoke plans;
3. D0 full-chain `lazy_place` on the exact same three plans;
4. then the expanded D0 eager-vs-lazy suite required by `CODEX_TASK_001_D.md`.

For every full-chain case record:

```text
mode
object/task id
full_chain_plan_success
failure_stage
selected_grasp
selected_place
executed_stage_names
segmented_plan_time_s
required/max interpolation waypoints if exposed
```

Also record peak CUDA memory if it can be collected without changing planner behavior; this is diagnostic only and is not a pass/fail criterion unless OOM occurs.

## 5. Gate for D1

D1 may start only if, after the buffer fix:

- all repository tests pass;
- the three frozen smoke cases complete full-chain planning without the interpolation-capacity exception;
- lazy-place shows no downstream planning regression relative to eager on the frozen cases;
- expanded D0 correctness satisfies the acceptance criteria already written in `CODEX_TASK_001_D.md`.

Do **not** switch the production relation-screen default yet.

## 6. What Codex should return

Append a short handoff to `docs/CODEX_TASK_001_D.md` containing:

```text
Infrastructure fix commit:
Interpolation buffer: 500 -> 640
Full tests:
D0 frozen eager full-chain:
D0 frozen lazy full-chain:
Expanded D0 eager/lazy correctness:
Any selected-relation differences:
Any full-chain success differences:
Max observed interpolation waypoint requirement:
Peak CUDA memory (if measured):
D1 started: yes/no
Failures:
Raw logs:
Open questions for ChatGPT:
```

Stop if the buffer fix exposes a new downstream correctness/planning blocker. Otherwise proceed to D1 exactly as previously specified.
