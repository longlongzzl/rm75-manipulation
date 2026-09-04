# ChatGPT Review — TASK 001-D0 after interpolation-buffer fix

**Reviewed head:** `16afd9b91fe0874119246b50729ca8284d2795ee`  
**Infrastructure fix:** `353c3c1e3a70f5c8f137297ec0c4922781123752`  
**Verdict:** `COMPLETE_D0_COVERAGE_BEFORE_D1`

## 1. Accepted evidence

The 500 -> 640 interpolation-buffer fix is accepted as a narrow infrastructure correction.

Current evidence is strong and should be preserved:

- full repository tests: `126 passed in 16.81s`;
- the previous 521-waypoint CUDA-graph capacity exception did not recur;
- frozen smoke full-chain eager/lazy success is identical:
  - tennis: 2/2 vs 2/2;
  - gluestick: 0/2 vs 0/2 at the same `segmented_chain` stage;
  - carrot (`carriot`): 2/2 vs 2/2;
- selected grasp/place relation matched for every paired smoke sample;
- 10 generated scenes x 2 repetitions/mode: eager 20/20 relation-found, lazy 20/20 relation-found;
- no selected tier/grasp/place differences in that generated comparison;
- `gluestick_desk_regression`: 2/2 vs 2/2 with the same selected relation;
- 24/40 paired generated samples exercised `search_tier > 0`.

The equal gluestick downstream failure is **not** evidence of a lazy-place regression. It may remain as a known baseline planner failure for TASK 001-D unless a lazy-only difference appears.

Do not roll back `lazy_place`. Do not change the production default yet.

## 2. Why D1 is still gated

The original D0 matrix explicitly required object-family diversity and expanded downstream planning coverage. The current rerun is still missing:

1. one generic asymmetric manipulated object;
2. one cubic/box manipulated object;
3. downstream full-chain eager-vs-lazy validation beyond the original three smoke plans.

These are validation gaps, not algorithmic blockers.

## 3. Use existing repository objects; do not invent new assets

The repository already contains suitable frozen objects in `assets/test_scenes/current_table.json`:

- cubic/box: `lvmukuai`, label `green cube.`;
- generic asymmetric: `shuazi`, label `white laundry brush.`.

Relevant meshes already exist:

- `assets/meshs/lvmukuai.glb` (+ collision mesh);
- `assets/meshs/shuazi.glb` (+ collision mesh).

Prefer these objects because they are already part of the current-table perception fixture and avoid introducing a new asset-generation/perception variable.

If either object cannot be compiled by the current pick/place task compiler without changing production semantics, stop and report the exact compiler/semantic-registry limitation. Do **not** add a special-case grasp family merely to make D0 pass. A fallback may use the existing `redcube` / `red_triangle` assets only if they already work through an existing generic task path; do not build a new algorithm around them.

## 4. Freeze two additional curated plans

Create or reuse deterministic, frozen manipulation plans for:

### D0-CUBE

A simple pick/place of `lvmukuai` within the existing `current_table` scene.

Requirements:

- use an existing destination/support already understood by the task compiler;
- do not add new grasp heuristics;
- freeze the resulting `manipulation_plan.json` as a benchmark fixture or record an immutable path/manifest in the committed D0 summary;
- record the selected grasp family/search tier.

### D0-ASYM

A simple pick/place of `shuazi` within the existing `current_table` scene.

Same constraints as above.

The purpose is relation-screen diversity, not to create a new manipulation capability.

## 5. Complete screen-only D0 coverage

Run eager and lazy on the two new curated plans, at least 2 repetitions per mode.

Acceptance:

- eager relation-found == lazy relation-found for every curated case;
- no lazy-only exception;
- log selected tier/grasp/place differences even if both succeed;
- combined curated relation recall must remain 100% wherever eager finds a relation.

Append these results to:

`benchmarks/task001/task001_d0_lazy_validation_summary.json`

## 6. Complete expanded downstream full-chain comparison

Use the existing `--full-chain` no-op executor. No robot or ManiSkill execution.

Run one full-chain repetition per mode for the following frozen matrix:

1. tennis smoke;
2. gluestick smoke;
3. carrot smoke;
4. `gluestick_desk_regression`;
5. `lvmukuai` cubic/box case;
6. `shuazi` asymmetric case;
7. the same 10 generated random scenes already used in the screen-only expanded comparison.

For the generated scenes, reuse the frozen plan plus explicit `--scene` overrides where valid; do not regenerate LLM plans for each timing run if the existing benchmark path supports scene overrides.

Record per case:

```text
case_id
object_id
mode
search_tier
relation_found
full_chain_plan_success
failure_stage
selected_grasp
selected_place
executed_stage_names
segmented_plan_time_s
```

Also summarize:

```text
eager_full_chain_success_count
lazy_full_chain_success_count
lazy_minus_eager_percentage_points
eager_success_lazy_failure_cases
selected_relation_difference_cases
```

Acceptance:

- lazy downstream success regression <= 1 percentage point relative to eager;
- **zero eager-success / lazy-failure cases in the curated set**;
- any eager-success / lazy-failure generated case must be frozen and diagnosed before D1;
- no interpolation-capacity exception;
- no crash/exception hidden as `relation_found=false`.

Equal eager/lazy failures may be recorded as existing downstream planner limitations and do not by themselves block D1.

## 7. D1 gate

D1 (`lazy_place_progressive_preplace`) may begin immediately after the above matrix passes.

Do not ask for another review between D0 and starting D1 if all acceptance conditions above are satisfied. In that case:

1. commit the completed D0 evidence separately;
2. then implement D1 exactly as specified in `docs/CODEX_TASK_001_D.md`;
3. stop after D1 benchmark/results and wait for ChatGPT review before any production-default switch.

If D0 exposes an eager-success/lazy-failure or requires production semantic changes to create the cube/asymmetric cases, stop before D1 and report it.

## 8. Still forbidden in this pass

Do not:

- reduce IK seeds;
- reduce grasp candidates/tilts/contact shifts;
- loosen tolerances;
- loosen collision checks;
- change interpolation settings again unless a new fixed-capacity exception is observed and reported;
- change `relation_screen_mode` default;
- modify real-robot execution;
- implement special object-family logic solely for benchmark coverage.

## 9. Required Codex handoff

Append to `docs/CODEX_TASK_001_D.md`:

```text
# D0 completion after ChatGPT post-fix review

D0 completion commit:
Cube fixture/plan:
Asymmetric fixture/plan:
Full tests:
Curated eager/lazy relation recall:
Generated eager/lazy relation recall:
Expanded eager full-chain success:
Expanded lazy full-chain success:
Eager-success/lazy-failure cases:
Selected-relation differences:
Max interpolation waypoints observed:
D0 accepted locally: yes/no
D1 started: yes/no

D1 commit (if started):
D1 tests:
D1 tennis lazy/progressive P50/P95:
D1 gluestick lazy/progressive P50/P95:
D1 carrot lazy/progressive P50/P95:
D1 suite lazy/progressive P50/P95:
D1 coarse solver work before/after:
D1 relation recall:
D1 downstream full-chain comparison:
D1 selected-relation differences:
D1 summary path:
Failures:
Raw logs:
Open questions for ChatGPT:
```

If D0 passes, proceed to D1 without changing production defaults. Stop after D1 evidence is committed.
