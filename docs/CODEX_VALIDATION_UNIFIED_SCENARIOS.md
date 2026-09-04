# CODEX Validation Board — Unified Sorting, Magnetic Assembly, and Push-T

> Target branch: `chatgpt/unified-three-scenarios`  
> Architecture: `docs/UNIFIED_THREE_SCENARIO_ARCHITECTURE.md`  
> Owner of physical-robot approval: user  
> Codex role: implement local adapters, execute tests/benchmarks, and record evidence. Do not represent unrun code as verified.

## 0. Collaboration and safety rules

### 0.1 Task states

Use exactly one of:

- `PROPOSED`
- `CODEX_IMPLEMENTED`
- `OFFLINE_VERIFIED`
- `CUROBO_VERIFIED`
- `SIM_VERIFIED`
- `REAL_VERIFIED`
- `NEEDS_REVIEW`
- `REJECTED`

### 0.2 Required handoff for every commit

Append a compact handoff to this file or a dedicated `CODEX_HANDOFF_*.md`:

```text
Task:
State:
Commit:
Parent commit:
Changed files:
Why each file changed:
Commands run:
Test environment:
Before metrics:
After metrics:
Correctness/safety checks:
Failures and raw statuses:
Raw log paths:
Committed summary path:
Unexpected observations:
Open questions for ChatGPT:
```

### 0.3 Strict scope

1. Never test physical motion from an uncommitted worktree.
2. One hypothesis per commit whenever practical.
3. Do not modify the existing production `lazy_place` relation screening while validating these scenarios.
4. Do not enable experimental reverse-grasp fallback by default.
5. Do not increase ignored-object sets, disable full gripper links, shrink collision geometry, loosen IK/pose tolerances, or lower test difficulty merely to produce a success.
6. Sorting and magnetic plans use real **instance ids**, not only asset names.
7. LLM output is untrusted. It must pass symbolic and geometric validation before cuRobo is invoked.
8. No real robot command until the corresponding OFFLINE and SIM gates pass and the user explicitly approves the trial.
9. Before every physical trial: verify emergency stop, speed limits, soft workspace, table collision model, TCP, gripper state, and a human observer.
10. Commit summaries under `benchmarks/unified_scenarios/`; large raw logs may remain under `/tmp` or `runtime_data`, but record exact paths.

---

# U0 — Branch integrity and pure-software baseline

**State:** `PROPOSED`

## U0.1 Checkout

```bash
git fetch origin
git checkout chatgpt/unified-three-scenarios
git status --short
git rev-parse HEAD
```

Record HEAD and require a clean worktree before modification.

## U0.2 Lightweight suite

```bash
python -m compileall -q rm75_app/scenarios
PYTHONPATH=. python -m pytest -q \
  tests/test_scenario_program_runner.py \
  tests/test_sorting_scenario.py \
  tests/test_magnetic_assembly.py \
  tests/test_magnetic_pickplace_adapter.py \
  tests/test_pusht_scenario.py \
  tests/test_pickplace_program.py
```

The GitHub Actions reference at implementation time is 25 passed. Local Codex must report its own result rather than copying that number.

## U0.3 Full repository regression

Run from the repository root:

```bash
PYTHONPATH=. python -m pytest tests -q
```

Acceptance:

- no new failure compared with `main` in the same environment;
- any environment-only skipped tests are listed explicitly;
- no changes are made merely to suppress unrelated failures.

## U0.4 Dry-run CLI

```bash
PYTHONPATH=. python tools/run_unified_scenario.py --help

PYTHONPATH=. python tools/run_unified_scenario.py \
  sorting-compile \
  --request configs/scenarios/sorting.example.json \
  --output /tmp/rm75_sorting_plan.json

PYTHONPATH=. python tools/run_unified_scenario.py \
  magnetic-generate \
  --catalog configs/scenarios/magnetic_panel_catalog.example.json \
  --inventory configs/scenarios/magnetic_inventory.example.json \
  --description "搭一面能稳定站立的磁吸墙" \
  --output /tmp/rm75_magnetic_wall.json

PYTHONPATH=. python tools/run_unified_scenario.py \
  magnetic-validate \
  --catalog configs/scenarios/magnetic_panel_catalog.example.json \
  --inventory configs/scenarios/magnetic_inventory.example.json \
  --assembly /tmp/rm75_magnetic_wall.json \
  --output /tmp/rm75_magnetic_wall_validation.json

PYTHONPATH=. python tools/run_unified_scenario.py \
  pusht-sim \
  --goal-x 0.08 --goal-y 0.02 --goal-yaw 0.0 \
  --system-identification \
  --output /tmp/rm75_pusht_dry_run.json
```

The example catalog/poses are not production calibration. U0 only verifies software plumbing.

## U0 deliverable

`benchmarks/unified_scenarios/u0_offline_summary.json` with commit, Python/library versions, test counts, CLI results, and all failures.

Stop for review if U0 is not clean.

---

# U1 — Sorting: complete program planning and continuous replay

**State:** `PROPOSED`

## U1.1 Freeze a real-scene suite

Create at least 20 valid sorting snapshots with:

- 3, 4, and 5 movable objects;
- at least three target arrangements;
- open and cluttered layouts;
- at least two target-occupancy swaps requiring a buffer;
- one deliberately impossible/no-buffer cycle;
- one target occupied by an unassigned obstacle;
- current real joint state in every snapshot.

Do not modify snapshots after the benchmark starts. Save a suite manifest with scene files, request files, expected valid/invalid frontend result, and random seed.

## U1.2 Frontend checks

For every request verify:

- each assigned object instance exists;
- target capacities are respected;
- slot poses are unique and within the declared target region;
- dependency order moves target occupants first;
- cycles are rejected without a buffer;
- cycles expand into a temporary buffer move and final return when a buffer exists;
- unassigned target occupants remain obstacles and are not silently relocated.

Render top-down before/after target layouts for manual review.

## U1.3 Complete cuRobo program compilation

Use the existing cuRobo2 environment, for example:

```bash
/home/zhangzhao/anaconda3/envs/curobo2/bin/python <local integration script>
```

Codex may add a `tools/benchmark_sorting_program.py` adapter, but it must call:

```text
SortingPlanCompiler
→ PickPlaceProgramCompiler
→ existing FixedSceneAtomTaskBuilder
→ existing PickPlaceCoordinator/Curobo2Backend
```

Do not create a second sorting-specific motion planner.

For every compiled atom record:

- candidate build time;
- relation-screen time and selected tier;
- segmented MotionGen time;
- selected grasp/place ids;
- all trajectory stages and point counts;
- inter-stage maximum start gap;
- source/predicted scene revision;
- total program planning time;
- failure stage and unabridged planner status.

Acceptance for the selected demo distribution:

- frontend validity agrees with suite annotations: 100%;
- no inter-stage discontinuity above 0.10 rad;
- no missing trajectory/gripper stage;
- no planner call occurs after the first execution command in full-program mode;
- compiled program begins within 0.12 rad of the snapshot joint state;
- no regression on previously successful PickPlace frozen cases;
- target full-program compilation rate: at least 90% on feasible, non-adversarial demo layouts; report honestly if lower.

Planning speed is secondary for sorting. Report total time, but do not reduce correctness to force a latency number.

## U1.4 ManiSkill replay

Replay every successfully compiled program with the same portable program package.

Verify:

- joint-name order;
- `dt` interpretation and controller rate;
- no teleport to the first trajectory point;
- gripper close/open timing;
- object attach/release event order;
- collision/limit violations;
- final object target error;
- maximum idle gap between consecutive motion commands.

Suggested no-stutter metric:

```text
p95 software idle gap between adjacent trajectory submissions < 150 ms
```

Exclude declared gripper dwell and optional atom-boundary observation checkpoints, but report them separately.

Do not claim dynamics equivalence merely because a joint path replays.

## U1.5 Real robot, staged

Only after U1.4 passes:

1. execute first trajectory only at reduced speed;
2. execute one complete PickPlace atom;
3. execute two precompiled atoms;
4. execute a 3–5 object sorting program.

Before each level compare actual joint state with the program's expected start. Abort if the gap exceeds the configured threshold.

Record:

- planned versus actual joint trajectories;
- inter-command idle time;
- success per atom and full task;
- grasp/place errors;
- collision/safety stops;
- whether a re-observation checkpoint invalidated a suffix.

Minimum paper-quality set after engineering validation: 20 random sorting trials with all failures retained.

## U1 deliverables

```text
benchmarks/unified_scenarios/u1_sorting_suite.json
benchmarks/unified_scenarios/u1_sorting_planning_summary.json
benchmarks/unified_scenarios/u1_sorting_sim_summary.json
benchmarks/unified_scenarios/u1_sorting_real_summary.json  # only after approval
```

---

# U2 — Magnetic assembly frontend, calibration, and rule validation

**State:** `PROPOSED`

## U2.1 Replace example assets with calibrated production assets

The checked-in example catalog is deliberately not executable. Build a production catalog from the actual magnetic pieces and existing tray.

For every panel asset record:

- exact mesh/ObjectSpec asset name;
- real scene instance ids and tray slots;
- local-axis convention, with X/Y in the panel plane and Z through thickness;
- measured X, Y, thickness;
- mesh-to-real scale;
- magnet-center and usable-edge offsets, when relevant;
- grasp clearance;
- engagement clearance;
- mass/friction if used in ManiSkill;
- measurement method and repeated measurements.

Do not guess dimensions from screenshots. Register missing objects in `ObjectSpec` and verify their meshes/axes in both cuRobo and ManiSkill.

## U2.2 Calibrate connection geometry

Measure or experimentally determine:

- vertical-slot gap between the two lying supports;
- required panel insertion depth;
- 90-degree half-edge overlap convention;
- magnet snap tolerance;
- safe preplace clearance;
- release height and gripper retreat clearance;
- repeatability over at least 10 manual assemblies.

Update the production catalog/config, not the generic rule code, unless evidence shows the rule parameterization itself is wrong.

Keep raw measurements and a photograph/diagram of the coordinate convention.

## U2.3 Symbolic adversarial suite

Create at least 40 structure graphs, including:

Valid:

- two-base + vertical wall;
- stable wall then 90-degree second wall;
- two stable walls + bridge/roof;
- 3, 4, 5, 7, and up to 12 pieces;
- different inventory instance assignments.

Invalid:

- one flat support + vertical child;
- free-standing vertical anchor;
- wrong 90-degree overlap;
- duplicate physical object use;
- duplicate child connections;
- support dependency cycles;
- bridge on one support;
- non-level support pair;
- slot too narrow/wide;
- excessive/zero gap or clearance;
- more than 12 pieces;
- missing tray object or wrong asset.

Acceptance:

- expected valid/invalid classification: 100%;
- no validator exception on malformed-but-schema-valid graphs;
- every rejection contains stable machine-readable rule codes.

## U2.4 LLM frontend

Use a mock client first, then the chosen real API. Run at least 30 natural-language prompts covering wall, corner, gate, bridge, small house-like structures, underspecified requests, and impossible inventory requests.

Record:

- raw first response;
- parser success;
- first-pass rule validity;
- repair prompt and response;
- final validity;
- template fallback use;
- piece count;
- duplicate/missing inventory references;
- generation latency and API cost.

Acceptance for the frontend software path:

- zero unvalidated graph reaches geometry/motion planning;
- zero raw Cartesian pose or trajectory from the LLM is trusted;
- every final structure uses <=12 available pieces;
- template fallback is labeled, not presented as LLM success.

Do not optimize the prompt by deleting hard cases from the test set.

## U2.5 Geometry render review

Resolve every valid structure and render:

- panel coordinate axes;
- panel collision boxes/meshes;
- selected mating edges;
- slot center and width;
- 90-degree edge line and overlap;
- support graph/order;
- target TCP/gripper clearance when available.

The strict geometry validator must pass. A human must inspect at least wall, corner, and gate/bridge renders before cuRobo motion planning.

## U2 deliverables

```text
configs/scenarios/magnetic_panel_catalog.json         # calibrated, not example
configs/scenarios/magnetic_inventory.json             # current tray ids
benchmarks/unified_scenarios/u2_measurements.csv
benchmarks/unified_scenarios/u2_rule_suite.json
benchmarks/unified_scenarios/u2_llm_frontend_summary.json
benchmarks/unified_scenarios/u2_geometry_summary.json
```

Stop if exact real geometry is not known. Do not continue to motion planning with the example catalog.

---

# U3 — Magnetic assembly motion planning, simulation, and real execution

**State:** `PROPOSED`

## U3.1 Multi-support contact audit

For every vertical-slot placement prove from logs that:

- both support instance ids are attached to the place candidates;
- both supports are temporarily ignored/disabled for contact endpoint solving and the final contact segment only;
- all unrelated obstacles remain enabled;
- no full gripper link is disabled;
- both supports are restored immediately after the planning call;
- free-space lift/transport sees both supports;
- scene revision/collision state does not leak into the next candidate or atom.

Add a real cuRobo regression test around `MagneticContactPlanningBackend`; pure mock tests alone are not sufficient for this gate.

## U3.2 Motion program matrix

Compile complete nominal programs for:

1. stable wall: 3 pieces;
2. wall corner: 4 pieces;
3. gate/bridge: 7 pieces;
4. one larger <=12-piece structure;
5. five perturbed tray/anchor layouts per structure.

Record per atom and per structure the same planning metrics as U1.

Acceptance before simulation:

- symbolic + geometry validation 100%;
- complete program compiles without missing stages;
- all dependencies match construction order;
- support objects are already placed before a dependent panel;
- no trajectory discontinuity >0.10 rad;
- no out-of-workspace target;
- no collision exemption outside intended contact segments.

Do not work around a motion failure by changing the symbolic rule result after the fact. Record whether failure is target reachability, approach, grasp, lift, transport, place, or retreat.

## U3.3 ManiSkill

Start with geometry-only replay, then physical objects:

1. two flat bases;
2. vertical panel inserted between them;
3. second 90-degree wall;
4. bridge/roof;
5. full structure.

Check:

- correct mesh scale/axes;
- magnet or snap approximation;
- release stability;
- collision forces/contact penetration;
- retreat does not disturb placed panels;
- accumulated pose error before every next atom;
- full-program versus atom-boundary re-observation.

If ManiSkill has no magnetic force model, label the approximation explicitly. A rigid attach/snap event can be used for workflow verification but is not evidence of magnetic dynamics fidelity.

## U3.4 Real robot safety ladder

Only after U3.3 and user approval:

1. dry-run target poses with no object;
2. grasp one panel from tray and return it;
3. place one flat base;
4. place second flat base and measure slot width;
5. insert one vertical panel at reduced speed;
6. construct one 90-degree corner;
7. execute the full selected structure.

At each level stop on:

- unmodeled collision/contact;
- panel slip or wrong mesh frame;
- unexpected magnetic snap;
- target error larger than calibrated tolerance;
- gripper retreat interference;
- actual joints outside expected tracking envelope.

Minimum paper-quality set after stabilization: at least 10 independent trials each for wall and corner, and 5 for the larger structure, including failures.

## U3 deliverables

```text
benchmarks/unified_scenarios/u3_contact_audit.json
benchmarks/unified_scenarios/u3_motion_program_summary.json
benchmarks/unified_scenarios/u3_mani_skill_summary.json
benchmarks/unified_scenarios/u3_real_summary.json  # only after approval
```

---

# U4 — Push-T pure simulation and tracker integration

**State:** `PROPOSED`

## U4.1 Define the real coordinate contract

Before any robot planning specify:

- table/planning frame;
- T-object local frame and mesh origin;
- x/y/yaw extraction convention;
- pusher TCP and radius/shape;
- contact TCP height;
- allowed workspace polygon;
- target-pose representation;
- tracker confidence and timestamp source.

Write one adapter from the existing tracker output to `PushTObservation`; do not duplicate perception/tracking logic inside the controller.

## U4.2 Pure simulated benchmark

Generate at least 200 random starts and goals within a reachable tabletop area. Separate:

- translation-dominant;
- rotation-dominant;
- combined;
- near-boundary;
- perturbed model parameters.

Compare:

1. direct center push heuristic;
2. one-step greedy model search;
3. H-step many-future MPC;
4. MPC + parameter ensemble under parameter mismatch.

Report:

- goal success within step limit;
- number of pushes;
- final position/yaw error;
- planning latency per cycle;
- best/median candidate cost;
- stalls/regressions;
- one-step prediction error;
- parameter-estimator effective sample size.

Do not tune and test on the same random seeds.

Suggested initial target, not a license to hide failures:

- MPC should materially outperform the direct heuristic on combined pose goals;
- p95 planning time should fit inside the desired observe–act cycle;
- under synthetic parameter mismatch, online fitting should reduce held-out one-step prediction error.

If parameter fitting does not improve prediction/control, leave it optional and do not make it a paper claim.

## U4.3 Tracker replay

Replay recorded RGB-D/video sequences without robot motion. Verify:

- confidence threshold behavior;
- yaw wrapping near ±π;
- no implausible > configured position jumps;
- pose jitter and latency;
- velocity estimates;
- loss/reacquisition behavior;
- each post-push observation is newer than the pre-push observation.

Store a tracker benchmark summary and representative plots.

## U4 deliverables

```text
benchmarks/unified_scenarios/u4_pusht_sim_summary.json
benchmarks/unified_scenarios/u4_pusht_sysid_summary.json
benchmarks/unified_scenarios/u4_tracker_replay_summary.json
```

---

# U5 — Push-T cuRobo, ManiSkill, and real closed loop

**State:** `PROPOSED`

## U5.1 cuRobo short-program validation

Build a `CuroboWaypointPushBackend` with:

- actual joint-state provider;
- calibrated pusher tool config;
- free scene containing the T;
- contact scene omitting/disabling only the T;
- existing trajectory sink in dry-run mode.

For at least 100 sampled short pushes verify:

- every hover/descend/contact/push/retract segment is planned before the first execution callback;
- all segment start gaps <=0.10 rad;
- contact and push direction are compatible;
- long pushes split into bounded subsegments;
- free scene contains the T;
- contact scene omits only the T;
- no disabled collision links;
- all unrelated objects and table remain active;
- no joint/workspace/collision violation.

Record the exact planner status for every failure.

## U5.2 Trajectory timing

The current adapter does not force exact Cartesian `speed_mps` retiming. Before real pushing, choose and verify one policy:

- retime the generated trajectories to a tested push speed; or
- configure the real/simulation trajectory sink to enforce the desired speed while preserving limits.

Measure actual pusher speed from joint/TCP logs. Do not ignore this item: Push-T response and system identification are meaningless when commanded speed is unknown.

## U5.3 ManiSkill closed loop

Use the same cycle as reality:

```text
observe simulated T
→ MPC
→ plan complete short robot program
→ execute
→ settle
→ re-observe
```

Do not directly apply the quasi-static state transition to the physics T during this test.

Compare:

- fixed model parameters;
- optional online fit;
- direct heuristic.

Record physical T pose after every push and simulator prediction error.

## U5.4 Real robot safety ladder

After user approval:

1. hover over marked contact points without touching;
2. descend to contact height with no planar motion;
3. execute 5–10 mm pushes at low speed;
4. execute one 20–30 mm push and track the T;
5. execute two closed-loop pushes;
6. run a full target-position task;
7. enable orientation goals;
8. only then evaluate online parameter fitting.

Abort on low tracker confidence, stale observation, large pose jump, planner failure, joint-start mismatch, pusher/table collision, unexpected object rotation, or three stalls.

Minimum paper-quality set after stabilization: at least 50 random real goals with all attempts retained, split into translation, rotation, and combined groups.

## U5 deliverables

```text
benchmarks/unified_scenarios/u5_curobo_push_summary.json
benchmarks/unified_scenarios/u5_mani_skill_push_summary.json
benchmarks/unified_scenarios/u5_real_push_summary.json  # only after approval
```

---

# U6 — Integration, web/frontend, and release gate

**State:** `PROPOSED`

After U1–U5 pass individually, connect them to the existing web/runtime entrypoints.

## U6.1 UI states

Every scenario UI must distinguish:

```text
observed
frontend_validated
geometry_validated
motion_compiled
simulation_replayed
approved_for_real
executing
completed / failed / invalidated
```

Never show “ready” merely because an LLM response parsed or endpoint IK succeeded.

## U6.2 Required controls

Sorting:

- select object instances and targets;
- choose full-plan or checkpointed execution;
- preview order/buffer moves;
- compile, replay, then explicit real approval.

Magnetic:

- text description;
- inventory and <=12-piece usage preview;
- symbolic graph and rule violations;
- 3D resolved structure/edge visualization;
- motion compilation and simulation replay;
- explicit real approval.

Push-T:

- current/goal T overlay;
- predicted best future and first push;
- tracker confidence;
- fixed versus adaptive model setting;
- step-by-step start/stop and emergency abort.

## U6.3 Release acceptance

Before merging into `main`:

- all full repository tests pass;
- all new CI tests pass;
- U0–U5 evidence is committed or clearly marked not run;
- no example calibration is used by a production entrypoint;
- no real execution can be triggered without explicit approval;
- all experimental modes are opt-in;
- README points to the architecture and validation docs;
- the three scenarios share the intended foundations rather than copying planner code.

## U6 final handoff

```text
Merged/target commit:
Full tests:
Sorting status: OFFLINE / CUROBO / SIM / REAL
Magnetic status: OFFLINE / CUROBO / SIM / REAL
Push-T status: OFFLINE / CUROBO / SIM / REAL
Known limitations:
Paper claims currently supported:
Claims not yet supported:
Raw evidence locations:
Open issues:
```

Stop for ChatGPT review before merging into `main`.
