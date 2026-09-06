# ChatGPT Review — Three Real Demos / Shared RRTrack / Simulation Brain

Date: 2026-09-06  
Reviewer: ChatGPT  
Branch: `chatgpt/unified-three-scenarios`

## 0. Corrected hard definition

This review adopts the user's clarified project definition:

1. **sorting / organization is a real RM75 demo**;
2. **magnetic assembly is a real RM75 demo**;
3. **Push-T is also a real RM75 physical demo**;
4. **all three reuse the latest PickPlace RRTrack perception/tracking chain**;
5. scenario code may adapt coordinate frames or state representations but must not
   build an independent tracker;
6. simulation is the planning brain / explicit world model; simulation success is
   supporting evidence, not a substitute for real-robot task success.

See `THREE_REAL_DEMOS_SYSTEM_DEFINITION.md`.

---

## 1. Review of the latest Codex handoff

The latest handoff provided meaningful new evidence:

- CPU suite reached 304 passed in the reported local environment;
- frozen cuRobo planning remained 14/16;
- original three-object replay still failed physically;
- read-only CPU-physics instrumentation now records per-substep joint tracking,
  contacts, object motion and release boundaries without changing the frozen
  program;
- the carrot case is close to its expected support-relative target immediately
  before release but deviates strongly after opening, making the release window a
  higher-priority diagnostic target than another grasp-candidate heuristic.

This does **not** establish a safe universal release delay. The evidence should be
used to learn/calibrate a readiness condition, not to tune one fixture until it
passes.

---

## 2. Code changes made in this review

### 2.1 Shared RRTrack scenario bridge

Added:

```text
rm75_app/scenarios/rrtrack_bridge.py
```

Key contracts:

- `RRTrackInstanceSample`: timestamped result from the existing RRTrack chain;
- `RRTrackSceneAdapter`: transforms accepted `T_cam_obj` into the planning scene
  for sorting/magnetic assembly;
- `RRTrackPushTTracker`: uses the same RRTrack pose and only projects it to
  table-frame `(x, y, yaw)` through `PoseMatrixPushTTracker`.

Important behavior:

- rejected/lost RRTrack outputs are never committed as new scene geometry;
- held objects are not overwritten by vision by default;
- camera->robot/table calibration must be explicit;
- the bridge is structural/duck-typed so importing the lightweight scenario API
  does not import the heavy OpenCV RRTrack runtime.

### 2.2 Release-readiness contract

Added:

```text
rm75_app/scenarios/release_readiness.py
```

It intentionally contains **no invented production thresholds**. With no
calibration, ordinary release state is `unknown`; strict joint-limit excursion is
`not-ready`. Once calibrated thresholds are supplied, the same API can classify a
short pre-open window in shadow mode before any real gate is enabled.

### 2.3 Tests / CI

Added unit coverage for:

- accepted/rejected/lost RRTrack pose propagation;
- held-object protection;
- RRTrack->PushT table-frame projection;
- RRTrack acceptance reuse as the PushT confidence gate;
- release readiness unknown/ready/not-ready semantics.

The branch CI has already passed the shared RRTrack bridge test revision. Continue
to require CI green after later wiring changes.

---

# 3. CODEX TASK R1 — wire the one RRTrack chain into all three real runtimes

**Do not implement a new T tracker.**

## R1-A Sorting / organization

At every observation checkpoint:

```text
latest RRTrack instance outputs
  -> RRTrackSceneAdapter(T_base_camera)
  -> TaskSceneState
  -> stale-suffix / atom validation
```

Requirements:

- preserve `instance_id`, never collapse by `asset_name`;
- never use a rejected/lost pose as a new planning pose;
- do not overwrite `HELD` attachment state;
- after release and stable reacquisition, RRTrack can again update the object;
- log frame/timestamp/precision/support with every checkpoint.

Test with at least one scene containing two physical instances of the same asset.

## R1-B Magnetic assembly

Use the exact same scene adapter. The only additional interpretation is structural:

```text
RRTrack measured panel pose
 -> resolved connection / support validator
 -> either accept current structure prefix or rebuild remaining suffix
```

Do not create a magnetic-specific pose tracker.

At minimum verify:

- one flat panel;
- two-support vertical slot;
- right-angle connection;
- bridge/roof prefix;
- released panel reacquisition after the gripper exits.

## R1-C Push-T

The physical loop must become:

```text
RRTrack(T)
 -> RRTrackPushTTracker => (x,y,yaw)
 -> world-model futures
 -> select one PushAction
 -> CuroboWaypointPushBackend plans complete short push
 -> RM75 executes
 -> RRTrack(T) again
 -> replan
```

Use the same T mesh / object identity / SAM6D initialization / Cutie / FoundationPose
/ recovery chain as PickPlace.

The PushT-specific tracker code must remain only:

- camera/table frame transform;
- 6D->planar projection;
- optional smoothing;
- finite-difference velocity for the planner.

It must not perform independent segmentation, registration or relocalization.

### Push-T physical safety/validity gates

Before enabling a full trial:

- workspace polygon for pusher and T;
- calibrated pusher TCP/contact height;
- plan the whole short contact program before descent;
- keep the T disabled only in the contact planning scene; all other obstacles stay;
- abort on RRTrack lost/recovering output;
- abort on stale timestamp;
- after an execution exception, do not retry the same push blindly;
- start with system identification disabled.

---

# 4. CODEX TASK R2 — finish release-window diagnosis before altering timing

Do not add another fixed dwell yet.

For each `gripper_open` boundary, summarize a fixed physical-time window before
and after opening. Required window signals:

- object linear/angular speed;
- TCP linear/angular speed;
- pad-origin opening rate;
- arm tracking error;
- object/gripper penetration;
- lateral contact impulse magnitude;
- support-relative object pose error;
- gripper joint-limit excursion;
- contact pairs and contact count.

Report both:

```text
max over pre-open window
value at immediate pre-open frame
max over first post-open window
settled value after release
```

Run successful and failed fixtures. Only after obtaining labelled windows should
thresholds for `ReleaseReadinessThresholds` be calibrated.

First deploy the threshold classifier as **shadow-only**. It must not delay/open
or block the gripper until its false-ready / false-not-ready rates are measured.

---

# 5. CODEX TASK R3 — world-model planning evidence

The paper claim is not “we have ManiSkill.” The evidence must show that predicted
futures change real decisions and improve physical outcomes.

## Shared internal ablation

Use identical RRTrack, task rules, robot and cuRobo:

```text
A. current-state / no future consequence evaluation
B. one-step future
C. multi-future simulation-guided planning (ours)
```

Report real RM75 success for all three scenarios.

## Sorting

Create cases where multiple kinematically feasible next moves differ in future
occupancy/reachability. Record whether the simulator rejects a locally plausible
move because of a later dead end.

## Magnetic

Create cases where multiple geometrically plausible assembly orders differ in
support stability / next-placement reachability / gripper exit feasibility.
Do not give ours more magnetic rules than the baselines.

## Push-T

Create initial/goal pairs where greedy position-only pushing and H>1 simulation
select different contact points/directions. Record one-step prediction error and
horizon error against RRTrack-measured real transitions.

Do not claim physical parameter identification; use “online simulator response
calibration” unless identifiability is separately established.

---

# 6. External baseline priority after the corrected physical-demo definition

### P0 — PWTF-adapted (overall nearest neighbor)

Closest conceptual comparison: interactive digital twin predicts candidate action
outcomes for real-world manipulation and uses those futures in MPC. Use the same
RM75 task endpoints and low-level skill budget where possible. If reconstruction
or action interfaces are replaced, label it `PWTF-adapted`.

### P0/P1 — PDDLStream + shared cuRobo (sorting / magnetic)

Use PDDLStream for task-and-motion ordering while exposing the same grasp/place
samplers, magnetic support rules, IK/collision tests and cuRobo. This is the most
important non-world-model planning comparison for the discrete tasks.

### P1 — Code-as-Policies style

Same LLM + RRTrack + bounded skills, but no simulated future outcomes. Retain
reasonable execution feedback/replanning so this is not an artificially weak
open-loop baseline.

### P1 — DINO-WM **or** LeWorldModel (Push-T)

Choose one first. Both have Push-T world-model planning resources. Either compare
on the standard Push-T domain using matched action/goal protocol, or adapt/train
on the RM75 observation/action domain and count that data/training cost. Do not
feed an off-domain checkpoint into the real camera stream and call failure a fair
comparison.

---

# 7. Acceptance state before calling the three-demo system “done”

The branch is not complete until all are true:

### Shared

- same RRTrack chain demonstrably feeds all three scenarios;
- real observation timestamps and transforms are logged;
- no simulator-predicted state is silently counted as observed state;
- physical task success comes from an outcome observer.

### Sorting

- multi-object real continuous program succeeds over randomized layouts;
- stale suffix can be invalidated by RRTrack evidence;
- planning/execution failure counts are included in denominator.

### Magnetic

- calibrated real panel/tray geometry is used;
- 2/4/6+ piece physical structures are demonstrated before attempting 12;
- text-generated blueprints pass the same deterministic rule/geometry validation;
- structure-prefix outcome is observed on the real scene.

### Push-T

- real T is tracked by the PickPlace RRTrack chain;
- one real push is planned/executed/reobserved end to end;
- closed-loop goal trials are run from multiple initial poses;
- fixed-model and optional response-calibrated variants are compared;
- simulation prediction error is measured against real RRTrack transitions.

Stop after R1/R2 instrumentation and provide a Codex handoff before changing
production release timing, magnetic physical tolerances, or Push-T model parameters.
