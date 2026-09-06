# Three Real-World Demos — System Definition

> This document is the hard project definition for the current paper branch.
> It overrides any earlier wording that treated Push-T as a simulation-only demo
> or implied different perception stacks for different scenarios.

## 1. Paper-level system identity

The three scenarios are **all physical RM75 experiments**. Simulation is the
planning brain / explicit world model, not the final evaluation environment.

```text
                       SAME REAL PERCEPTION
                    latest PickPlace RRTrack chain
                              |
                     6D tracked object state
                              v
Real observation -> planning-ready virtual world
                              |
                 simulate / reason about futures
                              |
                      select a task action
                              v
                         RM75 execution
                              |
                         RRTrack again
                              |
                  update / replan / continue
```

The main scientific story is therefore:

> **Use a task-conditioned simulator as an explicit world model for planning,
> while grounding every planning cycle in the same recoverable real-world
> RRTrack state and validating all three tasks on the physical robot.**

A simulator-only success is never counted as the final task success for any of
these three demos.

---

## 2. One perception stack, three task consumers

### Shared tracker

All three scenarios use the current PickPlace tracking chain:

```text
SAM3/SAM6D initialization
        -> Cutie mask propagation
        -> FoundationPose/local pose refinement
        -> rendered-mask agreement
        -> correction/recovery
        -> online/offline memory bank
        -> RRTrackOutput(T_cam_obj, accepted, agreement, state, ...)
```

No scenario should implement a second independent object tracker.

The new scenario adapter is:

```text
rm75_app/scenarios/rrtrack_bridge.py
```

It performs only coordinate/frame adaptation and task-state conversion. It does
not duplicate segmentation, pose estimation, recovery, memory, or relocalization.

### Sorting / organization

RRTrack 6D poses are transformed into the robot planning frame and update the
same `TaskSceneState` used by PickPlace. The planner can compile a full nominal
program, but atom-boundary RRTrack observations remain available to invalidate a
stale suffix when the physical world differs from the predicted state.

### Magnetic assembly

The same RRTrack scene state tracks tray pieces, already placed panels and the
current assembly prefix. The symbolic/geometry planner reasons about support and
connection rules; cuRobo plans the shared PickPlace primitive; RRTrack verifies
what was actually built before the next dependent placement.

A held object's planner attachment state is not overwritten by vision by default.
After release, RRTrack can again become the authoritative real-world pose source.

### Push-T

Push-T uses the same RRTrack output for the physical T object. The only extra
operation is a calibrated projection:

```text
T_cam_T
  -> T_table_cam @ T_cam_T
  -> (x, y, yaw)
```

The Push-T controller then performs:

```text
RRTrack observe
  -> simulate many future push sequences
  -> select first push
  -> cuRobo plans the complete short push program
  -> RM75 executes the real push
  -> RRTrack observes the actual T response
  -> optional simulator-response calibration
  -> replan
```

The existing `PoseMatrixPushTTracker` remains only a mathematical 6D-to-planar
adapter. It is **not** a second tracking algorithm. `RRTrackPushTTracker` now
wraps the actual RRTrack stream directly.

---

## 3. What “simulation as the brain” means in each real demo

The three scenarios intentionally exercise different kinds of future reasoning.

| Real demo | Simulator/world-model role | Real-world feedback |
|---|---|---|
| Sorting / organization | predict occupancy, reachability, ordering, collisions, and the consequences of a multi-object program | RRTrack scene refresh / atom validation |
| Magnetic assembly | predict whether a structure prefix is geometrically/physically admissible, whether the next connection is reachable, and whether the robot can exit after placement | RRTrack verifies the actual assembly prefix |
| Push-T | predict action-conditioned contact motion under many candidate short futures | RRTrack measures the real T motion after every push |

This is stronger than simply replaying a planned trajectory in ManiSkill. A
simulation result counts as “brain” evidence only when its predicted consequence
changes which real action/program is selected.

---

## 4. Real experiment requirement

Every main-table scenario must report physical RM75 outcomes.

### Sorting

Recommended minimum protocol:

- randomized real layouts;
- 3 / 5 / 8 objects if feasible;
- full nominal program versus atom-boundary re-observation;
- occupancy conflicts / buffer move cases;
- task success, planning failure, execution failure, recovery count, total time.

### Magnetic assembly

Recommended progression:

- 2-piece calibration fixtures;
- 4-piece stable wall/corner;
- 6–8-piece gate/bridge/house prefix;
- up to 12 pieces after geometry and release calibration are stable;
- text-generated blueprint and deterministic-template control;
- task success, structure-prefix validity, placement error, recovery/replan count.

### Push-T

Push-T is a **physical closed-loop demo**:

- random initial real T pose;
- random reachable target pose;
- execute one physical push at a time;
- RRTrack after every push;
- compare fixed simulator parameters versus optional online response calibration;
- report final pose error / success, pushes required, planning latency, prediction
  error, and model mismatch over horizon.

The quasi-static Python model is only an early world-model implementation and
software test backend. Final evidence requires real pushes, and the planning
simulator can be upgraded to ManiSkill/another contact simulator without changing
this experiment contract.

---

## 5. Baseline matrix under the corrected definition

All external baselines should ultimately be evaluated on physical tasks when the
interface makes that comparison meaningful. Simulator-only benchmark results are
supporting evidence, not substitutes for our physical success table.

### Overall closest baseline — Prompting with the Future (PWTF, RSS 2025)

PWTF is the closest conceptual comparison because it uses an interactive digital
twin to simulate candidate action outcomes and uses those futures for model
predictive control on real manipulation tasks.

Recommended fair adaptation:

- same RRTrack-derived real state when possible;
- same RM75 / object assets / task goal;
- same low-level motion primitive budget;
- PWTF-style future rendering + VLM evaluation versus our structured
  geometry/contact/task readout;
- report simulator calls, VLM/API cost, wall time, physical success and recovery.

If its reconstruction/action interface is replaced, label the implementation
`PWTF-adapted`, not an exact reproduction.

### Sorting / magnetic planning baseline — PDDLStream + shared cuRobo

Use PDDLStream for high-level TAMP while reusing the same grasp/place generators,
IK/collision tests and cuRobo motion planner. Give both methods the same magnetic
support rules. This isolates whether simulator consequence reasoning improves
multi-step ordering/assembly beyond symbolic task-and-motion search.

### Language-only baseline — Code as Policies style

Use the same LLM, RRTrack perception and bounded robot skills, but do not expose
simulated future outcomes. Keep execution feedback/replanning available. This
isolates the value of explicit future prediction from language decomposition.

### Push-T world-model baseline — DINO-WM or LeWorldModel

Both provide Push-T planning code/checkpoints. For the main physical comparison,
train/adapt the chosen learned world model on the same observation/action domain
or clearly separate a standard Push-T benchmark comparison from the RM75 real
experiment. Do not directly compare an off-domain checkpoint against our
calibrated real-state simulator and call it fair.

### Required internal ablations

For all three scenarios, the most important shared ablation is:

```text
current-state planning only
vs
single-future / short-horizon prediction
vs
multi-future simulation-guided planning
```

Keep RRTrack, robot, task rules and low-level cuRobo identical.

For Push-T additionally compare:

```text
open-loop multi-push
vs
RRTrack closed-loop replan after each push

fixed nominal simulation response
vs
online response-calibrated simulation
```

For sorting/magnetic additionally compare:

```text
frozen initial real scene
vs
RRTrack atom-boundary state refresh
```

---

## 6. Claim boundary

Safe wording:

- all three tasks are evaluated on the physical RM75;
- all three share the same recoverable RRTrack real-state estimator;
- the simulator acts as an explicit planning/world model;
- task-specific reasoning reads different aspects of predicted future states;
- no task-specific policy training is required by the core pipeline.

Do not claim until experimentally established:

- simulation perfectly predicts real dynamics;
- Push-T physical parameter identification recovers unique physical constants;
- magnetic simulation models actual magnetic force accurately;
- ManiSkill success guarantees real success;
- all three scenarios currently have completed real-robot success-rate tables.

The immediate implementation goal is to make this architecture true in code;
the next validation goal is to make it true in physical evidence.
