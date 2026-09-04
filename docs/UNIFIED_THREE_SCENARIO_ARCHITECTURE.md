# Unified Three-Scenario Architecture

> Branch: `chatgpt/unified-three-scenarios`  
> Status: software architecture and pure-Python invariants implemented; cuRobo, ManiSkill, perception, and physical-robot integration still require the staged validation in `CODEX_VALIDATION_UNIFIED_SCENARIOS.md`.

## 1. System objective

The three demos are organized around one common research story:

```text
Real observation
      ↓
Task-conditioned virtual state
      ↓
Explicit planning / many-future reasoning
      ↓
Portable action program
      ↓
Real execution and, when needed, re-observation
```

They are intentionally **not** forced into one control abstraction:

| Scenario | Dominant uncertainty | Planning policy |
|---|---|---|
| Tabletop sorting | static multi-object geometry and ordering | compile a complete PickPlace program before motion, then replay continuously |
| Magnetic assembly | discrete structural rules plus precise multi-contact placement | validate a symbolic structure, resolve geometry, compile through the shared PickPlace base |
| Push-T | state changes continuously during contact | simulate many short futures, execute one push, track, and replan |

Sorting and magnetic assembly share the existing `FixedSceneAtomTaskBuilder`, `PickPlaceCoordinator`, `Curobo2Backend`, portable trajectories, and robot/simulation executors. Push-T reuses the same real-to-sim and trajectory-planning boundaries but remains a closed-loop contact task.

## 2. Package layout

```text
rm75_app/scenarios/
├── contracts.py                 # backend-neutral program/scene contracts
├── program_runner.py            # full-plan, lookahead, closed-loop policies
├── pickplace_program.py         # full PickPlace program compilation + replay
├── sorting.py                   # target assignment, occupancy graph, buffer moves
├── sorting_io.py                # sorting JSON schema helpers
├── system.py                    # one facade for all three scenarios
├── magnetic/
│   ├── contracts.py             # panels, joints, inventory and structure graph
│   ├── rules.py                 # symbolic physical-support rules
│   ├── planner.py               # absolute placement resolution + ManipulationPlan
│   ├── geometry.py              # post-resolution geometric checks
│   ├── backend.py               # multi-support PickPlace contact adapter
│   ├── catalog.py               # calibrated panel catalog + tray inventory adapter
│   ├── io.py                    # JSON serialization
│   └── llm.py                   # text → symbolic graph → validate/repair
└── pusht/
    ├── contracts.py             # state, goal, action, model and report
    ├── model.py                 # fast quasi-static forward model
    ├── mpc.py                   # many-future random-shooting MPC
    ├── sysid.py                 # optional online simulator parameter fitting
    ├── controller.py            # push one step → track → replan
    └── backend.py               # short push → continuous cuRobo program
```

## 3. Shared explicit-program layer

### 3.1 Why compile an explicit program

Calling the planner between every physical trajectory stage creates visible stops and exposes the robot to partially generated plans. `PickPlaceProgramCompiler` replaces the physical executor with `InMemoryTrajectoryRecorder` and lets the existing coordinator complete all planning first.

For one atom the recorded program is approximately:

```text
atom_start
approach trajectory
grasp trajectory
gripper close
lift trajectory
preplace trajectory
place trajectory
gripper open
retreat trajectory
atom_end
```

For a multi-atom sorting or assembly task, every atom is compiled against a predicted scene transition. The final result is one `CompiledPickPlaceProgram` containing all trajectory and gripper commands.

### 3.2 Continuity and stale-state gates

The compiler rejects inter-stage joint discontinuities above `max_stage_start_gap_rad`. The executor checks the real current joint state against the first trajectory point before sending any command. A compiled package can be exported as:

```text
execution.json
000_<atom>_<stage>.npz
001_<atom>_<stage>.npz
...
```

The package contains no cuRobo tensors, so a separate ManiSkill or RealMan process can replay it.

### 3.3 Full-plan versus online planning

`ScenarioProgramRunner` implements three generic policies:

- `FULL_PLAN`: compile a complete predicted suffix, execute it, and invalidate/rebuild the suffix if the observed state diverges.
- `LOOKAHEAD`: plan the next step while the current one executes, but only when a runtime explicitly declares itself thread/process safe.
- `CLOSED_LOOP`: observe, plan, and execute one step at a time.

The current PickPlace integration chooses **full-plan replay** as the no-stutter default. Do not enable in-process lookahead on a shared cuRobo backend: its planner state, scene, attachment manager, and CUDA graph are mutable. A future lookahead deployment must use a separate planner process or a proven thread-safe backend.

For long real magnetic assemblies, the safest deployment sequence is:

1. compile the complete nominal structure and program offline;
2. replay in ManiSkill;
3. execute on real hardware with optional observation checkpoints only at atom boundaries;
4. if a checkpoint reports a material pose deviation, discard the remaining program and regenerate the suffix.

The checkpoint/recompile policy is intentionally a deployment decision; the initial implementation guarantees continuous replay of a frozen program and does not pretend that open-loop multi-step execution eliminates physical accumulation error.

## 4. Scenario 1 — tabletop sorting

### 4.1 Frontend representation

A `SortingRequest` contains:

- real scene file/snapshot;
- one final target assignment per object instance;
- target poses, support objects, capacities and slot spacing;
- optional buffer target;
- priority and success tolerances.

Use scene **instance ids**, not asset names, for assignments.

### 4.2 Ordering and occupancy

`SortingPlanCompiler` creates one target slot per assignment and forms an occupancy dependency graph. If object A's target is currently occupied by assigned object B, B must move before A.

A swap creates a cycle:

```text
A target occupied by B
B target occupied by A
```

Without a buffer the request is rejected. With a buffer, one object is moved temporarily, the cycle is cleared, and the buffered object is moved to its final target.

Unassigned objects remain in the planning scene as collision obstacles. The current compiler does not silently move them.

### 4.3 Execution

The sorting frontend produces a normal `ManipulationPlan`. The shared `PickPlaceProgramCompiler` then plans all atoms and the program executor replays them without planning calls between stages.

Use the full-program path only when the observed scene is expected to remain static. If a person moves an object after compilation, rescan and recompile; do not continue a stale program.

## 5. Scenario 2 — magnetic-panel assembly

### 5.1 Do not ask the LLM for Cartesian poses

The LLM emits only a bounded symbolic graph:

```text
pieces:
  logical piece id
  physical tray object id
  calibrated asset id
  role
  flat / vertical pose class
  optional grounded anchor

connections:
  flat_stack
  right_angle_edge
  vertical_slot
  bridge
```

Absolute poses and trajectories are generated deterministically after validation. This prevents hallucinated coordinates from reaching the robot.

The frontend allows at most 12 pieces, validates the first LLM response, sends rule violations back for one repair pass, and can fall back to deterministic wall/corner/gate templates.

### 5.2 Encoded physical rules

The symbolic validator currently enforces:

1. Every physical tray object is used at most once.
2. A free anchor is flat unless explicitly marked externally supported.
3. A vertical panel cannot stand on one lying panel.
4. A free-standing vertical panel must use `vertical_slot` with exactly two distinct flat supports.
5. A 90-degree wall corner uses `right_angle_edge`, has a stable vertical parent and vertical child, and uses approximately half-edge overlap (`0.5 ± tolerance`).
6. A bridge/roof is flat and requires at least two stable supports.
7. Gap and approach-clearance parameters stay inside calibrated bounds.
8. Every non-anchor piece has exactly one placement connection.
9. The dependency graph is acyclic and every prefix is structurally supported.

### 5.3 Geometry resolution

The planner uses a catalog convention:

```text
local X/Y = panel plane
local Z   = panel thickness
```

`vertical_slot` computes the center line between the two selected inner support edges and raises the child so its lower edge enters the slot. The expected inner-edge separation is:

```text
child thickness + 2 × requested gap
```

`right_angle_edge` aligns the mating edge lines and resolves the child plane orthogonal to the stable parent. The overlap parameter is retained in diagnostics and must be calibrated to the real magnet/edge geometry.

`bridge` computes a level pose over multiple support tops and rejects spans longer than the panel can cover.

### 5.4 Post-resolution checks

`MagneticGeometryValidator` rejects resolved structures when:

- two slot edges do not face each other;
- support edges are not parallel/coplanar;
- slot width does not match panel thickness and gaps;
- support heights differ too much;
- a vertical child is not vertical in the support frame;
- 90-degree mating edge lines do not coincide;
- panel normals are not approximately orthogonal;
- bridge supports are not level or the bridge is too short.

### 5.5 Shared PickPlace backend with two-support contact

The original PickPlace task exposed one `place_contact_object_name`. A vertical-slot placement intentionally approaches two supports. `MagneticPickPlaceTaskBuilder` therefore annotates the place candidates with all support instance ids.

`MagneticContactPlanningBackend` applies these exceptions only to:

- place/preplace endpoint contact screening;
- the final contact-adjacent place segment.

Free-space lift and transport continue to collide with every support. No entire gripper link is disabled and unrelated obstacles stay active.

### 5.6 Required real calibration

The example catalog is not a production asset. Before motion planning:

- register the real magnetic panel mesh in `ObjectSpec`;
- measure X/Y/thickness, magnet center/edge offsets and repeatability;
- define the tray object's real instance ids;
- calibrate `gap_m`, half-edge overlap, engagement clearance and release height;
- verify the object/TCP grasp relation and attached-object collision proxy.

## 6. Scenario 3 — Push-T

### 6.1 Receding-horizon loop

Push-T is implemented as:

```text
track current T pose
       ↓
simulate many H-step action sequences
       ↓
select lowest-cost future
       ↓
plan the first short robot push completely
       ↓
execute that one push continuously
       ↓
track the new T pose
       ↓
update simulator parameters (optional)
       ↓
replan
```

The controller never assumes that the simulated future after the first push is still valid.

### 6.2 Many-future planner

`PushTMPC` samples contact points, world push directions and distances. Every sequence is rolled out through `QuasiStaticPushTModel`; the cost includes final/intermediate position and yaw error, action effort, and regression penalty. Only the first action of the best sequence is returned.

This is a lightweight online model, not a claim of high-fidelity contact physics. It is deliberately fast enough to explore hundreds of futures per cycle.

### 6.3 Optional real-to-sim parameter fitting

`PushTParameterEnsemble` contains candidate friction, translation gain, rotational gain, contact efficiency and anisotropy settings. After a real push it predicts the transition under every hypothesis and updates weights according to pose error.

This feature is optional. First establish a stable tracker and pusher; then compare fixed nominal parameters against online fitting. Do not make system identification a prerequisite for the first Push-T demo.

### 6.4 cuRobo short-push program

`CuroboWaypointPushBackend` converts a planar action into:

```text
free-space hover
vertical descent
short contact approach
one or more short planar push segments
vertical retract
```

It plans every segment and checks joint continuity before execution starts, preventing a pause while the pusher is already touching the T.

The scene provider must expose two scenes:

- free scene: complete scene including the T;
- contact scene: identical scene with only the manipulated T omitted/disabled.

Every other object and self-collision remains active.

The existing trajectory executor determines real speed. The current adapter records `speed_mps` at the controller level but does not yet retime cuRobo trajectories to exactly match it; this is a required hardware-validation item.

## 7. Public API sketches

### 7.1 Unified facade

```python
from rm75_app.scenarios import UnifiedManipulationSystem

system = UnifiedManipulationSystem(
    planner=curobo_backend,
    task_builder=fixed_scene_task_builder,
    trajectory_sink=real_or_sim_executor,
    joint_state_provider=read_real_joints,
)
```

### 7.2 Sorting

```python
request = load_sorting_request("configs/scenarios/sorting.example.json")
prepared = system.prepare_sorting(request, scene)
assert prepared.compilation.success
prepared.compilation.program.export("runtime_data/sorting/latest")
report = system.execute_sorting(prepared)
```

### 7.3 Magnetic text frontend

```python
catalog = load_magnetic_catalog("configs/scenarios/magnetic_panel_catalog.json")
inventory = inventory_from_scene(scene, catalog, max_items=12)
structure = system.generate_magnetic_structure(
    "搭一个有两面垂直墙和顶部横梁的小门",
    catalog,
    inventory,
    anchor_pose=assembly_anchor,
    llm=openai_compatible_client,
)
prepared = system.prepare_magnetic_structure(
    structure,
    catalog,
    inventory,
    scene,
    scene_file=scene_path,
)
```

### 7.4 Push-T

```python
controller = UnifiedManipulationSystem.build_push_t_controller(
    tracker=t_tracker,
    cartesian_push_backend=curobo_push_backend,
    estimator=PushTParameterEnsemble.default_grid(),  # optional
)
report = controller.run(goal)
```

## 8. Dry-run CLI

The CLI never sends robot commands:

```bash
PYTHONPATH=. python tools/run_unified_scenario.py \
  sorting-compile \
  --request configs/scenarios/sorting.example.json

PYTHONPATH=. python tools/run_unified_scenario.py \
  magnetic-generate \
  --catalog configs/scenarios/magnetic_panel_catalog.example.json \
  --inventory configs/scenarios/magnetic_inventory.example.json \
  --description "搭一面能稳定站立的磁吸墙" \
  --output /tmp/magnetic_wall.json

PYTHONPATH=. python tools/run_unified_scenario.py \
  magnetic-validate \
  --catalog configs/scenarios/magnetic_panel_catalog.example.json \
  --inventory configs/scenarios/magnetic_inventory.example.json \
  --assembly /tmp/magnetic_wall.json

PYTHONPATH=. python tools/run_unified_scenario.py \
  pusht-sim \
  --goal-x 0.08 --goal-y 0.02 --goal-yaw 0.0 \
  --system-identification
```

To call an OpenAI-compatible LLM for the magnetic frontend, also provide:

```text
--llm-endpoint <chat-completions endpoint>
--llm-model <model name>
--api-key-env OPENAI_API_KEY
```

## 9. What is implemented versus what is verified

### Implemented and covered by lightweight tests

- backend-neutral program and scene contracts;
- dependency ordering and stale-plan invalidation;
- full PickPlace program recording/export/replay;
- initial-joint and inter-stage continuity gates;
- sorting target capacity, occupancy dependency and buffer-cycle expansion;
- magnetic symbolic support rules;
- magnetic geometry resolution and geometric validation;
- LLM JSON parse/repair/template fallback;
- magnetic two-support contact adapter;
- Push-T forward model, MPC, optional parameter fitting and closed loop;
- conversion of one Push-T action to a fully planned short robot program.

### Not yet verified by this implementation pass

- cuRobo full-program compilation on all real sorting objects;
- ManiSkill replay of the new full sorting and magnetic programs;
- real magnetic-panel mesh axes and measured connection geometry;
- real two-support contact and magnet engagement;
- text-generated structures beyond the pure symbolic/geometry layer;
- T-object tracker integration;
- pusher TCP/contact height and real planar push execution;
- Push-T model accuracy or benefit of parameter fitting;
- any physical-robot success rate.

Use `CODEX_VALIDATION_UNIFIED_SCENARIOS.md` before merging or enabling a real execution entrypoint.
