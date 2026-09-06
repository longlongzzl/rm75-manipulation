# RM75 Physical Execution Integration

Status: **CODE IMPLEMENTED / NOT YET PHYSICALLY VERIFIED**  
Branch: `chatgpt/realman-hardware-integration`

## 1. Why this is a small adapter, not a restoration of the old monolith

The historical `longlongzzl/lerobot-realman` repository contains physically used
RealMan SDK paths, but the old scripts mix robot IO, perception, task logic,
simulation, and planning.  The standalone repository should not re-import that
architecture.

The restored boundary is therefore:

```text
scenario / planner
    -> JointTrajectory (radians + dt)
    -> RealManTrajectoryExecutor
    -> RealManSDKSession
    -> Robotic_Arm.rm_robot_interface
    -> physical RM75
```

The same executor is shared by sorting, magnetic assembly, and Push-T.

Historical API behavior retained:

- connect: `RoboticArm(...); rm_create_robot_arm(ip, 8080)`;
- joint feedback: `rm_get_joint_degree()` with `rm_get_current_arm_state()` fallback;
- joint command: `rm_movej_follow()` in degrees;
- gripper: Modbus registers 258/264 with position readback from 259, or
  `rm_set_hand_follow_pos` when explicitly configured.

No legacy FoundationPose/SAM6D/ManiSkill/task orchestration is copied into the
hardware layer.

## 2. Files

```text
rm75_app/execution/realman_executor.py
    RealManSDKSession
    RealManTrajectoryExecutor
    RealManConnectionConfig
    RealManExecutionConfig

tools/realman_preflight.py
    connect + state/readiness only; never arms or moves the robot

tests/test_realman_executor.py
    fake-SDK offline coverage
```

## 3. Safety contract

Opening the SDK session never homes or resets the robot.

Physical motion has a separate explicit state:

```text
DISCONNECTED
  -> CONNECTED / DISARMED
  -> preflight passes
  -> explicit arm_execution()
  -> ARMED
  -> execute
  -> exception / stop => DISARMED
```

Preflight requires:

- SDK connection;
- seven finite joint values;
- cuRobo/RM75 joint-name order `joint_1..joint_7`;
- `rm_movej_follow` availability;
- a controller-side stop/pause method;
- configured gripper IO availability.

Every trajectory stage additionally verifies:

- actual q against the planned stage start;
- timestamps when required;
- finite 7-DoF samples;
- no excessive adjacent target jump;
- actual endpoint against the commanded endpoint.

An execution exception disarms the executor and attempts a controller stop.
This software guard is not a replacement for the physical emergency stop or a
human operator during bring-up.

## 4. First command on the robot PC: no-motion preflight

From repository root:

```bash
PYTHONPATH=. /home/zhangzhao/anaconda3/envs/realman/bin/python \
  tools/realman_preflight.py \
  --ip 192.168.101.20 \
  --port 8080 \
  --require-ready
```

This command does **not** call `arm_execution()`, `execute_trajectory`, or
`set_gripper`.  The output includes:

```json
{
  "ready": true,
  "checks": {
    "connected": true,
    "joint_feedback": true,
    "joint_names_match_rm75": true,
    "movej_follow_available": true,
    "stop_available": true,
    "gripper_io_available": true
  },
  "motion_submitted": false,
  "execution_armed": false
}
```

Do not continue to motion if any check is false.

## 5. Shared wiring for sorting and magnetic assembly

```python
from rm75_app.execution import (
    RealManConnectionConfig,
    RealManSDKSession,
    RealManTrajectoryExecutor,
)
from rm75_app.scenarios.system import UnifiedManipulationSystem, UnifiedSystemConfig

session = RealManSDKSession(RealManConnectionConfig(ip="192.168.101.20"))
session.connect()
robot = RealManTrajectoryExecutor(session)

system = UnifiedManipulationSystem(
    planner,
    task_builder,
    trajectory_sink=robot,
    joint_state_provider=robot.joint_configuration,
    scene_stamp_provider=scene_stamp_provider,
    atom_validator=rrtrack_atom_validator,
    pre_release_gate=release_readiness_gate,
    stop_callback=robot.stop,
    config=UnifiedSystemConfig(require_execution_feedback=True),
)

# Compile first.  No physical motion occurs during preparation.
prepared = system.prepare_sorting(request, observed_scene)

# Human/GUI preflight and simulation review occur here.
robot.arm_execution()
report = system.execute_sorting(prepared)
```

Magnetic assembly uses the same `robot`; only the deterministic structure and
contact planner differ.

## 6. Shared wiring for Push-T

```python
from rm75_app.scenarios.pusht import CuroboWaypointPushBackend

push_backend = CuroboWaypointPushBackend(
    planner,
    trajectory_sink=robot,
    joint_state_provider=robot.joint_configuration,
    scene_provider=pusht_scene_provider,
    tool=calibrated_push_tool_config,
)

# Controller chooses one short push from RRTrack-observed T state.
# The backend plans the whole short contact program before any motion.
robot.arm_execution()
result = system.run_push_t(goal, rrtrack_push_tracker, push_backend)
```

For early Push-T hardware tests run a single short push and reobserve before
allowing a full closed-loop trial.

## 7. Physical bring-up ladder

Do not jump from offline tests to a complete 3-object or multi-piece program.
Use this order:

1. no-motion preflight;
2. read and compare real q with the planner/model q convention;
3. manually place the robot at the first trajectory start and run one reduced-speed
   free-space stage;
4. validate actual joint tracking/error logs;
5. open/close gripper test away from objects;
6. one complete PickPlace atom with RRTrack outcome observation;
7. two PickPlace atoms with an observation checkpoint;
8. 3–5 object sorting program;
9. 2-piece magnetic assembly, then 4/6 pieces;
10. one short Push-T action, then multi-step closed-loop Push-T.

The frontend must expose these as guarded states.  It must not directly call the
RealMan SDK or bypass `RealManTrajectoryExecutor`.

## 8. Still required before REAL_VERIFIED

This integration restores the robot-command boundary, but it does not constitute
physical verification.  Still required:

- run `tests/test_realman_executor.py` in the repository environment;
- run no-motion preflight on the actual robot PC;
- verify the active RM75 SDK exposes the expected stop method;
- confirm robot IP and gripper backend for the current setup;
- measure real joint tracking at reduced speed;
- verify TCP/gripper calibration and workspace/table collision model;
- wire RRTrack outcome checks and timestamps into the real runtime;
- explicitly approve each first physical-motion ladder step.
