# Exact-native-equal drive write controls, workers 51-54

Production unchanged: 3425529. Run-tree HEAD: 28ef47a. Last full regression remains 1809 passed / 1 warning / 42.44 s; no new regression run or production optimization is claimed.

Worker 51 corrects worker 50's diagnostic inventory error. The original arm, gripper and gripper_passive controller groups are required; passive is type-checked and left untouched. Only selected arm/gripper PD set_drive_targets methods are wrapped on owned controller instances. Original set_action, cache updates and changed target writes still run. A write is skipped only when all requested positions exactly equal current native readback, with immediate unchanged-drive readback verification. No tolerance, parameter, collision offset, threshold or budget change.

```sh
PYTHONPATH="$PWD" python3 tools/run_network_isolated.py -- /home/zhangzhao/anaconda3/envs/curobo2/bin/python benchmarks/release/evidence/pen_closure/worker_51/probe.py --run-dir "$PWD/runtime_data/swm_release_pen_worker_51" --profile "$PWD/runtime_data/swm_release_pen_worker_51/profile.json" --app-root "$PWD"
```

For 52-54 substitute all directory suffixes. Use fresh directories and frozen worker_43 request/profile. Probes 51-53 call worker_49's owned-primary substep probe; 54 calls 51 and adds articulation feedback. All retain the separately declared process-local +4 mm planning-depth hypothesis, not a production default change.

- 51, both groups: arm skips 215 and writes 29; gripper skips 244 and writes 0. Pen sleeping in 1119/1220 substeps and final target velocities zero. Grasp nevertheless fails its original 200-step settle budget: maximum joint speed 0.001589460997 rad/s (joint_6); joint_4 also -0.001049020677 rad/s. End arm error 0.000178217888 rad. Object settle callback is not sampled when robot fails idle, so null object_idle is NOT a claim of moving pen; independent substep target readback shows sleeping.
- 52, arm only: 215 skips / 29 writes. Gripper original writes preserved. End robot speed 0.000072719253 rad/s, but pen remains moving with final world angular velocity [0.038024365902,0.080843910575,-0.002595341299] rad/s. Grasp budget failure.
- 53, gripper only: 244 skips / 0 writes. Arm original writes preserved. Same final robot/target values and grasp budget failure as 52.
- 54: additional robot-sleep diagnostic FAILS because it queries the PhysxArticulation aggregate for a sleep property not exposed by this installed API. No valid added articulation-feedback samples; do not infer its sleep state or zero reported velocity. The original velocity data and safety guards are untouched. A separate network-isolated type-introspection command confirms sleeping exists on PhysxArticulationLinkComponent, while PhysxArticulation exposes root/get_root but no sleep readback. The next diagnostic must query the owned root link and bind its identity.

All four worker results exit 1 and resource cleanup readbacks are true. No successful close/lift or grasp/place checkpoints. Both-group elision changes the target wake behavior but has NOT met simultaneous robot/object idle requirements. No optimization is promoted into production; no hardware/model inference was run.
