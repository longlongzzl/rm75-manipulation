# Verified sleep versus native joint cache, workers 55-57

Workers 55/56 ran on production 3425529, tree HEAD ffa8b8f. Worker 57 ran on WIP production 2f48069. Preserve that distinction when reproducing: the new production binding currently fails native joint name mapping before execution.

```sh
PYTHONPATH="$PWD" python3 tools/run_network_isolated.py -- /home/zhangzhao/anaconda3/envs/curobo2/bin/python benchmarks/release/evidence/pen_closure/worker_55/probe.py --run-dir "$PWD/runtime_data/swm_release_pen_worker_55" --profile "$PWD/runtime_data/swm_release_pen_worker_55/profile.json" --app-root "$PWD"
```

For 56 change all suffixes to 56. For 57 use worker_51/probe.py with worker_57 run/profile paths. Fresh run directories require the frozen worker_43 request/profile. All retain separately declared +4 mm process-local planning depth and both-group exact-native-equal drive-write elision, not changed production defaults.

Worker 55 fixes sleep lookup to the owned native root link and verifies root/articulation identity. 214 complete joint feedback samples, 192 sleeping. Last 20 position vectors are identical. Final cached joint_6 velocity remains 0.001589460997 rad/s while root reports sleep. Original guard still fails; raw velocities were not modified.

Worker 56 additionally verifies all 20 original links belong to the same articulation and records native spatial velocities. All 192 sleeping samples have exactly zero linear/angular link velocities. All final 20 links report sleep, while raw generalized joint velocity cache remains nonzero. This is native evidence distinguishing spatial rest from cached generalized velocity; it is not a successful grasp or a license to zero arbitrary moving feedback.

PhysX documentation states sleeping articulations have zero spatial velocity on all links: https://nvidia-omniverse.github.io/PhysX/physx/5.3.0/_api_build/class_px_articulation_reduced_coordinate.html . That documents engine semantics, not the installed SAPIEN cache implementation; the latter discrepancy is measured here.

2f48069 adds NativeArticulationVelocity and attaches evidence to primary readback, SWM robot observation and stage evidence. Effective zero is derived only from complete identity, unchanged q, all-link sleep before/after and exactly zero spatial velocities. Raw joint cache remains in evidence, no native setter or threshold change. Awake samples retain raw velocities; inconsistent/nonfinite evidence rejects. Targeted 47 passed / 0.12 s, full 1817 passed / 1 existing warning / 45.31 s.

IMPORTANT: worker 57 FAILED during scene registration with Native velocity joint identity mismatch. The new helper compared canonical wrapper names with namespaced native joint names instead of reusing the existing handle-bijection contract _native_drive_joints. No native action qualification is claimed. Primary/planner cleanup true; robot mirror not yet constructed (null). Next fix must reuse trusted robot.joints_map -> native handles, not suffix stripping or relaxed checks. Unit fixtures did not cover native namespacing, despite full regression passing.

All results exit 1. No hardware/model inference or six-checkpoint success. Raw logs remain under runtime_data; committed evidence is bounded.
