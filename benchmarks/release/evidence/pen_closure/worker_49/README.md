# Original primary physics-substep contact/sleep diagnostic

Production source remains 3425529, run-tree HEAD was f9cd92f. No production code change this increment. Last full regression remains 1809 passed / 1 existing warning / 42.44 s on 3425529; it was not rerun here.

```sh
PYTHONPATH="$PWD" python3 tools/run_network_isolated.py -- /home/zhangzhao/anaconda3/envs/curobo2/bin/python benchmarks/release/evidence/pen_closure/worker_49/probe.py --run-dir "$PWD/runtime_data/swm_release_pen_worker_49" --profile "$PWD/runtime_data/swm_release_pen_worker_49/profile.json" --app-root "$PWD"
```

Use a fresh run directory containing the frozen worker_43 request/profile. Both 49 and 50 retain the process-local planning depth hypothesis of 0.016777 m; production remains 0.012777 m. No geometry, material, damping, collision offsets, thresholds, solver policy or stage budget changes.

The probe wraps only the owned primary scene.step and demo.step_and_render after native initialization. Each original step executes once. It records target sleeping, velocities, pose, actual contact bodies and every point's position/normal/impulse/separation. Private prediction worlds are not instrumented or modified. Hooks are restored on exit.

Worker 49: 244 original control steps, 1220 physics substeps, 1709 total records including control boundaries/installation. Robot/target contact candidates occur in 1006 substeps and contain 21839 points. Every robot/target point has positive separation (0.032645259053 to 0.039997987449 m) and zero impulse; no robot/target penetration is recorded. This describes target contacts only, not an all-world collision audit.

First sleep-to-awake transition occurs at control 43 / substep 3 in grasp, together with left support/target contact candidates at about 0.03994 m separation and zero impulse. Another transition occurs at control 44 / substep 1. This supports a proximity-contact wakeup explanation rather than direct robot impact; it does not establish which controller operation prevents later sleep. The original settle guard still rejects after 200 grasp hold ticks. No close/lift or six-checkpoint skill completion. Resource cleanup readbacks all true.

Worker 50 attempted an exact-native-target-equality write-elision hypothesis with the same substep probe. It FAILED before installing the substep probe because its diagnostic assumed exactly arm/gripper controllers. The original configuration also contains gripper_passive (PassiveControllerConfig), confirmed by text inspection of the original and installed model. No drive writes were elided and no hypothesis execution steps recorded. This is a diagnostic implementation error, not a physical counterexample or successful optimization. Source/controller identity checks were not removed. Correctly preserve the third passive group before rerunning; do not blindly wrap every controller.

To reproduce that retained failure, substitute worker_50 for worker_49 in the command; probe 50 calls probe 49 by an absolute resolved repository path. Both worker results exit 1, cleanup readbacks true. Raw substep logs stay in runtime_data; bounded summary and SHA256 inputs are committed. No hardware, camera or model inference was run.
