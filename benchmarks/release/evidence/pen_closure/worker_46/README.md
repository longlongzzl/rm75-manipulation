# Pen motion/contact diagnostics, workers 46-48

Tested production source: 3425529. The only production-tree change this increment corrects the new NaN regression expectation to the original SceneInvalid exception; it does not weaken rejection. Targeted: 35 passed / 0.11 s. Full isolated tests: 1809 passed / 1 existing warning / 42.44 s.

Each diagnostic uses the original frozen worker_43 request/profile. Reproduce from repository root in a fresh run directory with those inputs:

```sh
PYTHONPATH="$PWD" python3 tools/run_network_isolated.py -- /home/zhangzhao/anaconda3/envs/curobo2/bin/python benchmarks/release/evidence/pen_closure/worker_46/probe.py --run-dir "$PWD/runtime_data/swm_release_pen_worker_46" --profile "$PWD/runtime_data/swm_release_pen_worker_46/profile.json" --app-root "$PWD"
```

For 47 or 48 replace all three directory suffixes. No real authorization argument is used. All use a process-local planning depth hypothesis of 0.016777 m versus the unchanged production default 0.012777 m. Neither geometry, physical parameters, thresholds nor original stage budgets change. These are not production configuration qualification.

Worker 46 instruments the complete object settle readback. 192 measured samples contain only target/table nonzero resultant forces and no target/robot resultant force. This only samples after robot endpoint conditions permit the object check. The bounded archive retains bi from the original full-inventory rows; the raw log preserves all objects. Worker fails at grasp settle budget, no close/lift, all three cleanup readbacks true.

Worker 47 calls the original per-control guard FIRST and then reads target state and all target/other-object/table/robot pairwise forces. It records 244 original control boundaries: approach 20, settle_approach 14, grasp 10, settle_grasp 200. All nonzero target contacts are with the table; no sampled robot resultant force. Target motion exists from the first approach control tick. This is NOT every physics substep and does not prove there was never a robot contact, overlap, zero-net-force pair, or wakeup influence. Same grasp settle failure and successful cleanup.

Worker 48 is a CONTROL EXPERIMENT, not a skill execution. It reaches the same audited pre-execution boundary but intercepts the first approach and instead holds the original measured initial seven arm joints with the original open-gripper command for exactly 200 original control ticks. It never executes the planned grasp path, sets q/qdot/actor pose, or changes scene/physics parameters. Pen is moving for ticks 1-8, then meets the original velocity thresholds on all ticks 9-200. No robot resultant contact is sampled. It deliberately raises diagnostic-complete after the hold, so worker exit 1 is expected diagnostic termination, not task success. Cleanup readbacks all true.

The stationary-initial-arm control disproves a claim that this pen simply cannot settle on the original table in 200 ticks. The grasp-path run differs and requires physics-substep contact/separation and wakeup evidence before attributing cause or changing a planning depth. No contact-model qualification, successful grasp/place, six-checkpoint success, model inference or hardware validation is claimed.
