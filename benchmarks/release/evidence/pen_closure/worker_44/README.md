# Bounded +4 mm planning-depth hypothesis

Worker 44 ran before production commit 53a912b (HEAD 7f710e3). Worker 45 ran on 53a912b. Both use this same probe and the same original request/profile. Archive-time source hashes in worker_44 describe 53a912b, NOT its earlier tested source; use the stated commit for worker 44 reproduction.

```sh
PYTHONPATH="$PWD" python3 tools/run_network_isolated.py -- /home/zhangzhao/anaconda3/envs/curobo2/bin/python benchmarks/release/evidence/pen_closure/worker_44/probe.py --run-dir "$PWD/runtime_data/swm_release_pen_worker_44" --profile "$PWD/runtime_data/swm_release_pen_worker_44/profile.json" --app-root "$PWD"
```

For worker 45 substitute both run/profile directory suffixes with 45. Each directory must be fresh with request.json and profile.json from the frozen worker_43 input. Native artifacts remain in runtime_data. No real authorization argument is used.

The probe replaces only the process-local bi ObjectSpec planning depth, from 0.012777 to 0.016777 m. It calls unchanged formal worker main, restores the spec on exit, and does not change production defaults, scene geometry, physical parameters, collision thresholds, or motion budgets. It is a hypothesis, not a qualified production configuration.

Worker 44: cached and exact-endpoint private closure screening no longer vetoed the selected candidate; original stage audit passed and primary approach/grasp movement ran. Independent pre-close capture rejected unsettled object motion. No primary gripper close or lift completed. Worker exit 1. Resource closure readbacks true.

Worker 45: complete registered object velocity feedback is now wired into the original stage settle loop. Approach settled in 14 control steps. Grasp exhausted its unchanged 200-step budget with robot idle but bi moving: linear velocity [-0.0004233875401,-0.0012440008076,0.0000183736030] m/s; angular velocity [0.0808439060899,-0.0380243754914,-0.0025953405092] rad/s. No close/lift command followed; three resource cleanup readbacks true. The cause of pen motion is not yet attributed to a contact pair or numerical instability. No default depth change is justified.

Neither run completes grasp/place or six fresh skill checkpoints. No model inference or hardware was run.
