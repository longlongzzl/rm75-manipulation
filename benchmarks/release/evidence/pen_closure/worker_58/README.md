# Native canonical velocity mapping repaired; late closure contact retained

Production commit: 120cd1a. Targeted 51 passed / 0.12 s. Full isolated regression: 1821 passed / 1 existing warning / 45.91 s. Previous native registration failure remains archived under worker_57.

Reproduction, repository root and fresh worker_58 request/profile copied from worker_43:

```sh
PYTHONPATH="$PWD" python3 tools/run_network_isolated.py -- /home/zhangzhao/anaconda3/envs/curobo2/bin/python benchmarks/release/evidence/pen_closure/worker_51/probe.py --run-dir "$PWD/runtime_data/swm_release_pen_worker_58" --profile "$PWD/runtime_data/swm_release_pen_worker_58/profile.json" --app-root "$PWD"
```

The helper now reuses original canonical wrapper-to-native handle bijection via _native_drive_joints. Native namespaced strings and joint order are not guessed. Added fixtures cover namespacing, reversed native order, missing/foreign/duplicate handles. Raw native joint cache remains in velocity evidence; sleep-derived zero requires all original link conditions. No threshold change or native velocity setter.

The run STILL uses the explicitly experimental +4 mm depth and exact-native-equal drive elision from probe 51. Those are NOT production defaults. The formal worker now passes initialization and stage audits, settles approach in 14 ticks and grasp trajectory endpoint in 13 ticks with complete robot/object idle, then performs the independent pre-close capture and private prediction. This is not completed grasp skill verification.

Private actual-preclose prediction runs only 100 physics substeps (20 control ticks), with phase no_forbidden_contact_predicted_not_qualified. Actual primary close executes 20 ticks, then at the 79th closed hold tick the original control-boundary guard detects left support/table force [0.000043121567,-0.000003777539,0.537089586258] N and aborts before lift. Target finger forces are not a holding/task success certificate. No successful after-grasp checkpoint, place, or six-checkpoint closure.

This is evidence that the existing private predictor lacks the full closed-settle execution window, not a reason to remove its check or relax collision policy. Next work must include bounded closed settling and matching actual drive-write semantics in the owned prediction before relying on its candidate ranking. Source/solver/contact thresholds and total execution budgets stay unchanged.

All created resource cleanup readbacks true; worker exit 1. No hardware or model inference. PARTIAL_DELIVERY.
