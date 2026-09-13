# Shared control semantics and full closed-hold prediction

Production: eb7f3f0. Targeted 39 passed / 0.09 s; full network-isolated tests 1831 passed / 1 existing warning / 48.21 s. Native workers ran serially, with full tests between workers 59 and 60.

Worker 59 retains ONLY the +4 mm planning-depth hypothesis from probe 44:

```sh
PYTHONPATH="$PWD" python3 tools/run_network_isolated.py -- /home/zhangzhao/anaconda3/envs/curobo2/bin/python benchmarks/release/evidence/pen_closure/worker_44/probe.py --run-dir "$PWD/runtime_data/swm_release_pen_worker_59" --profile "$PWD/runtime_data/swm_release_pen_worker_59/profile.json" --app-root "$PWD"
```

Worker 60 is the ordinary production entry with original 0.012777 m depth and no diagnostic wrapper:

```sh
PYTHONPATH="$PWD" python3 tools/run_network_isolated.py -- /home/zhangzhao/anaconda3/envs/curobo2/bin/python -m rm75_app.workcell.worker --run-dir "$PWD/runtime_data/swm_release_pen_worker_60" --profile "$PWD/runtime_data/swm_release_pen_worker_60/profile.json" --app-root "$PWD"
```

Use fresh directories with frozen worker_43 request/profile. Full regression command:

```sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 tools/run_network_isolated.py -- python3 -m pytest tests/ -q
```

NativeDriveCommands now binds the original arm/gripper/passive inventory and preserves original set_action and controller cache updates. A whole controlled position group is written when at least one native-precision target differs; exact-equal groups are skipped. Passive and velocity targets are never written in the control loop. Private prediction shares the same grouped rule; complete target/velocity/timestep initialization remains separate. Source wrappers are owned/restored on context exit. These runs install the source policy but execute no primary atomic commands (source write counters zero), so they do not newly qualify the source setter execution branch or model agreement.

Private closure now continues after 20 close ticks into a maximum of 200 closed-hold ticks. It requires original all-joint velocity, arm endpoint error and all 12 objects' velocity criteria for three consecutive control periods, with forbidden contact checks at every physics substep. It retains raw velocity cache alongside verified sleep-derived effective velocity. Complete idle does not qualify holding, model agreement, or skill success.

Worker 59: 72 actual private predictions (cached pre-screen plus exact endpoint checks), all rejected. 63 reject forbidden contact in gripper_close; 9 exhaust the closed-hold budget. All nine have arm endpoint error 0.061010956764-0.075673103333 rad, above the unchanged 0.02 rad criterion. Five finish with robot/object velocities idle but displaced arm; four still have moving robot and bi. No new prediction reproduces the particular 0.537 N late table event from worker 58, so do not claim exact event agreement. The new horizon eliminates previously incomplete acceptance by other original constraints.

Worker 60: 45 actual private predictions, all reject contact during gripper_close with original depth. Neither run executes approach/grasp/lift in the primary world. All private worlds closed and primary state/drive unchanged during prediction; all final resources released. Both worker results exit 1: no feasible trajectory, not successful task completion.

Next work: identify original contact-aware closing/geometry strategy capable of satisfying arm endpoint and stable bilateral holding, rather than merely extending motion budgets or loosening force/velocity/error criteria. Default depth remains unchanged. Models/hardware not run; PARTIAL_DELIVERY.
