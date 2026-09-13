# Native support collision geometry diagnostic

Source implementation: 487f993; preceding evidence commit: 1d223f1.
Input: 34 saved private predictions from worker_43. No source world is constructed.
Command (repository root):

```sh
PYTHONPATH="$PWD" python3 tools/run_network_isolated.py -- /home/zhangzhao/anaconda3/envs/curobo2/bin/python benchmarks/release/evidence/pen_closure/gripper_geometry_01/probe.py
```

The first invocation omitted PYTHONPATH and exited 1 before importing the project. The command above exited 0. Both raw logs remain in runtime_data/swm_release_pen_gripper_geometry_01/.

Native convex vertices, scale, shape-local pose and link FK are composed at saved open-prepared and rejected transient joint configurations. The private q readback is checked. Two actual support links yield 136 shape/state rows. Private articulation closed; zero physics steps; no primary world, controller actions, models, or hardware.

Open-prepared mesh maximum local TCP Z reaches 0.005021438228 m; rejected transient maximum reaches 0.025202232189 m. Across recorded poses minimum base Z is respectively 0.001218812432 m and -0.000440565498 m. These are extrema across different candidates, not one trajectory's calibrated grasp or contact surface. They do not identify a valid new grasp depth, predict stable holding, or replace original substep contact checks.

The bi ObjectSpec explicitly overrides grasp_z_offset with 0.012777 m. Changing only AtomTaskBuilderConfig does not change this object's depth. No production geometry, scene, threshold or budget changed.
