# Release boundary velocity and controller target audit

Task: determine whether the objects and arm are still moving at the moment opening begins, without changing original replay commands.
State: NEEDS_REVIEW.
Commit: containing commit (`git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U1_RELEASE_MOTION.md`).
Parent commit: 3596a77.

Changed files / reasons:

- `rm75_app/validation/maniskill_gate.py`: extend existing strict-replay boundary observation with actual object linear/angular velocities, arm joint velocities and arm controller target position. Tensor data are copied into detached serializable snapshots; controller target is explicitly not actuator effort. Robot velocity uses named arm indices; controller target preserves previously verified controller order.
- `tests/test_gripper_boundary_observation.py`: extend snapshot isolation and no-active-atom cases; add reordered robot-index versus controller-target-order test.
- Summary JSON and handoff: quantified motion evidence and unchanged baseline behavior.

Commands run:
```bash
PYTHONPATH=. /home/zhangzhao/anaconda3/envs/realman/bin/python tools/replay_sorting_program.py --compiled-dir /tmp/rm75_u1_cache_three_generated00_001 --output-dir /tmp/rm75_u1_release_motion_audit_001 > /tmp/rm75_u1_release_motion_audit_001.log 2>&1
PYTHONPATH=. python -m pytest tests -q > /tmp/rm75_u1_release_motion_tests.log 2>&1
git diff --check
```

Test environment: unchanged CPU Python 3.12.7; realman Python 3.11.14 / ManiSkill / SAPIEN, pd_joint_pos, 20 Hz, same seed and full frozen three-object program. Uses local Actor get_linear_velocity/get_angular_velocity and controller position target; no planner, model changes or real robot.

Before metrics: 227 tests; pose/contact observations did not expose release-time velocities. Original carrot release fails at 138.048 mm / 33.146 degrees.
After metrics: 228 tests pass, zero failures/skips, 21.08 s. Stage trace and task-check arrays compare exactly equal to the previous unmodified replay. All 18 stages run; tennis/glue pass, carrot still fails, exit 2.

Velocity observations immediately before opening:

| Object | Linear speed m/s | Angular speed rad/s | Maximum arm speed rad/s |
| --- | ---: | ---: | ---: |
| Tennis | 0.582852 | 18.6321 | 1.25958 |
| Glue | 0.184164 | 15.7129 | 0.204312 |
| Carrot | 0.136892 | 5.50406 | 0.289364 |

Controller target is unchanged across every opening. Actual arm maximum target error before opening is 0.0877495 / 0.0829806 / 0.0757096 rad respectively. After opening it drops to roughly 4.10676e-5 / 4.69685e-5 / 0.000835508 rad. Carrot still moves after opening at 0.0248881 m/s and 1.54039 rad/s.

Correctness/safety: getters only; no action, timing, friction, geometry, drive limit or target modification. Original failure retained. Existing limit/contact safety limitations remain. The observation confirms substantial motion at the release boundary, but does not establish a safe velocity threshold or fully separate gripper load from external support contact. No full SIM or REAL gate declared.

Raw logs and summary: `benchmarks/unified_scenarios/u1_release_motion_summary.json` lists exact raw and baseline paths. Complete vector values, not just magnitudes, remain in gripper_trace.

Unexpected observations: finishing timed place commands does not mean the arm/object is stationary. A position-only endpoint snapshot hid large object angular velocities. Static initial-placement stability was therefore not an equivalent initial dynamic condition.

Open questions / next work: evaluate motion speed/contact settling with explicit timing accounting and a broad frozen distribution, rather than applying the successful delay from one fixture. Do not infer actual actuator saturation from position error alone. Measured sorting suite and physical approvals remain outstanding; push authorization still missing. No main merge or physics identification.
