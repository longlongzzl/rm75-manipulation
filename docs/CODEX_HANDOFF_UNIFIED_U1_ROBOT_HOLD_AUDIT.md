# Runtime controller and unloaded hold audit

Task: verify action/configuration integrity and whether opening/closing an empty gripper reproduces the persistent loaded release tracking error.
State: NEEDS_REVIEW.
Commit: containing commit (`git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U1_ROBOT_HOLD_AUDIT.md`).
Parent commit: c454437.

Changed files / reasons:

- `tools/diagnose_robot_hold.py`: standalone simulator audit keeps the arm at measured environment-reset joints, commands 3 s open, 3 s closed, 3 s open, records actual controller config/source/URDF, link gravity flags, joint errors and all contact/limit samples. No planner, no true robot sink, no force or model modification.
- Summary JSON and this handoff: runtime evidence and alternative explanations not established.

Commands run:
```bash
PYTHONPATH=. /home/zhangzhao/anaconda3/envs/realman/bin/python tools/diagnose_robot_hold.py --scene-spec /tmp/rm75_u1_release_boundary_replay_001/maniskill_scene.json --output-dir /tmp/rm75_u1_robot_hold_audit_001 > /tmp/rm75_u1_robot_hold_audit_001.log 2>&1
PYTHONPATH=. python -m pytest tests -q > /tmp/rm75_u1_robot_hold_tests.log 2>&1
sha256sum assets/robot_models/RM75_gripper/RM75-B/urdf/RM75-B.urdf /home/zhangzhao/.maniskill/data/robots/RM75_gripper/RM75-B/urdf/RM75-B.urdf
git diff --check
```

Test environment: CPU Python 3.12.7; realman Python 3.11.14 / ManiSkill / SAPIEN; same RM75-MultiObjectTask-v1 scene, pd_joint_pos, 20 Hz, seed 0. Tool ran all 180 control steps successfully. No runtime control changes requiring a new production regression test; full existing suite still runs.

Before metrics: 227 tests; loaded place/open observations show persistent maximum arm error about 0.08 rad. Controller normalization, robot URDF drift, gravity and self-contact were unverified explanations.
After metrics: 227 tests passed, zero failed/skipped, 19.41 s. Final arm errors for open/closed/open phases are 4.28525e-5, 7.25105e-8 and 4.27331e-5 rad. Largest transient is 0.00340678 rad while reopening. Maximum sampled limit excess is 2.61185e-7 rad. No sampled nonzero robot/scene-object contacts; nonzero robot self-contact appears only between gripper_base_link and flange_spacer_8mm, maximum summed point impulse norm 0.0379691 Ns.

Configuration checks: runtime arm order is joint_1..joint_7, absolute unnormalized positions, stiffness 1000, damping 100, force limit 20, force drive, no controller interpolation. All robot link gravity flags are true for disable_gravity, matching BaseAgent's passive-force-balancing implementation. Do not attribute error directly to robot gravity without this caveat. Loaded URDF exactly matches repository SHA-256 e2f7d65c81b9f292f5bb8bf0a0f93df861e9a331e61276424ce53b5bfb899fea. Installed RM75 class has disable_self_collisions=False and manually grouped gripper links; flange_spacer_8mm is not in that group. Source inspection confirms flange -> gripper_base -> gripper_base_link are fixed connections, but contact causality has not been established.

Correctness/safety: no torque increase, stiffness tuning, mass changes, collision exemptions or gravity edits. Kept all scene objects. This is simulator empty hold at reset, NOT the user's stage-6 real low-speed empty trial. No real-stage credit or SIM safety pass inferred from low errors at this one pose.

Failures/raw statuses: no diagnostic runtime failure. Tiny sampled limit excess and fixed-mount self-contact remain visible. Attempt to import a guessed `realman.rm75` module failed during source discovery; actual registered class source was then located and verified at runtime. This import failure did not affect the completed experiment.

Raw logs and summary: exact paths in `benchmarks/unified_scenarios/u1_robot_hold_audit_summary.json`, including actual controller dataclasses and link gravity flags in raw result.

Unexpected observations: stable empty-gripper hold coexists with fixed mounting self-contact. Neither that contact nor a generic action mapping bug explains persistent loaded error by itself. Need pose/load-specific observations, not a blanket controller/collision change.

Open questions / next work: inspect contact and actuator response at loaded release poses. Actual mass/inertial calibration cannot be guessed from URDF values. Measured sorting suite, complete SIM safety gates and real trial authorization remain outstanding. Push still blocked without specific destination/payload approval. No main merge or physics identification.
