# Control-step joint-limit audit

Task: step 5, collect actual joint-limit evidence beyond stage endpoint tracking.
State: NEEDS_REVIEW.
Commit: containing commit, `git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U1_LIMITS.md`.
Parent commit: 90edc12.
Changed files / reasons: rm75_app/validation/maniskill_gate.py observes all active joints and model get_qlimits in strict replay after motion/gripper control steps, at reset, before checkpoint steps and after final settling. Records raw positions and per-joint excess without numerical slack; rejects nonfinite, reversed and shape-invalid inputs. Legacy observation disabled. tests/test_joint_limit_audit.py adds 5 cases for both limit sides and invalid inputs. Summary JSON and this handoff retain evidence.

Commands run:
```
PYTHONPATH=. python -m pytest tests/test_joint_limit_audit.py -q
/home/zhangzhao/anaconda3/envs/realman/bin/python tools/replay_sorting_program.py --compiled-dir /tmp/rm75_u1_inside_compile_001 --output-dir /tmp/rm75_u1_limits_audit_001 > /tmp/rm75_u1_limits_audit_001.log 2>&1
PYTHONPATH=. python -m pytest tests -q > /tmp/rm75_u1_limits_audit_tests.log 2>&1
git diff --check
```
Test environment: unchanged Python 3.12.7 CPU, realman Python 3.11.14 ManiSkill at 20 Hz control, same compiled program/seed/scene; no limit or dynamics changes.
Before metrics: 202 tests; stage-end tracking only, no per-control-step limit record.
After metrics: focused 5 passed; full 207 passed in 18.49 s, zero failed/skipped. 162 boundary samples; seven arm joints show zero sampled excess. Maximum among all joints 1.000361226e-5 rad on gripper_Left_Support_Joint. Right support maximum 1.19209e-7 rad; right first joint 1.50229e-9 rad. All other joints zero. Containment again passes, but no all-joint safety pass claimed.
Correctness/safety checks: actual model limits/active joint order retained with raw positions; no ignored joints, widened limits or post-hoc numerical allowance. Coverage includes gripper dwell and checkpoint settling. Control-boundary sampling cannot exclude violations between samples or at physics substeps.
Failures and raw statuses: no test failure; positive simulated gripper limit excursions retained and require interpretation. Existing gate.success reflects settled task outcome, not this newly reported joint-limit safety status; combined full SIM acceptance remains NEEDS_REVIEW.
Raw log paths: /tmp/rm75_u1_limits_audit_001.log; /tmp/rm75_u1_limits_audit_001/maniskill_gate_result.json (joint_limit_audit with all samples/limits), strict_replay_summary.json, video/0.mp4; /tmp/rm75_u1_limits_audit_tests.log.
Committed summary path: benchmarks/unified_scenarios/u1_joint_limits_audit.json.
Unexpected observations: very small gripper excursions occur despite successful containment; no assumption that they are acceptable numerical noise. Arm tracking error and limit excess are different measurements.
Open questions / next work: review simulated gripper excursions and complete contact auditing; add broader multi-atom distribution. User asked asynchronously to identify the required 20 measured snapshots with actual joints/targets, since current_table.json lacks measured joints. No synthetic replacement for that acceptance set. No real commands or physics identification. Push still awaits destination approval after earlier rejection, no retry.
