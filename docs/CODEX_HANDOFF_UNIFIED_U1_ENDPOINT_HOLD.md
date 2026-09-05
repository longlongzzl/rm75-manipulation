# Strict replay gripper endpoint hold

Task: step 5 physical failure diagnosis; one hypothesis, preserve commanded joint endpoint during gripper dwell.
State: NEEDS_REVIEW (improved single-fixture outcome, existing success criterion still fails).
Commit: containing commit, `git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U1_ENDPOINT_HOLD.md`.
Parent commit: 06b9811.
Changed files / reasons: rm75_app/execution/trajectory_executor.py keeps last successfully submitted arm target in strict timed mode and uses it throughout gripper dwell instead of freezing measured tracking error. Legacy mode and pre-trajectory gripper behavior remain unchanged. tests/test_timed_trajectory_executor.py verifies both open/close preserve the endpoint. Summary JSON and this handoff retain before/after evidence.

Commands run:
```
PYTHONPATH=. python -m pytest tests/test_timed_trajectory_executor.py tests/test_trajectory_executor.py -q
/home/zhangzhao/anaconda3/envs/realman/bin/python tools/replay_sorting_program.py --compiled-dir /tmp/rm75_u1_sim_start_compile_001 --output-dir /tmp/rm75_u1_strict_replay_004 > /tmp/rm75_u1_strict_replay_004.log 2>&1
PYTHONPATH=. python -m pytest tests -q > /tmp/rm75_u1_endpoint_hold_tests.log 2>&1
git diff --check
```
Test environment: same CPU Python 3.12.7 and realman Python 3.11.14 ManiSkill environment, same compiled trajectory, scene, seed and timing; no replan or target/collision/tolerance adjustment.
Before metrics: 191 tests; attempt 003 position error 0.1241348162 m, FAILED.
After metrics: focused 15 passed; full 192 passed in 18.41 s, zero failed/skipped. Attempt 004 remains FAILED, position error 0.0609966620 m. Final ball position (-0.298934, -0.281612, 0.042384); planned release target (-0.297130, -0.280102, 0.103335). All six stages replayed. No SIM_VERIFIED claim.
Correctness/safety checks: preserves planned arm reference while gripper opens/closes; no arm teleport, no artificial object attach, no extra dwell/speed change. Keeping commanded reference does not prove physical tracking convergence before release.
Failures/raw statuses: physics gate `target pose outside tolerance`, retained. Grasp/transport contacts show ball held by both support fingers; stage traces alone do not establish root cause or containment.
Raw log paths: /tmp/rm75_u1_strict_replay_003/maniskill_gate_result.json (before); /tmp/rm75_u1_strict_replay_004/{strict_replay_summary,maniskill_gate_result}.json (after); /tmp/rm75_u1_strict_replay_004/video/0.mp4; /tmp/rm75_u1_strict_replay_004.log; /tmp/rm75_u1_endpoint_hold_tests.log.
Committed summary path: benchmarks/unified_scenarios/u1_endpoint_hold_summary.json.
Unexpected observations: after correction, horizontal target alignment is close but final height differs from nominal release height. SortingPlanCompiler currently hardcodes success relation target_pose, whereas the historical plan specified inside. This is an identified semantic mismatch, not permission to relabel this failed trial as success. Current saved final_scene holder pose is inherited scene state, not sufficient proof of its observed post-replay pose.
Open questions / next work: preserve explicit target success semantics and validate containment using actually observed support geometry, with rejecting tests; inspect release video and avoid post-hoc success criteria changes. Retain this trial's original failed status. Broader U1 suite, idle/contact auditing and all real gates remain incomplete. Push still pending explicit destination confirmation after prior safety-review refusal; no push retry.
