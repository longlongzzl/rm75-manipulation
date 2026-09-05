# Strict replay gripper boundary observation

Task: distinguish opening displacement from retreat displacement in the failed third atom of the complete generated00 three-object program.
State: NEEDS_REVIEW.
Commit: containing commit (`git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U1_RELEASE_BOUNDARY.md`).
Parent commit: 50273f4.

Changed files / reasons:

- `rm75_app/validation/maniskill_gate.py`: strictly read-only gripper boundary snapshots before/after each close/open dwell, recording active instance ids and detached object/support/TCP/arm/gripper poses. Enabled only for strict replay. Observation host time remains outside excluded gripper dwell timing. No controller, trajectory, collision or tolerance changes.
- `tests/test_gripper_boundary_observation.py`: two tests verify instance lookup, detached snapshots, missing active atom handling, and observation with read-only fake interfaces that expose no physics-step or action API.
- `benchmarks/unified_scenarios/u1_release_boundary_summary.json` and this handoff: quantified evidence and limits.

Commands run:
```bash
PYTHONPATH=. python -m pytest tests -q > /tmp/rm75_u1_release_boundary_tests.log 2>&1
PYTHONPATH=. /home/zhangzhao/anaconda3/envs/realman/bin/python tools/replay_sorting_program.py --compiled-dir /tmp/rm75_u1_cache_three_generated00_001 --output-dir /tmp/rm75_u1_release_boundary_replay_001 > /tmp/rm75_u1_release_boundary_replay_001.log 2>&1
git diff --check
```

Test environment: CPU Python 3.12.7; realman Python 3.11.14 / ManiSkill / SAPIEN, unchanged RM75-MultiObjectTask-v1 pd_joint_pos at 20 Hz. Same frozen program as the previous commit, same environment seed, no replan or robot motion.

Before metrics: 219 unit tests; three-object replay failed carrot at 138.048 mm / 33.146 degrees, but lacked opening boundary poses.
After metrics: 221 passed, zero failed/skipped, 21.51 s. Six gripper before/after pairs captured. Stage trace and task checks compare exactly equal to the uninstrumented replay. Two atoms pass, carrot still fails; replay exits 2.

Correctness/safety checks: getters do not advance simulation. Object poses/arm arrays/gripper dictionaries are detached into serializable snapshots. Both object and supporting object use exact instance ids. Legacy replay does not invoke the new observers. No change in commands, physics timing, position/orientation tolerances or task relation. Existing gripper joint excess remains unresolved; no SIM_VERIFIED declaration.

Failures/raw statuses: carrot displacement during the one-second open dwell is 55.7217 mm, while TCP translation is 2.3873 mm and supporting brush translation is 0.07715 mm. Arm joints continue tracking the held commanded endpoint (maximum boundary joint change 0.075676 rad). These measurements show substantial disturbance before retreat, not proof of a specific friction/contact cause. Tennis drops 93.2159 mm into its container as expected; glue moves 6.5474 mm during opening. Do not treat displacement itself as failure independently of each task contract.

Raw logs and committed summary: exact paths in `benchmarks/unified_scenarios/u1_release_boundary_summary.json`; large logs remain local, not uploaded.

Unexpected observations: static-looking place endpoint is insufficient to establish stable release. Moving the retreat trajectory alone cannot explain displacement that already occurred before it began.

Open questions for ChatGPT / next work: examine opening-time gripper/object contacts and carrot/brush support stability. Keep the failed frozen program intact; do not reduce tolerances or remove the third object. Full measured 20-scene suite is still absent. Real stages await full simulation gates and explicit trial approval. Push remains permission-blocked; no retry without specific destination/payload approval.
