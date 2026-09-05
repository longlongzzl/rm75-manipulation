# Same-path time-scaling diagnostics

Task: test whether reducing commanded motion speed alone improves release stability, keeping path geometry and task difficulty unchanged.
State: NEEDS_REVIEW.
Commit: containing commit (`git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U1_REPLAY_SPEED.md`).
Parent commit: 8087ac1.

Changed files / reasons:

- `tools/diagnose_replay_speed.py`: explicitly opt-in in-memory retiming of loaded events before full strict replay validation. Multiplies scalar or interval dt by a finite factor >=1; copies position arrays unchanged and preserves gripper/event order. Records original/scaled dt, source position hashes and manifest hash. No source package writes or production default changes.
- `tests/test_replay_speed_diagnostic.py`: seven parametrized cases verify scalar/interval scaling, unchanged source arrays/events, rejected speedup/invalid factors and rejection of missing timing rather than inventing it.
- Summary JSON and handoff: complete outcomes including all failures and unexecuted suffixes.

Commands run:
```bash
PYTHONPATH=. /home/zhangzhao/anaconda3/envs/realman/bin/python tools/diagnose_replay_speed.py --compiled-dir /tmp/rm75_u1_cache_three_generated00_001 --output-dir /tmp/rm75_u1_replay_timefactor2_001 --time-factor 2 > /tmp/rm75_u1_replay_timefactor2_001.log 2>&1
PYTHONPATH=. /home/zhangzhao/anaconda3/envs/realman/bin/python tools/diagnose_replay_speed.py --compiled-dir /tmp/rm75_u1_cache_three_generated00_001 --output-dir /tmp/rm75_u1_replay_timefactor4_001 --time-factor 4 > /tmp/rm75_u1_replay_timefactor4_001.log 2>&1
PYTHONPATH=. python -m pytest tests -q > /tmp/rm75_u1_replay_speed_tests.log 2>&1
git diff --check
```

Test environment: unchanged CPU Python 3.12.7 and realman Python 3.11.14 / ManiSkill / SAPIEN; RM75-MultiObjectTask-v1, pd_joint_pos, 20 Hz, seed 0, same three-object frozen program. These are simulations, not robot speed trials.

Before metrics: 228 tests; original time-factor-1 task passes tennis/glue, fails carrot. Opening begins while objects have appreciable linear/angular velocity.
After metrics: 235 tests passed, zero failed/skipped, 19.24 s. Time factor 2 fails tennis at 264.058 mm after 6 stages; opening velocity 1.13945 m/s and 23.7362 rad/s, both above original tennis values. Factor 4 passes tennis but fails glue at 26.4525 mm after 12 stages; tennis opening velocity 0.279062 m/s / 8.30831 rad/s, glue 0.164821 m/s / 6.80417 rad/s. Neither completes the program; both exit 2 without runtime exceptions.

Correctness/safety: all 18 original position arrays unchanged, no point removal, no extra ignored obstacles/links, no geometry/material/controller tuning or task-tolerance relaxation. Strict time/joint-order/start-gap validation still executes on scaled events. Gripper dwells stay one second each; only trajectory time changes. Retiming necessarily changes physics/contact evolution, so these cannot be labeled original-timing replay passes. Maximum sampled joint-limit excess persists at 1.41344e-5 / 0.00133693 rad for factors 2/4. No complete SIM or REAL gate.

Failures/raw statuses: factor 2 stops after first atom, factor 4 after second; later atoms not executed and not counted as failures or successes. Keep original baseline failure and both retimed failures. No 20 mm tolerance increase to accept the 26.45 mm glue result.

Raw evidence: exact paths in `benchmarks/unified_scenarios/u1_replay_speed_summary.json`. Each diagnostic_summary retains dt audit and source hashes; maniskill_gate_result retains stage/gripper/velocity/contact/limit traces.

Unexpected observations: slower trajectory commands do not guarantee slower object motion while grasped. Outcome is sensitive to grasp/contact dynamics, not explained sufficiently by a global playback-speed multiplier. No default retiming adopted.

Open questions / next work: inspect grasp constraint/contact dynamics and calibration before choosing a general release policy. Measured 20-scene suite and real-stage approvals remain missing. Push still blocked without specific destination/payload authorization; no main merge or physics identification.
