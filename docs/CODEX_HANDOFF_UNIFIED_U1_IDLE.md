# Replay idle-time audit

Task: step 5, measure adjacent-trajectory software idle and separately report declared gripper/checkpoint intervals.
State: NEEDS_REVIEW (single-fixture timing check passes; full SIM gate not proven).
Commit: containing commit, `git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U1_IDLE.md`.
Parent commit: abc27c3.
Changed files / reasons: rm75_app/validation/maniskill_gate.py records monotonic host start/end times around synchronous motion execution, gripper dwell, and atom validation. Summarizer subtracts only declared gripper/checkpoint intervals from successive motion gaps; post-motion diagnostics remain counted as software overhead. tests/test_replay_timing_metrics.py covers exclusion arithmetic, no-data behavior, excessive unexplained gap, and overlapping interval rejection. Summary and this handoff retain raw evidence.

Commands run:
```
PYTHONPATH=. python -m pytest tests/test_replay_timing_metrics.py -q
/home/zhangzhao/anaconda3/envs/realman/bin/python tools/replay_sorting_program.py --compiled-dir /tmp/rm75_u1_inside_compile_001 --output-dir /tmp/rm75_u1_timing_audit_001 > /tmp/rm75_u1_timing_audit_001.log 2>&1
PYTHONPATH=. python -m pytest tests -q > /tmp/rm75_u1_timing_audit_tests.log 2>&1
git diff --check
```
Test environment: unchanged Python 3.12.7 tests and realman Python 3.11.14 ManiSkill; actual control period 0.05 s, same frozen compiled program, no motion retuning.
Before metrics: 198 tests; no committed measured idle-gap evidence.
After metrics: focused 4 passed; full 202 passed in 18.46 s, no failures/skips. Physical containment passed again. Five adjacent-motion software gaps: P95 0.001046453 s, max 0.001049812 s, below suggested 0.150 s. Gripper dwells each 1.0 s simulator time; corresponding host wall intervals 0.0727122 and 0.0911097 s separately retained. Final checkpoint host duration retained in raw events, not part of any gap because there is no next atom.
Correctness/safety checks: unknown/no-gap input yields no pass rather than zero-latency success; no fixed subtraction of simulated seconds from host seconds; all raw and excluded intervals retained. No target, speed, collision or tolerance change. No robot API.
Failures and raw statuses: no test or physical containment failure this run. Timing pass is not full dynamics/safety pass. No inter-atom gap is available in this single-atom fixture.
Raw log paths: /tmp/rm75_u1_timing_audit_001.log; /tmp/rm75_u1_timing_audit_001/maniskill_gate_result.json (replay_timing events/gaps, contacts and checks), strict_replay_summary.json, video/0.mp4; /tmp/rm75_u1_timing_audit_tests.log.
Committed summary path: benchmarks/unified_scenarios/u1_replay_timing_audit.json.
Unexpected observations: simulator advances faster than wall time, so simulated gripper dwell is much larger than measured host call duration. These clocks are reported separately; no real robot latency inference.
Open questions / next work: complete contact and joint-limit audit, multi-atom/multi-scene checks, and required 20 measured snapshots before full U1 acceptance. Real stages and physics identification remain unstarted. Push still awaiting explicit destination approval following earlier safety-review rejection; no retry.
