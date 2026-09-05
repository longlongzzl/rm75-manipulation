# Release and dwell contact audit

Task: step 5, close contact-observation gaps during gripper dwell and checkpoint settling.
State: NEEDS_REVIEW.
Commit: containing commit, `git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U1_CONTACTS.md`.
Parent commit: 90c6b27.
Changed files / reasons: rm75_app/validation/maniskill_gate.py includes all scene contacts in strict control-boundary samples, while legacy robot_contact_pairs still defaults to robot-only filtering. Preserves every point impulse vector and sum of point impulse norms alongside existing net impulse norm; opposing impulses cannot disappear from raw evidence. tests/test_replay_contact_audit.py covers filter compatibility and cancelling impulse vectors. Summary JSON and handoff report real replay.

Commands run:
```
PYTHONPATH=. python -m pytest tests/test_replay_contact_audit.py -q
/home/zhangzhao/anaconda3/envs/realman/bin/python tools/replay_sorting_program.py --compiled-dir /tmp/rm75_u1_inside_compile_001 --output-dir /tmp/rm75_u1_contact_audit_001 > /tmp/rm75_u1_contact_audit_001.log 2>&1
PYTHONPATH=. python -m pytest tests -q > /tmp/rm75_u1_contact_audit_tests.log 2>&1
git diff --check
```
Test environment: unchanged Python 3.12.7 tests and realman Python 3.11.14 ManiSkill; same compiled scene/program at 20 Hz control; no dynamics changes.
Before metrics: 207 tests; contacts only aggregated during motion, missing gripper/release and object-object evidence.
After metrics: focused 2 passed; full 209 passed in 18.57 s, no failures/skips. Containment passes again. 162 boundary observations cover initial, six stages, 20 close and 20 open steps, 20 checkpoint pre-step samples and final settling. Release phase includes left/right support finger–ball nonzero contact and ball–holder nonzero contact. Maximum per-record point impulse norm sums: left support–ball 0.4906424 N s, right support–ball 0.6829738 N s, ball–holder 0.01858479 N s. Ball–table reported records have zero point impulse norm sum in this phase.
Correctness/safety checks: no contact is removed from strict raw scene audit; no force conversion or collision exemption. Record counts are not control-step counts: multiple manifolds/records occur per pair per observation (2892 ball–holder records across 20 release observations). Contact existence and net impulse zero are not equivalent to absence of contact. Physical substeps remain unobserved individually.
Failures and raw statuses: no test or containment failure; full collision safety not established. No automatic allowed/forbidden contact interpretation added. Previously observed small gripper limit overruns remain review items.
Raw log paths: /tmp/rm75_u1_contact_audit_001.log; /tmp/rm75_u1_contact_audit_001/maniskill_gate_result.json (joint_limit_audit.samples now also carry all contacts), strict_replay_summary.json, video/0.mp4; /tmp/rm75_u1_contact_audit_tests.log.
Committed summary path: benchmarks/unified_scenarios/u1_release_contact_audit.json.
Unexpected observations: vector-summed net impulse alone can hide opposing contact-point impulses; raw vectors and norm sums now retained. Contact-record multiplicity is large and must not be misreported as impact frequency.
Open questions / next work: review scene/robot collision pairs and multi-object continuous programs; obtain the required measured 20 snapshots with matching assignments/joints. No full SIM_VERIFIED or real trial approval implied. No physics identification. Push remains pending destination authorization after previous rejection; no retry.
