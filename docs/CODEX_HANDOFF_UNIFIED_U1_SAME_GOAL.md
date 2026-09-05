# Same-goal IK and collision-context audit

Task: distinguish solver differences from changing goal/context between initial screening and grasp planning.
State: NEEDS_REVIEW.
Commit: containing commit, `git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U1_SAME_GOAL.md`.
Parent commit: 48dceeb.
Changed files / reasons: tools/diagnose_linear_solver.py records native and coarse sphere shapes/max parameter difference plus obstacle switches at the first native IK call per linear candidate. Optional mutually exclusive --cross-check-coarse sends the identical native goal to coarse IK, temporarily aligns coarse obstacle switches with native and restores all switches in finally. Native return unchanged. Summary JSON/handoff retain results; no production planner changes.

Commands run:
```
/home/zhangzhao/anaconda3/envs/curobo2/bin/python tools/diagnose_linear_solver.py --plan /tmp/rm75_task001_c1_plans/20260904_154547_llm_pick_place_pid21961_952932160/case_01/manipulation_plan.json --output /tmp/rm75_u1_gluestick_world_audit_001.json > /tmp/rm75_u1_gluestick_world_audit_001.log 2>&1
/home/zhangzhao/anaconda3/envs/curobo2/bin/python tools/diagnose_linear_solver.py --cross-check-coarse --plan /tmp/rm75_task001_c1_plans/20260904_154547_llm_pick_place_pid21961_952932160/case_01/manipulation_plan.json --output /tmp/rm75_u1_gluestick_same_goal_001.json > /tmp/rm75_u1_gluestick_same_goal_001.log 2>&1
PYTHONPATH=. python -m pytest tests -q > /tmp/rm75_u1_same_goal_tests.log 2>&1
git diff --check
```
Test environment: unchanged CPU Python 3.12.7 and curobo2 Python 3.11, same frozen old single-atom plan/default fixture joints. Source inspection confirms native IK and trajectory planner share their scene collision checker. Extra diagnostic calls synchronize and advance coarse solver internal state, so this is not a latency or identical-RNG comparison.
Before metrics: 213 tests; initial screen had feasible relations while native grasp goal IK failed; collision/goal equality not measured.
After metrics: full 213 passed in 18.37 s, no failures/skips. Eight first-native-IK snapshots: both sphere arrays [1,112,4], max parameter difference zero. Native gluestick disabled, all nine other scene entries enabled. Coarse gluestick initially enabled because previous screening restored its switch; no claim that this snapshot represents screen-time policy.
Cross-check: temporarily match native switches, solve exactly native goals with coarse IK current_state=None and four returned seeds. All eight calls fail (0/128 returned entries successful); position error range 0.0318281–0.0650397 m. Original native calls still fail; no path recovery. This contradicts treating initial endpoint success as proof the same goal/context succeeds at linear planning time.
Correctness/safety checks: target-only contact exemption matched, no added ignored object or disabled link. Coarse switches restored in finally; raw native results unmodified. Sphere equality is parameter-array equality, not full mesh/world equivalence. No robot or simulation motion.
Failures/raw statuses: all tested grasp goals remain linear_planner_failed. No assertion of root cause. Need matched candidate/criteria/world snapshots at original screening invocation.
Raw log paths: /tmp/rm75_u1_gluestick_world_audit_001.json and .log; /tmp/rm75_u1_gluestick_same_goal_001.json and .log; /tmp/rm75_u1_same_goal_tests.log.
Committed summary path: benchmarks/unified_scenarios/u1_same_goal_ik_audit.json.
Unexpected observations: coarse IK also fails when given the later actual goal; apparent solver discrepancy is not established by earlier aggregate relation count.
Open questions / next work: trace the selected candidate from screening through approach/grasp construction, comparing exact pose and criteria/state at each point. Full multi-object SIM, measured 20-scene suite and real stages remain incomplete. Push destination approval still absent after earlier rejection; no retry. Physics identification disabled.
