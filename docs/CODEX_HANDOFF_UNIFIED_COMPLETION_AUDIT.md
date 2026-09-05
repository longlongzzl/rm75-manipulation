# Unified-stage completion audit

Task: reconcile all stages and exact document deliverables with current authoritative evidence, preserving the full goal rather than equating diagnostic progress with completion.
State: NEEDS_REVIEW.
Commit: containing commit (`git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_COMPLETION_AUDIT.md`).
Parent commit: 42e99b9.

Changed files / reasons:

- `docs/CODEX_VALIDATION_UNIFIED_SCENARIOS.md`: current seven-board-stage and strict 14-stage evidence snapshot; U0 offline state and U1/U4 review states reflect existing results. Original requirements and safety gates unchanged.
- `benchmarks/unified_scenarios/u1_sorting_planning_summary.json`: canonical partial aggregation of frozen single-atom regression and independent full-program diagnostics, explicitly not a measured suite pass.
- `benchmarks/unified_scenarios/u1_sorting_sim_summary.json`: canonical partial aggregation of original failure, counterexamples, motion/safety evidence and missing gates; no false SIM pass.
- This handoff: audit method and external dependencies.

Commands/checks run: git status/log; rg over task requirements and actual files; JSON field inspection of U0/U4 evidence; presence check of every named board JSON/CSV deliverable; `rg -l '"joint_positions"|"joint_names"' assets/test_scenes -g '*.json'` returned no matches; `git rev-list --count origin/chatgpt/unified-three-scenarios..HEAD` returned 22 before this commit; JSON syntax checks and `git diff --check`.

Environment: local repository, read-only evidence audit; no new simulation, robot action, network write or test rerun. Latest tested code at parent has 235 passed, zero failed/skipped, 19.24 s (`/tmp/rm75_u1_replay_speed_tests.log`). This commit changes only documentation/summary artifacts.

Before metrics: main board still labeled every stage PROPOSED despite substantial offline work, while U1 results were dispersed among diagnostic summaries. Required canonical U1 summaries absent, required measured suite absent. Named actual magnetic catalogs/measurements, tracker replay and U3/U5 formal results absent. Historical U4 summaries describe synthetic-only limitations, including fitting identifiability/ensemble limitations; neither constitutes later physical identification approval.
After metrics: main board states and canonical U1 partial results now distinguish evidenced work from missing requirements. All-stage completion remains false. No measured suite, real result or calibrated production catalog fabricated. Required U1/U2/U3/U4/U5 scopes are not reduced to available examples.

Correctness/safety: do not merge main, enable physics identification, reinterpret simulator empty hold as physical low-speed testing, or relax simulation gates. Presence of a summary is not semantic completion. Existing camera calibration files do not satisfy panel geometry/pusher calibration. Search of scoped test-scene files cannot prove no measurements exist elsewhere; user must provide their location if available.

Failures/blockers: no user-supplied measured 20-scene sorting distribution, no actual magnetic calibration/inventory, no recorded tracker suite/pusher contract for later phases, no complete sorting SIM safety pass, and no current real-trial authorization. GitHub push remains explicitly rejected by permission review pending destination/payload approval; automated continuations are not that approval. No retry performed.

Raw/evidence paths: canonical summaries link to committed diagnostic evidence, which retains exact local raw paths. U0 file is `benchmarks/unified_scenarios/u0_offline_summary.json`; historical U4 files are `u4_pusht_sim_summary.json` and `u4_pusht_sysid_summary.json` under the same directory. No new large raw logs created.

Unexpected observations: repeated single-fixture successes and many green unit tests leave most real/calibration gates unevaluated. Additional arbitrary model/delay tuning cannot replace missing measurements or justify physical execution.

Open questions for user/GPT: provide measured sorting snapshot directory and actual calibration inputs; approve exactly which pending code/test/result payload may be pushed to the established GitHub branch. Review remaining release/contact model questions before any uncalibrated production change. This audit is not an all-stage completion claim.
