# U0 local validation handoff

Task: U0 branch integrity and pure-software baseline
State: OFFLINE_VERIFIED
Commit: the commit containing this handoff (resolve with `git log -1 --format=%H -- docs/CODEX_HANDOFF_UNIFIED_U0.md`)
Parent commit: ea1c1aa971a185c9ccc3c03925db6b93a2413de6
Target branch: chatgpt/unified-three-scenarios

Changed files and purpose:
- tools/validate_unified_u0.py: repeat the specified compile, test and dry-run commands and retain subprocess logs.
- benchmarks/unified_scenarios/u0_offline_summary.json: measured environment, subprocess results and CLI output evidence.
- this handoff: record validation limits and next inputs.

Commands run: `python tools/validate_unified_u0.py`; the exact subprocess commands are in the summary. Also ran `PYTHONPATH=. python -m pytest tests -q` on detached main 263cd7e at `/tmp/rm75-unified-u0-main-263cd7e`.
Test environment: local Python 3.12.7, NumPy 1.26.4; dependency versions and interpreter path are in the summary.
Before metrics: same-environment main 134 passed in 20.68s.
After metrics: target lightweight suite 25 passed; target full suite 163 passed (initial run 22.12s). No skipped tests or test failures. The recorded repeat is in full.log.
Correctness/safety checks: sorting compiled; magnetic symbolic and strict geometry reports valid; Push-T dry-run reports goal_reached. Example assets were used only for U0 software plumbing. No physical commands or external LLM API calls were made.
Failures and raw statuses: no U0 subprocess failures; no performance/dynamics claim is established by the dry-run.
Raw log paths: `/tmp/rm75_unified_u0/{compileall,lightweight,full,help,sorting,magnetic_generate,magnetic_validate,pusht}.log`.
Committed summary path: benchmarks/unified_scenarios/u0_offline_summary.json

Unexpected observations: the major update exists on the target branch rather than main. Most similarly named remote branches still point to 263cd7e. No merge into main was performed.

## Remaining gates and input inventory

- U1 remains PROPOSED. `assets/test_scenes/current_table.json` contains objects/source/version/scene_name but no joint snapshot. The sorting example explicitly requires verification against a current real-to-sim snapshot. A measured initial joint state and accepted scene capture are needed before freezing the required 20-scene suite; default planner joints must not be labeled real observations.
- U2 remains PROPOSED. Only an example panel catalog and inventory are checked in. Legacy magnetic modules and downloaded magnetic URDF/FBX archives exist, but are not a record of repeated physical panel/connection measurements. Need measured panel dimensions/axes, actual inventory ids, connection gap/overlap and manual repeatability data. Per the board, stop before motion planning if exact real geometry is unknown.
- U3–U6 remain PROPOSED and not validated. In particular, the U4 200-case comparison, tracker replay, cuRobo programs and ManiSkill dynamics have not been run by U0. U0's single Push-T dry-run must not be substituted for those gates.

Open questions for ChatGPT/user: provide the intended real scene/joint snapshot and production magnetic measurements/inventory, and identify the recorded T tracker sequence and pusher coordinate calibration. These inputs determine the next physical-scene validation gates. Pure-software follow-up benchmarks remain separate work, not completed by this handoff.
