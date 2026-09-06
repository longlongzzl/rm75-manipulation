# Codex handoff — Review 2026-09-06

## 结论与版本

状态：**NEEDS_REVIEW**。按要求先复跑离线全量与 GPU frozen，再记录原始程序释放证据。本轮没有改生产控制逻辑、轨迹、等待时间、碰撞集合、IK 参数或安全阈值；没有真实机器人运动，没有合并 main。未修改 ChatGPT 的审阅文件。

- 分支：`chatgpt/unified-three-scenarios`。
- 开始时工作区干净，从 `d9385fda6e07916ce7b4f30689314f5d5e3050d9` fast-forward 到本轮实测父提交 **`4a654558837f431a8f561519cc6077cf6e73ca9e`**。
- 本轮交付提交可用 `git log -1 --format=%H -- docs/CODEX_HANDOFF_REVIEW_20260906.md` 精确取得，避免文档嵌入自身提交哈希。
- 可审阅的逐案例结果、输入 SHA256、释放边界观测、分阶段指标和编译反例保存在 [机器摘要](../benchmarks/unified_scenarios/codex_review_20260906_summary.json)。原始大体积 trace 留在下述本机 `/tmp`，**不在 Git 内，也不保证永久保存**；摘要保留 trace SHA256。本机 portable program 依赖也不等于跨机器自包含数据包。

## 1. 回归结果及测试级别

| 验证 | 本轮实测结果 | 边界 |
| --- | --- | --- |
| 拉取后的 CPU 全量 | 294 passed，23.21 s | 本地离线，不引用 CI 数字充当复跑 |
| 最终 CPU 全量 | 304 passed，18.90 s | 新增 10 项测试，无失败 |
| 原安全 focused suite | 60 passed，0.75 s | no-op / fake observer，不是硬件停止证明 |
| GPU frozen full-chain | 14/16；原有 14 成功全部保留，0 回退 | cuRobo 规划 + no-op executor，不是物理执行成功率 |

GPU 用 RTX 5060 Ti、torch 2.11.0+cu128、`curobo2` 环境；详细 backend config 在机器摘要。保持 `lazy_place`、原抓取策略及 32 IK seeds 等原配置。逐 `(plan, scene, atom_id)` 对照 `/tmp/rm75_u1_cache_fix_frozen.jsonl`，不是只比较总数。两项原有失败：`current_table/gluestick` 的 `relation_screen`；`yaw_only_current_table_00_05/gluestick` 的 `segmented_chain`。本次每 case 一次，不声称 10× warm 性能结论。

新增测试：shadow 无观测/有越限/无越限三项；接触穿透与相反冲量不相消一项；scene stamp、release、outcome 回调异常，以及立即取消、中途取消、stop 回调异常六项。现有测试覆盖规划中场景变化、失败依赖与 outcome mismatch。取消仍只保证命令边界，不声称终止已下发的原生长轨迹。

## 2. 第 4 节记录方法

新增 `tools/record_original_release_evidence.py`，只在诊断进程包装原 replay 执行入口和每个 physics substep 回调。所有原命令照常执行；核对每段原轨迹位置和 dt，并核对 manifest、frozen inputs、scene、NPZ 的执行前后 SHA256 不变。三物体复跑的 `stage_trace` 和 `checks` 与 `/tmp/rm75_u1_release_motion_audit_001/maniskill_gate_result.json` **逐项完全一致**。

物理记录来自 ManiSkill **CPU physics** 即时状态：physics dt=0.01 s，control interval=0.05 s，每个控制帧 5 个子步。GPU physics 尚未支持，工具显式拒绝，避免把未 fetch 的 GPU buffer 当新观测；这与前面的 cuRobo GPU 测试是不同层级。

每子步保存 simulation frame/time、command frame、q/qdot、真实 controller target 和 tracking error、TCP pose/twist、所有物体 pose/twist、夹爪关节与限位、contact pair/点/normal/impulse/separation。时间原点是本次执行第一个 control step，不是 wall clock。夹爪还记录两 pad 原点距离与投影相对速度：这是**开度代理，不是已标定内表面净间隙**。trajectory dt 逐段保存在摘要。

释放前后各保存完整观测，区分 commanded target 与 actual movement。`tools/summarize_release_evidence.py` 分别统计限位、穿透、跟踪、接触 XY 冲量、支撑漂移，并保留 frame/time、按 atom/phase 分组。另保存冻结支撑中的目标位姿与观测支撑中的物体位姿，不用最终 task error 替代安全项。

Readiness 为 **shadow-only**：无观测为 unknown；存在原始数值越限为 not-ready；没有越限仍因未标定运动/接触阈值为 unknown。不存在未经标定的 ready。数值微小越限也记录，因此 not-ready **不等于已证明危险程度**。返回值不接入动作控制，没有 bounded hold 或统一延时补丁。

## 3. 原始程序与反例

| 原程序/布局 | 子步覆盖 | 原任务结果 | 解释 |
| --- | --- | --- | --- |
| generated00 三物体 | 2300/2300；460 controls | tennis、gluestick 通过；carrot 失败 | 原失败保留，不是完整 SIM 通过 |
| current-table 单 tennis | 800/800；160 controls | task 通过 | 单物体控制组，不算第二个多物体布局通过 |
| 新增 generated02 三物体请求 | 未执行 | 第三项 `sort_03_carriot` 在 relation_screen 编译失败 | 前两项规划不算完整程序成功 |
| 新增 generated03 三物体请求 | 未执行 | 同上 | 保留反例，不执行部分程序 |

两个新请求只改变冻结 scene 路径及 provenance，均明确是 synthetic；初始关节来自模拟器 reset 记录，不冒称实测。未搜索到第二个可执行的完整多物体布局，因此“多布局完整物理证据”仍不充分。

三物体 carrot 最终误差仍为 0.138048 m / 33.145641°。三个释放前 shadow 均 not-ready，分别 frame 500 / 1250 / 2050、simulation time 5 / 12.5 / 20.5 s；原程序仍完整下发全部 18 段，说明 shadow 没有截断或改变原失败。

以下为**整个程序**峰值，不是都发生在 release 瞬间；详细边界实测和分阶段峰值见 JSON：

| 独立指标 | 三物体原程序 | 单 tennis 控制组 |
| --- | --- | --- |
| 夹爪限位超出量 rad | 0.00148094（frame 1078） | 0.0000100036（155） |
| 夹爪-当前物体最大穿透 m | 0.00190786（416） | 0.000399016（206） |
| arm 最大绝对 tracking error rad | 0.215814（56） | 0.215151（56） |
| 当前物体接触点 XY 冲量范数之和 N·s | 3.040395（470） | 2.452894（227） |
| 支撑相对冻结位置最大漂移 m | 0.0136501（460） | 0.000796667（566） |

穿透/越限是观测值；tracking、支撑漂移的验收阈值未标定。侧向指标是每接触点 XY impulse 的范数之和，不是净冲量、持续力或夹爪造成侧推的因果证明，原 engine pair 符号约定保留。容器 inside 目标和最终落点本来可能不同，不能把 release 相对位移自动解释成任务失败。**单 tennis 的 task success 不覆盖这些安全判定。**

## 4. 可复跑命令与原始位置

仓库根目录执行；离线 `python`、GPU `/home/zhangzhao/anaconda3/envs/curobo2/bin/python`、物理回放 `/home/zhangzhao/anaconda3/envs/realman/bin/python`。

```bash
PYTHONPATH=. python -m pytest tests -q
PYTHONPATH=. python -m pytest -q tests/test_review_world_model_safety.py tests/test_scenario_program_runner.py tests/test_pickplace_program.py
```

全量原始日志：`/tmp/rm75_review_20260906_full_tests.log`（294）、`/tmp/rm75_review_20260906_final_tests_v3.log`（304）。Focused：`/tmp/rm75_review_20260906_safety_callbacks.log`。

GPU 的 16 对参数按旧 frozen 文件原顺序展开，等价实际命令如下：

```bash
python - <<'PY'
import json, subprocess
from pathlib import Path
rows = [json.loads(s) for s in Path('/tmp/rm75_relation_screen_benchmark/d2_full_chain_lazy.jsonl').read_text().splitlines()]
assert len(rows) == 16
cmd = ['/home/zhangzhao/anaconda3/envs/curobo2/bin/python', 'tools/benchmark_grasp_relation_screen.py', '--full-chain', '--repetitions', '1', '--output-jsonl', '/tmp/rm75_review_20260906_frozen16.jsonl', '--summary-json', '/tmp/rm75_review_20260906_frozen16_summary.json']
for row in rows:
    cmd += ['--plan', row['plan'], '--scene', row['scene']]
subprocess.run(cmd, check=True)
PY
```

GPU log：`/tmp/rm75_review_20260906_frozen16.log`。摘要也保留 16 行身份与成功/失败结果。

```bash
PYTHONPATH=. /home/zhangzhao/anaconda3/envs/realman/bin/python tools/record_original_release_evidence.py --compiled-dir /tmp/rm75_u1_cache_three_generated00_001 --output-dir /tmp/rm75_review_20260906_original_three
PYTHONPATH=. /home/zhangzhao/anaconda3/envs/realman/bin/python tools/record_original_release_evidence.py --compiled-dir /tmp/rm75_u1_inside_compile_001 --output-dir /tmp/rm75_review_20260906_original_tennis
PYTHONPATH=. python tools/summarize_release_evidence.py --evidence-dir /tmp/rm75_review_20260906_original_three --compiled-dir /tmp/rm75_u1_cache_three_generated00_001
PYTHONPATH=. python tools/summarize_release_evidence.py --evidence-dir /tmp/rm75_review_20260906_original_tennis --compiled-dir /tmp/rm75_u1_inside_compile_001
```

Recorder 不能覆盖已有输出目录，复跑应换新路径。三物体命令 exit 2 表示记录完成但 task 失败；tennis exit 0。每个目录有 `physics_substeps.jsonl.gz`、`evidence_summary.json`、`assessment.json`、`maniskill_gate_result.json`、replay initial state 与 video。对应 log 是目录路径加 `.log`。

```bash
PYTHONPATH=. /home/zhangzhao/anaconda3/envs/curobo2/bin/python tools/benchmark_sorting_program.py --request benchmarks/unified_scenarios/fixtures/sorting_review_20260906_generated02.json --initial-joints /tmp/rm75_u1_strict_replay_002/replay_initial_state.json --output-dir /tmp/rm75_review_20260906_compile02
PYTHONPATH=. /home/zhangzhao/anaconda3/envs/curobo2/bin/python tools/benchmark_sorting_program.py --request benchmarks/unified_scenarios/fixtures/sorting_review_20260906_generated03.json --initial-joints /tmp/rm75_u1_strict_replay_002/replay_initial_state.json --output-dir /tmp/rm75_review_20260906_compile03
```

两个 exit 2 编译失败的 `summary.json`、`frozen_inputs.json` 留在对应目录；对应 log 是目录路径加 `.log`。没有新增策略变体，本轮原始数据与此前延时/慢放诊断目录分开保存。

## 5. 仍未验收的项目与下一步审阅点

编译 source stamp 与冻结 scene 的 canonical fingerprint 一致：`71f14c62f898e67120e66b4449b87c46281d1cc134a2d341facef69e63a815a7`。对其拷贝的 tennis X 位移加 `1e-8 m`，原始哈希就改变。因此只验证了冻结数据契约；**没有验证真实观测降噪、更新与 freshness adapter**，更没有用永不更新的旧 stamp 冒充反馈。

20 个真实测量多物体输入未提供；U2 实物几何/料盘、U4 recorded tracker/独立物理 rollout、U6 实际前端集成均未因此完成。系统辨识仍未启用。本轮收集证据不改变此前阶段待审状态，不声称完整 SIM 或真机安全门通过。

下一步请审阅：子步证据能否区分持物 tracking、夹爪接触和支撑运动贡献，以及应怎样标定 release readiness 的物理阈值。只有阈值与反馈契约明确后，才讨论 bounded hold → reobserve → abort；不由本报告自动启用。
