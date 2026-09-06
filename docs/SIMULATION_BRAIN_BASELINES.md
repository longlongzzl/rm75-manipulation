# Simulation as a Planning Brain — baseline report

日期：2026-09-06。对象：RM75 三场景系统；不包含另一篇 PPO sim2real。结论基于作者论文、项目页与官方代码；下方“如何适配”均是本项目建议，不是宣称已复现。

## 1. 主线怎样定义才可检验

建议：**以真实观测构建显式世界模型，在其中预测候选操作的后果，再选择真实动作。**

```text
Real observation -> current state / geometry
                         |
Task goal -> candidate actions/programs
                         |
              action-conditioned prediction
                         |
              compare consequences + search
                         |
              execute action / checked prefix
                         |
                    re-observe
```

令 `s_hat = R(observation)`，`F(s_hat, action) -> predicted_state, contacts, constraints`，planner 搜索动作序列以最大化任务效用。F 可以是显式物理模拟，不必是学习型神经网络；但必须被决策实际调用，而不是只用于可视化回放。

当前的实现边界：

| 模块 | 已有机制 | 不能混称为 |
|---|---|---|
| 整理 | 目标占用/依赖编译、cuRobo 多阶段规划、预测场景更新 | 已用 physics 搜索不同任务顺序的结果 |
| 磁吸 | LLM/模板结构、规则/几何验证、共享 PickPlace | 已用真实磁力预测装配稳定性 |
| PushT | 解析 forward model + random-shooting MPC + 重新观测 | ManiSkill GPU MPC、CEM 或已标定真实动力学 |

因此文章可围绕这个方向组织，但不能声称首次用仿真作为世界模型，也不能将现有软件回归当成方法创新或三场景真机成功证据。

## 2. 最值得对比的外部工作

### A. Prompting with the Future（PWTF，RSS 2025）——最近的整体方法

作者用交互式数字孪生预测候选动作序列的结果，将预测画面交给 VLM 评价，再由采样式 MPC 决策。它明确将物理模型用于行动之前的预测，而不是训练一个任务 policy。[1]

官方代码已提供扫描环境 demo、重建流程、ManiSkill 相关依赖和可保存 trajectory/joint actions。[2] “开源”不表示 RM75 即插即用；还需要匹配机器人、动作接口、资产与渲染。

**本项目适配建议：**固定相同 real2sim 场景、相同候选动作集、相同底层 cuRobo。用 PWTF 风格的“渲染未来 + VLM 排序”与我们设计的结构化几何/接触/任务状态评价比较。保留原论文的核心未来评价机制。替换重建器/动作空间后必须写 `PWTF-adapted` 并披露差异，不把它当原版完整复现。

**实验价值：**回答结构化的 simulator readout 能否更稳、更省查询地支持长操作程序。必须同时报告 VLM token/API 成本、future 数、图像视角、wall latency；不能只比较动作执行时间。

**优先级：**最应该放相关工作和整体方法对照；GPU物理预测链可运行后再接，不让 3DGS 重建拖住核心实验。

### B. PDDLStream（ICAPS 2020）——整理与装配的强任务/运动规划基线

PDDLStream 将符号规划与黑盒采样器结合，原生面向 TAMP；官方仓库有 Adaptive/Focused 等算法及操作/搭建示例。[3]

**本项目适配建议：**符号层表示 object-at、slot-free、support-ready、holding；streams 复用当前 grasp/place candidates、cuRobo IK/碰撞/运动生成。磁吸的两底片支撑与半搭接规则同时提供给所有方法，不允许 ours 独享任务规则。

**比较什么：**未来占用与支撑依赖带来的计划选择、任务长度、死锁/缓冲搬运、计划成功率与时间。相同底层 planner 后，比较的是任务决策组织，不是 CPU RRT 与 GPU cuRobo 的速度差。

**不当用法：**不能把现有 sequential cuRobo 或简单 topological sort 改名 PDDLStream；确实运行它的 planner 才是外部 baseline。

**优先级：**整理/磁吸最值得实现的第一条强外部基线。

### C. Code as Policies（CaP）——不用显式未来预测的语言规划对照

CaP 通过语言模型生成调用感知/控制 API 的程序，也能表达反馈循环，不能把它描述成天生只能开环执行。[4]

**本项目适配建议：**same LLM + same perception + same pick/place primitives + same magnetic rule validator。移除对未来模拟结果的访问，但保留执行反馈和合理重规划。限制生成程序只能调用受控技能，不直接执行任意 Python/系统命令。

比较“语言+当前状态”与“语言+动作条件未来”是否真的有差异；记录首轮合法率、修复次数、实际成功、重试与成本。若我们只改了 prompt，要命名 `CaP-style shared-skill baseline`，不能声称完全复现。

**优先级：**成本较低、解释性强的辅助外部对照。

### D. DINO-WM——PushT 的学习世界模型对照

DINO-WM 在预训练 DINOv2 patch 特征上学习动作条件动力学，以目标视觉特征和 test-time action optimization 规划。[5] 官方代码有 PushT 数据、已训练 checkpoints 与 `plan_pusht.yaml` 入口。[6]

**推荐两种公平路线：**

1. 先在其标准 PushT 环境中接入我们的显式模拟规划器，与官方 checkpoint 对比，使用完全相同的初始/目标状态、观测、动作归一化、frameskip、horizon 与成功判据。
2. 真正比较 RM75 的 PushT，则在同一 RM75 数据/渲染域训练或适配 DINO-WM，并计入数据与训练成本。不能把标准 2D checkpoint 直接喂真实七轴相机画面，失败后宣称显式模拟更优。

位姿已知与纯视觉输入的先验不同，需额外提供统一感知版或 labeled oracle-state 上界。不得拿论文已有成功率直接拼在我们的真实 demo 表里。

**优先级：**PushT 首选学习型世界模型 baseline；不要求它代替磁吸结构规划。

### E. LeWorldModel（LeWM，2026）——更新的可选视觉世界模型

LeWM 是 raw-pixel JEPA 世界模型，作者用 CEM 做 goal-image planning，覆盖 PushT 等任务，提供代码和 checkpoints。[7]

它适合作为 DINO-WM 之外的较小模型对照，但不能用项目页上的相对加速倍数代替我们自己的相同硬件/预算测量。沿用上面的标准 PushT 或统一 RM75 数据路线；不要同时迁移多个学习基线而失去主系统验证时间。

**优先级：**可选第二学习基线，先完成 DINO-WM 或选择 LeWM 其中一个。

### F. SyncTwin（2026）——近邻，先放 related work

SyncTwin 聚焦重建、连续 real2sim 同步与动态/遮挡下安全运动规划。[8] 截至本次查看，官方项目页仍写 `Code (coming soon)`。因此不能把它列成“马上可完整复现”的已开源基线。

对齐关系：它的重要性在感知同步与动态几何，不是一定要与我们做同一套磁吸结构搜索。可对比同步/不更新状态的机制，但自实现的同步策略只能称 `synchronization ablation`，不能假装是 SyncTwin 原版。

## 3. 最终推荐的主表组合

| 场景 | 首选外部对比 | 必需内部对照 | 隔离的问题 |
|---|---|---|---|
| 整理 | PDDLStream + shared cuRobo；CaP-style | 当前一步贪心；完整任务计划；有/无新观测 | 占用/任务依赖与计划失效处理 |
| 磁吸 | PDDLStream + shared rules；CaP-style | LLM+规则；增加几何检查；增加物理预测（完成后） | 合法结构与物理后果是否混淆 |
| PushT | DINO-WM 或 LeWM；PWTF-adapted 可选 | H=1 vs H>1；解析 vs physics；open/closed loop | 动作条件预测与滚动反馈的收益 |
| 整体系统 | PWTF-adapted | 同候选集无未来评价 vs 有未来评价 | 仿真参与决策本身的必要性 |

不要求所有外部方法跑全部三个场景。把不适用任务写 N/A 并解释，比强行套错误接口后给零分更公平。

## 4. 第一批应做的实验

### E1：未来预测是否有决策价值

冻结同一候选动作/程序集；在几何都可行的候选间比较无未来评价、单步预测、多步预测。记录 chosen action 差异及真实后果，不只展示多条想象轨迹。

整理可用目标占用/缓冲顺序；磁吸可用每个搭建前缀、支撑与退让通道；PushT 可用接触点/转向/目标位姿。若规则已经唯一决定操作且无需物理预测，应诚实说该子任务考验符号/几何模块，不能夸大 physics 收益。

### E2：公平预算

固定 scene、资产、初始 q、机器人/夹爪模型、底层 planner 参数、安全边界、任务规则。报告：

- 每次决策的 forward model steps（例如 N×H）；
- 候选条数、horizon、模型/参数假设数；
- wall time P50/P95、硬件、显存；
- 重建/标定/训练数据成本和 API 成本；
- 实际执行动作次数、任务总时间、恢复次数。

相同 N 不等于相同计算，H=3 每条轨迹有更多预测；相同 N×H 也不等于同 FLOPs，因此两种预算都报告。不要只拿 warmed GPU 中一个 kernel 的时长作为总规划时间。

### E3：独立执行环境

规划器用 simulator A / learned model；评测环境 B 有独立状态、随机种子和 held-out 参数。不能把 planner 写入的预测目标位姿直接当执行结果，也不能让 planner 获得评测世界隐藏参数（除 labeled oracle）。

解析模型与解析模型之间的实验只用于软件/消融预检；真实主张需要 ManiSkill 的独立 rollout 与最终实物执行。当前参数模型里 friction、translation_gain、contact_efficiency 存在等效响应混淆；不得声称从位姿一步误差独立恢复所有物理属性。

### E4：准确 real2sim 的贡献

同一方法下比较 measured geometry/pose 与受控位置角度扰动，报告 pose/几何误差 → 预测误差 → 选错计划 → 任务失败。将未知障碍在线建模与未知被操作物体分开；不是所有未见物体都能抓取。

### 统一指标

task success 分子必须来自 outcome observer；分母含规划失败。另报 conditional transfer success（真实成功/已编译计划）但不能只用这一项隐藏 planner failure。报告错误/越界/异常接触，终点位姿或统一 IoU，碰撞预测 false pass，未来一步/多步误差以及误差随 horizon 的变化。小样本使用原始成功次数和置信区间，不夸大细小百分比。

## 5. 本轮附带的实验脚手架

```bash
PYTHONPATH=. python tools/benchmark_world_model_baselines.py \
  --episodes 50 --horizons 1 3 --budget 192 --max-steps 15 \
  --output /tmp/synthetic_horizon_ablation.json
```

此脚本只实现项目内部 random-shooting H1/H3 对照，明确输出 `ANALYTIC_SYNTHETIC_ONLY`，`external_baselines_reproduced=[]`。默认 nominal 模型不访问 execution parameters；`--parameter-modes nominal oracle` 中 oracle 明确标注特权。

它不是 PDDLStream/PWTF/DINO-WM/LeWM 的实现，也不是新的物理世界模型。先用它固定计时/预算/成功记录的协议，再迁移到独立物理模拟与外部 baseline。

## 6. 可用的论文表述

推荐：`Simulation as an Explicit World Model for Task-and-Motion Planning`。

可以主张的研究问题：在当前真实场景的显式模型中，哪些任务后果需要物理预测，哪些可以用符号/几何推理处理；在统一动作底座和反馈契约下怎样协同。

不能先写成结果：首次 real2sim2real、首次不训练策略、仿真保证安全、消除 sim-real gap、已支持任意未知物体、已实现通用磁吸物理。PWTF/PDDLStream 等近邻已经使这些宽泛 novelty claims 不成立。[1][3]

## 参考来源（原始作者/官方实现）

[1] Ning et al., Prompting with the Future, RSS 2025. https://arxiv.org/abs/2506.13761 ; https://prompting-with-the-future.github.io/

[2] PWTF official code. https://github.com/TritiumR/Prompting-with-the-Future

[3] Garrett et al., PDDLStream, ICAPS 2020. https://arxiv.org/abs/1802.08705 ; https://github.com/caelan/pddlstream

[4] Code as Policies. https://code-as-policies.github.io/ ; https://github.com/google-research/google-research/tree/master/code_as_policies

[5] Zhou et al., DINO-WM. https://arxiv.org/abs/2411.04983 ; https://dino-wm.github.io/

[6] DINO-WM official code/checkpoint instructions. https://github.com/gaoyuezhou/dino_wm

[7] LeWorldModel. https://le-wm.github.io/ ; https://github.com/lucas-maes/le-wm

[8] SyncTwin. https://arxiv.org/abs/2601.09920 ; https://sync-twin.github.io/
