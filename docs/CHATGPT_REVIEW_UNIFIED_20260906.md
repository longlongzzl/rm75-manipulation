# ChatGPT Review — Unified scenarios, 2026-09-06

审阅基线：`d9385fda6e07916ce7b4f30689314f5d5e3050d9`。工作分支：`chatgpt/unified-three-scenarios`。代码修复提交：`f59f4bec25c1137dc3f09e72690a4c954822a833`。本文件由 ChatGPT 编写；不代替 Codex handoff，不授权任何真实机器人运动，不合并 main。

## 1. 结论与证据边界

认可 `CODEX_VALIDATION_UNIFIED_SCENARIOS.md` 对阶段状态的修正。U0 的软件证据成立，U1/U4 仍待审，U2/U3/U5/U6 未完成。不要把文件存在、单测通过、单个 fixture 多次成功理解成完整阶段验收。

本次读取的记录：

- `CODEX_HANDOFF_UNIFIED_COMPLETION_AUDIT.md` 报告父提交全量本地测试 235 passed。这是 Codex 已报告的结果，不是本轮重新运行的结果。
- IK cache 修正后 frozen 单原子 full-chain 10/16 →14/16，没有已成功 case 回退；这不是 20 个实测多物体场景的 90% 全程序成功率。
- `u1_sorting_sim_summary.json` 原始三物体回放只通过前两项，carrot 失败，完整 SIM 安全门未通过。
- U4 的 200-case 结果和参数拟合仍属解析/合成环境，不是 ManiSkill 接触物理或真机数据。
- 本轮没有修改 `lazy_place`、碰撞集合、IK seeds/tolerance、抓取网格、关节速度限值或磁吸校准常数。

## 2. 这次实际修复的代码问题

### R1 — 规划后的旧观测不能直接用于下发

`program_runner.py` 原先 FULL_PLAN 先 observe、花时间规划，然后把仍然是初始观测的 current 用于验证第一条命令。重建 suffix 后也未重新确认；LOOKAHEAD 第一条缺少同等检查。

现在三个运行模式都在下发前重新 observe；重规划后再验证，按 `max_replans_per_step` 有界重试，持续变化则 INVALIDATED，不无限重试或发送已知过期计划。LOOKAHEAD 的 worker 单独记录规划耗时，不把与执行重叠的等待时间算成规划时间。

当 `stop_on_failure=False`，显式退回观察驱动 CLOSED_LOOP：依赖失败的步骤 SKIPPED，只允许不依赖失败步骤的独立任务继续。不能让一次失败后的预测状态成为后续事实。

注意：这些是步骤边界检查，不是连续碰撞监控；runtime.observe 必须返回真实更新，不能原样回传旧 snapshot。严格的 source stamp 兼容性仍由 runtime 负责实现。

### R2 — 程序下发完成不等于物理任务完成

`PickPlaceProgramExecutor` 新增可选 scene stamp provider、强制反馈开关、atom outcome validator、release readiness callback、cancel 与 stop callback。共享 `UnifiedManipulationSystem` 已透传这些参数。

- `require_execution_feedback=True` 时，没有 joints 或 scene stamp 就不下发。
- source fingerprint 不符或反馈异常即失败；关节偏差容差保持原值。
- outcome validator 失败，不把 atom 标成完成，不执行余下预编译程序。
- release gate 失败，不发送开夹爪命令；初始空夹爪 open 不误判为持物 release。
- cancellation 只保证在下一命令边界停止下发；原生长轨迹的紧急停止必须由真实驱动/硬件实现。
- diagnostics 区分 `command_dispatch` 与 `observed_atoms`。没有 outcome observer 时，`success=True` 仍仅表示兼容旧接口的下发成功，`task_success_verified=False`。
- command timings 记录软件函数提交/返回时间，不冒充真实运动起止或毫秒级无停顿证据。

为兼容离线 recorder，这些 gate 不是隐式启用的真实控制器。正式硬件接入必须显式启用反馈要求、接入观察和停止回调，重新通过原有 U1.4 门。

### R3 — PushT 停滞、无效结果与输入检查

现在用包含位置与 wrapped yaw 的目标误差衡量进展；仅转正方向也能累计进展。执行返回 success=false、规划异常、执行异常、跟踪异常、时间戳不递增或异常转角均停止本轮流程，不对无效 transition 做参数更新或自动重推。

拒绝 NaN/Inf 动作、容差、模型/规划参数。工作空间不再把预测结果 clamp 回内部：这等于凭空模拟了一堵不存在的墙。解析模型保留越界预测，planner 用保守整物体包围圆检查并拒绝无效 future。该规则可能拒绝近边界的可行姿态，必须报告保守性，不声称候选召回完全无损。

算法仍是 **random-shooting MPC**，不是 CEM；MPC diagnostics 已明确写出名称、无效 future 数量、模型步评估数量。没有在本轮伪装完成 ManiSkill rollout。

### R4 — 磁吸结构的语义与几何不能只校验父支撑

- 12 片上限是代码硬约束，不能由 LLM 的 max_pieces=100 绕开；拒绝非整数。
- 拒绝非有限值、反射矩阵、缩放矩阵冒充刚体变换。
- 90° 连接按实际选择的 child_edge 建立旋转基，不再默认每条 child edge 都沿局部 Y。
- 顶面按实际姿态计算，不再假定所有竖片局部 +Y 都指向上。
- slot 宽度使用跨槽方向投影，另行检查沿槽错位；还检查竖片实际底边与槽位置、厚度方向，而不仅是两个父片的位置。
- bridge 检查实际局部 footprint 和支撑高度，不用 max(width,height) 冒充任意方向跨度。
- 反向支撑法向不再让 validator 归一化零向量崩溃。

这些是几何自洽修复，不是磁力/极性/稳定性证明。用户所说的“半边搭接”仍必须通过实物照片、尺寸和局部坐标标注确认，未改变原有 overlap/gap 数值。

### R5 — 独立仓库的碰撞配置路径

全量 CI 暴露 `rm75.yml` 的 collision_spheres 仍指向旧 `/home/zhangzhao/Desktop/lerobot/rm75_pick_place_app/...`。已改为 `spheres/rm75_rough.yml`，沿用 loader 原有“相对 YAML 所在目录”解析规则；新增从不同 cwd 加载且几何字典相等的回归。球体数据、buffer、self-collision 设置没有变化。URDF 和 asset_root 的旧绝对路径仍需在实际 GPU 部署配置中显式核对，本轮没有用猜测路径静默替换真实模型。

## 3. 本轮软件验证

本地获得的是 GitHub Actions 导出的真实 source.zip，不是之前错误交付的空 ZIP。

运行：

```bash
python -m compileall -q rm75_app/scenarios tools/benchmark_world_model_baselines.py
PYTHONPATH=. python -m pytest -q \
  tests/test_review_world_model_safety.py tests/test_review_baseline_protocol.py \
  tests/test_scenario_program_runner.py tests/test_sorting_scenario.py \
  tests/test_magnetic_assembly.py tests/test_magnetic_pickplace_adapter.py \
  tests/test_pickplace_program.py tests/test_pusht_scenario.py tests/test_pusht_tracking.py
```

本地核心 focused 结果：**86 passed**；追加碰撞配置路径回归后最终 focused 为 **87 passed**。在本次最后增加 initial-open 回归前，新安全测试放回原始代码，结果 **44 failed / 8 passed**；其中包含新接口在原始版本不存在的预期失败，因此不能把 44 当成“44 个独立物理 bug”。

全量 CI 首先补齐 CPU/Flask 测试依赖，随后实际得到 292 passed / 1 failed；唯一失败是旧绝对碰撞配置路径。修复并补测试后，`1db7e3ecef65829310796220fa6e42e5b3c37b7a` 对应全量 workflow **34020841433** 实际结果：**294 passed，0 failed，0 skipped，23.77 s**。已下载其 source.zip 和 pytest.xml，逐文件验证本轮源码、测试、配置与本地版本完全相同。这个结果是 CPU/offline regression，不是 cuRobo GPU/ManiSkill/真机验收。

`tools/benchmark_world_model_baselines.py` 已增加解析环境下 H=1/H=3、等 model-step budget、独立 planning/execution model 的对比。四个 synthetic smoke 场景双方均 4/4：这只证明对比脚本能跑，**没有证明多步想象更好**。需更难的固定测试分布与独立物理执行环境。

## 4. 下一轮 Codex：先收集 release 证据，停止盲调等待时间

现有日志有用，但不支持统一等待时间补丁：.25/.5s 在单 fixture 成功，1s 失败；2×/4×慢放也失败。空载 hold 稳定并不排除持物时控制问题。不要改成“所有物体释放前等 .5s”。

请建立一次**不改原轨迹、不加真实动作**的可复现记录：

1. 固定 original portable program 与 scene；同时记录 trajectory dt、controller command interval、每个 physics substep 的 q/qdot、TCP pose/twist、object pose/twist、夹爪开度/速度、接触 pair/impulse、关节限位。
2. 用真实观测/模拟观测测量 release 前后，区分 commanded hold 与 actual still moving。记录 frame id / simulation time，不混用 wall time。
3. 单独判定夹爪 limit excursion、夹爪-物体穿透、控制 tracking、物体接触侧推、支撑目标不一致。不能把 final target error 当唯一成功标准。
4. readiness callback 首先运行 shadow-only，输出 ready/not-ready/unknown 与原因，不控制动作。仅在标定后评估 bounded hold → reobserve → abort 的策略；不直接设为生产默认。
5. 原始失败、诊断变体、策略变体分开保存；测多物体多布局，保留反例。不生成伪 measured scenes。

## 5. 下一轮 Codex 验收清单

- 拉取本分支最新提交；工作区干净；不要覆盖本文件冒称 ChatGPT approved。
- 全量 CPU pytest；GPU 16 frozen cases 保持已成功14项不回退；再做真实输入的多物体 suite。
- 验证真实 scene stamp 与编译 stamp 的生成规范一致，不把原始传感噪声直接哈希成每帧不兼容，也不返回永不过期的旧 stamp。
- 在 no-op executor 和可控 fake observer 上测试 scene changed during planning、callback exception、cancel、failed dependency、outcome mismatch。
- 不把 `command_dispatch` 成功计入 paper 的 real success。
- U2 完成实物几何/料盘测量后，补墙/转角/桥梁/≤12片、各边选择与翻转的全结构图测试；不只测私有 helper。
- U4 先接 recorded tracker，再接独立 ManiSkill rollout；动作/时间/坐标契约与实际 pusher 一致。
- 系统辨识继续保持实验性/未启用：当前 friction、translation_gain、contact_efficiency 的乘积/比值混淆仍在，不能声称分别识别真实摩擦或质量。
- handoff 保存到 `docs/CODEX_HANDOFF_REVIEW_20260906.md`，列精确 commit、命令、失败、测试级别。涉及实际运动仍需用户单独批准。

## 6. 前端与集成的实际边界

本轮没有新增 Web 页面，也没有将三个场景伪装成已完成的统一 Web 产品。当前共享能力主要经 `scenarios/system.py` facade 与 CLI 暴露；原有 `web/scene_workbench.py`、`web/control_panel.py` 到新 facade 的逐项调用、状态展示、取消和物理授权仍属于 U6 集成验收。LLM 接口能生成结构，不等于真实 API 的 30-prompt 或完整图形界面已经验收。

## 7. 论文主线

支持“仿真作为规划大脑”，建议正式名称为 **simulation-backed task-and-motion planning with an explicit world model**。但世界模型负责预测，搜索/规划器负责选择；不能把一个 simulator 类本身称作完整智能体。

整理当前 predicted scene 主要是预期放置效果更新；磁吸是显式规则和几何；PushT 才有动作条件下多未来搜索，且目前使用解析模型。不要把三者都写成已用 GPU 物理引擎进行候选未来比较。

外部基线和公平协议见 `docs/SIMULATION_BRAIN_BASELINES.md`。下一次增方法应优先形成“预测未来确实改变动作/结构/顺序”的证据，而不是继续改一个 cuRobo 调用时间。
