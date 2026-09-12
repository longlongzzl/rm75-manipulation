# SWM 软件交付滚动计划

目标：完整通过 `CODEX_SWM_RELEASE_GOAL_827A15E.md` 的 G0–G10。当前为 `PARTIAL_DELIVERY`，硬件未获许可。

基线：`7e3fee4196e7337e18217a0d72518789113eda77`，开始时工作树干净。

## 当前 M0 → M1

1. 修正导入资产发布验证与外部旧仓库状态的责任边界，增加临时源仓库负例及源闭包字节保护。
2. 完善 `swm/integration.py` 的受管理运行上下文，确保初始化异常、失败、取消、正常退出都清理资源。
3. 实装笔的正式 worker 上下文与阶段状态审计，串行运行实际原生 planner/主仿真，取得六个独立检查点证据。

## 后续顺序

- M2：动态参照与 Jimu 原 proof/两底板/原结构模板。
- M3：原 T 连续推送，反馈录制→实测辨识→下一规划消费新版本。
- M4：现有许可内实际离线 FP/SAM3D 与真实 Agent 程序/恢复。
- M5：受约束 pull/rotate、统一控制台与耐久、最终完整发布回归。

验收记录唯一入口：`benchmarks/release/swm_acceptance.json`。所有门槛保持原分母与阈值；运行前冻结具体输入/种子及摘要，不用事后成绩挑选输入。

执行约束：测试/仿真强制网络隔离，正式 pytest 范围限 `tests/`，GPU/PhysX 重任务串行；不连接或操作实体设备。不存在需要用户输入才能继续 M0/M1 软件实现的外部阻塞。

## 最新证据

G0 可移植验证与 S1 管理上下文已提交 `9c38a5449fbe02b4eb35481cb99f40749ded1d74`。定向 125 passed；全量 1518 passed、1 个既有弃用警告。源闭包字节摘要见验收表。下一实际动作：复用 frozen-world 初始化，创建笔的主物理世界、私有镜像、阶段 auditor 与正式 worker 工厂；禁止以当前上下文替身测试宣布 G1 通过。

## M1 阶段推进：349890e

已将抓取候选的后续放置可达性筛查接回共享阶段求解器；预筛路径不执行，place 继续使用后测 attachment 重算。NativeStage 摘要现绑定独立夹爪/attachment、释放几何和接触策略。新增 cuRobo 顺序阶段 auditor，逐段插值查询现有碰撞模型，审核状态切换端点，并在成功/失败后恢复观察对应的规划状态。

软件回归：SWM 120 passed；正式全量 1523 passed、1 个既有警告（45.32 s）。这些不是实际 GPU/主仿真成绩。未注册任何不完整 worker 工厂，缺适配器检查保留。

下一步仍是笔的正式 worker，而不是扩展 fixture 数量：先明确原生碰撞诊断对已附着目标的 world-obstacle 排除（当前诊断会临时启用场景对象，auditor 可能保守误拒绝），接独立 jaw 观测和原代理几何恢复；再复用原 frozen-world create_demo 创建主仿真及私有镜像，取得六次新采集和原生阶段审计证据。夹爪开合中间几何、候选恢复与联合筛查的原生结果仍待验证，不据此宣称 S2 全部完成。

## M1 初始化资源所有权：82be044

已复用原 create_demo 的初始化边界，并在 gym.make 返回后、reset/demo 构造之前注册清理；完整 actor 注册表缺失、初始化异常、取消、意外 live runtime、抓前持续冻结或额外世界创建均拒绝。仅软件替身测试：初始化生命周期加 managed runtime 共 18 passed（0.07 s）；未运行本轮全量。

原生启动尚未执行：源码确认原 resolve_planning_artifact_paths 会在 sim URDF 同目录生成 planning URDF/SRDF，当前新入口仍继承该行为。已询问是否修正为本次运行目录，未收到明确确认。不得运行 tools/validate_swm_native_bootstrap.py 来绕过这项待修复风险；该脚本仍为未提交草稿。无活跃仿真进程，不将待确认解释为真实运行中的等待。下一原生步骤仍为先消除此写入风险，再运行冻结输入初始化、真实读回并接正式 worker。

## S5 独立推进：7f51ceb

在原生启动输出路径确认待回期间，已接通另一明确断点：SharedPrimitiveExecutor 在 push 命令前调用 recorder.begin_action，完成反馈核验后将同一录制句柄放入 ExecutionReceipt；runtime 在后测曝光后调用原 transition observer。严格缺覆盖检查保留，命令异常/后测失败/未安装 observer 都释放对应动作窗口；完成窗口裁剪、无动作时有界滚动缓存，活动窗口满额仍拒绝丢弃起点。

定向 25 passed；正式全量 1540 passed、1 个既有警告（71.32 s）。均为软件回归，不是实际推送或 PhysX 成绩。尚未接实际关节反馈 FK/时钟映射、查询延迟、原 T 两动作辨识及 N+1 的下一规划消费；历史 action-id 集合长期有界策略仍需收口。M1 主线仍优先，初始化输出路径风险未绕过，未启动原生验证草稿。
