# Jimu 完整任务屋顶 IK / 原目标几何证据 — 2026-09-08

**NEEDS_REVIEW：三条非真机链仍未全部完成，完整 Jimu 任务的四个屋顶尚未修成通过。** 本轮把失败定位到已记录原释放目标的夹爪底座/墙体几何冲突；没有修改旧算法、抓放目标、候选、seeds、成功阈值或碰撞规则，没有机械臂/夹爪运动、SDK 或相机连接。

按 [回传模板](CODEX_THREE_SCENE_RESULTS_TEMPLATE.md) 记录。机器摘要：[roof IK summary](../benchmarks/unified_scenarios/three_scene_jimu_roof_ik_20260908_summary.json)。原始目标、关节解、世界坐标、轨迹和运行日志只留本地 `runtime_data/three_scene/jimu_roof_ik_20260908/`、对应 workcell job 及新 snapshot 的 ignored 运行目录。

## 0. 基本信息

- Tested base：`5fab0ba` + 本回传提交差异，branch `chatgpt/three-scene-software-closeout`。开始时有五个未提交的 Jimu 诊断文件，本轮继续完成它们，不丢弃已有工作。
- GPU 环境：foundationpose310 / Python 3.10 / torch 2.7.1+cu128 / 原 cuRobo1；CPU Python 3.12。Linux，RTX 5060 Ti 8151 MiB，driver 580.173.02。
- Native SIM 沿用既有 ManiSkill/SAPIEN 环境；本轮 cuRobo2、PickPlace、PushT GPU 不重复运行。没有安装/升级库或修改外部模型。
- 所有 GPU 运行串行，MemoryMax=9G、MemorySwapMax=512M、OMP_NUM_THREADS=2、MAX_JOBS=2。完整任务 runner 上限 900 s，没有因观察超时重启任务。
- 版本分别记录：bundle v1 是仅前台、无名义几何的早期诊断；bundle v2 / standard control v1 加入原 pregrasp 标签与名义几何；最终版本分别记录 prefetch / foreground，增加请求诊断的完整性判定。不能将早期运行当作最终字节版本。
- RealMan SDK connection / Robot IP used：NONE。

## 1. 旧仓库 dirty worktree 审计

旧 `Desktop/lerobot` 只读。没有 reset/clean/stash、覆盖、修改、整体复制 untracked 或重新纳入 overlay。沿用 fixed `7aaff9d` + 已审阅 12 个 overlay 的六入口/依赖分类。

完整输入仍是旧 `tag1_standard_three_layer` 的 manifest、builder、fixed scene，以及只读读取七个起始角度的 `JIMU_BUILDER_JSON_EXECUTION.md`。输入文件 hash 前后相同；old status SHA256 为 `15fd1ac40352579a761b4134244c2a74158dd723b2b146c6a39ccf458fe20968`。

完整读取原 86 行启动文档及 manifest builder 选项，未发现被遗漏的屋顶修复启动参数。文档的 live anchor 参数不用于本轮固定输入，也没有执行文档中的任意 shell 文本。三个旧 untracked 任务文件仍只是只读运行输入，不是新批准迁移。

## 2. 迁移完整性

未重新迁移；`migrate_working_sources.py --target-repo . --verify-only` PASS，**807 文件**，source commit `7aaff9da22486b7d25557b3795dd258f9b65f10d`。原受管源码/资产不变，全面 dependency/runtime 资格仍 false。

bundle v2 / v3 实际 worker 的 `builder_scene.json` 与旧原文件经完整 JSON 比较 **相等**；不仅比较件数或中心点。原 21 件（9 locked + 12 movable）、层序、料槽角色和逐层高度配置保留。此证据排除这次入口序列化改写设计，不证明旧输入每个目标本身都可行或现场标定正确。

## 3. 代码完整性与 CPU 测试

- compileall：rm75_app / tools / tests，包含 snapshot，PASS。
- 完整 `tests/three_scene`：**456 passed**，1 warning，20.12 s。
- 完整 `tests`：**778 passed**，1 warning，32.71 s。
- 两个屋顶诊断测试文件共 32 项；相对本轮开始已有的 11 项，新增/扩展 21 项。无 failed/skipped；唯一 warning 是既有 trimesh Scene.dump 弃用提示。

覆盖原返回解/误差一一对应、参数/返回对象保留、原配对顺序、原 pregrasp 标签、前后台独立观测额度、刚体 FK 一致性、几何未资格化拒绝、碰撞状态变化 fail closed、请求诊断缺失/不完整拒绝、聚合分母与原始数据排除。

## 4. 前端

本轮未修改前端。frontend/CLI/request/input/Stop/overlay 离线覆盖随完整 suite 复跑；实际浏览器页面、导入/编辑/round-trip、按钮 Stop：NOT_RUN。

完整 builder 通过实际 WorkcellService → worker → 原 triangle entry 运行；不把 worker 运行当作新浏览器交互或相机验证。

## 5. PickPlace 回归

本轮新 PickPlace GPU / 相机 / benchmark：NOT_RUN。最新结果见 [上一轮 PickPlace / PushT 回传](CODEX_THREE_SCENE_PICKPLACE_PUSHT_20260908.md)：原五场景通过 2/5，附加诊断后 2/6；原七物体 4/7 且退让警告；两个固定统一协调器任务各 10 warm 及 no-op 全链通过。它们互不替代，七物体问题仍未关闭。

## 6. Magnetic / Jimu 回归

### 6.1 实际运行与版本

| 运行 | 原程序终态 | 屋顶诊断资格 |
| --- | --- | --- |
| original_bundle_v1 | 8/12，final=False，worker failed，229.314 s | 8 个前台 batch；原 pregrasp 标签尚未覆盖，无名义目标几何 |
| original_bundle_v2 | 8/12，final=False，worker failed，240.523 s | 12 个前台 batch 全部完整/状态不变，含精确目标几何 |
| standard_roof_control_v1 | 12/12，final=True，283.024 s | **0 个 batch，诊断未观测**；任务成功不算诊断通过 |
| standard_roof_control_v2 | 12/12，final=True，280.537 s | 12 个 prefetch batch 完整，四个 roof × 三阶段齐全，audit PASS |
| original_bundle_v3 | 8/12，final=False，worker failed，228.916 s | 15 个 batch（12 前台、3 预取）全部完整/状态不变，audit PASS；**任务 FAIL** |

每次完整 builder 原程序均保留失败和 source retry；v1/v2/v3 都是 20 个 cycle 标记（8 true + 12 false），失败重试不是新物体。标准 control 是原完整 12 件场景，不是改成两块积木。五次 GPU 运行均已终止，无取消或重启；原完整 builder 三次失败，标准场景两次任务通过。独立 four-wall 本轮 NOT_RUN，既有 4/4 证据保留。

五次运行共记录 47 个诊断 batch / 6206 个目标，已记录 batch 均完整且状态不变；0 个 batch 的早期 control 单独标为未观测，不能通过空集判定。48 次原释放/返回执行门审计通过，clearance 共 2699 点、return 共 8502 点；搬运审计共 2693 点，world exemptions 均为空。这些执行门证据只覆盖实际到达该阶段的原路径，不使未成功放置的四个屋顶变成通过。

### 6.2 新仓库中的只读诊断

`jimu_roof_ik_diagnostics` 安装在原批量查询外，不新增 IK/规划、改候选、改返回结果或提升 success。先复制同一原查询返回的 raw seed/error，再做当前范围内的 native validity、FK、解析球盒接触，最后检查 world / attached / collision model fingerprint。

每个 source × phase 只记录首个 batch；最终版 **prefetch 与 foreground 分开计算额度**。所有原查询、后续候选及重试照常运行。采样范围是首批证据，不宣称穷尽所有后续重试或所有可能抓法。

发现并修正两处诊断覆盖问题：原 pregrasp 刷新标签实际为 `winner_chain_ik_preselect_grasp`；标准成功场景直接复用预取计划，前台没有新 IK。新验证器要求四个 roof 的 pregrasp / grasp / paired_place 证据都出现且状态不变；**不得因 0 条记录而通过**。没有关闭缓存、强制重新规划来改变原运行行为。

### 6.3 near-IK 行误差假设没有解释本次失败

原 single-IK near fallback 存在“debug 第一行误差 vs 最近关节行”的可构造差异，CPU 用原 helper 证明了这个可能性。但 bundle v1/v2 实际记录的批量查询每目标仅返回 **1 行**，逐行 debug / raw 误差一致；本次证据不支持用该假设解释屋顶 paired-IK 全失败。

bundle v2：四个 roof 的 pregrasp 成功分别 14/17、14/17、14/17、10/17；grasp 分别 14/17、12/17、12/17、13/17。成功抓取端点并不代表对应释放目标可行。

这里的原 result.success 经过已有 near-IK collision guard；摘要另列 raw success rows。没有改 near 阈值或把低误差的碰撞解提升成成功。

### 6.4 精确释放目标的必要几何条件

原 paired 调用的顺序通过原源码 AST 核验：`q_grasps + q_grasps` 与 `hover_poses + release_poses`，不是交错排列。每候选必须同时满足自己的 hover / release。

名义几何使用原 `gripper_base_link` 正半径 sphere，依据原 GPU FK 变换到 EE，再放到**完全相同的原 EE 目标**；不使用失败解替代目标。用原 start 和一个原返回配置交叉核对其 EE 刚性，bundle v2 最大偏差约 4.41e−8 m。原半径/偏置未改。

这是一项 box-world 必要条件检查，**不是新的 IK、全机械臂/自碰撞/mesh/连续轨迹或物理几何资格**。实际失败返回配置另用 native GPU validity 检查；不能把解析几何方法称为对所有名义目标的 native GPU 距离查询。

bundle v2 首个前台 paired batch：

| roof | 原候选 / IK 目标数 | result success | hover 底座/墙重叠 | release 底座/墙重叠 | 每个 release 最大重叠的范围 |
| --- | --- | --- | --- | --- | --- |
| right | 192 / 384 | 0/384 | 172/192 | **192/192** | 31.494–40.339 mm |
| back | 176 / 352 | 0/352 | 160/176 | **176/176** | 26.952–40.401 mm |
| left | 176 / 352 | 0/352 | 160/176 | **176/176** | 26.952–40.401 mm |
| front | 144 / 288 | 0/288 | 116/144 | **144/144** | 12.732–40.130 mm |

即：**已观测首批 688 个原配对候选的 release 全部存在该刚体几何冲突**。不是由 seed 没收敛造成的同一精确目标误差；但不外推成所有其他抓法、输入或后续重试全局不可行。

最终 bundle v3 在最终源码版本上复现同样的前台 **688/688** release 重叠；额外观测的 right roof 预取批次为 **192/192**。预取与前台重复来源分别计数，不冒充新增任务物体或全局抓取空间覆盖。原程序已生成 `roof_face_z180` 与 `target_sym_yaw_z_+180` 备选，不能把失败解释为没有尝试过这些原备选。

实际 paired 免检事件只列 `gripper_Left_Support_Link` / `gripper_Right_Support_Link`，不含 gripper_base_link。`_disabled_collision_links=[]` 本身不足以证明没有 world-cost 过滤，因此同时核对真实 filter 事件。解析接触记录是全部球盒几何，不把受豁免 link 的接触误称为 native 阻挡。

### 6.5 标准场景对照

最终 standard control 的四个 roof 均在后台预取时观察：

| roof | 首批 paired 目标 | result success | release 底座/墙重叠 |
| --- | --- | --- | --- |
| front | 320 | 200 | **0/160** |
| right | 288 | 180 | **0/144** |
| back | 288 | 174 | **0/144** |
| left | 288 | 154 | **0/144** |

标准场景 592 个首批 release 目标没有上述底座/墙必要条件冲突，且原全链 12/12。两场景的输入/目标/抓放绑定和候选数并不相同，这是独立正例，不是只改一个变量的因果消融，更不是用标准场景替代完整 builder。

### 6.6 仍未实施的改变

没有删除/缩小夹爪底座、墙体或桌面碰撞体；没有新增底座接触免检、移动屋顶/墙体、翻转任务朝向、改 source pose、加统一延时、开额外 fallback 或减少候选。搬运仍保留全世界/桌面/负载/自碰撞，payload 自碰撞豁免仅 finger pairs；原释放/返回执行门继续运行。

下一步需要核对完整任务的目标位姿/抓放绑定与实际夹爪模型约定，不能直接把“更多 seed”或“更宽碰撞容差”当修复；本文尚未确定是旧任务输入、模型实测还是某个原变换定义应修改。

## 7. PushT GPU / cuRobo2

本轮 NOT_RUN，不改 PushT。上一轮实际 6 次 GPU：完整链 3/6，另有 2 个下降失败和 1 个障碍正确拒绝；3/3 追加障碍、30/30 注入执行前门禁、快/慢速度验证通过。真实推头选择、TCP/contact 映射和现场 profile 仍缺，不因 Jimu 进展自动资格化。

## 8. Camera / tracking

新相机、RRTrack、遮挡恢复、tagless 多实例身份与装配精度：NOT_RUN。只读 frozen inputs 不冒充新定位。原橙色薄片 RRTrack 29/30 记录保留，不能当完整装配证明。

## 9. RealMan no-motion

SDK connection/preflight、真实 joint feedback、控制器 Stop、gripper backend：NOT_RUN。未使用真实机器人地址，没有机械臂/夹爪运动。

## 10. Physical motion ladder

自由空间、夹爪、单/多物体 PickPlace、Jimu 装配、PushT 单推/新观测/多步闭环：全部 NOT_RUN。

## 11. Final summary

完整 Jimu 屋顶仍未通过，但已取得原 batch、同一返回行误差、实际 world-contact scope、精确原目标刚体几何和标准成功场景的对照证据。修复的是诊断覆盖及验证误报风险，不是放宽任务成功。

结果只做本地提交，未 push；此前原始数据上传审核限制仍在，无新许可，不重试或改写历史绕过。原始关节、世界、轨迹和日志保留本地。三条非真机链目标保持未完成。
