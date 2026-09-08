# PickPlace / PushT 无运动验证续报 — 2026-09-08

后续范围澄清：[完整冻结世界 / 新 PushT GPU 续报](CODEX_THREE_SCENE_PICKPLACE_AUDIT_20260908.md)。下文四个单胶棒运行虽读取九物体 JSON，起始实际仅加载 desk 及虚拟障碍；`world_exempt_links=[]` 不证明所有输入物体都在碰撞世界。原 2/5 保留，不外推为九物体完整世界通过。payload/base overlap 的原 40 mm self-buffer 分解亦见该续报，不改原失败或容差。

**NEEDS_REVIEW：本轮可运行的固定输入验证已结束，但三条链尚未全部完成，Jimu 也不是只差真机。** 未修改生产规划算法、候选、seeds、碰撞几何或成功阈值；没有连接机器人 SDK、相机，没有机械臂或夹爪运动。

机器摘要：[本轮结果](../benchmarks/unified_scenarios/pickplace_pusht_followup_20260908_summary.json)。原始日志、逐点轨迹、IK 返回解与失败渲染只留本地 `runtime_data/three_scene/pickplace_pusht_followup_20260908/` 及新 snapshot 的 ignored 运行目录。

## 0. 基本信息

- Tested HEAD：`7bd4a348bf0aa48b3625c077566a976b43aa59b1`，branch `chatgpt/three-scene-software-closeout`。
- 开始时已有五个未提交的 Jimu 屋顶诊断相关文件。原样保留，其 SHA256 单独记录在摘要 `preexisting_dirty_files_sha256`；不能将本轮测试描述为 clean HEAD。本轮未跑该诊断的 Jimu GPU。
- 本轮新增的是结果聚合工具、24 项聚合单测、摘要与本文。最终源码哈希见摘要；没有改 PickPlace/PushT 生产逻辑。
- Linux；CPU Python 3.12；native PickPlace 为 foundationpose310 / Python 3.10 / torch 2.7.1+cu128 / cuRobo1；PushT 和统一协调器为 curobo2 / Python 3.11 / torch 2.11+cu128。
- RTX 5060 Ti，8151 MiB，driver 580.173.02。GPU 串行，MemoryMax=9G、MemorySwapMax=512M、OMP_NUM_THREADS=2、MAX_JOBS=2，未升级依赖或改安装模型。
- PickPlace 使用原 native SIM 路径；PushT 为 authored GPU fixture，不是 ManiSkill 推动物理。RealMan SDK connection / Robot IP used：NONE。

## 1. 旧仓库 dirty worktree 审计

旧 `Desktop/lerobot` 保持只读，未 reset/clean/stash/覆盖/迁移文件。前后 status SHA256 均为 `15fd1ac40352579a761b4134244c2a74158dd723b2b146c6a39ccf458fe20968`。

六入口与依赖分类沿用已审阅 fixed `7aaff9d` + 12 approved overlay；没有重新决定纳入任何 dirty/untracked 内容。为寻找旧性能基准输入做过只读文件名搜索，没有发现历史 `/tmp` plans 的原文件；未据此重造输入或声称原 16 cases 已复跑。

## 2. 迁移完整性

未重新迁移。`tools/migrate_working_sources.py --target-repo . --verify-only` 在 GPU 前后均 PASS，**807 文件**，固定 source commit 为 `7aaff9da22486b7d25557b3795dd258f9b65f10d`。六入口/资产按 manifest 验证；dependency/runtime 全面资格标志仍 false。未改旧仓库、snapshot 受管源码或外部 cuRobo/模型。

## 3. 代码完整性与 CPU 测试

最终 compileall PASS；完整 `tests/three_scene` **435 passed**（16.08 s）；完整 `tests` **757 passed**（32.50 s）。结果导出 PASS，snapshot verify PASS；无 failed/skipped，只有既有 trimesh Scene.dump 弃用 warning。

新增 24 项聚合测试覆盖：失败不能移出分母、原 cycle 成功不能覆盖退让警告、完整 GPU 轨迹仍须满足动态/障碍/执行前门禁、拒绝 real 结果、排除原始轨迹/内部路径、保留附加诊断失败、snapshot count 类型。

首次结果导出发现 verify-only 的 `files` 是整数而非数组，导出器报 TypeError；已修复并增加整数/数组/非法值单测。这是后处理错误，不是 GPU 崩溃，不删改任何原测试结果。

## 4. 前端

- 按 browser 技能检查及其故障流程确认没有可连接浏览器；**本轮浏览器页面实操、截图、点击 Stop、builder round-trip：NOT_RUN**。未换用其他浏览器控制方式冒充已完成。
- 实际 loopback workcell server 未使用 `--allow-real`，HTTP info 显示 `allow_real=false`、`real_latched=false`，三场景入口存在；HTML/CSS/JS 实际 HTTP 获取成功。资产哈希/字节数见摘要。
- 实际 CLI 的 PickPlace、Magnetic、PushT preview 全部完成，均为 `command_completed_unverified` / `task_success=null`，不是对象任务成功。
- 实际 CLI PushT SIM：6 次 surrogate push，3 次成功观测确认，`verification=surrogate_pose`，`model_validated_on_robot=false`。不是 GPU 或真实物理验证。
- frontend/CLI/request/原 input bridge/Stop/互斥/real 授权与任意 argv 拒绝的离线覆盖随完整 suite 复跑。本轮实际 native input 提示、真实控制器 Stop：NOT_RUN。

## 5. PickPlace 回归

### 5.1 原 native 固定场景矩阵

使用现有 `tools/run_native_pickplace.py` 的全部五个固定 case，全部传 `--transport-world-checked --audit-clearance`；原扩展缓存、固定场景、候选/seed 和成功规则不变。

| 原 case | native / 终态 | runner 严格退让检查 | elapsed |
| --- | --- | --- | --- |
| legacy_gluestick | 1/1，final=True | PASS | 13.267 s |
| gluestick_jitter_00 | 1/1，final=True | PASS | 13.961 s |
| gluestick_yaw_00 | 0/1，final=False | FAIL | 8.610 s |
| gluestick_swap_00 | 0/1，final=False | FAIL | 12.484 s |
| current_table_all | 4/7，final=False | FAIL，2 条退让 warning | 81.657 s |

固定场景矩阵完整通过 **2/5**。七物体共 9 次原尝试，4 true / 5 false；不是 9 个不同物体，**4/7 也不是 4 个严格完整抓放成功**。七物体仅 3 次 clearance 执行审计 / 20 点，6 次 transport 审计 / 373 点。

所有 native 运行 `loaded_mplib_modules=[]`，搬运审计 `world_exempt_links=[]`，原 attached payload 保留；搬运世界/桌面/自碰撞没有放宽。非搬运接触阶段仍按此前批准的原兼容范围，不将 `strict_clearance_success` 偷换为所有阶段零豁免或物理成功。

### 5.2 两个单物体失败的证据

- yaw：原候选的 pregrasp → grasp 直线验证在 4/20 点失败，位置离线 **18.4 mm > 原 15 mm**；方向误差 3.36° < 原 8°。候选被原门拒绝，未改阈值或开 fallback。
- swap：原第一候选的直线验证先失败；后续候选进入原抬升/退让分支后 IK 失败。另做一次 `--audit-lift-ik` 原输入复跑，**仍 0/1、final=False，13.202 s**。因此本轮 native 总运行数为 **6，完整通过 2**；附加失败不藏在五场景分母外。
- 附加诊断观察首个仍挂载负载的原抬升查询：起点 valid、6 payload spheres、原 64 个返回解均被 native self collision 检查拒绝。最低位置误差约 3.839 µm 的返回解仍有 payload/base 重叠约 10.604 mm。
- 原精确名义目标的刚体 payload/base 几何核验得到 **10.607735 mm、1.052024 mm** 两对重叠，均不在 ignore pair；底座交叉 FK 坐标差为 0，诊断前后状态一致。只证明当前模型下这一个原目标的必要几何条件不满足，不认证物理模型精度，不证明所有抓法都不可行。
- 原七物体胶棒/薯片罐更早的失败现场见 [抬升证据报告](CODEX_THREE_SCENE_PICKPLACE_LIFT_20260908.md)。本轮没有删底座球、改目标/抓法或增加接触豁免来让测试通过。

### 5.3 统一协调器：两个既有 D0 固定任务的 10× warm + 完整规划

与上面的旧 native 程序明确分开。使用仓库现有 `d0_cube_lvmukuai`、`d0_asym_shuazi` 两个 manipulation_plan，不重新生成目标。

`benchmark_grasp_relation_screen.py --repetitions 11`：每任务一次冷调用 + **10 warm**，总 22 行、warm 20/20 找到关系。planner construction **5.493 s** 单列，不混入 warm。

| 固定任务 | warm P50 / P95 | coarse calls / requested rows / padded rows（每次均值） |
| --- | --- | --- |
| lvmukuai | 0.1825 / 0.1831 s | 3 / 101 / 192 |
| shuazi | 0.3186 / 0.3689 s | 2 / 54 / 128 |

套件 warm P50/P95 **0.2500 / 0.3234 s**。另有 pose_tolerance 求解，方块 1 call / 40 requested / 64 padded，刷子 1 / 2 / 64。未把 padding 或其他 solver rows 省略成 coarse 全部工作量。

随后同输入 `--full-chain --repetitions 1`：**2/2**，approach → grasp → lift → preplace → place → retreat 全部分段规划通过，执行器为原 no-op。方块 16 grasp / 9 place、刷子 20 / 2，均无 reverse fallback 或 tool-axis retry；`lazy_place + primary_only` 保持默认。它不是 native SIM 或物理验收。

历史三 smoke 的 `/tmp` plan 和原 16-case JSONL 本机已不存在。本轮**不声称**复现历史三 smoke / 16 frozen，也不与其历史 P95 作提速比较。本轮只新增这两个可追溯固定任务的现版本基准；原 native 候选筛选 10× warm 仍 NOT_MEASURED。

## 6. Magnetic / Jimu 回归

本轮 Jimu GPU、相机定位、真实装配均 NOT_RUN。保留上轮 [独立返回执行门回传](CODEX_THREE_SCENE_INDEPENDENT_RETURN_20260908.md)：标准 four-wall 4/4、标准 triangle-roof 12/12；原完整任务仍 **8/12、四个屋顶失败**。无 Tag 多实例身份/装配精度也尚未资格化，因此不是“只差真机”。

五个预存屋顶诊断文件未丢弃、未纳入本轮结果提交。其离线测试通过不等于新 GPU 证据，更不能断言 near-IK 行选择就是当前屋顶失败原因。

## 7. PushT GPU / cuRobo2

全部使用现有 `run_pusht_gpu_validation.py` 原 `low_table` authored fixtures，带 `--audit-blocker --audit-execution-gates`；无运动 arm sink 不允许 execute。

| GPU run | 完整五段链 | 原拒绝点 / 审计 |
| --- | --- | --- |
| orthogonal_tool，15 mm/s | PASS，2309 点，76.800 s 轨迹 | 最大 TCP 14.036 mm/s |
| orthogonal_tool，5 mm/s | PASS，6043 点，201.267 s 轨迹 | 最大 TCP 4.688 mm/s |
| yaw_matched_tool，15 mm/s | PASS，2164 点，71.967 s 轨迹 | 最大 TCP 14.024 mm/s |
| baseline | FAIL | descend 10/16，0 IK variants |
| rotated_orthogonal | FAIL | descend 11/16，0 IK variants |
| orthogonal_neighbor_blocked | 正确拒绝，非完整链 PASS | approach 无合格规划 |

全部实际 GPU 运行完整链 **3/6**，正常输入 **3/5**，另 1 个预设障碍负例。三条通过链均在任何执行前完成 approach → descend → contact → push → retreat，并通过原逐点碰撞/TCP corridor/姿态和速度、关节速度/加速度检查。

- 三条完整链分别再加入无关障碍：**3/3 被实际 GPU 路径审计拒绝**。
- 每链注入 stale、replayed、非单调、换 session、低置信度、错误 frame、simulation source、平移/yaw drift、joint drift：**30/30 拒绝，execute 调用 0**。这是复用真实 GPU 已规划链的输入注入，不是相机实测或完整执行闭环。
- 原 10 push candidates、3 friction samples、32 IK variants、4-row padding、5 mm Cartesian step、0.02 rad edge、3 mm corridor、0.05 rad 姿态、0.25 rad/s 和 0.5 rad/s² 限值均未改。没有缩减候选或增加目标接触豁免。
- 两个下降失败与 [此前真实 GPU 工具几何诊断](CODEX_THREE_SCENE_PUSHT_ENVELOPE_20260908.md) 的提前接触点一致。本轮不把它们归零为成功，不重新移动 standoff/contact。
- **所有 fixture 的 hardware_profile_qualified=false**。当前 5 mm 圆推头平面模型与模拟闭合夹爪的接触包络不一致；最终用闭合夹爪还是独立推头、相应 TCP/contact 映射仍需确认，不编造实际尺寸。

## 8. Camera / tracking

本轮新 RGB-D、RRTrack、同场景 PickPlace 定位、PushT fresh observe → push → fresh observe：全部 NOT_RUN。原橙色薄片 RRTrack 29/30 记录不是多物体抓放或 T 形物体闭环证明。

## 9. RealMan no-motion

现有示例 profile 通过 `check_pusht_chain.py` 做只读资格检查，按预期 exit 2：缺 10 项规划/定位字段，另缺 observation、joints、goal。未进入 GPU 或 SDK，`motion_authorized=false`。这些是当前示例 profile 的缺项，不能照搬成已选 RRTrack 方案的标定清单。

SDK/preflight、关节反馈、实际 Stop、gripper backend：NOT_RUN。**本轮没有任何机械臂/夹爪运动。**

## 10. Physical motion ladder

低速自由空间、独立夹爪、单/多物体 PickPlace、单件/完整 Jimu、PushT 单推/推后新观测/多步闭环：全部 NOT_RUN。

## 11. Final summary

- PickPlace：五固定场景完整通过 2/5；附加一次失败诊断后总 2/6。七物体全链、释放退让与部分原固定目标几何冲突仍未关闭。
- PushT：6 次实际 GPU，完整链 3/6，保留两种失败和一个正确拒绝负例；3/3 障碍、30/30 注入门禁、快慢速度验证完成。实际工具/接触和定位 profile 待确认。
- Jimu：标准案例通过不代表完整旧任务通过；完整任务仍有四个屋顶缺口。
- 本轮结果不是三条链全部完成或可以直接上真机的许可。没有通过碰撞放宽、减少候选、延时或成功条件修改来收尾。
- 结果只做本地提交，**未 push**。此前未推送父提交的原始日志/轨迹、内部路径和硬件标识上传被环境审核拒绝，无新增许可；不重试、不改写历史绕过。原始数据保留本地。
