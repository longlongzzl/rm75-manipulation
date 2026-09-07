# 三场景续报：PickPlace 抬升失败现场证据 — 2026-09-08

**NEEDS_REVIEW；三条非真机链尚未全部关闭，Jimu 不是只差真机。** 本轮完成 3 次原 PickPlace cuRobo GPU/SIM 运行：单胶棒正例通过；两次原七物体均为 native 4/7、final=False。确认两个物体的首个失败抬升目标与现有底座碰撞模型冲突，不修改目标、几何或豁免来通过。

按 [回传模板](CODEX_THREE_SCENE_RESULTS_TEMPLATE.md) 填写。机器摘要：[pickplace lift summary](../benchmarks/unified_scenarios/three_scene_pickplace_lift_20260908_summary.json)。逐返回解、完整日志和轨迹仅保留本地 `runtime_data/three_scene/pickplace_lift_20260908/`。

## 0. 基本信息

- Tested base：`83940552b72ca169dfd30f28376500cead1e871e` + 本回传提交差异；branch `chatgpt/three-scene-software-closeout`。
- 最终 Python 文件 SHA256 见机器摘要，适用于 `table_lift_v2`、`legacy_gluestick_v1` 和最终 CPU suite。`table_lift_v1` 是本轮较早版本，不冒充最终字节版本：尚无名义目标几何检查和 elapsed 字段，目标 Pose 通用序列化得到空属性，未据此使用或回填其名义目标。
- CPU：Python 3.12/Linux；GPU：foundationpose310 Python 3.10、torch 2.7.1+cu128、原 cuRobo1，RTX 5060 Ti 8151 MiB，driver 580.173.02。ManiSkill/SAPIEN 和模型沿用原环境，没有升级。
- PushT 的 cuRobo2 本轮 NOT_RUN；其前轮实际 GPU 结果见第 7 节。
- GPU 串行：MemoryMax=9G、MemorySwapMax=512M、OMP_NUM_THREADS=2、MAX_JOBS=2，每次 timeout 900 s；三次均自然结束，未超时取消。
- RealMan SDK connection / Robot IP used：NONE；execute_real=false，无机械臂、夹爪动作或相机采集。

## 1. 旧仓库 dirty worktree 审计

- 旧目录 `Desktop/lerobot`；HEAD 沿用只读审计 `36798efbd12814841607951c9af470b309b34fd3`，本轮未迁移新文件。
- 本轮前后 status SHA256 均为 `15fd1ac40352579a761b4134244c2a74158dd723b2b146c6a39ccf458fe20968`。
- 六入口/依赖分类沿用已审阅 fixed `7aaff9d` + 12 approved overlay；[三个 untracked Jimu 任务依赖](CODEX_JIMU_TASK_BUNDLE_AUDIT_20260908.md) 仍待逐文件审核，未擅自纳入。
- 旧目录修改、覆盖、reset、clean、stash：NO。新 snapshot 的 ignored 运行/失败渲染输出仅留本地，不声称 native 没有写运行产物。

## 2. 迁移完整性

- 无重新迁移；manifest 为 `rm75_app/_vendor/working_snapshot/MIGRATION_MANIFEST.json`。
- `PYTHONPATH=. python tools/migrate_working_sources.py --target-repo . --verify-only`：exit 0，**807 文件**通过，包含六入口。
- source commit 仍为 `7aaff9da22486b7d25557b3795dd258f9b65f10d`；dependency/runtime 资格标志仍 false。
- 未改 snapshot 源码、外部 cuRobo 库或安装模型。

## 3. 代码完整性与 CPU 测试

| 最终检查 | 实际结果 |
| --- | --- |
| compileall：rm75_app、tools、tests，含 snapshot | PASS，exit 0 |
| 完整 tests/three_scene | **382 passed**，1 warning，11.89 s |
| 完整 tests | **704 passed**，1 warning，28.80 s |
| snapshot verify / git diff --check | PASS |

无 failed/skipped；唯一 warning 是既有 trimesh Scene.dump 弃用提示。新增 19 项测试覆盖：原结果对象/seed 参数保留、失败查询现场 world、每源有界观测、SIM/挂载/阶段筛选、异常传播与上下文恢复、同一 seed 对齐误差、非有限返回解保留、碰撞状态变动 fail closed、名义负载/底座几何以及摘要脱敏。

## 4. 前端

- PickPlace/Magnetic/PushT frontend、CLI、overlay、request、Stop/input bridge 的既有测试随完整 suite 复跑。
- 本轮实际浏览器页面、preview/polling/Stop、builder 导入/round-trip：NOT_RUN；未修改前端。
- 不把这些离线测试称为真实控制器 Stop 或新浏览器操作证据。

## 5. PickPlace 回归

### 本轮改动范围

增加 opt-in `--audit-lift-ik` 和新仓库 `pickplace_lift_diagnostics`，只在原 `_plan_short_linear_segment_via_goal_ik` 内、原 IK 返回失败时采集证据：

- 每个 source 只观察首个仍挂载负载的 `joint_start_*lift*` 失败；所有原查询、64 个返回解和后续重试照常执行。不是减少规划候选。
- 在同一原 GPU 锁内记录当时 world、attached spheres、各 rollout 碰撞开关与模型 SHA；先复制原始解/各自误差，再做 FK 和现有碰撞查询。最后验证状态未变；不卸载负载，不清空 world，不做碰撞 ablation，不把失败改成成功。
- 每个返回解的误差与同一行配置对应，不使用原 debug 中仅第一行的标量替代所有解。
- 名义目标核验：将起点的原刚体 payload spheres 按原 EE 目标变换，与固定 base_link 原 spheres、原 self buffers/ignore pairs 比较；起点与一个原返回配置的底座球坐标差为 0。它不是新 IK 或新路径，也不认证实际模型精度。
- 这一采样是诊断，不是执行门。抬升沿用已批准非搬运 world-contact 兼容范围；此处 native validity 是在原范围内观察，不能声称全 world 未过滤。解析球盒接触另行列出未过滤几何；mesh 笔筒没有被球盒解析法覆盖。**自碰撞未豁免底座，仅保留原邻接规则和已批准 payload/finger 接触。**

### 实际 GPU 运行

三次均用 `tools/run_native_pickplace.py`，传入原扩展目录、`--transport-world-checked --audit-clearance --audit-lift-ik`；case 为原 `current_table_all`（两次）或 `legacy_gluestick`（一次）。完整参数留在本地 result.json。

| run | 原 cycle 结果 / 完整终态 | 搬运审计 | 释放执行审计 |
| --- | --- | --- | --- |
| table_lift_v1 | **4/7**，9 次尝试，final=False，exit 42；elapsed 未记录 | 6 次 / **373 点** | 3 次，5/12/3 点；2 条 clearance warning |
| table_lift_v2 | **4/7**，9 次尝试，final=False，exit 42；87.637 s | 6 次 / **388 点** | 3 次，5/12/14 点；2 条 clearance warning |
| legacy_gluestick_v1 | **1/1**，final=True，exit 0；13.058 s | 1 次 / **61 点** | 5 点通过，warning=0，失败抬升诊断=0 |

两次七物体都保留 4 true / 5 false 原 cycle 标记，重试不是新物体。**4/7 是原程序 cycle 数，不是 4 个严格完整抓放通过**：刷子仍打印释放退让 warning，网球亦有 warning；两次独立 clearance 执行审计都只有 3 次。v1 出现原 reverse-reuse/candidate 路径新增 28.687/15.457 µm 接触而被拒；v2 没有相同 selection 负例，不能以 v1 路径解释 v2 每条 warning。

三次 `loaded_mplib_modules=[]`，所有搬运 `world_exempt_links=[]`；搬运 payload/table/world/self 检查保留。单胶棒 runner `strict_clearance_success=true` 仅代表该字段规定的规划/退让检查，不等于全阶段无接触豁免或实物成功。

### 抬升失败定位

v1/v2 各记录胶棒、薯片罐两次首个失败查询，**4/4 状态未变**，每个保留原 64 个返回解；四次起点均 valid，负载均为原 6 spheres。v2：

| 原目标 / source | 原返回解 | 最小位置误差解 | 名义原抬升目标的 payload ↔ base_link 重叠 |
| --- | --- | --- | --- |
| gluestick，top_bias_center 抓取后原 8 cm 抬升 | 0/64 IK success；63 self collision / 1 world collision | 3.975 µm 位置误差，仍与底座重叠 **7.813897 mm** | **7.817607 mm**，1 对球，不在 ignore pair |
| hongshupian，首个原抓取后的 8 cm 抬升 | 0/64 IK success；63 self collision / 1 world collision | 5.623 µm 位置误差，仍与底座最大重叠 **28.968629 mm** | **28.974598 mm** 和 8.170718 mm，2 对球，不在 ignore pair |

v1 胶棒同为 63 self / 1 world；薯片罐为 64 self。不能把两次随机/后台求解的样本差异写成性能或成功率改善。

结论限于**当前模型和这两个原固定目标**：名义目标自身就有 payload/base 几何冲突，不只是失败 seed 的姿态不好；只更换 seed 无法消除精确同一目标的这对刚体冲突。不是证明所有抓法/场景都无解，也不是证明真机一定发生相同毫米数碰撞。未删底座球、恢复旧 payload/base 免检、移动物体、重设抓取/抬升目标或开启原已禁用的 fallback。

`lazy_place + primary_only`、所有候选/seed、成功阈值、原 fallback 和几何不变；新增诊断有额外开销，10× warm 性能基准 NOT_MEASURED。实时定位 → 原七物体全链 NOT_RUN；physical_success=null。

## 6. Magnetic / Jimu 回归

本轮 Jimu GPU NOT_RUN，未改 Jimu 逻辑。沿用 [已完成 GPU 及分段执行门](CODEX_THREE_SCENE_RELEASE_GATE_20260908.md)：默认四墙 **4/4**、默认屋顶 **12/12**；完整 21 件任务（9 固定 + 12 移动）仍 **8/12、final=False**。返回段不免检，24 次已观测释放/返回 gate 通过，不覆盖所有未运行分支。

因此 Jimu 仍有完整任务屋顶规划、未覆盖执行分支和 tagless 装配精度证据缺口，不是“只差真机”。未把旧 untracked 配置未经审核迁移，也未改 RRTrack 回 AprilTag。

## 7. PushT GPU / cuRobo2

本轮未重复 PushT GPU、不改规划逻辑。已实测结果可由两份报告直接复查：

- [六次真实 GPU 及执行前门禁](CODEX_THREE_SCENE_RELEASE_CACHE_20260908.md)：完整五段链 3/6 次运行通过；含快/慢同姿态和 yaw-matched，完整正常输入为 3/5，另一个为预设障碍拒绝。完整链后的无关障碍 3/3 拒绝，注入观测执行前门禁 30/30 拒绝，execute=0。
- [实际夹爪几何 GPU 交叉核验](CODEX_THREE_SCENE_PUSHT_ENVELOPE_20260908.md)：四种原姿态的 GPU/解析工具接触掩码一致；两个原失败下降分别在 10/16、11/16 首次发生夹爪—T 接触。不是新增四条完整链通过。

当前 5 mm 圆推头接触模型与闭合夹爪模拟几何不一致；最终用闭合夹爪还是独立推头及真实 TCP/contact 映射仍未确认。没有猜测工具选型、改 contact offset/姿态或通过提前接触来满足原短推成功条件。

## 8. Camera / tracking

新相机采集、RRTrack 推理、真实遮挡/恢复、同场景 observe → plan → fresh observe：本轮 NOT_RUN。既有橙色单薄片 29/30 + 坏深度拒绝/恢复不是多实例和装配精度证明，不改写为已完成。

## 9. RealMan no-motion

SDK connection/preflight、实际 joint feedback、真实 Stop、gripper backend：NOT_RUN。本轮只有 GPU/SIM 和离线测试；没有控制器连接。

## 10. Physical motion ladder

机械臂自由空间、夹爪、真实 PickPlace/Jimu/PushT：全部 NOT_RUN。未增加运动授权。

## 11. Final summary

- PickPlace：单胶棒正例保留；原七物体仍未通过。本轮把两个失败从笼统 IK_FAIL 定位到仍挂载负载时的底座自碰撞，并用原名义目标几何交叉核验，未降低碰撞保护。
- Jimu：默认场景恢复，完整任务仍 8/12；PushT：已有真实 GPU 完整链和负例，但工具几何/接触映射待确认。三条链不宣称全面关闭。
- 全量离线 **704 passed**；三场景 **382 passed**；807 文件 snapshot verify PASS。
- 本轮修改限于新仓库 opt-in 诊断、runner 计时/输出、摘要与单测、回传文档；不修改旧仓库/snapshot/模型。
- **仅本地提交，未 push。** 之前上传未推送父提交中的原始日志/轨迹、内部路径和硬件标识被环境审核拒绝，未收到新的内容授权；本轮不重试、不改写历史绕过。原始结果继续本地保留。
