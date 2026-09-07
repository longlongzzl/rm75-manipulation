# 三场景无运动后续：Jimu 分段释放执行门 — 2026-09-08

**NEEDS_REVIEW，三条非真机链未全部完成。** 在 [上轮释放状态回传](CODEX_THREE_SCENE_RETURN_STATE_20260908.md) 的正确夹爪模型观测上，新增原融合退让/返回路径的独立 SIM 执行前检查。没有机械臂或夹爪运动，没有 SDK connection，没有相机采集。PickPlace/PushT 本轮生产算法未改，其历史失败不删除。

按 [回传模板](CODEX_THREE_SCENE_RESULTS_TEMPLATE.md) 填写。机器摘要：[三次 GPU gate summary](../benchmarks/unified_scenarios/three_scene_jimu_gate_20260908_summary.json)；原始数据只在本地 `runtime_data/three_scene/jimu_gate_20260908/` 和各 result.json 关联的 worker job 中，不上传原始轨迹或几何。

## 0. 基本信息

- Tested base：`6b308a574ded1cb0e934ec00472ecd0d458dd503` + 本回传提交差异；最终 Python SHA256 见机器摘要。
- Branch：`chatgpt/three-scene-software-closeout`；Linux；RTX 5060 Ti 8151 MiB，driver 580.173.02。
- CPU Python 3.12；本轮 GPU：foundationpose310 / Python 3.10 / torch 2.7.1+cu128 / 原 cuRobo1。cuRobo2 环境未改。
- ManiSkill：沿用原 portable 依赖和已迁移模型；RealMan SDK connection / Robot IP used：NONE。
- GPU 串行，MemoryMax=9G、MemorySwapMax=512M、OMP_NUM_THREADS=2、MAX_JOBS=2；SIM fixed inputs，不使用相机 fallback，不带 execute-real。

## 1. 旧仓库 dirty worktree 审计

- 旧目录 `/home/zhangzhao/Desktop/lerobot`，HEAD `36798efbd12814841607951c9af470b309b34fd3`，固定基线 `7aaff9da22486b7d25557b3795dd258f9b65f10d`。
- 本轮前后 status SHA256：`15fd1ac40352579a761b4134244c2a74158dd723b2b146c6a39ccf458fe20968`。旧文件修改/覆盖 **NO**，未 reset/clean/stash/整体复制 untracked。
- 无新增待迁移文件；原任务三文件与 documented-start 文档沿用 [既有只读分类](CODEX_THREE_SCENE_RETURN_STATE_20260908.md#1-旧仓库-dirty-worktree-审计)，仍未自动纳入 overlay/snapshot/Git。
- 原文档 `[45,0,0,-90,0,-90,60]°` 只用于完整任务的独立 SIM comparison，不修改原程序默认启动姿态。

## 2. 迁移完整性

- 未重新迁移；fixed `7aaff9d` + 12 approved overlay、**807 files** 不变。
- `PYTHONPATH=. python tools/migrate_working_sources.py --target-repo . --verify-only`：PASS，包含六入口 manifest 校验；无 denied RL/policy 树恢复。
- 不修改 snapshot、安装的 cuRobo 或模型；manifest 的 dependency/runtime 全面资格标志仍不置 true。

## 3. 代码完整性与 CPU 测试

| 最终检查 | 实际结果 |
| --- | --- |
| compileall：rm75_app、tools、tests，含 snapshot | PASS |
| 完整 tests/three_scene | **354 passed**，1 warning，11.74 s |
| 完整 tests | **676 passed**，1 warning，28.95 s |
| git diff --check | PASS |

相对上一轮新增 24 项测试，无 skipped；仅既有 trimesh 弃用 warning。GPU 前已完成 compileall 和完整 three_scene，再执行 GPU；完整 CPU suite 在 GPU 串行运行期间复跑。

测试覆盖原 native concat 输出身份/数值/重复点语义、原 join edge、同一对象被修改拒绝、拷贝/跨线程/已消费/过期记录拒绝，以及 table/neighbor/self/arm-target/palm-target/return-target 六类负例不进入执行。还覆盖模型同步、released/source/table/binding 资格缺失及真实执行/prefetch/其他阶段不继承接触例外。

## 4. 前端

PickPlace/Magnetic/PushT frontend、CLI、overlay、权限、nonce/input/Stop 合约随完整离线 suite 复跑；本轮实际浏览器 page load/preview/polling/input/Stop **NOT_RUN**，未改前端，不把 worker 的 GPU 结果当成浏览器或控制器资格验证。

## 5. PickPlace 回归

本轮新 GPU/相机/10× warm benchmark **NOT_RUN / NOT_MEASURED**。原 `lazy_place + primary_only`、候选/seeds/几何/阈值不变。此前单胶棒实际 workcell 1/1、七物体 4/7（9 次原尝试、final=False）保留；本轮 Jimu gate 不安装到 PickPlace，不把独立 Jimu 结果算成 PickPlace 修复。

## 6. Magnetic / Jimu 回归

### 6.1 分段依据与执行顺序

只包裹原 `_concat_joint_paths`：调用原函数一次，返回完全相同的 list 对象，记录最多 16 个 thread-local 路径身份及原 clearance 前缀。重复点只为边界元数据按原规则识别，**不重写/平滑/截断/重定向返回路径**。执行时要求同一对象、全部值及前缀一致，记录一次消费；边界不明时 fail closed，不靠阶段名字猜测接触结束位置。

实际调用顺序：原 Jimu partial-open → 上轮模型同步/只读观测 → 新 SIM gate → 原执行函数。融合路径拆为原 clearance 前缀与 return 后缀；共享边界点和 join edge 必须进入无豁免返回检查。只支持已审阅的 `post_place_clearance` 和 `post_place_clearance_return_to_cycle_start`，未知 clearance label 拒绝。

### 6.2 接触策略没有扩展到返回

先验证 released SIM、native binding、模型开度与模拟夹爪精确一致、payload detached、table/source 存在、world/self 开启、无 disabled links 或其他 world masks。两段均按最大 .01 rad 加密检查。

clearance 全 world check 已 valid 的样本不加任何例外；若 invalid，必须同时通过以下两个 GPU 检查：

| 检查 | fingers ↔ released target | fingers ↔ table/其他物体 | arm/palm ↔ target | self |
| --- | --- | --- | --- | --- |
| 原目标单物体暂时排除，所有连杆检查其他 world | 由下一检查限定 | 保持检查 | 由下一检查覆盖 | 保持原始输入 |
| 完整 world，仅 world cost 输入过滤左右 fingers | 允许释放接触 | 由上一检查覆盖 | 保持检查 | 保持原始输入 |

两项取交集，仅获得原先批准的 finger ↔ just-released target 成对例外，不免检桌面、邻物、手臂或掌部，不改共享碰撞球/self 输入。实际过滤调用为零则不能资格化。所有作用域退出后恢复原 world/cost state。

**return 段不使用任何接触豁免，包括刚释放目标也完整检查。** 任一不合格样本、缺失资格或状态污染都抛出不可被原宽泛 `except Exception` 重试吞掉的类型，在原执行调用之前停止。未新增 solver/fallback、未减少候选，未套用 PickPlace 的 +1 µm penetration 规则覆盖原 Jimu 释放接触语义。

### 6.3 实际 GPU 结果

| 运行 | 原 native 结果 | 新执行门 |
| --- | --- | --- |
| `jimu_four_wall_gate_v1`，原默认四墙固定输入 | **4/4、final=True**，59.284 s | **4/4 PASS**，clearance 219 + return 618 个样本；无接触例外触发 |
| `jimu_original_bundle_gate_v1`，原三文件任务 + documented start | **8/12、final=False**，224.160 s，exit 42 | 已完成八件 **8/8 PASS**，clearance 444 + return 1430；仅首件 2 个接触样本触发成对 GPU 检查 |
| `jimu_triangle_gate_v1`，原默认 triangle-roof 固定输入 | **12/12、final=True**，268.285 s | **12/12 PASS**，clearance 655 + return 2159；无接触例外触发 |

三项均完整运行到 native final，没有限次诊断取消。运行顺序为默认四墙 → 使用 triangle entry 的完整 builder → 独立默认 triangle-roof。完整 builder 共 20 次 cycle 标记：8 true + 第九件原四种屋顶源的 12 个 false 重试；最终没有完整 grasp/place paired IK relation，失败发生在执行门之前。没有跳过这些候选，也不把 8 个已完成步骤的 gate PASS 写成 8/8 全任务通过。

24 次实际执行前 gate 均 PASS，**clearance 1318 + return 4207** 个分段样本（共享边界点在两段各计一次，不是 5525 个互异状态），均 `return_world_exempt_links=[] / self_collision_input_modified=false / state_unchanged=true`。完整 builder 首件实际 `world_filter_calls=2`，不是只靠 mock 宣称 target-local 分支已运行；两个接触点得到成对检查许可，后续返回没有继承例外。

三次搬运审计分别 **176 / 403 / 737** 点，world exemptions 均为空；不合格 near-IK promotions 分别 **51 / 1427 / 267** 次拒绝。三次 `loaded_mplib_modules=[]`。完整任务原三文件和原启动文档前后 SHA256 一致；全流程没有改变原候选、几何或成功阈值，也不将单次耗时差异称为性能提升。

`jimu_release_execution_audits` / 同名事件是本轮新增门禁证据；原 `independent_clearance_execution_audit_observed` 字段仍专指旧公共 PickPlace wrapper，此 Jimu 运行中为 false。不能只读旧字段认定没有新门，也不能以新门通过替代 native final/物体级成功。

### 6.4 范围和仍未关闭项

- 本轮 gate 覆盖原 Jimu pose 执行边界的 standalone clearance 与 fused clearance+return；**分离的 joint-return、其他渲染/执行分支尚无本轮 GPU 覆盖，不宣称全入口均已资格化**。
- 原 floor/wall/roof 依赖、同类料槽 retry、partial/full-open、角色/料位/层高、候选/seeds、magnetic 原逻辑保留；loaded transport 仍全 world/table/self，payload 接触只 finger-local。
- 完整 21 件 builder、默认 triangle、分离返回、tagless 多实例身份/精度分别记录，不能以默认四墙代替完整装配。

## 7. PushT GPU / cuRobo2

本轮生产算法未改，新 GPU **NOT_RUN**；完整 CPU suite 已复跑。前轮真实 GPU 正常输入 3/5 完整链（含一次慢速重复）、两种姿态下降失败及预设阻挡拒绝保留；三条完整链上的额外邻物 GPU 审计 3/3 拒绝、注入执行前门禁 30/30 拒绝且 execute=0。真实工具/观测 profile 未资格化，不用 authored SIM 参数冒充实测。

## 8. Camera / tracking

本轮新采集、RRTrack 新推理、fresh/re-observe/遮挡恢复全部 **NOT_RUN**。历史橙色单片 29/30 与坏深度拒绝→恢复保留，不等于装配精度。原任务固定 AprilTag 数据只作旧工作离线基准，不是切回 AprilTag。

## 9. RealMan no-motion

SDK connection/preflight、joint feedback、真实 Stop API、gripper backend 全部 **NOT_RUN**。没有机械臂或夹爪运动，不把 GPU SIM 称为真实控制器验证。

## 10. Physical motion ladder

自由空间、夹爪、PickPlace、Jimu、单推、推后新观测、多步 PushT 全部 **NOT_RUN**。

## 11. Final summary

本轮从“正确模型上的释放观测”推进到“原分段路径的实际执行前门”：原四墙 4/4、默认屋顶 12/12，24 次已执行步骤的 gate 通过且返回无豁免；完整 builder 仍 8/12、final=False。全量 CPU 676 passed / three_scene 354 passed；不宣称三条非真机链已全部完成。

本地提交见本报告所属 commit；**未 push**。此前父提交包含原始数据的上传仍被环境审核阻止，没有新的上传许可，不重试或改写历史绕过。
