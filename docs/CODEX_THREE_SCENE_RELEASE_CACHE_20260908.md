# 三场景无运动后续：释放路径、原任务闭包与 PushT 门禁 — 2026-09-08

**NEEDS_REVIEW，三条链路尚未全部完成；Jimu 不是只差真机。** 本轮实际执行 6 次原 PickPlace/Jimu GPU 运行、6 次 PushT GPU 运行及一次 CPU 资格检查。没有机械臂或夹爪运动，没有 SDK connection，没有相机采集。失败和超时保留在分母。

按 [回传模板](CODEX_THREE_SCENE_RESULTS_TEMPLATE.md) 填写。机器摘要：[release cache summary](../benchmarks/unified_scenarios/three_scene_release_cache_20260908_summary.json)，共 13 条（**其中 1 条是 CPU 资格检查，不是 GPU**）。原始轨迹、固定定位和日志仅保留本地 `runtime_data/three_scene/release_cache_20260908/`，worker 对应 job_id 在各自 result.json 中。此前结果：[native gaps](CODEX_THREE_SCENE_NATIVE_GAPS_20260908.md)。

## 0. 基本信息

- Tested base：`c3a70d3` + 本回传提交差异；机器摘要记录最终 Python 源文件 SHA256。以下注明运行期间仅增加的报告字段，未把缺字段填成实测值。
- Branch：`chatgpt/three-scene-software-closeout`；Linux，RTX 5060 Ti 8151 MiB，driver 580.173.02。
- CPU Python 3.12；PickPlace/Jimu foundationpose310、Python 3.10、torch 2.7.1+cu128、原 cuRobo1；PushT curobo2、Python 3.11、torch 2.11+cu128、cuRobo2。
- ManiSkill：沿用原 portable 环境及已迁移模型，未升级依赖。RealMan SDK connection / Robot IP used：NONE。
- GPU 串行，MemoryMax=9G、MemorySwapMax=512M、OMP_NUM_THREADS=2、MAX_JOBS=2；单次上限 600–900 s。未复制旧 ABI 扩展、改外部 cuRobo 或修改 snapshot 源码。

## 1. 旧仓库 dirty worktree 审计

- 旧目录 `/home/zhangzhao/Desktop/lerobot`，HEAD `36798efbd12814841607951c9af470b309b34fd3`；固定基线 `7aaff9da22486b7d25557b3795dd258f9b65f10d`。
- 前后 status SHA256：`15fd1ac40352579a761b4134244c2a74158dd723b2b146c6a39ccf458fe20968`。不声称与重启前历史哈希相同。旧文件修改/覆盖：**NO**；未 reset/clean/stash，未整体复制 untracked。
- 新发现的 `tag1_standard_three_layer/manifest.json`、`builder_scene.json`、`fixed_scene_pose_results.json` 均为 untracked 的直接任务依赖，分类 **candidate_final_fix（配置/固定数据）**。逐文件 SHA256、依赖和理由见 [三文件审计](CODEX_JIMU_TASK_BUNDLE_AUDIT_20260908.md)。
- **三项未纳入 approved overlay / snapshot / Git**，仅只读原文件进行 SIM 重放，前后三文件哈希一致。是否逐文件迁移仍待明确审核，不擅自决定纳入或丢弃。

## 2. 迁移完整性

- 本轮无重新迁移；fixed `7aaff9d` + 原 12 approved overlay 不变，snapshot **807 files**。
- `PYTHONPATH=. python tools/migrate_working_sources.py --target-repo . --verify-only`：PASS，六入口随完整 manifest 校验。
- 原 MIGRATION_MANIFEST 的 `dependency_completeness_verified=false`、`runtime_gpu_verified=false` 不擅自置 true。独立场景规划正例不等于全部入口资格化。

## 3. 代码完整性与 CPU 测试

| 最终检查 | 实际结果 |
| --- | --- |
| compileall：rm75_app、tools、tests，含 snapshot | PASS |
| 完整 tests/three_scene | **301 passed**，1 warning，6.32 s |
| 完整 tests | **623 passed**，1 warning，23.15 s |
| git diff --check | PASS |

测试日志 SHA256 见机器摘要 `checks`；无 skipped，未跳过失败测试。仅有既有 trimesh Scene.dump 弃用 warning。本轮新增 18 项测试，覆盖原反向退让分支 AST 不变/整段校验、失败候选继续原分支、原任务闭包防路径逃逸、Jimu 固定入口、PushT GPU 后执行前门禁及摘要保留部分失败证据。

## 4. 前端

- 本轮三场景 frontend/CLI/overlay/request 权限随完整离线 suite 复跑；**实际浏览器点击 NOT_RUN**，未改前端代码。
- PickPlace：实际 WorkcellService → native worker → 原 fixed-world entry 复跑；页面/preview/polling 沿用前轮证据，不冒充本轮浏览器结果。
- Magnetic：实际 workcell 导入完整 21 件 design 并启动原 builder；浏览器导入/未知字段 round-trip/preview/input prompt/Stop 本轮 NOT_RUN。
- PushT：此前 browser CPU loop/preview/polling/Stop 证据保留。本轮 GPU 使用独立无运动入口，不把 CPU SIM 写成 GPU/物理回归。
- 本轮未启用 allow-real；原 prompt bridge、控制器 Stop 不以本轮离线单测冒充现场验证。

## 5. PickPlace 回归

### 释放后反向路径复用修复

前轮网球失败时，原程序仅验证 reverse-clearance **终点**，随后直接复用整段；执行前独立审计拒绝中间新增接触。本轮只在这一已审阅 AST 分支条件中添加整段资格检查，原 true/else 分支体、候选和 solver 调用不变：

- 若整段不合格，进入原已存在的 fresh-clearance 候选分支；不是新增 fallback 或跳过失败。
- 原 released-candidate 选择也使用同一安全检查；预释放原逻辑不替换。
- 仅 SIM、已释放、已知 source cuboid 可用原 finger ↔ just-released target 规则，关节加密 .01 rad、相对起点 penetration 增量上限 +1 µm 保留。所有其他物体/桌面/连杆及 self checks 不豁免；未知碰撞状态继续 fail closed。
- 单测实际覆盖“端点有效但中间碰撞”以及继续下一个原候选。GPU 本轮记录的新增选择审计均为正例；**没有观察到新 reverse hook 的 GPU 负例触发，不声称它造成了本轮某次 replan 或成功率提升**。

| GPU 运行 | 原结果 | 审计 |
| --- | --- | --- |
| `pickplace_workcell_selection_v1`，原 gluestick_desk_regression | **1/1、final=True**，14.265 s，worker=command_completed_unverified | 1 次搬运 / **62 点**、6 payload spheres；5 clearance 点 |
| `pickplace_table_selection_v1`，原 current_table_all 七物体和原重试 | **4/7、9 次 cycle 尝试，final=False**，exit 42 | 6 次搬运 / **386 点**，payload 1/5/6；3 次 clearance 为 5/12/14 点 |

第二项保留 4 次 true、5 次 false。原物体尝试顺序为笔、绿木块、刷子、胶棒、胡萝卜、薯片罐、网球、胶棒、薯片罐；重试不算新的物体。刷子/网球退让 warning 共 2 条；目标局部路径分别出现 penetration **4.260131 → 4.322036 mm**、**.728871 → .775354 mm**，均被拒绝。胶棒/薯片罐原规划失败未消除。原程序这次打印了 final=False，不同于前轮无 final 的中止；两次都未全链通过。

两个运行 `loaded_mplib_modules=[]`，搬运 `world_exempt_links=[]`。单胶棒运行后才增加 legacy 的 selection/独立审计观测字段，故该原 result 未记录新字段；已记录的 clearance 5 点是实际证据，不补造新 selection 数据。

`lazy_place + primary_only`、原候选/seed/物体几何/成功阈值不变。原 planning_profile rows 留本地；受控 10× warm benchmark / 性能提升 **NOT_MEASURED**。同场景 RRTrack/SAM6D → PickPlace 完整工作台定位链 **NOT_RUN**；本轮是 fixed-world planning/preview，实物 task_success=null。

## 6. Magnetic / Jimu 回归

修复独立验证入口仍使用旧 planner 加载方式的问题：现在经 source_adapter + 原 cuRobo 绑定，并安装与工作台相同的搬运、grasp-contact IK 和 near-IK 保护；按四墙 → 默认屋顶 → 完整 builder → 原完整任务闭包顺序执行。

| GPU 运行 / 输入 | 原结果 | 搬运审计 |
| --- | --- | --- |
| `jimu_four_wall_current_v1`，原默认固定 anchors | **4/4、final=True**，55.433 s | 4 次 / 177 点 |
| `jimu_triangle_current_v1`，原默认 triangle 场景 | **12/12、final=True**，271.452 s | 16 次 / 740 点 |
| `jimu_builder_current_v1`，21 件 builder + 通用 synthetic anchors | **8/12、final=False**，273.648 s，worker=failed | 8 次 / 353 点；2029 次 near promotion 被拒 |
| `jimu_original_bundle_v1`，原 manifest + 原 builder + 原 fixed scene | **5/12 后 900.218 s 超时取消，无 final**，9 次 cycle 标记（5 true / 4 false） | 50 次 / 2586 点；含被预释放返回检查拒绝的候选，**不是 50 次已执行搬运** |

默认 triangle 的 12/12 **不代替**完整 21 件 builder（9 固定 + 12 移动）回归。前三项终态模块 census 均为 `loaded_mplib_modules=[]`；原任务超时被 SIGTERM 停止，没有最终 census，记 **NOT_RECORDED**，不补成 []。

原完整任务 manifest 已实际生效：right → back → left → front，明确料位 role order、triangle slots `[4,5,6,12]`、layer_z_extra_m `[.005,.004,.004]`、final-contact low hover `.01 m`。此前只传 builder 会使用 fallback 料位；相对默认场景，首个 front roof 源 y 相差约 **149.875 mm**。恢复完整配置并未让任务全过：第六步及同类重试中，原程序反复报 **no post-place clearance candidate can also return to cycle start**，在抓取/释放前拒绝，不跳过返回要求。超时运行不是完整失败穷尽或性能基准。

原角色依赖、source retry、partial/full-open + retreat 与候选/seed/目标几何保留；已批准非搬运接触仍属兼容策略，搬运检查全部 world links、table、自碰撞与 payload，payload 自身接触仅 finger-local。未放行 near-IK 的 arm/table 碰撞。

### 独立退让审计覆盖缺口

原 portable 的 `_install_dry_run_motion_window_wrappers_jimu` 明确跳过 direct dry-run wrapper，以保留 partial-open 状态；因此公共 PickPlace 的独立 clearance **执行**审计未成为 Jimu 最外层执行门。已完成 Jimu 原 result 的 `clearance_path_audits=[]`，不能由原 release/return 日志推定这一独立审计已通过。现在增加显式 `independent_clearance_execution_audit_observed`；four-wall/默认 triangle 实测后才加此报告字段，原结果缺字段仍保留，完整 builder 实测为 false。超时 job 无最终审计列表，仅使用已落盘事件。

本轮没有盲套 PickPlace 的 penetration 规则覆盖 Jimu partial-open/接触语义，也没有将兼容 4/4、12/12 写成全严格或实物资格。原 fixed scene 来自历史 AprilTag，是旧工作离线回归，**不是当前 RRTrack 切回 AprilTag**。

## 7. PushT GPU / cuRobo2

真实 cuRobo2 GPU、原人工 `low_table` fixture，不是硬件参数。原 10 个 push 候选、3 friction scales、32 返回 IK seeds、5 mm Cartesian 采样、.02 rad joint edge、3 mm corridor、.05 rad 姿态、.25 rad/s 和 .5 rad/s² 限制保留；本轮不改生产规划代码。

| 本轮 GPU 运行 | 完整链 | 关键实际结果 |
| --- | --- | --- |
| `pusht_execution_gates_v1`，orthogonal_tool，15 mm/s | PASS | 2309 点，76.800 s；最大 TCP 14.035721 mm/s |
| `pusht_slow_gates_v1`，同姿态，5 mm/s | PASS | 6043 点，201.267 s；最大 TCP 4.687830 mm/s |
| `pusht_yaw_matched_tool_v1`，工具随 T 朝向旋转 | PASS | 2164 点，71.967 s；最大 TCP 14.023798 mm/s |
| `pusht_baseline_v1`，原另一工具朝向 | FAIL | approach 1324 点通过；descend 10/16 无成功 IK variant |
| `pusht_rotated_orthogonal_v1`，T 旋转而工具固定 | FAIL | approach 1481 点通过；descend 11/16 无成功 IK variant |
| `pusht_orthogonal_neighbor_blocked_v1`，预设阻挡 | REJECTED，非完整链成功 | approach planning 失败；未到完整链后障碍审计 |

完整五段 approach → descend → contact → push → retreat **3/6 次运行通过**（正常输入 3/5，含一个慢速重复，不是三个独立场景）。此前失败/重复分母保留。两个失败下降的最后原 IK batch 均为 4×32 个 returned rows 全 false，伴随位置残差和部分 self collision；不是仅末端 corridor 被拒，也不凭失败 seed 诊断断言全局不可达。

三条完整链最大 corridor 偏差不超过 **.016981 mm**，最大姿态误差不超过 **2.747057e-5 rad**；原阈值未放宽。最大 joint speed / acceleration 分别：快 `.128698 / .494959`、慢 `.063884 / .449213`、yaw-matched `.135285 / .490735`。每段原样本及时间重采样后全点检查；完整路径后再注入无关障碍，**3/3 正确拒绝**。不把预设阻挡 IK_FAIL 当作这三项完整路径碰撞负例。

### 新增完整 GPU 规划后的执行前门禁

对上述 3 条真实 GPU prepared chains，各自复用该已审计链，调用生产 `execute_push` 的观测/起点检查；复制 executor、注入测试 observer 和拒绝所有 execute 的 arm sink，不连接 SDK、不改生产 profile、不重新赋时真实帧。

每条 10 项：stale、replayed、非递增 timestamp、capture session 重启、低 confidence、错误 frame、simulation source、4 mm 平移漂移、.05 rad yaw 漂移、关节起点变化。**30/30 在任何 execute 调用前正确拒绝，execute=0**。还补单测防止把无关异常或 arm.execute 抛错当成门禁通过。明确是 **injected observation / reused GPU plan**，不是相机的真实 re-observe 或移动物体实验。

`tools/check_pusht_chain.py --profile runtime_data/three_scene/machine.json`：exit 2，qualification_incomplete，10 个 motion/observer 字段及 observation/joints/goal 未提供；integration_qualified=false、hardware_reviewed=false。原配置仍为未填充 bring-up observer，不能据其字段认为现有 Tag 标定有效，也未伪填 RRTrack/T 工具参数。

## 8. Camera / tracking

本轮新采集、RRTrack 推理、fresh timestamp 在线测试、真实遮挡/恢复、observe → push → fresh observe：全部 **NOT_RUN**。前轮橙色单薄片 29/30 accepted、坏深度拒绝→恢复证据保留，但不是多实例身份或装配精度证明。固定历史 AprilTag 输入不充当本轮 tagless 证据。

## 9. RealMan no-motion

真实 SDK connection/preflight、joint feedback、Stop API、gripper backend：**NOT_RUN**。没有机械臂或夹爪运动。NoMotionArm 与 worker cancel 都不等于真实控制器 Stop 验收。

## 10. Physical motion ladder

低速自由空间、单夹爪、单/多物体 PickPlace、单件/2/4/6+ Jimu、一次短推、推后新观测、多步真实 PushT：全部 **NOT_RUN**。

## 11. Final summary

- PickPlace：单胶棒 workcell GPU 正例通过；整段反向退让复用保护已补；七物体仍 4/7、final=False，同场景感知链未资格化。
- Jimu：默认四墙/屋顶规划恢复；完整 builder 和原三文件工作任务均未通过；新增任务闭包待逐文件审核，独立执行退让门和 tagless 装配精度仍缺证据。
- PushT：本轮 6 次真实 GPU，完整链 3 次通过，快慢对照、3 个碰撞负例和 30 项执行前注入门禁通过；两个原姿态仍失败，真实工具/场景/观测 profile 未确认。
- 改动限于新仓库 PickPlace 源适配和释放选择检查、legacy 报告字段、native/PushT 验证及摘要工具、单测、审计/回传文档；旧仓库/snapshot/外部规划库未改。
- 本轮本地提交见本报告所在 Git commit；**未 push**。此前未推送父提交中的原始日志/轨迹、内部路径和硬件标识上传被环境审核拒绝，尚未得到该上传内容的补充许可；不重试、不改写历史绕过。原始数据仍本地保留。
- 三条非真机链尚未全部关闭，不宣称可以直接开始机械臂运动。
