# 三工作流迭代 R2（2026-09-11）

日期：2026-09-11  
分支：`chatgpt/three-scene-software-closeout`  
起点 HEAD：`b5935e1bc50bef9b253133705aed9524c218de70`（13a5cb8 之上 + 交接文档）  
审阅依据：`docs/CHATGPT_REVIEW_13A5CB8_WORKFLOW_20260910.md`

全部测试与仿真经 `tools/run_network_isolated.py`；GPU 全部串行（MemoryHigh=6 GiB、MemoryMax=7 GiB、禁 swap、CPUQuota=200%）。真实 LLM 调用在独立进程内（网络放行），preview/sim worker 保持内核隔离。本轮未连接或操作真机与夹爪，未修改 vendor 源码与原底板数据，未降低任何碰撞球、缓冲、阈值或成功条件。

## 0. 固定版本与回归

- 定向新增测试（审阅要求）：`tests/test_pusht_merge_epoch_review.py` + `tests/test_jimu_completion_provider_review.py` → **25 passed**。
- 全量隔离回归（起点）：**1322 passed, 1 warning**（43.94 s），日志 `runtime_data/three_scene/isolation_regression_r2_20260911.log`。
- 运行时关键文件 SHA256 记录于 `runtime_data/three_scene/r2_evidence_20260911.json`（有界证据 JSON，随本文提交）。

## 1. Jimu：实际前端模型入口的同一设计闭环

### 1.1 生产路径实连修正

- `tools/import_jimu_design_library.py` 从未在真实数据上运行过，实连发现两处原生契约不符并修复：`layer_z_extra_m` 列表需序列化为逗号字符串（原生 CLI 为单字符串参数）；`tray_slot_role_order` 改为经 per-job manifest 传递（原实现发出完整 14 项 CLI 参数，会覆盖生成子结构的裁剪）。另补 `sys.path` 引导（与其它 tools 一致）。
- 模板库由两份只读 bundle 建成：`runtime_data/three_scene/llm_jimu_20260910/design_library_r2.json`（grid_3x3_00：9 locked/12 可动；arc_00：7 locked/12 可动）。
- 生成子结构 manifest 裁剪：新增 `rm75_app/magnetic/generation.py::subset_task_manifest`——build_layers/tray 顺序/三角槽索引按选中角色过滤，并经 `tray.slot_layout` 保留原物理槽位；`iteration_workflows.py` 在物化 per-job manifest 时使用。原生边界错误（unknown build_layers role、tray role）因此消除。
- 本机 LLM 实连：`AnthropicJsonClient` 经 sanitized 环境（不修改模块内全局代理；socks:// 环境变量会使 SDK 构造崩溃，见 jimu_design_llm 轮记录）真实调用 DeepSeek Anthropic 兼容端点；模型推理耗时超过原 120 s 墙钟预算 → 按审阅授权将 `timeout_s` 上限提至 600 并配置 300 s、`max_tokens` 32768（审阅上限），每次调用记录实际 usage（input/output tokens）。首次失败（120 s 墙钟）如实保留在 generation 记录中。
- 首份生成的接触策略失败：生成的 SIM profile 未设 `simulation_contact_policy`，magnetic 默认 `strict` 拒绝指垫 link-world toggle；对齐到 19 件模板运行同款 `transport_world_checked_compatibility` 后通过。这是配置对齐，不是豁免放宽。

### 1.2 同一设计证据（生成 → 提交 → 原生）

弧底板结构 1（job `c9696af259524ecca72e5907fa0a9bd3`）：**3/3 搭建周期全部成功**（`native_final_success=true`，`generated_design_used_unchanged=true`）。

| 字段 | 值 |
| --- | --- |
| generation_id | `d2b1dc68573a48779dc3fabede1b0ed8` |
| selected_roles | `square_9, square_10, square_11`（3 可动） |
| locked 底板 digest | `6643a158…`（原 7 块锁定弧形件，新增锁定 0、移除 0） |
| design_digest（生成 proof） | `68e246ba1c673e50bef8a37d83563e71a023565ebb74437cd9e61cdad26b7ae0` |
| 浏览器/请求 design_digest | `68e246ba…`（一致） |
| 作业 builder_scene.json 规范化 digest | `68e246ba…`（一致） |
| native 实际完成角色 | cycle 1–3 = square_9/10/11 全部 success |

网格底板结构 1（job `da4161b2412a4f97845f85e36be54e4b`）：`front_wall` 完成，`front_second_wall`（二层墙）在 `pre_release_post_place_clearance` 端点约束失败 ×2 → 任务失败。**如实失败**：二层墙是更难的真实结构，不是生成/提交链路问题（digest 一致、原生实际消费了生成设计）。

其余结构与运行结果见 §1.4。汇总：本轮通过真实前端生成入口跑完的 **6 个结构全部逐周期成功**（`arc_gen_2` 3/3、`arc_gen_3` 3/3、`arc_gen_4` 2/2、`grid_gen_3` 3/3、`grid_gen_4` 2/2、`grid_gen_1b` 3/3，共 16/16 个搭建周期），`grid_gen_2` 为 1/3（真实结构难度，见 §1.4）。**失败不隐藏**：`arc_gen_1` 的两次边界契约失败、`grid_gen_1` 的过期 proof、`grid_gen_2` 的物理失败都保留在案。

### 1.3 禁止新增 locked 支撑

两种底板所有生成结构的 locked 集合与原模板完全一致（grid 9 块 / arc 7 块，新增 0、移除 0）；可动角色均为原模板父子闭合子结构（`compile_selection` 拒绝未知/固定底座角色与缺失支撑），坐标/类型/变换全部来自原模板编译。

### 1.4 结构矩阵（生成 → 原生结果，随队列补全）

每个结构一行，结构名即生成记录文件名（`runtime_data/three_scene/llm_jimu_20260910/<name>.json`）；列出全部 8 个已生成结构，便于核对分母。

| 结构文件 | board | selected_roles（可动） | 原生结果 |
| --- | --- | --- | --- |
| `arc_gen_1` | arc_00 | square_9, square_10, square_11 | 边界修复前两次失败：job `403893d6`（`unknown build_layers role`）、job `61d99dec`（`tray_slot_role_order` 含未参与角色）——均为**生成子结构裁剪契约**问题，非物理失败 |
| `arc_gen_2` | arc_00 | square_9, square_10, square_11 | 裁剪修复后 job `b8497eb5` 因 `strict_contact_not_supported` 失败（配置对齐）→ 对齐后 job `c9696af2` **3/3 成功** |
| `arc_gen_3` | arc_00 | square_9, square_13, triangle_17 | job `0e9ebb5e` **3/3 成功**（见下方闭环表） |
| `arc_gen_4` | arc_00 | square_8, square_9 | job `a49aa129` **2/2 成功**（见下方闭环表） |
| `grid_gen_1` | grid_3x3_00 | right_wall, right_second_wall, right_roof_triangle | **不能原生提交**：生成于 15:59:20（真实入口，generation `9157213b…`），早于 16:04:20 最终库重建；design digest `8aa0eb42…` 完整一致，但 proof 的 `native_recipe_digest` 已过期 → `validate_generated_request` 拒绝。已用同一真实入口（`IterationAPI.generate` + 原 prompt）重新生成为 `grid_gen_1b`，见下方补充行 |
| `grid_gen_2` | grid_3x3_00 | front_wall, front_second_wall, front_roof_triangle | job `da4161b2` **1/3**（cycle 2 二层墙失败，如实记录） |
| `grid_gen_3` | grid_3x3_00 | right_wall, back_wall, left_wall | job `12e3d576` **3/3 成功**（见下方闭环行） |
| `grid_gen_4` | grid_3x3_00 | right_wall, right_second_wall | job `5b7a0d50` **2/2 成功**（见下方闭环表） |

`arc_gen_1`/`arc_gen_2` 是同一角色的两次独立生成（不同 generation_id），保留两者是为了区分「边界契约失败」与「配置对齐失败」；`grid_gen_1` 与 `grid_gen_4` 角色集合不同（前者含 roof triangle）。

**同一设计闭环逐字段表**（每次运行都从作业目录取回，非生成端自述）：

| 结构 | job | generation_id | gen=request=job design_digest | locked digest / 作业内 locked 数 | movable roles | native build_layers | cycle | final |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `arc_gen_2` | `c9696af2` | `d2b1dc68…` | `68e246ba…` ✓ | `6643a158…` / 7 | square_9,10,11 | 逐层单件 | 1,2,3 | true |
| `arc_gen_3` | `0e9ebb5e` | `4155915b41f644cdb96e61766d187bc4` | `690f70b9…` ✓ | `6643a158…` / 7 | square_9, square_13, triangle_17 | `[square_9]`, `[square_13]`, `[triangle_17]` | 1,2,3 | true |
| `arc_gen_4` | `a49aa129` | `647e13ec7e9f471fa4393582e2ac2b1a` | `bf29774b…` ✓ | `6643a158…` / 7 | square_8, square_9 | `[square_8, square_9]`（同层两件） | 1,2 | true |
| `grid_gen_3` | `12e3d576` | `db9587100ea54c85a6949456c780a03b` | `780aec2b…` ✓ | `5ea0b144…` / 9 | right_wall, back_wall, left_wall | `[right_wall, back_wall, left_wall]` | 1,2,3 | true |
| `grid_gen_4` | `5b7a0d50` | `359d6dfd9ef04b228bf8f526e891e579` | `6149e95f…` ✓ | `5ea0b144…` / 9 | right_wall, right_second_wall | `[right_wall]`, `[right_second_wall]` | 1,2 | true |

三个结构的共同性质：生成 proof、浏览器/请求、作业 `builder_scene.json` 三处 `design_digest` **完全相等**（`request_equals_generation_proof=true`、`job_builder_scene_matches=true`），作业内 locked 角色数与模板一致（arc 7 / grid 9，新增 0），接触策略统一为 `transport_world_checked_compatibility`（与 19 件模板同款，未新增豁免），且 `generated_design_used_unchanged=true`——原生执行消费的就是生成的设计本身。

`arc_gen_3` 的 `tray_slot_role_order = [square_9, triangle_17, square_13]` 与 selected_roles 顺序不同，但**集合相同**：槽位顺序来自 `tray.slot_layout` 保留的原模板物理槽位，不是角色替换。（`compile_selection` 会拒绝任何不在选中闭合子结构内的角色。）

| `grid_gen_1b` | grid_3x3_00 | right_wall, right_second_wall, right_roof_triangle | **3/3 成功**（job `c957130d`）：`IterationAPI.post('generate')`（浏览器生成路由同入口）+ 原 prompt，最终库验证通过；**LLM 再次选中同一角色集合，design digest 与原版逐位相同（`8aa0eb42…`）**——同一设计在最终库下重获有效 proof。提交闭环：作业 builder digest = 请求 digest = `8aa0eb42…`，locked 9，`build_layers=[right_wall]→[right_second_wall]→[right_roof_triangle]`，`generated_design_used_unchanged=true` |

角色集合关系：`grid_gen_1`（right_wall, right_second_wall, right_roof_triangle）是 `grid_gen_4`（right_wall, right_second_wall）的严格超集，二者构成「同一底板加/减一件」的对照；`grid_gen_3` 与它们只共享 `right_wall`，是独立的三角摆放结构。

## 2. Jimu：弧形第 11 件返程问题（不再排除出范围）

### 2.1 只读同候选 trace（区分三类失败）

`--stop-after-return-diagnostic` 运行（job `01dab6f951b540dabd6bde3b6eee80a0`）在首个失败返程查询捕获：

- source `square_16`、阶段 `pre_release_post_place_clearance_post_plane_main_m_near_up_return_check`、`released=false`、`attached=false`；
- start 端点 `INVALID_START_STATE_WORLD_COLLISION`，goal 端点 **valid**；
- 具体碰撞对：`gripper_base_link`(sphere 10) vs `scene_obstacle_square_16` 线性重叠 **8.63 mm**、`gripper_Right_2/Left_2_Link` vs 同件各 1.7 mm；
- 规划器模型夹爪锁 0.6 rad，而仿真实际夹爪 0.7189 rad（模型落后于仿真）；
- `state_unchanged=true`：不是接触状态未恢复（phase 参与者/`excluded_sources` 恢复链路干净），也不是目标不可达——是**已放置物体碰撞 + 模型夹爪状态滞后**。

### 2.2 最小修改：返程检查边界夹爪模型同步

新增 `rm75_app/workcell/jimu_return_model_sync.py` + `pickplace_gripper_state.sync_gripper_to_demo`（与既有 `update_from_demo` 同守卫/同拓扑校验，仅去掉 released 门槛），在 `_plan_return_to_start_joint_curobo` 入口把规划器锁关节同步到仿真实际值；`--jimu-return-model-sync` 显式启用，事件 `jimu_return_model_sync` 记录 before/after 与错误。

复跑（job `45a037254d7f4e23b1aa920de97d632a`，569.5 s）：同步在 **17 个返程检查点全部生效**（0.6 → 0.7189），**前 10 件不退化（10/12 完成）**；第 11 件 `triangle_18` 的返程起始碰撞**仍然存在**——真实夹爪状态下的返程起点本身与已放置件重叠，不是模型滞后造成的。`--jimu-return-diagnostic-limit 8` 复跑（job 见 §2.3）以捕获 triangle_18 自己的碰撞对。

### 2.3 triangle_18 返程起点碰撞对（失败专用命名 + limit-1 实测）

limit-1 复跑（job `635f460b2940458f9a86706221d23f0a`，751.3 s，`--jimu-return-model-sync --jimu-return-diagnostic-limit 1`）：**前 10 件不退化（10/12 完成）**，第 11 件 `triangle_18` 的首个失败返程查询即被捕获；错误名现在只在失败时带碰撞对明细（`_detail_suffix` → `jimu_collision_details` 只读几何助手）：

```
Jimu return full-world collision at 0: INVALID_START_STATE_WORLD_COLLISION
| detail={dense_index: 0, source: triangle_18, geometry_detail_recorded: true, ...}
```

- 世界接触对（32 对，按 clearance 升序，最接近的前 7 对）：

| 机器人 link（sphere） | 障碍 | clearance |
| --- | --- | --- |
| `right_pad` (18) | `scene_obstacle_square_16` | 34.9 mm |
| `left_pad` (17) | `scene_obstacle_square_16` | 36.0 mm |
| `gripper_Right_Support_Link` (15) | `scene_obstacle_square_16` | 47.7 mm |
| `gripper_Left_Support_Link` (12) | `scene_obstacle_square_16` | 48.8 mm |
| `base_link` (0) | `virtual_table_plane` | 57.5 mm |
| `gripper_Right_2_Link` (16) | `scene_obstacle_square_16` | 76.4 mm |
| `gripper_Left_2_Link` (13) | `scene_obstacle_square_16` | 77.0 mm |

（其余 25 对 ≥ 85 mm，主要是 `base_link` 与其它已放置件。）
- cuRobo 原始世界碰撞（`curobo_raw_world_collision`）：25 个约束中**恰好 1 个非零**——`gripper_Right_2_Link` 球 16 vs `square_16`，value 0.0472；与最接近接触对一致。
- 结论（最小修改判定）：triangle_18 的**返程起点关节 q 本身**把夹爪（两指垫、支撑件、Right_2）带到距已放置 `square_16` 仅 35–77 mm 的位置，cuRobo 原始世界约束报 1 处碰撞。这是真实几何问题（第 11 件的返程起点与相邻已放置件过近）——不是夹爪模型滞后（本轮模型同步生效、前 10 件无退化），不是接触状态未恢复，也不是目标不可达（§2.1 三类区分已排除后两类）。本轮保留两项正确性修复（返程检查边界夹爪模型同步 + 失败专用碰撞对命名），不放松任何阈值、不修改 vendor 几何；第 11 件的返程起点几何修复超出最小修改范围，如实列入 §6 未证明项。

## 3. PushT：合并后控制器复跑与干预

- 原案例复跑（合并后控制器，含 SessionControl epoch/干预修复）：**2 推成功**，终态 `[0.375463724, -0.176387921, 0.007207535]` 与合并前记录一致（176.1 s）。证据：`runtime_data/three_scene/pusht_r2_20260911/original_regression/`。
- 脚本扰动恢复复跑（`--disturb-after-pushes 1`）：**3 推成功**（207.8 s），扰动轮 `response_fit_skipped`、epoch 复位语义生效。证据：`runtime_data/three_scene/pusht_r2_20260911/disturb_recovery/`。
### 3.1 成功确认窗口内的干预恢复（pause → relocate → resume）

会话契约改为**分层实连**，不再由测试工具直接写 worker 文件：

- 请求层：`--confirm-window-relocate` 时请求参数携带 `run_until_goal=true`（`spec.py` 已定义的类型化参数，real 模式仍被拒绝）；机器 profile 携带 `pusht.session_policy`（`position_replan_m/yaw_replan_rad/poll_s/max_wall_s`）。**worker 自己**在建控制器前写 `session_policy.json`。
- 控制层：干预经 `rm75_app/workcell/iteration_api.py::IterationAPI.control`——与浏览器路由 `POST /api/workcell/iterate/sessions/<id>/control` 同一入口与同一校验（relocate 必须已确认 paused 且为物理 SIM、命令序列号必须连续、上一命令必须已确认）。工具不再写 `session_policy.json`/`session_commands/*.json`；仅 HTTP 传输层不在本沙箱内（本进程禁非 Unix socket）。
- 触发点：worker 事件流出现第一条 `goal_confirmation`（即确认窗口已打开）时下发 pause，等 `phase=paused & safe_to_adjust` 后再 relocate，确认后 resume。

| 运行 | job | 干预落点 | 结果 | 耗时 |
| --- | --- | --- | --- | --- |
| `confirm_intervention_f`（直接写命令文件，过渡实现） | `616004aa70ae4678a87c470aeba87d15` | epoch 0→3，`last_command=3` | **成功**：3 推，`external_scene_epochs=3`，终态 `[0.3761,-0.1835,0.0156]` | 327.8 s |
| `confirm_intervention_g`（会话 API + 请求层使能；physics.py=漂移测量版 `7ccfff83a539`） | `34fd4f62901c4292a5f59946033f76aa` | t=204.74 s pause / 204.94 s relocate / 205.09 s resume，epoch 0→1→2→3，`IterationAPI.control` 应答 sequence 1/2/3（`queued=true`） | **成功**：4 推，`task_success=true`，`success_observations=3`，`external_scene_epochs=3`，终态 `[0.376089841,-0.183515742,0.017613385]` | 323.6 s |
| `confirm_intervention_h`（冻结树复跑，physics.py=流式版 `95491d4b9ad9`） | `439fcc9af3d246658d2162ef753b438a` | t=205.54 s pause / 205.71 s relocate / 205.92 s resume，epoch 0→1→2→3，应答 sequence 1/2/3 | **成功**：4 推，`task_success=true`，`success_observations=3`，`external_scene_epochs=3`，终态 `[0.376089841,-0.183515742,0.017613385]` | 328.1 s |

f 与 g 的关键差别是**状态可见性**：f 的干预前会话状态为 `run_until_goal:false`（工具中途写策略文件无效），说明「策略必须在构造前存在」这一约束真实存在；g 由 worker 依请求写入，`session_status.json` 自第一帧即为 `run_until_goal:true`。

g 的干预三段全部经**浏览器同款入口**走通，逐段回执与相位如下（`result.json.confirmation_intervention.steps`）：pause 前 `phase=observing` → 应答后 `phase=paused`、`epoch=1`；relocate 前等待到 `phase=paused & safe_to_adjust=true` 才下发 → `epoch=2`；resume → `phase=observing`、`epoch=3`。物理侧记一条 `pusht_external_intervention`：`before=[0.375381,-0.176406,0.006386]` → `after=[0.358,-0.186,0.09]`，`counted_as_push=false`、`response_fit_allowed=false`，随后 3 推重新逼近并在确认窗口内达成目标。**干预被测对象是真实会话状态机与真实物理，不是打桩**。

g 与 h 的差别只是 `rm75_app/pusht/physics.py`：g 为「两处 1e-5 门限 + 漂移实测记录」版本，h 在其上追加了 §3.3 的流式证据写盘。g 的 `source_sha256` 与工作树相比仅 physics.py 一项 DIFF（其余 11 项全 SAME），故 g 的证据按「冻结前一版源码」标注；h 的 **12 项全部与工作树一致**（`source DIFF vs tree: none`），即 h 的整条链路（控制器、物理、会话控制、`IterationAPI`、规划后端）都是提交前源码，且 h 与 f/g 三次成功结果一致（终态 `[0.376089841,-0.183515742,0.017613385]` 逐位相同），说明流式改造没有改变物理与判决。

### 3.2 交互额外 observe 与计划起点 q（不放宽 1e-5）

审阅要求检查「交互额外 observe 推进物理步后计划起点 q 不一致」。定位：`controller.py` 交互分支在**规划之后、执行之前**再取一次观测（用于判定 T 是否在规划期间移动、以及接触特征是否仍属于规划位姿），而 `observer.observe()` 会推进 `1/control_freq` 物理步。若该步使机械臂关节被伺服移动超过原 1e-5 rad，`physics.py::execute_push` 会抛 `Simulated start changed after planning`。

处理（**不改判据**）：两处 `>1e-5` 门限保持原值，新增实测记录 `last_planning_start_q_drift_rad`、`last_pre_execution_start_q_drift_rad` 写入 `physics/summary.json`，错误信息带上实测量值（`… : 2.000e-05 rad > 1e-5 rad`）。测试 `test_interactive_extra_observe_measures_start_q_drift_without_widening_the_gate`（AST 断言两处比较常量仍为 `1e-5`）与 `test_start_q_drift_still_raises_at_the_original_tolerance_with_measured_value`（2e-5 仍抛错并记录）。

实测（g 漂移版与 h 冻结树各一次，`physics/summary.json`）：两次均为 `last_planning_start_q_drift_rad = 1.1920928955078125e-07`、`last_pre_execution_start_q_drift_rad = 1.1920928955078125e-07`（单精度浮点的 1 ULP，约为门限的 1/84），即交互额外 observe 在该伺服配置下**没有**把关节推动到可测状态变化，5927 个控制步内无一触发。门限与记录保留，供长会话/不同伺服配置复现。

### 3.3 长会话证据：流式磁盘 + 有界缓存

`PhysicsSession` 原先每控制步向内存数组追加一行，长会话（`run_until_goal`）会无界增长。改为：每控制步行以 JSONL **流式写盘** `physics/observations.jsonl`（每 30 步 flush），内存仅保留 `deque(maxlen=4096)` 有界显示缓存；`close()` 由流文件**逐行重建** `observations.json`，与原先一次性 `dumps(list)` **逐字节相同**（测试 `test_observation_evidence_streams_to_disk_and_rebuilds_the_same_array`），并新增 `observation_samples`/`observation_display_cache_rows` 两个计数便于核对。显示路径（`service.job`）原本已有界：只读 events.jsonl 末 120 KB / 末 40 条、stdout 末 16 KB。

**冻结树实测（h，job `439fcc9af3d246658d2162ef753b438a`）**：`observation_samples = 5927`、`observations.jsonl` 5927 行、重建的 `observations.json` 5927 行，而内存显示缓存 `observation_display_cache_rows = 4096`——即刚好被 `maxlen` 截住。该会话共 5927 个控制步，若按原实现会把 5927 行全部留在内存并在 `close()` 时一次性序列化；现在内存占用与行长解耦。两次 drift 门限实测均为 `1.1920928955078125e-07` rad。

12 例原 T 随机套件：`tools/build_pusht_suite_profile.py`（复用冻结基线 + 校准 v3，`--geometry-id original`）+ `tools/run_pusht_random_suite.py --run --seed 77 --count 12`，结果见 §3.4。

### 3.4 12 例随机套件（seed 77，完整分母，0/12 如实记录）

- **12/12 全部尝试**（`attempted=12, requested=12, successes=0, hardware_connected=false`），无跳过、无换种子、无放宽护栏。
- 失败模式分布：**9×** `RuntimeError: Physical T left planar/support workspace`（首次推即出支撑面，9–30 s）；**2×** `Operator requested a stop`（案例跑满 900 s 超时被停）；**1×** `RuntimeError: Physics replanning rejected: PushPathRejected`（无可执行推送候选）。
- 案例分布：seed 77 的 12 例多为**大偏航差**（例：initial yaw −2.5711 rad → goal −1.5797 rad，≈0.99 rad 旋转）。冻结模型 `maximum_push_length_m=0.05`，响应校准 v3 来自平移为主的批会话——大偏航旋转在若干 0.012 m 小步中被推出支撑平面，物理护栏如实拦截（护栏、门限、校准均未改）。
- 对照：R1 的 3 例随机（小偏航）全成功；平移案例与确认窗口干预（f/g/h）全部成功。结论：合并后控制器保持平移/小扰动能力，**大偏航随机案例是真实未覆盖区域**，如实列入 §6 未证明项——不以换种子或放宽护栏掩盖。每例的 job id、请求、错误均存于 `runtime_data/three_scene/pusht_r2_20260911/random_suite_seed77/results.json`。

## 4. PickPlace：七物体分母与第一失败阶段

全部 5 次运行使用同一冻结桌面（`assets/test_scenes/current_table.json`）、`transport_world_checked_compatibility` 接触策略、无种子参数、无阈值修改；每次运行 2 次尝试即如实停（`native_cycles` 完整记录每次尝试）。

### 4.1 建议顺序 vs 原顺序（七物体分母不变）

| 运行 | 顺序 | job | 逐周期结果 | 结论 |
| --- | --- | --- | --- | --- |
| `suggested` | shuazi, bi, lvmukuai, carriot, tennis, gluestick, hongshupian | `c5f0a8c2` | cycle 1–5 全部**一次通过**；cycle 6 gluestick ✗✗ → 停 | **5/7 干净通过**；gluestick/hongshupian 的失败与顺序无关 |
| `original_control`（对照） | shuazi, bi, lvmukuai, carriot, gluestick, hongshupian, tennis | `54c80d42` | cycle 1–4 一次通过；cycle 5 gluestick **✗✗✓**（第 3 次成功）；cycle 6 hongshupian ✗✗ → 停 | 与 R1 基线逐位复现（4 干净 + 1 重试成功） |

建议顺序的收益：tennis 提前后在第 5 位一次通过（原顺序下 tennis 排最后、从未被轮到）。分母保持完整 7 物体，未通过降低阈值或跳过对象来抬高成功率。

### 4.2 gluestick 第一失败阶段：抓取 IK 无交集

`gluestick_paired_on/off`（同一冻结对 gluestick+hongshupian，job `def62cbe`/`5eeca0bd`）：gluestick 两次尝试均失败，stdout 定位到**抓取阶段**——`[winner_chain] IK preselect found no candidate with both pregrasp and grasp IK (pregrasp_ok=5/13)`，grasp IK 候选 **0/13**（垂直长轴抓取的 grasp IK 无一可解），legacy 候选级 grasp MotionGen fallback 已禁用。

- **paired-endpoint-repair 开/关无差异**（27.7 s vs 26.2 s，同样 ✗✗）：该修复作用于放置端点对，不改变抓取 IK 候选的不可达性——与 R1「垂直候选几何不可达」结论一致。
- 对照：`original_control` 里 gluestick 在第 3 次尝试成功（✗✗✓），说明同一候选集偶有可解初值——修复开/关的 2 次尝试协议无法区分这一差异，如实记录为「修复未改善首败阶段」而非「修复有效」。

### 4.3 hongshupian 第一失败阶段：带载 transport hover

`hongshupian_stage`（hongshupian+tennis，job `774b0425`）：hongshupian 两次尝试失败，失败渲染文件名定位到 **`transport_hover_face_robot`** 阶段——grasp 候选已建（24 对 built）、抓取成功、**带载 transport hover 失败**；随后 tennis 也失败（cycle 1 全部目标失败即停）。与 R1 的 lift 0.12/0.15 试验一致：抬升参数不是该失败的关键变量。

### 4.4 汇总

七物体分母下：**5/7 成功**（suggested）/ **5/7**（original，含 1 次重试成功）。两个失败对象的确定性第一失败阶段：gluestick = 抓取 IK（0/13 grasp 候选），hongshupian = 带载 transport hover。`clearance_failures` 全部为空、`clearance_path_audits` 全部 `all_valid=true`——**不是放置间隙审计的问题**。未降低任何碰撞、缓冲、阈值或成功条件。

## 5. 测试与验证

- 新增测试：`test_return_model_sync_skips_real_and_prefetch_and_emits_evidence`、`test_subset_task_manifest_*`（2）、`test_confirmation_intervention_commands_*`、`test_paired_endpoint_repair_requires_*`；`pickplace_gripper_state` 重构后既有 release 测试全过。
- 最终全量隔离回归（冻结代码 + 租约空闲，`runtime_data/three_scene/isolation_regression_r2_final_20260911.log`）：**1334 passed, 1 warning in 44.62 s**。起点回归 1322 项 → 1334 项（+12：PushT 漂移/流式 4 项、扰动-会话契约 2 项、Jimu 子结构 manifest 2 项、返程模型同步/端点修复 4 项）。`tests/iteration/test_runtime_wiring.py` 的 3 项此前失败确认为 workcell 租约占用（`ResourceLease` 单作业），本次租约空闲后全部通过——非代码缺陷。

### 5.1 机器两次黑屏与运行环境修复（非代码缺陷，如实记录）

2026-09-11 当天机器两次黑屏死机（19:34 与 21:27 各一次重启）。两次死机暴露/造成以下运行环境问题，全部已修复并记录；**所有源码/测试未因死机改动**：

1. **cuRobo JIT 缓存被半成品 + 残留锁卡死**：第一次死机前 arc 运行正在 JIT 重编译内核时挂起（scope 内 nvcc 卡死），在 `runtime_data/curobo_native_extensions/geom_cu/` 留下 0 字节 `lock` 文件与半写 build.ninja——之后每次原生运行都在 torch 构建锁上无限等待（第二次黑屏前 pickplace 步即为此现象，900 s 超时退出，exit=42）。修复：清除残留锁与半成品目录，在 scope 外重建全部内核（约 1 分钟），scope 内验证导入通过。
2. **coacd 包缺失**：vendor 场景加载要求 `decomposition="coacd"`，重启后触发未缓存网格的分解 → `ModuleNotFoundError: coacd`。修复：`pip install coacd`（vendor 自身提示的同名依赖），并预热 8 个未缓存网格的 `.coacd.ply`（与 9 月 8 日既有缓存的同一模式，落在 vendor 目录旁的生成缓存文件，不动 vendor 源码）。
3. 队列脚本全部重建到 `runtime_data/three_scene/`（`/tmp` 被重启清空），每一步加 `timeout -k 60` 看门狗与 stale-lock 自愈。
4. 死机时刻的任务对照：第一次死机时有挂起的 GPU 编译进程（可能有关系）；第二次死机时 GPU 完全空闲、仅剩秒级 CPU 诊断脚本——两次死机无法归因于单一任务，疑似机器/驱动层面问题。真机未动。

## 6. 未证明项与 NOT_RUN

- 相机与真机：未运行。
- triangle_18 返程修复：模型同步是正确性修复且无退化，但未解除第 11 件——真实几何碰撞仍在调查。
- 12 例随机套件（seed 77）：0/12，失败模式见 §3.4——大偏航案例为真实未覆盖区域，未通过换种子/放宽护栏规避。
- 确认窗口干预：已证明（f/g/h 三次成功，g 为会话 API + 请求层使能，h 为冻结树复跑）。
- 响应校准 v3 仍为同场景调试校准。
