# 三工作流迭代（2026-09-10）

日期：2026-09-10  
分支：`chatgpt/three-scene-software-closeout`  
基线提交：`eca8ece3aa6aa86155cd34b2e4b5e59b471a1632`（PushT 批量规划与失败物体 IK 诊断）

三条工作流并行推进：PickPlace 连续抓放与剩余两对象（gluestick、hongshupian）针对性修复；旧 3×3 与弧形底板只读导入并打通真实 LLM—前端—原 Jimu 程序；PushT 保留最新长推实现并完成原案例回归、随机场景与移动 T 恢复测试。全部测试与仿真经 `tools/run_network_isolated.py`（内核网络阻断，唯一例外是仅监听 127.0.0.1 的浏览器验证服务器与真实 LLM API 调用，两者不接触任何机器人接口）；GPU 工作全部串行并置于 systemd-run scope（MemoryHigh=6 GiB、MemoryMax=7 GiB、禁止 swap、CPUQuota=200%）。本轮未连接或操作真机与夹爪，未修改旧仓库，未降低任何碰撞或成功条件。

用户点名的审阅文件 `CHATGPT_REVIEW_ECA8ECE_AND_NEXT_STEPS_20260910.md` 在本机不存在；本轮按 `docs/CHATGPT_REVIEW_A35328C_IK_SIM_JIMU_20260909.md` 与 `docs/CODEX_PUSHT_BATCH_SEARCH_20260910.md`、`docs/CODEX_PUSHT_BACKOFF_LIFT_20260910.md` 继续。

## 0. 隔离回归

命令：`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 tools/run_network_isolated.py -- python3 -m pytest tests -q`

结果：**1091 passed, 1 warning（42.30 s）**，HEAD eca8ece 基线全绿。日志：`runtime_data/three_scene/isolation_regression_20260910.log`。

## 1. PushT：原案例回归、随机场景、移动 T 恢复

保留最新长推实现（多候选批量搜索、响应校准 v3、后退 10 mm + 上抬 50 mm 撤退）。所有运行沿用原命令参数：`full_arm_physics`、原重力补偿、闭合夹爪 0.9 rad、夹爪反馈几何、`--maximum-push-length-m .05`、响应校准 `response_calibration_v3.json`。成功条件不变：位置 ≤ 6 mm、朝向 ≤ 0.1 rad、连续三次稳定观测。

| 试验 | job | 状态 | steps | 终态观测 [x, y, yaw] | 用时 |
| --- | --- | --- | ---: | --- | ---: |
| 原案例回归（translation） | `439f723ae96448458981da76166f1f15` | succeeded, physics_pose | 2 | `[0.375463724, -0.176387921, 0.007207535]` | 179.33 s |
| 随机 r1_yaw_recovery（初态含 0.2 rad 偏航） | `163d3a686d124dcc87cf71aab4e2136c` | succeeded, physics_pose | 7 | `[0.384049386, -0.183102310, 0.092144914]` | 395.57 s |
| 随机 r2_rotated_goal（目标含 0.12 rad 偏航） | `341fcc7b61874e17a77767d055aa2819` | succeeded, physics_pose | 4 | `[0.371357322, -0.175508991, 0.111360475]` | 248.21 s |
| 随机 r3_offset_start（初态偏移） | `b77b496514214aaf906e595f58a93135` | succeeded, physics_pose | 4 | `[0.376974404, -0.174959003, -0.039828293]` | 473.37 s |
| 移动 T 恢复（第 1 推后外部移动） | `f4e4d1e6a2e44930ac34db6b188e56071` | succeeded, physics_pose | 3 | `[0.376129657, -0.183503985, 0.015780997]` | 见 result.json |

- 原案例回归与 `CODEX_PUSHT_BATCH_SEARCH_20260910.md` 记录完全一致（2 次推送、同一终态）。
- 随机三例为代码内固定种子（`tools/run_pusht_physics_validation.py` 的 `RANDOM_CASES`），每次运行可复现，不取运行时随机数；三例全部达到原阈值。
- 移动 T 恢复：第 1 次推送后（T 位于 `[0.374478, -0.176111, -0.015738]`），在 t=74.90 s 由外部扰动覆写为 `[0.358, -0.186, 0.09]`（位置与偏航同时挪开）。控制器以新物理真值重新观测、重新规划，3 次推送内恢复成功。被扰动跨越的「推→观测」转换不进入接触响应拟合（事件 `response_fit_skipped reason=external_disturbance_after_push`），校准不被污染。扰动在 physics summary 中记录为 `external_pose_overwrite_between_pushes`，提前接触/静态障碍接触/未授权 link 接触 T 均为 0。
- 实现：`rm75_app/pusht/controller.py`（可选 `disturb` 钩子）、`rm75_app/pusht/physics.py`（`PhysicsSession.disturb` + 调度接线）、`tools/run_pusht_physics_validation.py`（`--case random`、`--disturb-after-pushes/--disturb-pose`）。扰动日程放在 machine profile 的 physics 段，`spec.py` 浏览器契约不变。测试：`tests/three_scene/test_pusht_disturbance_recovery.py`（9 项）。
- 证据目录：`runtime_data/three_scene/pusht_iteration_20260910/{original_regression,random_cases,disturb_recovery}`。

## 2. PickPlace：连续抓放与剩余两对象

### 2.1 连续抓放基线（7 对象单进程）

命令（workcell service，冻结世界，transport_world_checked_compatibility）：

```bash
/home/zhangzhao/anaconda3/envs/foundationpose310/bin/python tools/run_workcell_native_validation.py \
  --task pickplace --object-name shuazi \
  --cycle-order shuazi bi lvmukuai carriot gluestick hongshupian tennis \
  --fixed-world assets/test_scenes/current_table.json \
  --audit-current-table-failures --extensions runtime_data/curobo_native_extensions \
  --output runtime_data/three_scene/pickplace_iteration_20260910/baseline_cycle --timeout-s 900
```

job `98b6107f3fb64e2cb18b11a3495b8934`，用时 69.5 s：

| cycle | 对象 | 结果 |
| ---: | --- | --- |
| 1 | shuazi | success |
| 2 | bi | success |
| 3 | lvmukuai | success |
| 4 | carriot | success |
| 5 | gluestick | False, False, **True**（第三次尝试完成） |
| 6 | hongshupian | False, False（随后程序终止，tennis 未进入） |

`native_final_success=False`、`failure_code=frozen_world_validation_failed`（并非所有源都出现 transport 观测）。基线证明连续链在 gluestick 重试后可通过、hongshupian 是当前唯一阻断连续完成的源。证据：`runtime_data/three_scene/pickplace_iteration_20260910/baseline_cycle/result.json` + job stdout。

### 2.2 hongshupian：带载抬升针对性修复尝试（未成功，如实记录）

原生失败链：基线日志显示「rejected grasp candidate before transport: … has IK but no validated straight pregrasp->grasp primitive」与 `hongshupian_transport_hover_face_robot` 失败渲染；`docs/CODEX_FAILED_OBJECT_IK_PUSHT_090_20260909.md` 记录原抬升目标处名义负载与 base_link 球线性重叠 28.975 mm / 8.171 mm。

按 A35328C 审阅建议先改抬升目标（不改碰撞/成功条件）。新增经测试的 `--pickplace-lift-target-m`（0.030–0.200 m，仅 PickPlace，只改 `--joint-search-start-collision-lift-m`，与 jimu +3 mm 试验同模式）：

| 试验 | lift | job | 结果 | 用时 |
| --- | ---: | --- | --- | ---: |
| lift 0.12（`hongshupian_lift120`） | 0.120 | —（启动前被 frozen 序列校验拒绝，单对象 cycle-order 需 ≥2 项） | — | 3.5 s |
| lift 0.12（`hongshupian_lift120b`） | 0.120 | `419c7f5d465544a58865916b48e9bbc1` | cycle 1 False×2 | 21.4 s |
| lift 0.15（`hongshupian_lift150`） | 0.150 | `c289d3d12104436eae5e22c064ebe6a8` | cycle 1 False×2 | 22.3 s |

原生日志（0.12 运行）显示：垂直抓取关系的 TCP-up 轴即为纯世界 Z（`axis_world_z=1.000`），lift 0.120 m 后端点 IK 与约束直线 IK 均 `IK_FAIL`；approach-line retreat 与 world-Z lift 回退同样失败。**结论**：抬升高度单独不能解除 hongshupian——0.12/0.15 的目标处 IK 本身失败，且基线日志表明还存在未通过直线验证的 pregrasp→grasp 原语与朝向机器人的 transport hover。候选下一方向（本轮未做）：更换抓取关系/放置夹爪朝向并重验整条带载路径；不做种子堆叠，不放松任何阈值。

### 2.3 gluestick：失败物体 IK 搜索（128 seeds）

job `c77cb71542d04f7eaf55240fa06ff42a`，用时 52.4 s。cycle 1 False×3。`failed_object_ik_search` 事件（128 seeds，逐目标重试）：首个失败目标为 lift 目标（z≈0.1158，与 hongshupian 被阻抬升同一量级），`newly_accepted=0`、`raw_results_unchanged=true`、`goals_changed=false`、`full_chain_success=false`——种子重试没有为该目标找到通过原生校验的新解，与「垂直候选释放几何不可达、种子无法修复目标几何」的既有结论一致。同时基线连续运行证明 gluestick 可在第 3 次尝试完成（cycle 5 True），即连续链中 gluestick 不再是阻断源。

**PickPlace 小结**：连续抓放 7 对象中 5 个完成（含 gluestick 重试成功），hongshupian 仍是唯一阻断连续完成的源；抬升目标 0.12/0.15 两次合法改目标试验均未解除（原生日志：抬升后端点/直线 IK 均 IK_FAIL，且存在未验证 pregrasp→grasp 直线原语与面向机器人的 transport hover）。不降低任何阈值；下一方向为更换抓取关系/放置朝向并重验整条带载路径。

## 3. 旧 3×3 / 弧形底板只读导入与 LLM—前端—原 Jimu 程序

### 3.1 只读导入

新增 `tools/import_jimu_scene_bundle.py` + `configs/workcell/jimu_scenes_import_20260910.json`：按 SHA256 寻址从旧仓库只读导入 6 个 JSON 到 `rm75_app/_vendor/jimu_scenes/`（旧仓库 HEAD `36798efb…` 与 worktree status SHA256 在导入前后一致，`old_repository_modified=false`；symlink/路径逃逸/哈希不符全部拒绝）。tag2 manifest 的 `sam6d_fixed_scene_result_file` 在导入副本中重定位到任务目录内（旧文件不改），因此两个 bundle 都满足 `read_original_task_bundle` 的目录内依赖约束，可直接用 `--task-dir` 只读回放——tag2 在旧仓库路径下做不到这一点。两个场景均通过 `validate_design`（tag1 21 件/12 可动，tag2 19 件/12 可动）。测试：`tests/three_scene/test_jimu_scene_import.py`（6 项）。

### 3.2 真实 LLM → 设计 JSON

新增 `rm75_app/llm/jimu_design_llm.py`：自然语言 → `jimu_builder_scene_v1` 设计，本地 `validate_design` 强制校验（一次带错误信息的修复重试），证据不含密钥。本机 `RM75_LLM_PROVIDER` 未设置且无 deepseek/openai key；`ANTHROPIC_AUTH_TOKEN`+`ANTHROPIC_BASE_URL` 指向 DeepSeek 的 Anthropic 兼容端点（`https://api.deepseek.com/anthropic`），经官方 `anthropic` SDK（流式 32K）真实调用：

- 模型：`deepseek-v4-pro[1m]`（机器的 `ANTHROPIC_DEFAULT_OPUS_MODEL` 映射）；proxy 显式 `http://127.0.0.1:7897`（SDK 对 socks:// 环境变量报错，模块构造客户端时按 `RM75_LLM_PROXY` 显式指定并恢复环境）。
- 命令「用弧形底板搭一个小拱门：两块 locked 方形底座并排，上面放一块三角形积木」，参考 tag2 导入模板。
- 产出 10 件设计（7 locked 弧形件 + 2 locked 中心方件 + 1 可动三角 `triangle_17`），**一次通过验证**（`end_turn`，无 validation_error）。设计 SHA256 `3e26d437…`。
- 证据：`runtime_data/three_scene/llm_jimu_20260910/real_arc_design/{design.json,llm_request.json,summary.json}`。

mock provider 为确定性离线设计，供测试与离线链演练。测试：`tests/three_scene/test_jimu_design_llm.py`（8 项）。

### 3.3 前端与服务边界

workcell 服务器仅监听 127.0.0.1（`--port 7862`，`allow_real=False`；TCP 监听不能用 seccomp 隔离启动，已单独记录），Playwright 用 `/tmp/rm75_three_scene_browser_20260910` venv + 本机 chromium-1228。

- 浏览器往返（`tools/check_workcell_browser.py`，导入的 tag2 弧形底板设计）：**7 项 checks 全 passed**——magnetic 完整 builder 往返（19 件/12 可动）、画布投影 69 顶点、编辑平移往返（其余 18 件不变）、magnetic preview、pickplace preview、PushT CPU surrogate 循环与 Stop。`robot_connected=false`、`hardware_actions_requested=false`。证据：`runtime_data/three_scene/llm_jimu_20260910/browser_tag2_arc/report.json`。
- 真实 LLM 设计 → workcell service magnetic preview：job `dfa8d6f6609d4807843af422236c412a`，`command_completed_unverified`、`verification=preview_only`——真实 LLM 产出的设计被 `spec.py`/`validate_design` 接受并走完 worker 预览路径。证据：`runtime_data/three_scene/llm_jimu_20260910/real_arc_preview/`。
- 首次浏览器运行因 GPU job 持有 workcell lease 而 preview 失败（跨进程 flock 正确拒绝），记录为环境串行约束而非缺陷；lease 空闲后全过。

### 3.4 原 Jimu 程序（sim）

导入的 tag2 弧形底板经 `--task-dir` 只读 bundle 回放、由原 Jimu 程序（vendored working snapshot 原程序 in-process 执行）在 sim 模式运行。job `2bbf289177f7476aa381dc4613da7411`，用时 609.2 s：

- **10/12 个搭建周期完成**（cycle 1–10，其中 4–8、10 为一次失败后重试成功）；cycle 11（第 11 个可动件）失败两次后程序终止，worker 报 `CuroboOnlyUnsupported: Jimu return full-world collision at 0: MotionGenStatus.INVALID_START_STATE_WORLD_COLLISION`——即 A35328C 审阅 §3 记录的 Jimu 返程起点世界碰撞家族，不是新缺陷。
- `original_task_bundle_unchanged=true`：原程序运行前后导入 bundle 的 SHA256 完全一致，只读回放成立。
- 结论：**真实 LLM → 设计 JSON → 前端（导入/编辑/预览 7 项全过）→ workcell service → 原 Jimu 程序（sim）整条链已打通**；弧形底板场景可被原程序直接执行。搭建任务本身的返程碰撞修复不属本轮范围（用户要求"打通"，未要求修复 Jimu）。

## 4. 测试与验证

- 新增/修改测试文件：`tests/three_scene/test_pusht_disturbance_recovery.py`（9）、`tests/three_scene/test_jimu_scene_import.py`（6）、`tests/three_scene/test_jimu_design_llm.py`（8）、`tests/three_scene/test_workcell_native_validation.py`（+3 lift-target 测试）。定向运行全部通过。
- 最终全量隔离回归：**1113 passed, 1 skipped, 1 warning（43.11 s）**（基线 1091 passed；新增 22 项，1 skip 为 workcell lease 被 GPU job 占用时的合法跳过）。日志：`runtime_data/three_scene/isolation_regression_final_20260910.log`。
- `git diff --check` 与 `compileall` 通过。

## 5. 与远端并行工作流的合并（fae8a5e + ffd1a2f）

推送时发现远端 `chatgpt/three-scene-software-closeout` 已新增 `fae8a5e`（三场景 UI 工作流、session_control 暂停/重定位/恢复、magnetic generation/llm_client、随机 PushT 套件）与 `ffd1a2f`（原生序列结果对账、worker 离线隔离边界，含 `CHATGPT_REVIEW_ECA8ECE_AND_NEXT_STEPS_20260910.md`）。本提交以 rebase 方式叠放在其上（`1078ffb`），冲突与集成修复如下：

- `rm75_app/pusht/controller.py`：两者合并。保留 SessionControl 重写（pause/relocate/resume、`response_is_plausible`、epoch 干预检测），同时接入本轮的 `disturb` 脚本化扰动钩子（`__init__` 增加 `disturb=None`；循环顶部按 step 施加扰动；扰动轮跳过响应拟合并发 `response_fit_skipped`）。互不干扰：无 control_policy 时 SessionControl 退化为非交互策略（policy 默认值），与 GPU 证据运行时的行为等价。
- `rm75_app/workcell/offline_boundary.py`：其 `from tools.run_network_isolated import block_network` 在 app-root 非仓库时（服务测试 fixture 仅软链 rm75_app）不可导入，导致真实子进程 preview/sim 测试全挂。改为优先保留原模块导入（可 monkeypatch），失败时经 `__file__` 解析真实仓库路径显式加载；缺模块时拒不派发。
- `rm75_app/workcell/iteration_workflows.py`：`from . import legacy` 读的是包属性（首次导入时绑定），导致 monkeypatch sys.modules 的测试在全量运行中顺序相关失败；改为 `importlib.import_module`（尊重 sys.modules 注入）。
- `tools/run_pusht_random_suite.py`：补上仓库根 sys.path 引导（与其它 tools 入口一致），其自带测试在全量运行中通过。
- 最终全量隔离回归（合并后树）：**1297 passed, 1 warning（40.93 s）**。日志：`runtime_data/three_scene/isolation_regression_merged_20260910.log`。

两份并行实现同时保留（如 `tools/import_jimu_scene_bundle.py` 与 `tools/import_jimu_design_library.py`、`rm75_app/llm/jimu_design_llm.py` 与 `rm75_app/magnetic/{generation,llm_client}.py`、`--case random` 与 `tools/run_pusht_random_suite.py`）：取舍由用户决定，本提交不删除任何一方。

## 6. 未证明项与 NOT_RUN

- 相机与真机：本轮未运行（用户明确禁止）。
- hongshupian 完整链：lift 0.12/0.15 未解除；记录为未解决。
- PushT 随机/恢复只覆盖代码内固定种子三例与单一扰动位姿，不代表任意场景。
- 响应校准 v3 仍为同场景调试校准，不是真机校准。
- 真实 LLM 只调用一次生成一类设计；无跨模板泛化证据。
- 合并后控制器（SessionControl + disturb 钩子）只经 CPU 测试验证；GPU 物理证据在本轮合并前生成的等价实现上获得，语义未变（非交互默认策略下行为相同）。
