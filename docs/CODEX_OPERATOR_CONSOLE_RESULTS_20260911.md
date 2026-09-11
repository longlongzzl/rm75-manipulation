# 统一操作控制台本机验收（2026-09-11/12）

日期：2026-09-12
分支：`chatgpt/three-scene-software-closeout`
验收 HEAD：`6dbb6a1811fb188ea6527818574fc51d506f216d`（`feat: add consolidated operator console with durable request recovery`）
验收依据：`docs/CHATGPT_OPERATOR_CONSOLE_20260911.md`（其实现基线 `7086fbd…` 是 HEAD 的父提交）
证据目录：`runtime_data/console_acceptance_20260911/`、`runtime_data/console_acceptance_20260911_single/`、`runtime_data/console_launch_20260911/`

本轮只做本机复验：一切 pytest / node / 原生 worker 经 `tools/run_network_isolated.py` 的内核隔离；控制台 Web 进程按 §5 例外在 loopback 监听（seccomp 会禁 TCP），preview/SIM worker 保留原隔离；GPU 相关只有 magnetic 原生 SIM 与 PushT `full_arm_physics` 单例，二者串行、无并发、启动前用 `nvidia-smi` 观测（未杀任何其它进程）；真实 LLM 调用发生在 Web/API 进程内，未把 worker 隔离关掉。**本轮未改动任何仓库文件**（验收前后 `git status --porcelain` 均为空），未改 vendor、未新增碰撞豁免、未放宽阈值或成功条件、未连接或操作真机与夹爪。

## 0. 结论摘要

| 复验项 | 结果 | 关键证据 |
| --- | --- | --- |
| 三套命令（43 / 32 / 1377） | PASS | `report.json`、`test_suites.log` |
| 正常浏览器 HTTP 导航（补上 doc §4 的 BLOCKED 缺口） | PASS（`normal_navigation: true`） | `report.json`、`report_single/report.json` |
| 只读页面与模板（shipped 工具默认路径） | PASS | `report.json`、`report_template.json` |
| 单对象预览 `--preview-object bi` | PASS | `runtime_data/console_acceptance_20260911_single/report.json` |
| 多对象预览（笔 → 网球，原隔离 worker） | PASS | `report_preview.json`，job `2854afa6722248ef85f6ec873f06d915` |
| LLM 生成 + 刷新恢复 + 同 proof SIM | PASS（19/19） | `report_jimu_llm.json`，generation `33b54f3d…`，job `e6a2f633…` |
| 同设计 SIM（设计包导入，digest 一致） | PASS | `report_jimu_sim.json`，job `a438f76d…` |
| 共享 Stop | PASS（`cancelled`，非成功） | `report_stop.json`，job `d32249dc…` |
| PushT 暂停 / 移动 T / 继续（同一请求） | PASS（`succeeded`，`external_scene_epochs: 3`） | `report_session.json`，job `45da3178…` |
| 失败项 | 9 条，见 §8（**5 条**由本轮新提交引入，含 1 条假红项；4 条继承自上一提交 `7086fbd`） | §8 |
| GPU 随机套件 / PhysX / 相机 / 真机 / 完整私有仓库 | NOT_RUN | §9 |

## 1. 启动配置与环境

- 启动命令（等价于 doc §1 的形式，profile 用本机真实 R2 配置副本）：

  ```bash
  env -u ALL_PROXY -u all_proxy \
    python3 tools/launch_workcell_console.py \
      --profile runtime_data/console_launch_20260911/console_profile_r2_nodiag.json --port 7863
  ```

- **启动配置名**：`console_profile_r2_nodiag.json`（bootstrap 自报同名）。内容为上一轮 R2 已跑通配置的副本，仅去掉 `pickplace.audit_current_table_failures` 与 `pickplace.frozen_source_order` 两个只与 PickPlace 冻结世界诊断有关的开关（原因见 §8 问题 6）。
- 进程：pid 48598，systemd scope `workcell-console-acceptance-7863b`，仅 `127.0.0.1:7863`；`ConsoleAPI` 版本 `2026.09.11-console.1`；`allow_real` 未启用（bootstrap 字段为 false，启动器本身无 `--allow-real`）。
- 用户自己的控制台（pid 34971，`:7861`）全程未触碰、未占用、未停止。
- 移除 `ALL_PROXY` 的理由见 §8 问题 7（本机 shell 的 `ALL_PROXY=socks://127.0.0.1:7897/` 会让生成 provider 的 SDK 构造直接抛错）；`http(s)_proxy` 保留，DeepSeek 端点经 `http://127.0.0.1:7897` 正常出网。
- bootstrap 配置检查：10/11 项 `ready`，唯一 `error` 是 `magnetic.scene`（假红项，见 §8 问题 1）；`magnetic.library`、`magnetic.llm`、`pusht.physics`、两个原生快照与解释器均为 `ready`。

## 2. 三套命令（doc §5 原样）

| 命令 | 结果 |
| --- | --- |
| `PYTHONPATH=. PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 tools/run_network_isolated.py -- python3 -m pytest tests/console -q` | **43 passed**, 0 failed, 0 skipped（1.60 s） |
| `python3 tools/run_network_isolated.py -- node --test --experimental-default-type=module tests/console/client_state.test.mjs` | **32 passed**, 0 failed（Node v20.19.5） |
| `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 tools/run_network_isolated.py -- python3 -m pytest tests -q` | **1377 passed**, 1 warning（48.03 s）；1377 − 43 = 1334 与 doc §4“完整仓库 1334 项”的分母一致 |

日志：`runtime_data/console_acceptance_20260911/test_suites.log`。三套均通过隔离工具运行，输出首行为 `Network isolation active: non-Unix sockets blocked in this process and its descendants.`

## 3. 正常浏览器导航（doc §4 自认 BLOCKED 的项）

doc §4 记录 `ERR_BLOCKED_BY_ADMINISTRATOR`、停在页面加载前、未绕过策略。本机复验该项**不再阻塞**：

- shipped 工具两次运行（`report.json`：只读页面+模板；`report_single/report.json`：加 `--preview-object bi`）均为 `status: PASS`，`normal_navigation: true`，`blocked_requests: []`，`errors: []`。
- 我自己驱动的 6 个阶段（template / preview / stop / session / llm / sim）在同一真实 HTTP 服务器上均为 `no page exceptions` + `no browser console errors (favicon excluded)`。
- 唯一非 2xx 请求是 `/favicon.ico` 的 404（每个新会话一次；见 §8 问题 2）。CSP 未产生任何拦截记录。

阶段截图（保留在本地 gitignore 目录）：`01_pickplace.png`、`02_jimu.png`、`03_pusht.png`、`04_mobile.png`（390 px 无横向溢出）、`template_loaded.png`、`preview_*.png`、`llm_*.png`、`sim_*.png`、`stop.png`、`session_*.png`。

## 4. 预览：多对象与单对象

- **多对象**（本机驱动，最终跑在 §1 的 R2 配置上）：勾选「笔」「网球」→ 顺序渲染为 `笔 → 网球` → 预检查 `配置与请求检查通过` → 预览 job `2854afa6722248ef85f6ec873f06d915` 完成，`verification: preview_only`、`task_success: null`。断言“预览结果不是运动成功”通过：即使作业记录里 `command_success: true`（原生命令结束），页面也只标 `preview_only`，未被读成任务成功。
- **单对象**（shipped 工具，`--preview-object bi`，输出目录 `runtime_data/console_acceptance_20260911_single/`）：`checks` 含 `explicit single-object preview through actual server/worker`，`preview_requested: "bi"`，`normal_navigation: true`，`status: PASS`。
- 两次预览都走原隔离 worker，网络隔离未关闭；未提交任何 SIM 或真机请求。

## 5. LLM：真实慢生成 + 刷新恢复 + 同一份 proof

**第一次尝试（保留在案）**：`report_jimu_llm_attempt1_profile_bug.json`、`llm_stage_attempt1.log`。真实模型调用成功、刷新恢复实质成立，但整份驱动 `FAIL`，原因是两条**我自己的断言写错**（把 `robot_command_submitted` 当作 magnetic 结果字段；把刷新后的草稿提示文案当成“已找回服务器接收记录”）**加上我合并的配置带了 PickPlace 诊断开关**导致原生 SIM 被 `legacy.py` 的守卫拒绝。三条都已定位并修正，第一次记录原样保留，不删。

**第二次尝试（终态，19/19 PASS）**：`report_jimu_llm.json`、`llm_stage.log`。

| 项 | 值 |
| --- | --- |
| request_id / generation_id | `bcbb3bab2623f181ec13bd84f756bd8d` / `33b54f3d21764b529e53cb509c91e129` |
| POST `/console/generate` 次数 | **1**（刷新前后各记一次响应，仍是同一次提交） |
| 轮询过的 generation_id 集合 | `["33b54f3d21764b529e53cb509c91e129"]`（**只有一个**） |
| 生成期间刷新 | 生成 15 s 时 `page.reload()`；刷新后仍在轮询同一 id，计时器 `已等待 0:15` 继续（来自服务端 `created_at`，非前端新计时） |
| 刷新后的本地恢复文案 | `已恢复此机器配置的浏览器草稿。没有自动发起任务、重试或模型请求。` |
| 模型调用次数 | `model_calls: 1`；provider usage `input 8 / output 1084` tokens |
| 生成结果 | `status: succeeded`，board `grid_3x3`，movable 3，roles `right_wall / right_second_wall / right_roof_triangle` |
| design_digest（proof） | `8aa0eb420b2318d4bda65bc04a86b73bed9f200ba08cf0938c70d1f65e348295` |
| 生成说明 | 「三层单柱小塔（右侧三角顶）…」（同一 digest 下文案与上一轮不同：digest 覆盖结构选择，不覆盖自然语言） |
| 预检查 | `配置与请求检查通过` |
| SIM 确认对话框 | 内文含同一 `design_digest`（不是重新生成或另一份设计） |
| 原生 job | `e6a2f633d2e24583825692573cd8101f`，请求内 `generation_proof.design_digest` == 生成 proof digest |
| 原生终态 | `command_completed_unverified`，`native_completed_cycles: [1,2,3]` 全 `success: true`，`generated_design_used_unchanged: true`，`command_success: true`，`verification: not_observed`，`task_success: null` |
| 页面标签 | `指令结束 · 未验证任务成功`（没有把原生命令结束显示为“全部成功”） |
| 真机 | 请求 `mode: sim`；生成记录 `robot_command_submitted: false`、`auto_execute: false`；结果无真机字段 |

GPU 串行门在两处生效：第一次门等了 540 s 直到用户自己的 GPU 作业退出；第二次 `other_compute_apps: []`、`waited_s: 0`。门只观测，不杀进程。

## 6. 同设计 SIM（设计包导入路径）

用 §5 生成的设计包（`same_design_bundle.json`，含 `generation_proof`，digest `8aa0eb42…`）走控制台的文件导入路径，不调用模型：

- 导入后摘要：`导入设计（来源已校验）· 固定 9 件 / 活动 3 件`（来源与 proof 一起导入）。
- 预检查：`配置与请求检查通过`；SIM 确认对话框含同一 digest。
- 原生 job `a438f76d1d0b4e4a898b20a8c8be3c16`：`command_completed_unverified`、`command_success: true`、`native_final_success: true`、`native_full_chain_passed: true`、`native_completed_cycles: [1,2,3]`、`expected_cycles: 3`、`episode_command_results: [true×5]`、`clearance_failures: []`、`generated_design_used_unchanged: true`、`original_algorithms_preserved: true`、`contact_policy: transport_world_checked_compatibility`。
- 发布执行审计（`jimu_release_execution_audit`）：`passed: true`，`execution_guard: true`，clearance 采样 49 次、release 接触采样 2 次、return 采样 157 次，允许接触目标 `scene_obstacle_right_wall`，`physical_success: null`。
- 结果自带诚实边界：`verification: not_observed`、`task_success: null`、`note: "Normal process return is not proof of a real grasp or magnetic connection"`。
- 两次 magnetic SIM（§5 生成路径、§6 导入路径）是**同一 digest、同一 proof**，均 3/3 周期、`generated_design_used_unchanged: true`；成功没有靠换设计或叠种子取得。

## 7. Stop 与 PushT 会话操作

**共享 Stop**（`report_stop.json`，PushT `surrogate`，CPU、无物理后端）：

- request_id `4a2b9d78f7e44410b935af9fd47e8547` → job `d32249dc5f2942be95a8c5c2fef4ce42`。
- 预检查 digest `456a8df…` 与提交后的 `canonical_request_digest` 一致（浏览器往返没有改变数值表示）。
- 轮询序列 `accepted → running → running → cancelled`；终态 `status: cancelled`、`command_success: false`、`error: worker exited -15 without a final result`、`task_success: null`（停止不被读成成功，也不被读成“已安全停止”以外的结论）。
- 同一阶段第一次运行的 job `433957ba0bfb4506a48a212f97c875a9` 同为 `cancelled`（那次整份报告只因 favicon 记了一条控制台错误而 FAIL，见 §8 问题 2）。
- 另一条诚实记录：驱动在会话阶段崩溃时留下正在运行的 job `42b86f749637499b92540686e17ef312`，我用控制台自己的 Stop 端点（`POST /api/workcell/jobs/<id>/stop`）停掉它，回执 `{"requested": true, "physics_cooperative_stop": true, "physical_estop": false}`，此后 `cancelled / managed_active: false`。没有解除锁、没有杀其它进程、没有动用户的服务。

**PushT 暂停 / 移动 T / 继续**（`report_session.json`，11/11 PASS）：按上一轮记录的单例参数，经 UI 的“运行中移动 T（仅仿真）”面板操作：

| 项 | 值 |
| --- | --- |
| job | `45da3178d0f240df98c1d733cfd94bfd` |
| 提交的实际参数 | `initial [0.35,-0.18,0]`、`goal [0.38,-0.18,0]`、`full_arm_physics`、`geometry_id: original`、`max_steps: 12`、`run_until_goal: true`、`speed 0.015`、`maximum_push_length_m 0.05` |
| 暂停 | `指令 1 已确认 · 已在动作边界暂停` |
| 移动 T | `指令 2 已排队，等待服务器确认；尚未允许修改 T。`（等待上一条指令确认后才允许继续） |
| 继续 | 同一请求继续，终态 `succeeded`、`task_success: true`、`verification: physics_pose`、`steps: 2`、`success_observations: 3` |
| 移动是否真的生效 | `external_scene_epochs: 3`（不是只改了 UI 草稿） |
| 终态位姿 | `[0.37614, -0.18354, 0.01494]`（目标 `[0.38,-0.18,0]`） |
| 真机字段 | `mode: sim`、`hardware_connected: false`、`model_validated_on_robot: false`、`hardware_profile_qualified: false` |

表单细节两条（UI 契约，不是绕过表单）：`未到位则继续`勾选时「步数上限」被置灰，因此驱动程序先取消勾选、写入 12、再重新勾选，最终请求同时带 `max_steps=12` 与 `run_until_goal=true`；暂停/继续/移动控件位于默认折叠的 `<details>` 面板内，驱动按操作员动作先展开该面板再点击，断言里记录了展开后的标题「运行中移动 T（仅仿真）」。

## 8. 失败项与问题清单（本轮新增 / 已存在分开）

### 本轮 HEAD（`6dbb6a1`）新引入

1. **bootstrap 里 `magnetic.scene` 是假红项。** `console_api.py:157-162` 只在“已有 spec 且 spec 带 `generation_proof`”时才跳过该项；bootstrap 没有 spec，所以配置检查恒定报 `error`，而提示语本身已写明“生成结构使用其原模板的场景”。实际影响：操作员看到一个红项，但本轮 3 次 magnetic 原生 SIM 全部跑通。建议改为按“该 profile 是否能经模板库解析出场景”判断，或明确标为 `warning`。
2. **`/favicon.ico` 404。** 每个新浏览器会话产生一条控制台错误（shipped 工具与我的驱动都遇到；我的驱动后来按 shipped 工具的口径排除 favicon 再判定）。静态白名单里没有 favicon，属外观问题。
3. **doc §5 的 shipped 工具在本机默认解释器下不可直接运行。** `python tools/check_workcell_console.py …` → `ModuleNotFoundError: No module named 'playwright'`（本机 playwright 装在 `~/.venvs/console-acceptance`，与 `tools/launch_workcell_console.py` 用的解释器不同）。此外该工具在 `import playwright` **之前**就 `mkdir` 输出目录，失败后目录已存在，重试报 `FileExistsError: Choose a new evidence directory`，容易被误读成“证据目录被占用”。建议把依赖检查移到建目录之前，或在 doc §5 注明需要的解释器/venv。
4. **`--list-profiles` 暴露不到真正可用的本机配置。** `tools/launch_workcell_console.py:18-27` 只扫 6 个 glob（`runtime_data/iteration/machine.json`、`examples/workcell/machine.example.json`、`runtime_data/three_scene/*/machine*.json`、`runtime_data/three_scene/*/*/machine*.json`、`configs/workcell/*machine*.json`、`runtime_data/iteration/*.json`）。本机输出 125 个候选（44 个带 physics 后端），`has_jimu_library` **全为 false**，而本轮实际跑通的 R2 配置（文件名不是 `machine*.json`）根本不在列表内；`--profile` 却能正常读它。这与 doc §1“读取已有本机配置候选”的意图有落差，建议支持任意路径/名称或明示扫描范围。
5. **离线 11 项 DOM 组件夹具不可移植。** `tests/console/dom_fixture.py`（本轮新增）把浏览器路径硬编码为 `/usr/bin/chromium`、默认输出写 `/mnt/data/...`，本机两者都不存在，因此 doc §4 表里的“离线 DOM/组件工作流 11 PASS”本轮 **NOT_RUN**（既没运行也没改动该文件）。同批新增的 `tools/check_workcell_console.py` 反而支持 `--chromium`，建议夹具也接受同名参数与输出参数。

### 已存在（继承自 `7086fbd`，非本轮前端改动引入）

6. **PickPlace 诊断开关会让无关任务硬失败。** `legacy.py:300-303`：profile 带 `pickplace.audit_current_table_failures: true` 时，要求 `task=='pickplace' && mode=='sim'` 且有冻结校验，否则 `ValueError('Focused current-table diagnostics require checked frozen-world PickPlace SIM')`。本轮我第一次合并配置时带上该开关，magnetic SIM 直接被拒（`report_jimu_llm_attempt1_profile_bug.json` 里的 `error` 就是它）。开关与任务不匹配时更适合忽略+告警，而不是让无关任务失败。
7. **生成 provider 的代理处理不如旧客户端。** `rm75_app/magnetic/completion_provider.py` 的 `AnthropicJsonClient._make_client()` 虽传 `trust_env=False`，但 anthropic 1.4.0 的 `DefaultHttpxClient.__init__` 仍会无条件按环境构造代理表；本机 shell 的 `ALL_PROXY=socks://127.0.0.1:7897/` 触发 `ValueError: Unknown scheme for proxy URL URL('socks://127.0.0.1:7897/')`，随后被统一映射成不具信息量的 `RuntimeError('Anthropic-compatible design request failed or timed out')`。旧路径 `rm75_app/llm/jimu_design_llm.py:44-68` 明确剥离 `('http_proxy','https_proxy','HTTP_PROXY','HTTPS_PROXY','ALL_PROXY','all_proxy')` 并留了注释，新客户端没有。本轮按“不改仓库代码”的验收口径未修，只用启动时 `env -u ALL_PROXY -u all_proxy` 规避；建议把旧客户端的剥离逻辑搬过来，并把 SDK 构造异常单独归类，避免下一次仍要花时间定位。
8. **`geometry_ids` 重复项。** `iteration_api.py:50` 生成 `['original', *sorted(variants)]`，而 profile 的 `pusht.physics.geometry_variants` 里本身就含 `original`，于是 bootstrap 返回 `["original","original","wide"]`，PushT 几何下拉出现两个同名项。功能未受影响（本轮会话阶段按 `original` 提交成功），但会让人误以为有两个几何。
9. **全量 pytest 会污染控制台“最近记录”。** 跑完 `pytest tests` 后 `runtime_data/workcell/jobs` 多出若干假作业目录，控制台 history 会显示它们；本轮验收时先记下该现象再去读 history，避免把测试残留当成本次作业。

## 9. 未运行项与边界（NOT_RUN）

- **真机与夹爪**：全程未连接、未操作、未解锁（AGENTS 禁止）。所有请求 `mode: sim`；PushT 会话结果 `hardware_connected: false`、`model_validated_on_robot: false`。
- **GPU 随机套件 / PhysX / 相机标定 / 完整私有仓库其它测试**：NOT_RUN（本轮只按 doc §5 要求跑已知单例，没有立刻铺开大量 GPU 随机任务）。
- **离线 11 项 DOM 组件夹具**：NOT_RUN（理由见问题 5）。
- **`--doctor`、多机访问、非 loopback 暴露、CSP 在非本机浏览器上的表现**：NOT_RUN。
- **资格影响**：本轮不改变任何资格字段；`integration_qualified`、`model_validated_on_robot`、`hardware_profile_qualified` 在证据里仍是 false，UI 完成不等于硬件验收。
- 与 doc §4 的一致性：该文档自述的三项边界（`normal_browser_http_navigation` BLOCKED、`full_repository_tests` NOT_RUN_PARTIAL_SOURCE_TREE、GPU/PhysX/相机/真机 NOT_RUN）中，**第一项在本机复验为 PASS**，其余仍然成立并如实保留。

## 10. 复现与证据清单

```bash
# 三套命令（§5 原样，隔离工具）
PYTHONPATH=. PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 tools/run_network_isolated.py -- python3 -m pytest tests/console -q
python3 tools/run_network_isolated.py -- node --test --experimental-default-type=module tests/console/client_state.test.mjs
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 tools/run_network_isolated.py -- python3 -m pytest tests -q

# 控制台（loopback 例外，worker 保持隔离）
env -u ALL_PROXY -u all_proxy python3 tools/launch_workcell_console.py \
  --profile runtime_data/console_launch_20260911/console_profile_r2_nodiag.json --port 7863

# shipped 工具：只读页面+模板；单对象预览
~/.venvs/console-acceptance/bin/python tools/check_workcell_console.py \
  --url http://127.0.0.1:7863/workcell/console/ --output runtime_data/console_acceptance_20260911 \
  --chromium /home/zhangzhao/.cache/ms-playwright/chromium-1228/chrome-linux64/chrome
~/.venvs/console-acceptance/bin/python tools/check_workcell_console.py \
  --url http://127.0.0.1:7863/workcell/console/ --output runtime_data/console_acceptance_20260911_single \
  --preview-object bi --chromium /home/zhangzhao/.cache/ms-playwright/chromium-1228/chrome-linux64/chrome

# 本机浏览器驱动（真实点击；GPU 门只观测不杀进程）
~/.venvs/console-acceptance/bin/python runtime_data/console_launch_20260911/browser_acceptance.py <template|preview|stop|session> http://127.0.0.1:7863 runtime_data/console_acceptance_20260911
~/.venvs/console-acceptance/bin/python runtime_data/console_launch_20260911/browser_jimu.py llm http://127.0.0.1:7863 runtime_data/console_acceptance_20260911
~/.venvs/console-acceptance/bin/python runtime_data/console_launch_20260911/browser_sim.py runtime_data/console_launch_20260911/same_design_bundle.json http://127.0.0.1:7863 runtime_data/console_acceptance_20260911
```

| 证据 | 路径（`runtime_data/` 已 gitignore，随本文提交的是 JSON/日志，截图留在本机） |
| --- | --- |
| shipped 工具（只读 / 单对象） | `console_acceptance_20260911/report.json`、`console_acceptance_20260911_single/report.json` |
| 三套命令 | `console_acceptance_20260911/test_suites.log` |
| 模板 / 预览 / Stop / 会话 / LLM / 同设计 SIM | `report_template.json`、`report_preview.json`（旧的样例配置运行留存为 `report_template_sample_profile.json`、`report_preview_sample_profile.json`）、`report_stop.json`、`report_session.json`、`report_jimu_llm.json`、`report_jimu_sim.json` |
| 第一次 LLM 尝试（失败在案） | `report_jimu_llm_attempt1_profile_bug.json`、`llm_stage_attempt1.log` |
| 阶段日志 | `template_stage.log`、`preview_stage.log`、`stop_stage.log`、`session_stage.log`、`llm_stage.log`、`sim_stage.log` |
| 驱动脚本与启动配置 | `console_launch_20260911/{browser_acceptance.py,browser_jimu.py,browser_sim.py,console_profile_r2_nodiag.json,same_design_bundle.json}` |
| 作业原始记录 | `workcell/jobs/{2854afa6…,e6a2f633…,a438f76d…,d32249dc…,45da3178…,42b86f74…}/`（request/result/events/machine_profile） |

结论：本机复验下，控制台的真实 HTTP 导航、单/多对象预览、真实慢生成刷新恢复（单一 generation_id 与同一份 proof）、同设计原生 SIM、共享 Stop 与 PushT 暂停/移动 T/继续全部按 doc 描述工作，且没有把原生命令结束、预览或仿真成功说成真机成功；§8 的 9 条问题里只有 4 条由本轮新提交引入，其余为继承或环境问题；真机相关项保持 NOT_RUN。

OPERATOR-CONSOLE-20260911-END
