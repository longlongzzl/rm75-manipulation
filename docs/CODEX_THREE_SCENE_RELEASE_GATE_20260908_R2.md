# Three-scene release gate — 2026-09-08 R2

第 0–11 节保留 `d9f2190` 的原 R2 验收结果；用户随后澄清需要的是**仿真失败录像而非桌面相机视频**，纠正后的独立录像工作见第 12 节，不回填或改写原实验分母。

**NEEDS_REVIEW：本轮无机械臂/夹爪运动，三条 demo 尚未完成验收。** 按 [本轮审阅要求](CHATGPT_REVIEW_CODEX_PUSH_20260908.md) 顺序完成干净 HEAD 离线基线、精确资产恢复、Jimu 三个 GPU 控制实验，再做三个 current-table 请求的有限诊断。没有降低碰撞检查、减少候选/seeds、放宽成功条件或重新设计规划算法。

结果摘要：[机器可读证据及哈希](../benchmarks/unified_scenarios/three_scene_release_gate_20260908_r2_summary.json)。本轮唯一新增结果 MD 为本文；原始轨迹、关节数组、设备标识、图像/视频和大日志仅留本地。

| 链路 | 本轮实际结果 | 尚未通过的门 |
| --- | --- | --- |
| Jimu | 两个批准资产已恢复；four-wall 4/4；standard roof 7/12；原 builder 7/12 | 默认屋顶场景出现墙片搬运起点碰撞；原 builder 最终拒绝残留世界物体免检状态；不是只差真机 |
| PickPlace | 三个请求对象 0/3；含诊断复跑共 0/5 次 worker 验收通过 | gluestick 抓取 world collision；hongshupian 带载抬升 payload/base collision；tennis 释放退让接触不合格 |
| PushT | 实际工具 profile 清单检查完成，qualification_incomplete | 工具选择/实测几何/TCP/接触 link/观察标定缺失；没有继续调 synthetic fixture |
| Camera | D435 真实 RGB-D 采集；保存 60 帧、12 秒编码预览视频 | 人工看图仅 1 块橙色正方形 Jimu；多实例 RRTrack/遮挡恢复 NOT_RUN |

## 0. 基本信息

- Branch：`chatgpt/three-scene-software-closeout`。
- 起始干净 HEAD：`a62ae5b`，由 origin fast-forward 获取本轮审阅 MD；完整 SHA 见摘要 `tested_base_commit`。上轮 reviewed HEAD 为 `4c741184e0ec32595e8c08efd916cd2c8e52bb62`。
- 被测版本：上述基线 + 本提交差异。三个 Jimu GPU 实验先运行，使用原基线适配逻辑和恢复后的 snapshot；之后才加入 PickPlace 只读诊断 v1/v2。摘要 `source_sha256` 是最终导出源码，**不冒充所有较早运行的源码版本**。
- CPU Python 3.12.7；GPU `foundationpose310` Python 3.10.20 / torch 2.7.1+cu128 / 原 cuRobo1；Linux 6.8.0-137-generic，RTX 5060 Ti，driver 580.173.02。
- ManiSkill/SAPIEN、cuRobo 和外部安装模型沿用现有环境，未安装/升级/改写。cuRobo2 本轮 GPU NOT_RUN。
- `execute_real=false`；RealMan SDK connection / Robot IP used：NONE。相机实际连接采集与机械臂连接分开记录。
- 本地证据根目录（下文记作 R2）：`runtime_data/three_scene/release_gate_20260908_r2/`。实际 worker 日志在 `runtime_data/workcell/jobs/`，可由各本地 `result.json` 追溯；不上传运行 ID 或关节日志。

## 1. 旧仓库 dirty worktree 审计

- Old repo：`/home/zhangzhao/Desktop/lerobot`；HEAD `36798efbd12814841607951c9af470b309b34fd3`。
- 固定迁移基线：`7aaff9da22486b7d25557b3795dd258f9b65f10d`。
- 只读 status 审计使用 `git status --porcelain=v1 -z` 的原始字节，前后 SHA256 均为 `15fd1ac40352579a761b4134244c2a74158dd723b2b146c6a39ccf458fe20968`（不是换行分隔的 `--short` 文本哈希）。**没有 reset、clean、覆盖或修改旧文件；没有整体复制 untracked。**
- 原单一 overlay 的 12 项批准路径/内容 SHA 保持原批准值；只追加本轮明确批准的两个模型。新 status hash 不是对所有 dirty 内容的重新批准。

发现一个必须单独报告的已审阅直接依赖漂移：`lerobot/common/robots/realman_lerobot/realman_arm.py`。

| 内容 | SHA256 | 处理 |
| --- | --- | --- |
| 先前批准并已迁移的原始字节 | `87fa8a423ee93f0a96d41cbadb21318c3fa90f759e150d154a66bd290f000e2f` | 从新仓库内可恢复 snapshot 备份读取，逐字节匹配批准 SHA 后继续保留 |
| 当前旧 dirty 文件 | `ab14203dd87a8fb54816c2d24a73f15529bf5ab6d8da29813e1d7535b0c60ab6` | 未纳入；旧文件保留原样，等待另行审阅 |

新漂移分类为 **unknown**，不是已证明的 `candidate_final_fix`：包括 UDP 单调时间/陈旧反馈处理、有限值检查、`sendall`/TCP_NODELAY、网络接口与 UDP 路由选择、实时反馈设置返回值检查等硬件通信变化；另有 `STACKCUBE_GRIPPER_RELEASE_FIX` 及打开夹爪 windup 处理，属于单独 StackCube 实验性变化，同样未自动批准。不能因其可能有用就混入本轮两模型恢复。

其余 11 个原批准 overlay 文件仍匹配原 SHA。为不采用新 dirty 代码，也不丢失先前批准版本，迁移工具增加显式 `--approved-archive`：只接受新目标仓库内、旧源仓库外的档案，且必须匹配**批准的原始 SHA**；不反向替换路径字符串，不容许 hash 不符、symlink/path escape 或过大文件。档案不是额外 overlay。

## 2. 迁移完整性

批准新增范围仅为：

| Snapshot 相对路径 | Bytes | SHA256 |
| --- | ---: | --- |
| `Demo_Triangle/red_triangle_74x135x6p5.glb` | 1493004 | `fac68dee16a4f8d5b17d7de0f4007ac2445472c70d338c46dbb98908e38f4175` |
| `Demo_Triangle/red_triangle_74x135x6p5.glb.coacd.ply` | 8146 | `f6d655ec58ef7db661241935c6d7f9ee3d0ac35c6807e0d2d1b57f9772582a12` |

先将新仓库的生成 snapshot 移至 R2 下 `snapshot-backup.HNDIbt/working_snapshot`，可恢复、未删除；随后执行：

```bash
GIT_OPTIONAL_LOCKS=0 PYTHONPATH=. python tools/migrate_working_sources.py \
  --source-repo /home/zhangzhao/Desktop/lerobot --target-repo . \
  --source-ref 7aaff9da22486b7d25557b3795dd258f9b65f10d
GIT_OPTIONAL_LOCKS=0 PYTHONPATH=. python tools/apply_audited_worktree_overlay.py \
  --source-repo /home/zhangzhao/Desktop/lerobot --target-repo . \
  --manifest configs/workcell/approved_worktree_overlay_20260907.json \
  --approved-archive runtime_data/three_scene/release_gate_20260908_r2/snapshot-backup.HNDIbt/working_snapshot
PYTHONPATH=. python tools/migrate_working_sources.py --target-repo . --verify-only
```

- 固定导出 803 项 + 单一完整 overlay 14 项（8 替换、6 新增）= **809 个 manifest 文件**。没有叠加两个 overlay。
- 恢复前的 807 个 installed 文件 SHA **全部不变**，仅新增上述 2 个模型，无删除；snapshot 源码/算法没有改写。
- 六个入口的 fixed blob 检查 PASS；snapshot manifest verify 在恢复前、恢复后和 Jimu 后均 PASS。
- Manifest：`rm75_app/_vendor/working_snapshot/MIGRATION_MANIFEST.json`，源固定提交仍为 `7aaff9d`，按项记录 `byte_source`；只有上述 realman_arm 来自 approved archive。
- `runtime_gpu_verified=false`、`dependency_completeness_verified=false` 保留；局部实验不能认证全部运行依赖。
- 旧仓库 status、原 builder/fixed scene/manifest 与 start 文档前后不变。日志：R2 的 `migrate_baseline.log`、`migrate_overlay.log`、`snapshot_restored_verify.log`、`snapshot_after_jimu_verify.log`。

## 3. 代码完整性与 CPU 测试

| 阶段 | 命令/范围 | 实际结果 |
| --- | --- | --- |
| 干净 `a62ae5b` 基线 | `PYTHONPATH=. python -m compileall -q rm75_app tools tests` | PASS |
| 干净基线 | `PYTHONPATH=. python -m pytest -q tests` | 878 passed，34.13 s |
| 迁移后 | 同一 compileall，包含 vendored snapshot | PASS；原第三方 invalid-escape 警告保留 |
| 迁移专项 | approved archive / overlay / triangle asset audit | 30 passed |
| 迁移后完整 three_scene | `PYTHONPATH=. python -m pytest -q tests/three_scene` | 566 passed |
| 最终完整 three_scene | 同上 | 579 passed，16.43 s |
| 最终完整 tests | `PYTHONPATH=. python -m pytest -q tests` | **901 passed** |
| 最终 compileall | 同上，覆盖应用、工具、测试和 snapshot | PASS |

最新离线结果及 log SHA 见摘要 `checks`；早期 900 项结果与所有诊断失败日志保留，不替换成最终数字。最终 901 比干净基线增加 23 项测试；无 pytest failure/skip，1 条原 trimesh 弃用 warning。

新增测试覆盖批准档案 SHA/边界拒绝、SIM/指定对象诊断约束、原结果/碰撞状态不变、接触局部单调性指标、失败分母和隐私字段导出、人工相机计数与 RRTrack 结论分离。没有修改测试成功阈值掩盖 GPU 失败。

两个原无运动 import/parser contract 均 PASS：`check_working_entrypoints.py --task pickplace|magnetic`，安全 machine 模板及对应 preview 示例，`missing_required_flags=[]`，`native_main_called=false / planner_called=false / robot_control_called=false`。该项不等于 GPU 或 SDK 通过。

## 4. 前端

本轮范围集中在指定 GPU 恢复/诊断和物理 profile 清单，没有开浏览器重新操作三场景页面。完整 three_scene 的已有 frontend/CLI/overlay/preview/status/prompt/Stop 离线测试已复跑通过，但不冒充真实页面操作。

| 场景 | Page / Preview / Status / Stop 实际浏览器操作 | 其他 |
| --- | --- | --- |
| PickPlace | NOT_RUN | 原输入 prompt bridge：本轮浏览器 NOT_RUN |
| Magnetic | NOT_RUN | `jimu_builder_scene_v1` 导入、round-trip 与 prompt：CPU suite 覆盖，本轮浏览器 NOT_RUN |
| PushT | NOT_RUN | Sim request / live status：本轮实际前端 NOT_RUN |

## 5. PickPlace 回归

仅使用 `assets/test_scenes/current_table.json`，SHA256 `e5dd3154bdb6b509791861478170ffc5f0b6e2c8f08ea89b9b7a5111ee2fd908`；每次是独立完整 frozen-world reset，**不是依次取走物体的清桌实验**。9 个世界对象为 `bi / bitong / carriot / desk / gluestick / hongshupian / lvmukuai / shuazi / tennis`，活动源之外 8 个对象保留；源身份、foreground、transport、release-clearance 仍由实际 worker 验收。

生产 `lazy_place + primary_only`、候选、seeds、world/self/payload、已有接触范围、near-IK guard、缓存策略均不变。没有扫描 yaw/swap 或另外扩大物体矩阵；它们的历史失败保留、本轮 NOT_RUN。

| 本地 run | 耗时 s | Worker 验收 | 请求对象首个失败阶段 |
| --- | ---: | --- | --- |
| `pickplace_before_gluestick` | 24.251 | FAIL | pregrasp 5/13，grasp 0/13；未到 lift，原 fallback 的 bi 成功不计入 |
| `pickplace_before_hongshupian` | 46.129 | FAIL | 仍 attached 的 TCP-up lift endpoint IK；fallback bi 不计入 |
| `pickplace_before_tennis` | 23.974 | FAIL | post-place clearance；native final=True 不计入通过 |
| `pickplace_diagnostic_v2_gluestick` | 23.861 | FAIL | 同一 grasp 批次，增加原返回行/碰撞只读证据 |
| `pickplace_diagnostic_v2_tennis` | 16.792 | FAIL | 同一 clearance，增加相邻样本接触变化度量 |

五次均为 `WorkcellService → worker → 原 cuRobo1 native SIM`，全部 exit 42 / `frozen_world_validation_failed`。整体 **0/5 worker 请求验收成功，三个选定对象 0/3**；每次原 native 都可能产生 successful cycle/final=True，不能用这些替代请求对象端到端完成。后台预取来源在摘要中单列，未当作前台成功。

### Gluestick：本轮先失败在 grasp，不沿用旧 lift 结论

v1 原 lift 观测没有触发，因为阶段未到达；v2 仅观察原 `winner_chain_ik_preselect_grasp_contact` 的 **13 goals、原 requested 32 seeds**，0/13 success，13 个最近返回配置均为 world collision；观测完整、碰撞状态和原返回行不变、无新增求解。

前 5 个低位置误差配置为 21.12–33.10 µm，`gripper_Right_Support_Link` 与 `scene_obstacle_shuazi` 的启用 box 模型分别重叠 **5.573 / 6.535 / 8.454 / 10.381 / 12.303 mm**，没有 self pair。其余 8 个最近配置误差更大（约 8–14 mm），不能据此证明所有精确目标全局不可达。

证据指向当前 frozen 模型中的邻近刷子 world 阻挡，不是“再加 IK seeds”或已证明的实物标定误差。**没有把刷子 1.125 scale 缩小，没有挪物体或改抓取目标。** 与旧实验失败阶段不同的事实保留；不未经对照就把整个结果变化归因于三角资产。

### Hongshupian：带载目标与 robot base 自碰撞模型冲突

首个失败为 `joint_start_tcp_up_lift_grasp_direct_top_bias_center_vertical_fast_ik_straight_ik`。原 64 seeds / 64 返回行，native success=0，start 有效、payload 仍 attached（6 spheres）。低误差行约 6.9 µm 仍被 native self-collision 拒绝。

精确未改 lift 目标的必要几何检查得到 `attached_object` 对机器人 **`base_link`（不是 gripper_base_link）** 两处重叠 **28.974682 / 8.170788 mm**。原 base self buffer 40 mm 保留；诊断 state_unchanged/complete=true。未得到真实测量证明该模型错误，因此没有缩 sphere/buffer、豁免 payload/base 或改 lift 目标。

### Tennis：已有 reverse 并不满足逐步不加深接触

- 只读取原程序已经生成、被 endpoint world check 拒绝的 reverse-incoming 路径：7 点 → 原密度 25 审计样本。保持 full world/self/non-target 检查，目标接触仍仅限左右 finger link；`selected=false / new_solver_calls=0 / state_unchanged=true`。
- 现有 full-path gate PASS 的含义是“不超过**初始**接触深度 + 原 1 µm 数值余量”，**并不证明相邻样本单调递减**。v1 缺局部指标，未据此采纳。
- v2 实測：初始 4.987516 mm → 末端 1.007717 mm，25/25 样本仍接触；最大相邻加深 **0.031243 mm**，`monotonic_contact_nonincreasing=false`。因而不符合本轮审阅允许选择的单调退路，而且末端也不是完全脱离。
- 原 fresh 20 mm / 3 点退让仍在第 1 个审计样本由 4.987516 加深至 5.123030 mm（**+0.135514 mm**）被拒。原 planning profile 的 `post_place_clearance` 记录 `retreat_candidate_count=1 / success=false / status=PLAN_FAIL / path_waypoints=0`，没有现成另一个安全备选可替换。
- 仅为 `audit_release_path` 返回值增加局部变化**指标**，接受条件完全未变；未改原 endpoint 判断、候选选择、容差、世界免检或轨迹。没有把旧 initial-bound gate 的通过误报为新审阅的安全单调性通过。

**本轮没有证据支持安全的 PickPlace 规划/模型修复，因此没有实施选择逻辑或几何改动。** “修复后受影响请求复验”NOT_RUN/不适用；以上两次 v2 是诊断复验，不能标为修复后的 PASS。需要真实测量/审阅后再决定下一步。

复跑命令（分别以三个指定 source 和表中独立 output 运行）：

```bash
python tools/run_workcell_native_validation.py --task pickplace \
  --object-name gluestick --fixed-world assets/test_scenes/current_table.json \
  --audit-current-table-failures --extensions runtime_data/curobo_native_extensions \
  --output runtime_data/three_scene/release_gate_20260908_r2/pickplace_before_gluestick --timeout-s 600
```

原始本地每次 `result.json` 可追溯 worker stdout/events；摘要只导出碰撞对象/link 名、计数、标量误差/重叠和 SHA，不导出 raw q/FK/世界位姿。

## 6. Magnetic / Jimu 回归

### 恢复模型后的等价性

只读 `audit_jimu_triangle_assets.py` 再次比较旧工作目录与新 snapshot，`compared_asset_selection_equivalent=true`。两个环境现在均选中批准主模型：**74.000 × 6.500 × 135.000 mm（X/Y/Z）、scale=1、local tip +Z**，builder tip 与上方向 dot=1；四个屋顶原 nominal edge gap 都为 **4 mm**。旧 fallback 约 120 mm、builder local tip −Z、gap 11.5 mm 的历史差异已消除，没有修改 builder 轴或目标。

原 COACD 字节保留，边界约 75.956 × 8.975 × 136.850 mm，未缩小/重新生成。其 source MD5 匹配；完整 native cache reuse 资格仍未单独证明。几何等价不等于实物装配公差合格。

### 按规定顺序完成三个 GPU 实验

| Run（按执行顺序） | 任务完成数 | 保留的 cycle marker 分母 | 最终状态 / 耗时 | 释放/返回审计 |
| --- | --- | --- | --- | --- |
| `four_wall` | **4/4** | 4 true + 0 false | final=True，exit 0，63.279 s | 4/4 PASS，clearance 218 + return 617 样本 |
| `standard_roof` | **7/12，FAIL** | 7 true + 5 false = 12 | final=False，exit 42，348.403 s | 已完成部分 7/7 PASS，383 + 1184 样本；不是全任务通过 |
| `original_builder` | **7/12，FAIL** | 7 true + 7 false = 14；另保留终端安全异常 | 无 final marker（null），exit 42，498.781 s | 已完成部分 7/7 PASS，379 + 1080 样本；不是全任务通过 |

搬运审计采样分别 **176 / 292 / 413**，所有已观测 transport `world_exempt_links=[]`；所有已观测 return 免检集合也为空。统计可能包括预取/重试，不能转换为执行物体数量。原 role/dependency、同类 source retry、partial/full-open/retreat、attached payload 和 near-IK guard 保留；未加新 collision allowance。

标准场景首个中间失败在 cycle 4 的 front_wall after-lift transport 起点 `INVALID_START_STATE_WORLD_COLLISION`；原重试后该墙完成，但最终 cycle 8 的 front_second_wall 同类失败，**未到 roof batch（0 个）**。原 builder 同样出现墙片 after-lift transport 失败。

现有只读 native world 诊断的首个非零 link 为 `gripper_Right_Support_Link`，无禁用 link；标准场景原 GPU 标量约 5.452264、builder 约 5.511379。这些是 **native cost 值，不是米或穿透深度**。启用 cuboid 的解析负间隙列表为空；确切 mesh 障碍配对没有资格化，不能编造“已证明是哪块 mesh”，也不能据此放开搬运。

### 原 builder 右屋顶改善及终端停止原因

输入为原只读 `Beta_demo-codex-v0.9/jimu_tasks/tag1_standard_three_layer` manifest / builder / fixed scene（9 固定 + 12 移动），使用原 `JIMU_BUILDER_JSON_EXECUTION.md` 记载的 SIM start。两个输入不变标志均 true；文档没有作为脚本执行，没有迁移整个任务目录。

原程序在墙片失败重试后实际观测到 `right_roof_triangle`：

| 指标 | 历史 fallback 模型 | 本轮批准 135 mm 模型 |
| --- | --- | --- |
| 原 paired place IK 成功 goal rows | 0/384 | **240/384**（不是 240 个完整抓放配对/成功件） |
| 原 pregrasp / grasp | 见历史审计 | 14/17 / 14/17 |
| 192 个名义 hover 的 gripper_base / box 重叠 | 172/192 | **0/192** |
| 192 个名义 release 的 gripper_base / box 重叠 | 192/192 | **0/192** |

对照来源：[原屋顶 IK 审计](CODEX_THREE_SCENE_JIMU_ROOF_IK_20260908.md)。本轮 3 个右屋顶 foreground batch 完整且 state_unchanged，另外三块屋顶的 batch 未观测，`roof_ik_audit.passed=false`。这支持恢复资产确实解除**已观测右屋顶**的旧名义底座/墙体几何障碍，不能宣称四屋顶都已修好或 full builder 12/12。

最终 worker 拒绝：`failure_code=strict_contact_not_supported`，event `transport_policy_rejected`，reason **`preexisting_disabled_world_objects`**，step `grasp_direct_prez_00_120mm_final_approach_gripper_world_relaxed`。残留 disabled objects 为 `active_target_object / scene_obstacle_right_roof_triangle / virtual_table_plane`，disabled links 为空。**门禁保持拒绝，没有把桌面或其他 world object 加入搬运免检。** 后续应审查原阶段切换的禁用状态生命周期；本轮未贸然重新启用/豁免/修改 roof 算法来通过。

历史标准控制曾 12/12、原 full builder 8/12；本轮实际为 7/12 / 7/12，不能用“几何恢复有改善”覆盖这两个退化结果。three-real-demo release 仍 FAIL。

GPU 命令主体：

```bash
python tools/run_native_contact_audit.py --scene four-wall \
  --transport-world-checked-compatibility --extensions runtime_data/curobo_native_extensions \
  --output runtime_data/three_scene/release_gate_20260908_r2/four_wall
python tools/run_native_contact_audit.py --scene triangle-roof --audit-roof-ik \
  --transport-world-checked-compatibility --extensions runtime_data/curobo_native_extensions \
  --output runtime_data/three_scene/release_gate_20260908_r2/standard_roof
python tools/run_workcell_native_validation.py --task magnetic \
  --task-dir /home/zhangzhao/Desktop/lerobot/Beta_demo-codex-v0.9/jimu_tasks/tag1_standard_three_layer \
  --jimu-start-command-doc /home/zhangzhao/Desktop/lerobot/Beta_demo-codex-v0.9/JIMU_BUILDER_JSON_EXECUTION.md \
  --audit-roof-ik --extensions runtime_data/curobo_native_extensions \
  --output runtime_data/three_scene/release_gate_20260908_r2/original_builder --timeout-s 900
```

上述三个及第 5 节五个 GPU 命令均**串行**使用 foundationpose310 Python，以 `systemd-run --user --scope -p MemoryMax=9G -p MemorySwapMax=512M` 限内存，环境 `PYTHONPATH=<repo> / PYTHONDONTWRITEBYTECODE=1 / PYTHONUNBUFFERED=1 / OMP_NUM_THREADS=2 / MAX_JOBS=2 / TORCH_EXTENSIONS_DIR=<repo>/runtime_data/curobo_native_extensions`；两个控制 runner 外层 timeout 900 s。没有 `--execute-real`。原 native SIM 的 auto-execute 不是机械臂执行。

完整终态模块检查：four-wall、standard roof、五个 PickPlace 请求 `loaded_mplib_modules=[]`；原 builder 安全异常提前返回，最终 census 为 null/未取到，不补造空集合。本轮未恢复或调用 MPLib 后端。

## 7. PushT：停止 fixture 调参，转向实际 profile

已向用户询问“闭合夹爪还是专用/打印推头”及真实尺寸/TCP/接触/标定资料，尚未收到选择或实测值。只读检查现有 configs/examples/calibration 后没有得到可认证的实际工具 profile。没有拿模板数值假装硬件测量。

执行 `tools/check_pusht_chain.py --profile R2/machine.json --output R2/pusht_physical_profile`，使用本轮生成的**未认证模板**：exit 2，`status=qualification_incomplete`，`hardware_profile_qualified=false`，hardware_reviewed/integration_qualified/motion_authorized/planning_inputs_complete 均 false；缺 observation/joints/goal。

| 必须确认的 profile 字段 | 当前证据/缺项 |
| --- | --- |
| `pusht.motion.tool_frame` | 模板为 gripper_tcp；未确认真实工具选择，不是 measured qualification |
| `pusht.motion.push_tcp_z_m` | 缺实测值 |
| `pusht.motion.tool_quaternion_wxyz` | 缺实测姿态 |
| `pusht.motion.pusher_contact_links` | 缺真实接触 link 映射 |
| `pusht.motion.tool_collision_geometry_verified` | 未认证 |
| `pusht.motion.object_centroid_z_m` | 缺实际物体质心高度 |
| `pusht.motion.object_height_m` | 缺实际物体高度 |
| `pusht.motion.static_collision_objects` | 缺实测静态世界 |
| `pusht.observer.kind` | 模板为 realsense_apriltag，不等于用户已选此生产输入 |
| 该模板 observer 的 `T_base_camera / T_marker_object / marker_size_m` | 均缺合格标定；若选别的 observer，按其协议重新验证 |
| 实际 observation / current joints / push goal | 均缺本轮真实输入 |

校验器共 10 个字段错误，具体内容保留在摘要；模板合法的字段也不自动变为真实工具事实。没有把用于 frozen SIM 的相机外参直接冒认为 PushT 的当前标定。

本轮 actual GPU chains / CPU 实际输入规划 / TCP corridor / collision / speed timing / stale-replay / object-moved reobserve：**NOT_RUN**（已有离线保护测试复跑）。历史审阅认可的 3 条五阶段 GPU 链、两种方向失败、阻挡负例和 30/30 precondition 拒绝仅为历史证据，不加入本轮分母；没有再次移动 contact/standoff 或缩几何调 authored fixture。

## 8. Camera / tracking 与供用户查看的视频

- D435 可用性探测：1 台；不导出 serial。先探测的 `camera_stream_started=false` 与随后真实采集是不同阶段，摘要明确分开记录。
- 使用原 `capture_rrtrack_rgbd.py` 采集 1 帧 overview，再采集 60 帧 RGB-D 预览；没有启动 RRTrack solver，也没有 AprilTag 隐藏真值。
- 人工检查 overview 与视频末帧，只看到 **1 块橙色正方形薄片 Jimu**，未具备同时至少 4 块同类物体条件；没有安排受控遮挡。不能把 60 帧采集计为 60 个 accepted track。
- 多实例 ID 稳定性/交换检测、per-instance confidence、LOST/reappearance、同一 planning base frame 变换与 pose spread：全部 **NOT_RUN**；tagless-ready=false。
- 新视频为本地 `R2/camera_review_clip_5fps.mp4`，H.264 / 640×480 / 5 fps / 60 frames / **12.000 s 编码时长**，198887 bytes，SHA256 `3b1621897259ca83aa16eedc2b381ba5b55ba0d032ed909022954c6a955f1f9b`。ffprobe 与全片 ffmpeg 解码检查 PASS。
- 采集 host record 时间跨度 13.075445 s，device timestamp 跨度 13073.099 ms；5 fps 为编码速度，不是认证的曝光时间/相机 freshness。没有用 host 写入时间证明跟踪延迟。
- **这是真实桌面/夹爪静态预览，不是上述规划失败的仿真动画，也不是机械臂任务成功录像。** 本轮 render_mode=none，没有生成失败回放 MP4。用户可先看此视频确认桌面布置；raw RGB/depth/capture metadata 与视频全部不上传 Git。
- observe → push → fresh observe 物理闭环：NOT_RUN。

## 9. RealMan no-motion

SDK connection/preflight、关节反馈/顺序/单位、真实 Stop API、夹爪后端读写语义、实际 API/signature 调用：全部 **NOT_RUN**。没有真实机器人连接，没有发机械臂/夹爪命令，没有由本轮程序启动的机械臂/夹爪运动。相机采集不是机器人 SDK 验证；SIM 成功也不是硬件 permission。

## 10. Physical motion ladder

| 项目 | 状态 |
| --- | --- |
| Reduced-speed free-space trajectory | NOT_RUN |
| Isolated gripper | NOT_RUN |
| One PickPlace atom / multi-object PickPlace | NOT_RUN / NOT_RUN |
| One Magnetic piece / 2/4/6+ structure | NOT_RUN / NOT_RUN |
| One PushT short push | NOT_RUN |
| PushT push → fresh observation | NOT_RUN |
| PushT closed-loop multi-step | NOT_RUN |

本轮在无运动证据提交后停止；不自动进入 SDK ladder、自由空间运动或对象任务。

## 11. 总结、提交范围与待审阅事项

- PickPlace：三种当前桌面请求仍 FAIL；只增加被限定于本轮三对象 SIM 的原候选只读观测和接触度量，没有未经证据的规划“修复”。
- Magnetic：两模型漏迁移已恢复并 SHA 验证；右屋顶原 IK/名义几何实测改善，但 standard/full builder 均 7/12，仍需查搬运起点 world collision 与禁用世界状态生命周期；不是已发布可真机 demo。
- PushT：不再调 synthetic fixture；需要用户明确工具并提供测量/标定，profile 保持未认证。
- Camera：真实预览视频已保存，四块同类 Jimu 身份测试条件不足，不能补造 tracking 通过。
- 代码/配置提交范围：单一 overlay、snapshot manifest、两个批准模型；`apply_audited_worktree_overlay.py`；`legacy.py` 的 opt-in 诊断入口、`pickplace_focused_diagnostics.py`、`pickplace_release_contact.py` 的指标、native validation CLI；对应 tests；有界摘要导出工具/测试；本文和一个 summary JSON。
- 未改：旧仓库、snapshot 原 807 项已安装字节、外部 cuRobo/模型、三场景规划算法、碰撞容差/候选/seeds、真机授权配置。原始日志、关节、设备标识和视频未纳入提交。
- 本轮提交以 `Restore approved Jimu assets and record strict no-motion release gates` 为主题；本文随代码/摘要一起提交并 push 到本分支，最终 SHA 与 push 结果在交付消息给出，避免自引用 commit hash。

待 ChatGPT/用户审阅：

1. Jimu 新 first-failure 及 residual world-disabled 生命周期如何在保持搬运全检查下修复；不得直接豁免桌面。
2. Gluestick 邻近刷子 collider 与 hongshupian payload/base 是否有可提供的真实尺寸/标定证据；没有之前不缩模型。
3. Tennis 已有 reverse 不满足单调性、fresh 失败；若继续，需要单独审阅安全的后续路径方案，不能把容差放宽作为修复。
4. PushT 最终使用闭合夹爪还是专用推头，以及对应测量/观察 profile；多实例 Jimu 测试需在 D435 视野中放至少 4 块同类片并安排一次临时遮挡。

## 12. 用户澄清后的仿真失败录像（非新的发布验收）

先前保存桌面视频是对用户意思的误解，不能帮助检查上述仿真失败。本节从 `d9f21906e4b5b528c32b492c8053d32d53fca8fd` + 新的显式录制开关出发，交付 PickPlace/Jimu native SIM 画面，以及 PushT 已保存 GPU 工具碰撞证据的离线可视化。三者表示方式不同，不能统称完整执行轨迹。**没有再次采集真实相机，没有机械臂/夹爪或 SDK 操作。**

### 实际交付视频

视频根目录（V）：`runtime_data/three_scene/sim_failure_video_20260908/`，仅本地保存，不上传 Git。

| 视频 | 已核验规格 | 重点位置 | 真实含义 |
| --- | --- | --- | --- |
| `V/tennis_failure.mp4` | H.264，1024×600，10 fps，224 帧，22.4 s | 18.4–22.4 s | current-table Tennis 原 SIM 的放置与释放后退让拒绝；原 motion-window 之外有端点跳转，不是完整物理连续录像 |
| `V/jimu_failure.mp4` | H.264，1024×600，10 fps，906 帧，90.6 s | 84.6–90.6 s；最后 4 s 为静态候选 | 原 builder 已接受路径的运动学回放，以及 front_wall 搬运起点被拒时的候选静态检查 |
| `V/pusht_failure.mp4` | H.264，1280×600，10 fps，248 帧，24.8 s | 8–12 s：baseline；20.8–24.8 s：rotated_orthogonal | 两个历史 synthetic 失败案例的原碰撞球与名义 TCP 采样可视化；不是已通过 IK 的机械臂轨迹 |

三片均通过 ffprobe 与整片 `ffmpeg -v error -i <video> -f null -` 解码验证；人工查看了 Tennis 18/20 s、Jimu 30/89 s，以及 PushT 两个首次碰撞采样的关键帧。Tennis/Jimu 保留全景与 active-object 近景；PushT 保留斜视和侧面细节。字幕来自对应原始证据，不是肉眼测量。

- Tennis：仍为 worker FAIL / `frozen_world_validation_failed`。实际原退让检查在样本 1 的接触深度由 `0.004987516 m` 加深到 `0.005123030 m`，即约 **0.135514 mm**。被拒退让没有执行；视频末尾显示最后接受的 SIM 状态，不能从 native `final=True` 推导任务成功。
- Jimu：到第 4 件 `front_wall`，原 after-lift transport 起点为 `INVALID_START_STATE_WORLD_COLLISION`。首个只读 native 非零 link 是 `gripper_Right_Support_Link`，cost 约 5.511379，**不是米**；确切 mesh 障碍配对仍未证实。记录诊断后受控取消，不继续完整 builder，也不把 3 个成功 cycle marker 写成 3/3 验收。
- Jimu 最后 4 秒是**静态被拒机器人姿态**：来自原 `diagnosed_q`，只临时设置虚拟 articulation 的 qpos 用于渲染；没有路径插值/物理步进/solver/执行调用，随后恢复原 qpos/qvel。场景物体保持当时 SIM 位姿，**没有伪造目标已经被抓住后的物体姿态**。此视图仅供看姿态/邻接关系，不能认证精确碰撞配对或 payload 动态。
- GPU 计算等待被省略；固定 10 fps 和原路径播放（部分被放慢）不是 wall-clock、实时跟踪、碰撞连续性或物理时序资格化。

### 录制尝试全部保留

| 本地 run | Worker / 运行时长 | 录像结果 |
| --- | --- | --- |
| `tennis_v1` | FAIL，41.066 s；请求源仍失败 | 224 帧，完整录制元数据，无录制错误；只录 requested tennis，不录 fallback/后台预取 |
| `jimu_first_failure_v1` | 诊断后取消，81.468 s；2 个成功 cycle marker | 第一版只截到阶段状态，85 帧/8.5 s；取消前未完成 recorder 元数据封装，不作为连续失败过程交付 |
| `jimu_first_failure_v2` | 诊断后取消，130.737 s；3 个成功 marker | 866 帧/86.6 s；补录原安全门已接受的路径，失败候选未显示，保留本地 |
| `jimu_first_failure_v3` | 诊断后取消，132.937 s；3 个成功 marker | 最终 906 帧/90.6 s，完整元数据、无录制错误；额外 40 帧静态被拒姿态，作为交付版本 |

三个 Jimu 运行均是预先声明的**首次诊断限次录制**，不是完整 12 件任务的新通过/失败矩阵；录像播放时序也不同于 R2 benchmark。原任务三文件和 start 文档前后 unchanged=true。原 7/12 发布阻塞以及所有中间失败保留。

本节没有重跑 gluestick/hongshupian 或 PushT GPU 规划。前两者的原始失败证据仍在第 5 节；PushT 继续停在真实工具 profile 缺失，下面只是回放已经存在的失败几何证据，不是新增完整链验收。

### PushT 补录：原碰撞证据，不补造失败轨迹

用户追问 PushT 为什么没有视频后补齐。此前按审阅停止 synthetic 调参、没有新的 PushT 运行，不能成为遗漏已有失败证据可视化的理由。但原失败 result 没有保存完整关节路径，不能生成假冒的整臂运动。

- 输入为 `runtime_data/three_scene/pickplace_pusht_followup_20260908/gpu_{baseline,rotated_orthogonal}/input.json`、同目录原 `result.json`，以及 `runtime_data/three_scene/pusht_envelope_20260908/envelope_v1/{baseline,rotated_orthogonal}.json`。来源和历史 GPU 查询边界见 [工具包络诊断](CODEX_THREE_SCENE_PUSHT_ENVELOPE_20260908.md)。
- `tools/render_pusht_failure_video.py` 核对原 input SHA、case/motion/config/push、no-hardware/未认证标志、原 GPU 诊断完整性和失败采样。使用原 `_scene` 构造全部 cuboid，再用原球体半径/偏置逐样本复算 sphere-box contacts，要求与已保存 GPU/CPU 一致证据完全匹配。
- SAPIEN 只渲染原 **38 个夹爪碰撞球**、T 两个 cuboid 和原桌面。没有 arm mesh/关节解，不能据图认证 arm/self/IK；没有 physics step、求解器调用、真实设备或新 GPU 规划。没有更改 gap、接触点、姿态、半径、世界物体或允许接触阶段。
- 每个原名义 TCP 采样显示 0.8 s，首个失败点停留 4 s，之后停止；不插值/外推，也不把离散采样称作实时轨迹。片中持续标注 `HISTORICAL SYNTHETIC FIXTURE`、`NOT an IK / executed robot trajectory`、`hardware profile NOT qualified`；蓝球为原碰撞模型，红球为该点与原世界实际重叠的球。
- baseline 首次 overlap **10/16**：`gripper_Right_Support_Link / right_pad` ↔ `pusht_target_0`，最大 **4.800047 mm**；rotated_orthogonal 首次 **11/16**：`gripper_Left_Support_Link` ↔ 同一目标 collider，最大 **3.049097 mm**。各自与原 GPU `cartesian_ik_failed` 采样编号相符，不代表真实夹爪已经发生接触。
- 保留所有渲染尝试：baseline v1 在相机姿态类型接口处报错，未生成 MP4；baseline v2 / rotated_orthogonal v1 完整但顶视被底座球遮挡，改用侧面相机后交付 baseline v3（120 帧）与 rotated_orthogonal v2（128 帧），合并为上述 24.8 s 视频。只是渲染视角修正，没有重算/修改 fixture，失败结果和历史规划分母不变。

复现方式：对上述每个 case 调用 `python tools/render_pusht_failure_video.py --input <原 input.json> --envelope <原 envelope.json> --result <原 result.json> --output <新的本地目录>`，使用 foundationpose310/SAPIEN，9G/512M 内存限制、OMP_NUM_THREADS=2，串行运行。输出目录必须不存在；不覆盖原始证据。

### 实现、复跑与验证

- `tools/run_workcell_native_validation.py --record-sim-video` 是显式选择，默认关闭；只允许 frozen PickPlace/Jimu SIM。原 argv 仅增 `--dry-run-motion-window-scale 1.0`；候选、seeds、成功条件、world/self/payload、原任务几何和真机权限不变。
- 使用第 5/6 节的相同 frozen 输入和 GPU 内存限制，分别追加 `--record-sim-video`；Jimu 追加 `--stop-after-collision-diagnostic`，输出为表中独立目录。最后两次 Jimu 在录制元数据封装后才执行已声明的 SIM 取消，避免把半封装文件当成成片。
- `sim_failure_video.py` 捕获原 direct motion-window 帧；Jimu 保留其独立原执行入口，在原安全门返回成功且原终点匹配时，使用同一 native player 做已接受路径的运动学渲染，恢复原终点。不是新的轨迹规划或第二次物理测试，片中标为 `accepted-path replay`。
- 静态被拒配置检查不调用会同步 planner/attachments 的 native helper，只设置并恢复虚拟 articulation；从未执行被拒路径。没有隐藏世界碰撞体、缩小几何或更改任何碰撞规则来让片子好看。
- 默认不录制、SIM-only/固定输入拒绝、排除预取/替代源、原返回值/失败不变、只有原成功路径可回放、静态 qpos/qvel 恢复均有单测。
- 补 PushT 后最终 `compileall` PASS；完整 `tests/three_scene` **602 passed，17.62 s**；完整 `tests` **924 passed，36.55 s**，1 条原 trimesh warning。新增 14 项 PushT 视频契约测试覆盖原几何/输入不变、禁止 live/硬件/未验证证据、拒绝不匹配碰撞/失败编号、停止于首次失败且不补造轨迹。较早的 907/909/910 项测试日志保留；首次新增单测因 fixture 的 tuple/JSON list 比较不一致为 1 failed/13 passed，修正测试序列化后 14 passed，未放宽实际输入检查。
- snapshot verify **809 项 PASS**。本轮对旧仓库执行的操作是只读；再次 status 审计 SHA 为 `50c216317a3a41b8eeb023f0a1911d552a2b6ba412ee2489c348f3000e1cc2f5`，与第 1 节原 R2 的 `15fd1a…` 不同，不能声称本节前后全仓状态相同。当前发现旧 gripper 模型目录下 `docs/asserts/001.png / 002.png / 003.png` 为 untracked（文件 mtime 均为 2026-01-31）；仅去除该目录的 status 记录也不能恢复旧 hash。本节未重做全量 dirty diff，不猜测变化来源、不擅自纳入/丢弃/清理；原 Jimu task/start 文档逐次 unchanged 标志仍为 true，已批准 snapshot 字节不变。
- 新增/修改范围：录制模块、legacy 的 opt-in 接入、native validation CLI/已声明取消的封装等待、PushT 独立离线渲染工具、对应测试及本文。视频/截图/逐帧元数据/raw stdout 不纳入提交。

### 可追溯哈希

| 本地证据 | SHA256 |
| --- | --- |
| 交付 Tennis MP4 | `83e98cb0c386b06522f8924188f47b20f43aa860ce501c9ff6f9a6dcc029c2e2` |
| Tennis 本次 result JSON | `b02fd940227d22fdfcb84cd037179fdfbf26810b38d4d28340f0e22784f865ee` |
| Tennis 原录制 metadata JSON | `d9e388f7702b4a0e910ae4b8f65bbccbb77a3fa7d4312bfc10bd35325548c193` |
| 交付 Jimu MP4 | `8621397e6c2a42a5d1d5527df0368ff6d33af583767189591451a7299b64ce37` |
| Jimu v3 result JSON | `6a1c6eb441ad240e3371143dfa0c5631195b82bdba5ab99a89c5c45f5f7e1608` |
| Jimu v3 原录制 metadata JSON | `7003131df4f4bebc254d41b1bc739fd40800047663cb89502957c756f8cc679e` |
| 交付 PushT MP4 | `3e07e1e7ff57d9a6c2b7758cdde8b65e9473b2f435f4e25167fe7608e85ef780` |
| PushT baseline v3 metadata | `69c1483116a6df135d3939aaef4984387799b5ff755d79065a4767618180984c` |
| PushT rotated_orthogonal v2 metadata | `980890f3486628a590837cf371ebcc423e07f66851294525b397be7fb148fab1` |
| 最终 full tests log | `c43b5276096b3b5ce3bfd8363e90f4d2b2856892dba9bd686b825ece671c1656` |
| 最终 three_scene log | `e9f41aa162107bfd5e5e247666628b3ee135541759730e72cf25b739110533d1` |

录制工具与本节说明随本分支提交并 push；提交 SHA 在交付消息给出。等用户查看这三条链的失败证据再讨论下一步，**不自动放宽碰撞或进入真机运动**。
