# 实际工作台 fixed-world SIM 接线回传 — 2026-09-08

**NEEDS_REVIEW；三条链尚未全部完成。** 本轮把上一轮仅在独立测试 runner 中的 PickPlace 完整世界与请求物体验收接入实际 `WorkcellService → worker → 原 cuRobo1 native SIM`。实际完成 11 个独立 GPU 请求，6 PASS / 5 FAIL；另做 1 次 GPU 搬运审计后的 service Stop，取消成功。没有真机运动。

机器结果：[workcell frozen-world summary](../benchmarks/unified_scenarios/workcell_frozen_world_20260908_summary.json)。原始 worker 日志、位姿、轨迹、events 与 native argv 只留本地；摘要仅含必要计数、范围与 SHA256。

## 0. 基本信息

- Tested base：`bf23c62def82444feb98794808c154753244a46c` + 本轮源码差异；branch `chatgpt/three-scene-software-closeout`。最终源码哈希见摘要，不宣称 clean HEAD 测试。
- CPU Python 3.12；GPU foundationpose310 / Python 3.10 / torch 2.7.1+cu128 / 原 cuRobo1、ManiSkill/SAPIEN；RTX 5060 Ti 8151 MiB、driver 580.173.02。
- 全部 GPU 串行，MemoryMax=9G、MemorySwapMax=512M、OMP_NUM_THREADS=2、MAX_JOBS=2；原扩展缓存与依赖版本不变。工作台内部 task/mode 均为 PickPlace/SIM，`allow_real=false`，profile 的 hardware_reviewed / integration_qualified 均 false。
- SDK connection / Robot IP used：NONE；未连接或驱动相机、机械臂、夹爪。
- 只读 `git ls-remote --heads origin chatgpt/three-scene-software-closeout` 与本地 origin ref 均为 `3d2aba1bf375e516e27b353dd0c1387e2327da79`，指定分支没有新增审阅提交。没有 fetch/merge 覆盖文件，也没有 push。

## 1. 旧仓库 dirty worktree 审计

旧 `/home/zhangzhao/Desktop/lerobot` 保持只读，未 reset/clean/stash/覆盖/修改。status SHA256 前后均为 `15fd1ac40352579a761b4134244c2a74158dd723b2b146c6a39ccf458fe20968`。六入口与依赖分类沿用既有审计，没有擅自纳入或丢弃 dirty/untracked 文件。

两份待批准的 `Demo_Triangle/red_triangle_74x135x6p5.glb` 及 `.glb.coacd.ply` 尚未迁移；自动续跑或默认选项不当作批准。

## 2. 迁移完整性

没有重新迁移。GPU 前后 `tools/migrate_working_sources.py --target-repo . --verify-only` PASS，固定 `7aaff9da22486b7d25557b3795dd258f9b65f10d` + 12 approved overlay，807 文件。snapshot 受管源码/资产不变；dependency completeness / runtime GPU 全面资格标志仍 false。

使用现有原 `T_world_obj` 冻结输入，不生成、变换或改写场景；启动前验证每个物体都有有限 4×4 矩阵，拒绝空场景、缺请求源、非法名称、重复 JSON key 或损坏输入；结果验收再次核对输入 SHA，变化则失败。

## 3. 代码完整性与 CPU 测试

- `compileall -q rm75_app tools tests`：PASS（含 snapshot）。
- 完整 `tests/three_scene`：**556 passed，16.83 s**。
- 完整 `tests`：**878 passed，33.44 s**。
- 均只有 1 个既有 trimesh 弃用 warning，无 failed/skipped。新增 24 项测试；原 native-world CLI 测试改用有效场景并断言全部名称保留。

测试覆盖严格输入、原文件不修改、缺对象拒绝、前台/预规划与请求源绑定、独立 clearance 必须存在、输入变更拒绝，以及实际安装后的 world JSONL 落盘和原返回值保留。默认 SAM6D 入口、real 禁止冻结世界、Jimu 原 parser、原 source retry 返回值、浏览器不能选入口/格式/接触策略/任意 argv 的完整测试均复跑。

## 4. 前端 / 实际入口

发现并修复：上一轮完整世界 gate 仅接在 `tools/run_native_pickplace.py`，实际工作台 fixed-world SIM 仍没有该覆盖与源身份验收。

现在 trusted profile 选择 `fixed_scene_format=native_world` 时自动通过原 `--tracked-scene-object-names` 传入该冻结文件的全部名称；原程序负责去除当前活动源。原同轮换目标重试与后台预规划均保留。worker 还必须确认请求源在前台成功、完整搬运世界已观察、独立退让有效，才能给出 `command_completed_unverified`；即使通过，`task_success=null` / `verification=frozen_world_sim_only`，不是物理成功。

这个接线**只用于原 native-world PickPlace SIM**；默认相机/SAM6D 入口、real 路径、Jimu 算法和接触策略没有改变。没有新增浏览器任意 argv 或选择碰撞豁免的入口。profile 中原 `transport_world_checked_compatibility` 仍显式选择，不自动提升硬件资格。

实际页面方面按 Browser skill 复核：getForUrl 报 `No browser is available`；完整读取其 bootstrap 故障流程后复用 runtime，唯一一次 recovery list 为 `[]`。没有切换到无关控制工具绕过。三场景页面点击、实际 input prompt、builder 导入/round-trip、preview/sim 按钮、浏览器状态轮询和浏览器 Stop：本轮 **NOT_RUN**。

实际 service/worker 请求、任务状态与 GPU 后取消已测试，不能用这些后端证据替代浏览器实操。

## 5. PickPlace GPU 回归

命令使用现有 `tools/run_workcell_native_validation.py --task pickplace --object-name <请求源> --fixed-world <原输入> --extensions runtime_data/curobo_native_extensions --output <全新目录> --timeout-s 600`，经真正 service 提交、worker 子进程及原 native 入口执行，没有直接调用测试函数冒充该通路。

| 输入 / 请求源 | 工作台完整结果 | elapsed |
| --- | --- | --- |
| legacy_gluestick / gluestick | PASS | 14.583 s |
| jitter_00 / gluestick | PASS | 15.485 s |
| yaw_00 / gluestick | FAIL：请求源失败，原程序改做 bi | 27.465 s |
| swap_00 / gluestick | FAIL：请求源失败，原程序改做 bi | 29.284 s |
| current_table / lvmukuai | PASS | 14.379 s |
| current_table / carriot | PASS | 14.479 s |
| current_table / shuazi | PASS | 15.484 s |
| current_table / hongshupian | FAIL：请求源失败，原程序改做 bi | 31.494 s |
| current_table / gluestick | FAIL：请求源失败，原程序改做 bi | 23.847 s |
| current_table / bi | PASS | 15.987 s |
| current_table / tennis | FAIL：释放后退让失败 | 16.894 s |

总计 **6/11**。四个胶棒旧输入 2/4；原桌面七个**独立单对象请求** 4/7。每个请求都从相同原始桌面重新开始，不能当作七物体连续整理，也不替换 [前轮完整连续任务的 5/7](CODEX_THREE_SCENE_PICKPLACE_AUDIT_20260908.md)。失败全部留在分母。

11 个请求的原 native 都打印了 final=True，但 5 个 worker 正确拒绝完成：4 个是替代源/后台预规划不能满足请求源，另 1 个是退让失败。原 native marker、前台 source outcome、后台 capture-only outcome、worker 成功四者分别保留。没有把别的物体成功或进程退出算作目标完成。

全部输入的 9 个名称实际传入原 argv；记录的 loaded world 均含其余 8 个对象，没有 world coverage 拒绝。摘要核对 worker 世界 JSONL 与完整 events 一致，再用实际 source outcome 重算验收结果，逐项与 worker 返回比较通过。所有 `loaded_mplib_modules=[]`；transport 的世界豁免为空且 payload spheres 保留。transport 审计计数可能含后台规划，摘要明确该范围，不叫实际搬运次数。

### 网球独立请求的原始失败点

反用 final-contact 路径的端点与已放物体冲突，原程序转为 fresh clearance。该原 20 mm 退让的 endpoint IK 成功，3 点直线验证也通过（最大线偏差约 0.2 mm）。但 target-local 释放检查在第 1 点发现：初始穿入 4.987516 mm → 当前 5.123030 mm，增加 **0.135514 mm**，违反“不加深原接触”。原代码随后跳过未启用的 legacy planner rescue，产生 clearance warning；没有执行 clearance 审计。

这些是保存的 native 碰撞模型数值，不是实物接触测量。没有加容差、删目标、屏蔽更多 links、降低碰撞或启用 MPLib/fallback 来通过。普通物体原 box_scale 0.9、desk 0.99、shuazi 1.125 等原缩放未变；名称覆盖完整不等于真实几何精度资格化。

### GPU 后 Stop

额外一次原胶棒请求带 `--stop-after-transport-audit`。实际 GPU 搬运已逐点审计 61 samples / 6 payload spheres 后，service 收到取消，worker 以 -15 终止，job 为 cancelled；没有 native cycle 或 final 完成标记。该项 **PASS：实际在途 SIM 进程取消**，不是 task 完成。

取消请求返回 1.449 ms，仅是请求接口耗时，不是机器人停止耗时。物理急停与 SDK Stop 均未调用、未资格化。全部实际 GPU 运行数因此为 **12：11 个正常请求 + 1 个主动取消测试**。

原候选筛选 10× warm、历史缺失的 16 frozen：本轮 NOT_RUN；未拿上轮其他输入/协调器的性能结果替代。原候选、seeds、放置目标、碰撞余量及成功条件均不放宽。

## 6. Magnetic / Jimu

本轮新 Jimu GPU、模型迁移、实际组装：NOT_RUN。原 full builder 8/12、四个屋顶失败以及两份漏迁移三角片资产的问题见 [资产审计](CODEX_THREE_SCENE_TRIANGLE_ASSET_AUDIT_20260908.md)。fixed+12 overlay 校验通过不等于最终旧工作版模型闭包完成；未收到两文件 overlay 批准，也未用其他模型替代后宣称通过。

标准 four-wall / triangle 控制结果与原依赖、同类重试、grasp-release binding、partial/full-open、retreat、attached payload 等既有证据保留。默认 Jimu 入口及算法未改，相关完整 CPU suite 已复跑；无 Tag 多实例身份与装配精度仍未资格化。

## 7. PushT

本轮 PushT GPU：**NOT_RUN**，避免把前轮数据说成本轮重复验证。当前生产 PushT 代码没改，CPU 全套含其门禁测试已跑。最新真实 cuRobo2 GPU 证据仍为 [前轮第 7 节](CODEX_THREE_SCENE_PICKPLACE_AUDIT_20260908.md)：6 runs、3 条完整链、3/3 追加障碍拒绝、30/30 执行前输入注入拒绝、execute=0。

实际工具（闭合夹爪还是独立推头）、尺寸/TCP/contact、桌面/目标几何与定位 profile 未确认；`hardware_profile_qualified=false`。不能用 authored fixture 或现有 5 mm 圆推头模型代替现场确认，不调模型/目标或减碰撞来取消两个下降失败。

## 8. Camera / tracking

新 RRTrack RGB-D、真实 fresh/lost/recovery、手眼标定实测、PushT observe → push → fresh observe：本轮均 NOT_RUN。以前单橙色薄片和注入 timestamp 数据不能证明多实例或实际闭环。本轮没有新相机访问。

## 9. RealMan no-motion

实际 SDK/preflight、关节反馈、硬件 Stop API、夹爪 backend：NOT_RUN。仅有 no-motion 模拟 worker / service 取消证据，不是硬件 Stop 资格。没有任何机械臂或夹爪运动。

## 10. Physical motion ladder

低速自由空间、独立夹爪、单/多物体 PickPlace、单/完整 Jimu、PushT 单推/新观测/多步闭环：全部 NOT_RUN，保持本轮禁止真机试验的边界。

## 11. 完成审计与交接

| 要求 | 当前证据 / 状态 |
| --- | --- |
| fixed + approved overlay / CPU 完整性 | 807 文件 verify PASS；compileall + 556 / 878 tests PASS |
| PickPlace 实际入口完整世界、源身份、独立退让 | 已接入，11 次实际请求 6/11；仍有原规划/退让失败，不是软件全链全通过 |
| 前端三 tab、builder/input/按钮实操 | Browser 通道不可用，NOT_RUN；后端测试不能替代 |
| Jimu 最终旧工作版迁移、完整四墙/屋顶 | 两资产待批准；完整屋顶仍未通过，尚未完成 |
| PushT 实际工具/定位资格与完整 GPU 链 | authored GPU 已测，实际 profile 未确认，尚未完成 |
| 新相机/跟踪闭环 | 未运行，不从 CPU/GPU 注入测试推断完成 |
| Git 回传 | 本轮本地提交，未 push；指定 origin 分支未新增审阅提交 |

本轮修改 `legacy.py`，新增 `native_frozen_world.py`、24 项单测、摘要与本文，并给前轮报告加实际入口续报链接。旧仓库、snapshot、生产规划算法、几何、seeds 与接触策略未变；完整原始数据在 `runtime_data/three_scene/workcell_frozen_world_20260908/` 及对应 `runtime_data/workcell/jobs/`，不导出原始轨迹。

剩余工作不能靠再次自动续跑当前模型就算完成：需要明确批准两份 Jimu 三角片资产 overlay；确认 PushT 实际工具与已有标定；提供可连接的浏览器通道。PickPlace 剩余失败若要改变原路径/模型/候选选择行为，也需审阅该方向，不能擅自减 buffer、加接触容差、改目标或开额外 fallback。

上传此前含原始轨迹/日志、内部路径与硬件标识的未推送父提交曾被环境审核拒绝；本轮只读查远端 ref，没有重试 push、改写历史或绕过限制。远端目前没有本轮结果。
