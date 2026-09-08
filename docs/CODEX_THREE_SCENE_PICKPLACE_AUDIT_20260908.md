# PickPlace 完整冻结世界 / PushT GPU 续报 — 2026-09-08

后续实际入口接线见 [workcell frozen-world 回传](CODEX_THREE_SCENE_WORKCELL_FROZEN_WORLD_20260908.md)：下文本轮 gate 最初只在独立 runner；后续已接入真实 service/worker，并做 11 个独立 GPU 请求和 1 次 GPU 后 Stop。它们不是重复七物体连续任务，前后结果分别保留。

**NEEDS_REVIEW：免运动验证已推进，三条链没有全部关闭，Jimu 仍不是只差真机。** 本轮完成 15 次 PickPlace native GPU/SIM 和 6 次 PushT GPU，共 21 次，全部自然结束。最终五个 PickPlace 场景完整通过 2/5；七物体原生结果 5/7；PushT 完整链 3/6（5 个正常输入中 3 个通过，另 1 个正确拒绝障碍负例）。所有中间失败保留。

机器回传：[GPU 完整矩阵](../benchmarks/unified_scenarios/full_frozen_world_20260908_summary.json)、[迁移与原余量审计](../benchmarks/unified_scenarios/pickplace_relocation_audit_20260908_summary.json)。前者复用 `tools/summarize_no_motion_matrix.py` 的 `summarize_run` / `totals` 重算结果，并核对实际 world coverage JSONL 数量、拒绝项、前台物体绑定；不导出原始轨迹、关节或世界位姿。

## 0. 基本信息

- Tested base：`3dce41977c73f360dc7ca30fc74c2319829e9a93` + 本报告列出的源码差异；不是 clean HEAD 测试。Branch：`chatgpt/three-scene-software-closeout`。
- 最终源码 SHA256 见 GPU 摘要 `final_source_sha256`。v1/v2 为中间审计版本，不冒充最终版本验证；三个目录的原 `argv`、JSON、日志各自保留且摘要记录哈希。
- CPU：Linux / Python 3.12。PickPlace：foundationpose310 / Python 3.10 / torch 2.7.1+cu128 / 原 cuRobo1、ManiSkill/SAPIEN。PushT：curobo2 / Python 3.11 / torch 2.11+cu128 / cuRobo2。
- RTX 5060 Ti，8151 MiB，driver 580.173.02。GPU 串行，MemoryMax=9G、MemorySwapMax=512M、OMP_NUM_THREADS=2、MAX_JOBS=2；未升级依赖、重建旧扩展或改安装模型。结束后 GPU compute process 列表为空。
- RealMan SDK connection / Robot IP used：NONE。相机、机械臂、夹爪均未连接或驱动。

## 1. 旧仓库 dirty worktree 审计

旧 `/home/zhangzhao/Desktop/lerobot` 只读；HEAD 沿用 `36798efbd12814841607951c9af470b309b34fd3`，可复现基线 `7aaff9da22486b7d25557b3795dd258f9b65f10d`。前后 status SHA256 均为 `15fd1ac40352579a761b4134244c2a74158dd723b2b146c6a39ccf458fe20968`。

未 reset/clean/stash/覆盖/修改旧文件，未重新决定任何 dirty/untracked 内容的纳入或丢弃。六入口及直接依赖的既有分类、12 文件 approved overlay 不变。额外只读审计了五份原场景、九类 object spec 的 18 个模型引用、原碰撞配置、原 CLI 和同轮重试/前台/预规划源码。

## 2. 迁移完整性

未执行新迁移。GPU 前后 `PYTHONPATH=. python tools/migrate_working_sources.py --target-repo . --verify-only` 均 PASS，807 文件、固定 `7aaff9d` + 12 approved overlay。六入口与受管文件按 manifest 校验；dependency completeness / runtime GPU 的全面资格标志仍 false。

- 五份新旧 JSON 的 `objects`（位姿、分数、placed 等）全部一致，各含 9 个物体。
- legacy_gluestick / current_table 全文件一致；jitter / yaw / swap 仅 `generated_metadata.base_scene_file` 的来源路径迁移不同，不能说三份文件 SHA 或完整 JSON 相同。
- 经实际 `source_adapter(snapshot)` 解析的九类模型，18 个 mesh/sim 引用逐字节相同，尺度和 ObjectSpec 非路径字段一致。未发现这些引用中有 Jimu 三角片式漏迁移；这不是整个动态依赖闭包已经资格化。
- 新旧原碰撞球文件 SHA256 同为 `9216f499e1d7a2855dd4d8ce2ee04137bc59922b919db13d99c7852063d5b2ec`。

## 3. 代码完整性与 CPU 测试

最终 `compileall -q rm75_app tools tests` PASS（包含 vendored snapshot）；`tests/three_scene` **532 passed，16.82 s**；完整 `tests` **854 passed，34.24 s**，均仅 1 个既有 trimesh 弃用 warning，无失败或跳过。

相对本轮基线增加 57 项测试：22 项资产/余量审计；5 项固定入口全名称参数；30 项覆盖门、原函数跨目标去重、请求物体绑定及前台/后台证据隔离。测试原函数时只 AST 提取目标函数，不导入机器人入口。较早阶段的 497/819、517/839、528/850 全套结果也保留，不拿旧计数替代最终源码验证。

新增模块只服务显式 no-motion runner。没有修改快照、生产 PickPlace/Jimu 算法、PushT 规划代码、候选、seeds、原目标位姿、尺寸、球、buffer 或容差。原非搬运接触兼容策略不变；搬运仍全查世界、桌面、payload 和自碰撞，仅保留此前批准的 payload—finger/pad 自接触处理。

## 4. 前端

按 Browser skill 检查实际 `/workcell/` 入口，`getForUrl` 返回 `No browser is available`；按其 bootstrap 故障流程复用 runtime，`browsers.list()` 为 `[]`。没有换用独立浏览器/其他控制协议绕过该限制。

三场景实际页面加载、点击 preview / sim / Stop、实时状态轮询、原 input 确认、Jimu builder 导入与 round-trip：本轮均 **NOT_RUN**。相关 frontend/CLI/overlay/input/Stop/互斥/任意 argv 拒绝的完整离线测试已复跑。既有 loopback HTTP 资产检查及三个 preview、PushT CPU surrogate 证据见 [前轮回传第 4 节](CODEX_THREE_SCENE_PICKPLACE_PUSHT_20260908.md)，不是本轮浏览器实操或物理证据。

## 5. PickPlace 回归

### 5.1 先纠正旧单物体测试的世界范围

前轮五个输入均含 9 个物体，但四个单胶棒运行起始实际障碍只有 `desk`（另有虚拟墙/桌面）；七物体运行起始有其余 8 个对象。因此 `world_exempt_links=[]` 只能证明已加载世界没有 link 豁免，不能证明 JSON 所有物体已加载。前轮 2/5 不能外推为完整九物体世界验收；原结果未删改，日志哈希见资产摘要。

本轮增加 `--full-frozen-world`，必须同时使用 `--transport-world-checked`。它通过原 CLI `--tracked-scene-object-names` 载入完整名称，不重写规划器。只读 gate 在有效世界刷新后检查每个非活动源对象是否存在/启用；loaded 阶段必须有其余 8 个物体。发生缺项时直接失败，不继续在残缺世界里规划。空审计、不足覆盖和替代对象不能判通过。

### 5.2 三轮都保留，最终采用 v3

1. **v1，5 次 GPU**：新入口最初只传“非初始源”名称。原程序在 cycle 1 失败后切换源仍沿用该列表，导致胶棒遗漏；两个困难场景被新 gate 拦下。这是本轮测试参数错误，不归咎于迁移丢失或擅自补改旧算法。2/5 原 runner 通过；七物体 4/7、2 条退让 warning。
2. **v2，5 次 GPU**：传入全部 9 个名称。已读原 `_load_fixed_scene_capture`、`capture_or_reuse_foundationpose_scene` 和 `_single_scene_sync_obstacles`，它们会排除“当前活动源”，不会重复创建其障碍；原函数单测和 GPU 均证实跨目标保留其余对象。世界无遗漏；另发现本轮 source observer 包含后台 capture-only 返回，因此不将 v2 的未分范围 episode 计数作为最终物体验收。2/5，七物体 5/7、无退让 warning。
3. **v3，5 次 GPU**：保留 v2 CLI，仅给审计增加原 `_planning_prefetch_capture_only` 标志和前台线程标识。前台实际成功才可满足请求对象，后台成功不能补足 foreground task 或 transport coverage。再次完整 CPU + 五例 GPU；结果如下。

| 最终固定 case | 请求对象结果 | runner 完整通过 | elapsed |
| --- | --- | --- | --- |
| legacy_gluestick | 胶棒 1/1 | PASS | 13.526 s |
| gluestick_jitter_00 | 胶棒 1/1 | PASS | 14.116 s |
| gluestick_yaw_00 | 胶棒失败；原重试改做 bi | FAIL | 26.079 s |
| gluestick_swap_00 | 胶棒失败；原重试改做 bi | FAIL | 27.517 s |
| current_table_all | 原 native 5/7，final=False | FAIL | 85.698 s |

最终 **2/5**，不是进程退出或 native final=True 就通过。yaw/swap 的原 `final=True` 实际对应前台 bi 成功，后台 lvmukuai 仅预规划成功；请求的胶棒没有完成，所以仍 FAIL。未减少原重试/候选池来规避该区别。

- yaw 原胶棒：pregrasp 13/13、grasp 9/13、hover 5/13、release 4/13，四图交集 0/13，无完整关系。此处有完整场景障碍，不照搬旧仅桌面用例的 18.4 mm 直线失败作为本次失败原因。
- swap 原胶棒：原 paired straight lift 失败、无完整 transport-hover chain；未开 skipped direct transport fallback。
- 七物体前台 9 次尝试，5 true / 4 false：bi、lvmukuai、shuazi、carriot、tennis 成功；gluestick、hongshupian 失败并原样重试。后台还有独立预规划事件，不混入这 9 次前台尝试。
- 七物体本次 5 个 clearance 执行审计 / 41 点、无退让 warning；6 次 transport 审计 / 387 点包含前/后台规划，不冒称 6 次实际搬运。世界刷新 126 次，其中前台 77、前台 transport 12；全部 loaded 刷新预期/实际非源对象均为 8，无覆盖拒绝。
- 两个胶棒正例分别 clearance 5 / 6 点、transport 62 / 63 点；每例 9 次前台世界刷新，其中 transport 3 次，请求源绑定通过。
- 三轮共 **15 次 native GPU** 全留分母。v2/v3 七物体都是 5/7；v1 的 4/7 和退让 warning 不删除。没有据此宣称算法优化收益或退让问题普遍消失。所有运行 `loaded_mplib_modules=[]`。
- 原 native 候选筛选 10× warm：本轮 NOT_MEASURED。前轮两个既有 D0 任务的 20 warm / 2 full-chain 结果仍仅属于统一协调器；原历史 16-case 输入缺失，未重造后冒称复跑。

### 5.3 原抬升证据的余量分解，不放宽碰撞

只读追踪原代码确认 payload 使用 `inv(T_world_base) @ T_world_obj` 挂载，TCP 目标转换到 robot base / EE 后交给原 IK。保存证据的三个首个抬升目标相对起点 FK 都约 +80 mm Z、XY 极小；未发现这些代码路径缺 world→base 变换的证据，**不代表实际相机标定已验证**。

原 `base_link` 是机械臂底座，不是 `gripper_base_link`。原粗略底座球中心 `[0,0,0.12]`、半径 70 mm，另有原 **40 mm link self-collision buffer**。该 buffer 在 fixed baseline、旧 worktree、snapshot 与现行 approved payload pair 配置中完全一致；本轮没有减小它。

对三条已保存 GPU 名义目标诊断做代数分解（没有新增较宽松碰撞 query）：

| 保存的原目标 | enforced overlap（含原 self buffer） | 仅减去 link self buffer 后的球间 gap |
| --- | --- | --- |
| table / gluestick | 7.818 mm | 32.182 mm |
| table / hongshupian | 28.975 / 8.171 mm | 11.025 / 31.829 mm |
| swap / gluestick | 10.608 / 1.052 mm | 29.392 / 38.948 mm |

这五对记录主要触及保守的 self-buffer 区，不能写成实物网格必然相交。另一方面，所列球仍含原全局 sphere inflation，正 gap **也不是实物净空、安全许可或通过依据**；原 native 失败及 `safe_to_execute=false` 全部保留。既有 approved 策略只允许 payload—finger/pad 自接触，不恢复 payload/base 或 payload/gripper-base ignore，不改球/余量/目标来过关。

原配置 README 已说明 rough 手工球并非最终生产资格模型。当前只做模型/场景来源核验和证据澄清，没有擅自“标定”一个更小的模型。

## 6. Magnetic / Jimu 回归

本轮 Jimu GPU / 新迁移：**NOT_RUN**。两份原 `Demo_Triangle/red_triangle_74x135x6p5.glb` 及其 `.glb.coacd.ply` 漏迁移证据已在 [三角片审计](CODEX_THREE_SCENE_TRIANGLE_ASSET_AUDIT_20260908.md) 逐文件记录，尚未收到将其纳入 SHA-addressed overlay 的批准；未整体复制 untracked，也未改旧目录。

标准案例通过不等于旧完整 12 件任务通过：当前原完整 builder 为 8/12，4 个屋顶仍失败，而且所用 fallback 三角片并非原工作版 74×135×6.5 mm 模型。因此 Jimu **不是只剩真机**。恢复模型后是否消除屋顶问题仍需复跑，不能预先保证。已有角色/依赖/同类重试/释放退让与 attached payload 的 CPU 测试本轮均复跑；无 Tag 身份/装配精度仍未完成资格化。

## 7. PushT GPU / cuRobo2

本轮实际重新执行现有 `run_pusht_gpu_validation.py` 的六例，保持原 `low_table` authored fixtures，并传 `--audit-blocker --audit-execution-gates`。无运动 arm sink 拒绝任何 execute；没有改 TCP、接触 links、工具球、目标、桌面、候选或路径限值。

| GPU case | 完整五段链 | 结果 |
| --- | --- | --- |
| orthogonal_tool，15 mm/s | PASS，2309 点，76.800 s 轨迹 | 最大 TCP 14.036 mm/s；运行 13.299 s |
| orthogonal_tool，5 mm/s | PASS，6043 点，201.267 s 轨迹 | 最大 TCP 4.688 mm/s；运行 18.857 s |
| yaw_matched_tool，15 mm/s | PASS，2164 点，71.967 s 轨迹 | 最大 TCP 14.024 mm/s；运行 12.104 s |
| baseline | FAIL | descend 10/16，无合格 IK；9.032 s |
| rotated_orthogonal | FAIL | descend 11/16，无合格 IK；9.452 s |
| orthogonal_neighbor_blocked | 正确拒绝，非完整链通过 | approach 失败；7.036 s |

全部实际运行完整链 **3/6**，正常输入 3/5，另 1 个预设障碍负例。三条完整链在任何执行前已规划 approach → descend → contact → push → retreat，并通过原逐点碰撞、TCP corridor、姿态、速度/关节速度/关节加速度检查。

三条完整 GPU 链加入无关障碍后 **3/3 拒绝**；各注入 10 个 stale/replay/session/frame/confidence/drift 等门禁输入，**30/30 拒绝、execute 调用 0**。这是 GPU 已规划链上的执行前输入注入，不是实时相机测试。原 10 candidates / 3 friction samples / 32 IK variants 及原所有路径限值不变。

所有 `hardware_profile_qualified=false`。当前 5 mm 圆推头平面模型与闭合夹爪碰撞包络不一致；实际采用闭合夹爪还是独立推头、测量 TCP/contact/桌面/目标和定位 profile 尚未确认。不能通过把仿真 fixture 写成实际 profile 来解除资格门。两个下降失败保留；工具几何证据见 [PushT envelope 回传](CODEX_THREE_SCENE_PUSHT_ENVELOPE_20260908.md)。

## 8. Camera / tracking

本轮 RRTrack 新 RGB-D、实际目标跟踪、T_base_camera 标定实测、fresh/lost/recovery、observe → push → fresh observe 均 **NOT_RUN**。此前单橙色薄片记录不是三场景实时闭环验证。CPU freshness 门禁和本轮 GPU 输入注入已完成，但不代替真实相机证据。

## 9. RealMan no-motion

实际 SDK connection/preflight、关节反馈、Stop API、夹爪 backend 与实际签名调用均 **NOT_RUN**。未连接设备，未发任何机械臂或夹爪命令；运行中没有机械臂/夹爪运动。GPU 中的模拟关节与夹爪状态不是硬件 no-motion preflight。

## 10. Physical motion ladder

低速自由空间、独立夹爪、单/多物体 PickPlace、单件/完整 Jimu、PushT 单推、推后新观测和多步闭环：全部 **NOT_RUN**。没有新增统一延时，也没有因测试通过而申请或开启 real 执行。

## 11. Final summary

- PickPlace：补齐完整冻结世界、跨目标障碍保留、请求物体绑定、前后台证据隔离。最终 2/5 全场景通过，七物体两轮为 5/7；胶棒/薯片罐及困难胶棒输入仍有原规划失败。原模型保守余量只作澄清，未改动。
- PushT：六例实际 cuRobo2 GPU 复验结束，3/6 完整链、3/3 追加障碍、30/30 执行前门禁；实际工具、接触、定位和物理闭环资格尚未完成。
- Jimu：仍有两份待批准的原模型资产及完整屋顶复验、无 Tag 定位资格，不是只差真机。
- 代码改动：新增 `audit_pickplace_relocation.py`、`pickplace_world_coverage.py` 和相应测试；扩展 no-motion `run_native_pickplace.py` 及入口测试；新增两份摘要与本文，给旧报告追加范围/余量说明。snapshot、旧仓库和生产算法均未修改。
- 原始数据只在本地：`runtime_data/three_scene/pickplace_relocation_20260908/`、`full_frozen_world_20260908/`、`full_frozen_world_v2_20260908/`、`full_frozen_world_v3_20260908/`（后三个目录均位于 `runtime_data/three_scene/`）。
- 本轮仅本地提交，**未 push**。此前未推送父提交含原始轨迹/日志、内部路径与硬件标识，上传被环境审核拒绝；没有新增许可，不重试、不改写历史绕过。远端尚无本轮报告。
- 后续需要明确批准 Jimu 两资产 overlay，并确认 PushT 实际工具/测量 profile；浏览器通道不可用，真实前端验证仍待补。不得把这些未完成项写成三条链已完成或可进入真机。
