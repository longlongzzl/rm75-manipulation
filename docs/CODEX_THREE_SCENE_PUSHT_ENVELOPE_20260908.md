# 三场景无运动后续：PushT 下降失败的工具几何证据 — 2026-09-08

**NEEDS_REVIEW：两个失败姿态存在明确的提前工具接触，未修成完整链成功。三条非真机链仍未全部关闭。** 本轮使用真实 cuRobo2 GPU FK 和 raw sphere-distance 查询，对四个原 saved authored fixtures 做工具包络诊断；不是四次完整机械臂规划，也不是实机实验。没有连接相机或 SDK，没有机械臂/夹爪运动。

按 [回传模板](CODEX_THREE_SCENE_RESULTS_TEMPLATE.md) 填写。机器摘要：[PushT tool-envelope summary](../benchmarks/unified_scenarios/pusht_tool_envelope_20260908_summary.json)。此前完整 GPU 链路分母仍见 [六次 PushT 回传](CODEX_THREE_SCENE_RELEASE_CACHE_20260908.md#7-pusht-gpu--curobo2)；Jimu 最新见 [分段释放执行门](CODEX_THREE_SCENE_RELEASE_GATE_20260908.md)。

## 0. 基本信息

- Tested base：`ae0ec0e05909a8ff1c22dc72856da47d193f403b` + 本回传提交的两个新 Python 文件，SHA256 见机器摘要。
- Branch：`chatgpt/three-scene-software-closeout`；Linux；RTX 5060 Ti 8151 MiB，driver 580.173.02。
- CPU Python 3.12；GPU curobo2 / Python 3.11 / torch 2.11+cu128 / cuRobo2。未升级库，未修改安装的 cuRobo 或机器人模型。
- ManiSkill：本轮未启动；RealMan SDK connection / Robot IP used：NONE。
- 一个 GPU scope 串行处理四个原输入，6.232 s；MemoryMax=9G、MemorySwapMax=512M、OMP_NUM_THREADS=2、MAX_JOBS=2。

## 1. 旧仓库 dirty worktree 审计

本轮只使用新仓库已保存的四个 SIM input.json，没有迁移/修改旧 `lerobot` 文件，没有 reset/clean/stash 或复制 untracked。旧固定基线与 approved overlay 不变；上轮 status hash 的只读审计保留，不冒充本轮新做完整 dirty diff。

## 2. 迁移完整性

未重新迁移。fixed `7aaff9d` + 12 approved overlay、807 个 manifest 文件不变；本轮 `migrate_working_sources.py --verify-only` PASS。未更改六入口或 snapshot 受管源码/资产，全面 dependency/runtime 资格标志仍不置 true。

## 3. 代码完整性与 CPU 测试

| 最终检查 | 实测 |
| --- | --- |
| compileall：rm75_app、tools、tests，含 snapshot | PASS |
| 完整 tests/three_scene | **363 passed**，1 warning，11.56 s |
| 完整 tests | **685 passed**，1 warning，27.81 s |
| git diff --check | PASS |

新增 9 项单测，验证 rigid sphere 旋转/平移往返、半径和原数组不变、非法几何拒绝、原 fixture 观测重建与 selected push 精确匹配、live/unknown/execute-real 输入不得进入诊断。无 skipped，仅既有 trimesh 弃用 warning。原生产规划代码未改。

## 4. 前端

三场景 frontend/CLI/权限/overlay 的离线 suite 已复跑。实际浏览器 page load、preview、polling、input、Stop：本轮 **NOT_RUN**；未改前端，不将本次 GPU 几何查询冒充页面/控制器验证。

## 5. PickPlace 回归

本轮新增 GPU、相机和 10× warm benchmark：**NOT_RUN / NOT_MEASURED**。此前单胶棒 1/1、七物体 4/7（9 次原尝试、final=False）保留，原 `lazy_place + primary_only`、候选、seeds、阈值未改。仍未全链完成。

## 6. Magnetic / Jimu 回归

本轮新增 GPU **NOT_RUN**。上轮原四墙 4/4、默认屋顶 12/12、完整 builder 8/12、24 次分段 release/return gate 通过的证据保留。完整 builder 屋顶 paired IK、分离 joint-return 的独立覆盖和 tagless 装配精度仍未关闭；不把本轮 CPU green 当作新的装配结果。

## 7. PushT GPU / cuRobo2

### 7.1 为什么追加此诊断

前轮 baseline 在下降 10/16、rotated fixed-tool 在 11/16 没有成功 IK variant。失败 IK 的关节解偏离目标且含一些 self contacts，单凭那些返回解不能区分目标 TCP 本身的工具/world 几何问题与求解分支问题。

当前 `pusht.Config.pusher_radius_m=.005` 把平面接触模型视作半径 5 mm 的圆；原 GPU fixture 的 tool_frame 是 `gripper_tcp`，碰撞模型却是原闭合夹爪，不是新增 5 mm 圆形推头。只有 `tool_collision_geometry_verified=true` 不能证明二者的接触位置映射一致；这些 authored fixtures 的 `hardware_profile_qualified` 始终为 false。

### 7.2 实际方法和边界

- 新工具 `tools/diagnose_pusht_tool_envelope.py` 读取四个原 input.json，逐一 SHA256 记录；不改原 pose、tool quaternion、geometry、gap、候选、seeds 或成功条件。
- 原 input 未保存 observation.pose；按原生成器的 named fixture 重建 `[.35,-.18,0/.3]`，重新执行原 choose_push，要求其结果与 saved push **完全一致**。这是原固定输入重建，不是 live observation。
- 从原 cuRobo2 GPU FK 提取全部 **38 个正半径 gripper/pad spheres**，变换到 gripper_tcp；用另一个仅作 FK 的关节参考验证其相对 TCP 的刚性，最大误差 **2.780888e−8 m**。没有把第二个 FK 参考当运动指令或可达路径。
- 在原名义下降线的 17 个 TCP 姿态上刚性放置同样球体，保留半径/偏置；0 是 hover，1–16 是下降采样。对全场景的 native raw GPU sphere-distance 和原 cuboid SDF 作逐球、逐样本交叉核对，四项 mask 全部一致。
- 查询不移除世界物体、不改共享机器人球体；四项 query 前后 world enabled flags 与原模型球体完全一致。
- 这只检验 **指定精确 TCP 姿态下的 tool/world 必要条件**，不提供 arm/self/IK/连续轨迹资格。无工具碰撞的两个对照不能单独叫全链 PASS。

### 7.3 四项实测

| 原 case（均 low_table） | 前轮完整规划 | 本轮首个 tool/world 碰撞样本 | 本轮具体接触 |
| --- | --- | --- | --- |
| baseline，斜置工具 | descend 10/16 失败 | **10/16**，名义 TCP z=.050 m | `gripper_Right_Support_Link` / `right_pad` ↔ `pusht_target_0`；首点最大重叠 **4.800047 mm** |
| rotated_orthogonal，T 转 .3 rad、工具固定 | descend 11/16 失败 | **11/16**，名义 TCP z=.045 m | `gripper_Left_Support_Link` ↔ `pusht_target_0`；首点最大重叠 **3.049097 mm** |
| orthogonal_tool，原正例 | 前轮完整链 PASS | 无，17 个样本均无 tool/world overlap | 本轮只证工具几何必要条件，不复跑整链 |
| yaw_matched_tool，工具随 T 转动 | 前轮完整链 PASS | 无，17 个样本均无 tool/world overlap | 同上 |

baseline 从第 10 点到末点共 7 点有重叠；rotated_orthogonal 共 6 点。两个首次重叠位置与前轮 IK 失败位置一致。至少原指定的精确 TCP 姿态在当前固定夹爪模型/场景中存在确定接触；不能把它归为单纯随机 seed 不佳，也不据此断言所有其他工具姿态全局不可达。

### 7.4 未采用“把下降起点移远”来凑通过

仅增加 standoff，随后仍让 TCP 走到旧 5 mm 圆形推头定义的 contact，可能使真实夹爪在名义 contact 阶段提前接触并推动 T，产生未计入后续 12 mm push 的位移。这样即使碰撞允许阶段通过，也没有证明原短推模型语义成立。

因此本轮**没有修改 standoff、接触/TCP 偏置、工具朝向或允许接触阶段**，没有把上述两个失败重新定义为完整链成功。已向用户询问最终采用闭合夹爪直接推还是独立推头；后续必须据实际选型建立可信的接触几何映射，再做完整 approach → descend → contact → push → retreat GPU 验证。真实硬件 profile 仍未资格化。

### 7.5 分母与未复跑项

本轮 **4/4 几何诊断完整**，不是 4/4 完整链成功。此前六次 GPU、正常输入 3/5 完整链（含慢速重复）、两个下降失败和预设邻物拒绝全部保留；此前完整链上的额外 GPU 邻物审计 3/3 和注入执行前门禁 30/30 也不冒充本轮新做。

原始 FK-relative sphere 坐标、名义 TCP pose 与日志只在本地 `runtime_data/three_scene/pusht_envelope_20260908/`。提交摘要只保留 case、输入/源代码/测试哈希、接触对及标量测量，不包含原始关节、球体坐标或硬件标识。

## 8. Camera / tracking

相机采集、RRTrack 新推理、freshness、遮挡恢复和 push 后新观测：本轮全部 **NOT_RUN**。旧橙色单薄片 RRTrack 29/30 等证据保留，不等于 PushT 接触参数/真实跟踪链已经确认。

## 9. RealMan no-motion

SDK connection/preflight、joint feedback、真实 Stop API、gripper backend：全部 **NOT_RUN**。本轮只有 GPU FK/距离查询，无机械臂或夹爪运动。

## 10. Physical motion ladder

自由空间、夹爪、PickPlace、Jimu、单推、推后新观测、多步 PushT：全部 **NOT_RUN**。

## 11. Final summary

本轮把两个 PushT 下降失败从泛化的 IK_FAIL 定位到具体的夹爪—T 提前接触，GPU/解析几何交叉验证一致；未放宽碰撞、删候选或伪造接触偏置。新增 9 项单测，完整 suite 685 passed。工具选型与 TCP/contact 映射待确认，三条非真机链仍未全部完成。

本地提交见本报告所属 commit；**未 push**。此前父提交中的原始数据上传仍受环境审核阻止，无新增许可，不重试或改写历史绕过。
