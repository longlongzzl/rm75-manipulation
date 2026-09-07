# 三场景无运动回归与 IK 碰撞诊断 — 2026-09-08

**尚未全部完成，Jimu 不是只差真机。** 本轮补齐 PickPlace 实际工作台固定世界入口，定位并保护 Jimu 的碰撞 IK 误放行，重新跑了 PushT 快慢两组完整 GPU 链。PickPlace 七物体和 Jimu 完整 builder 仍未通过。没有连接机器人 SDK，没有机械臂/夹爪运动或新相机采集。

按 [回传模板](CODEX_THREE_SCENE_RESULTS_TEMPLATE.md) 填写。机器摘要：[native gaps](../benchmarks/unified_scenarios/three_scene_native_gaps_20260908_summary.json)。原始结果、轨迹和日志仅保存在本机 `runtime_data/three_scene/native_gap_20260908/`；其中 worker 的 `result.json` 记录对应本地 job_id。前轮证据：[工作台与完整 builder](CODEX_THREE_SCENE_CLOSEOUT_FOLLOWUP_20260908.md)、[PushT GPU / RRTrack](CODEX_THREE_SCENE_NOMOTION_20260907.md)。历史失败没有覆盖或删除。

## 0. 基本信息

- Tested base：`ea13776` + 本回传提交内差异；摘要记录最终被测 Python 源文件 SHA256。
- Branch：`chatgpt/three-scene-software-closeout`。
- OS / GPU：Linux、RTX 5060 Ti 8151 MiB、driver 580.173.02。
- CPU Python 3.12；PickPlace/Jimu：foundationpose310、Python 3.10、torch 2.7.1+cu128、原 cuRobo1；PushT：curobo2、Python 3.11、torch 2.11+cu128、cuRobo2。没有混用两个后端或恢复 MPLib。
- ManiSkill 沿用迁移后的原环境；RealMan SDK connection / Robot IP：NONE / NOT_RUN。
- GPU 串行，MemoryMax=9G、MemorySwapMax=512M、OMP_NUM_THREADS=2、MAX_JOBS=2。任务有 600–900 s 上限，无并行 GPU 压测。

## 1. 旧仓库 dirty worktree 审计

- 旧仓库 `/home/zhangzhao/Desktop/lerobot`，HEAD `36798efbd12814841607951c9af470b309b34fd3`。
- 只读使用原完整 builder JSON；没有 reset、clean、stash、覆盖旧文件或复制整批 untracked。**旧文件修改：NO**。
- 前后 `status --porcelain=v1 -z` SHA256 均为 `15fd1ac40352579a761b4134244c2a74158dd723b2b146c6a39ccf458fe20968`，不冒称与重启前历史哈希相同。
- 六入口/直接依赖分类沿用已批准审计；本轮没有新增 dirty diff 纳入或丢弃决策。

## 2. 迁移完整性

- 固定 `7aaff9da22486b7d25557b3795dd258f9b65f10d` + 12 approved overlay 不变，snapshot **807 files**。
- `python tools/migrate_working_sources.py --target-repo . --verify-only`：PASS。六入口随完整 manifest 校验，未改 snapshot、外部 cuRobo 或模型安装。
- 改动为新仓库进程内适配器。未将 manifest 中全入口资格标记擅自置 true。

## 3. 代码完整性与 CPU 测试

| 检查 | 最终结果 |
| --- | --- |
| compileall：rm75_app、tools、tests，含 snapshot | PASS |
| 完整 tests/three_scene | **283 passed**，1 warning，6.16 s |
| 完整 tests | **605 passed**，1 warning，23.43 s |
| git diff --check | PASS |

新增 27 项测试：碰撞诊断、预筛桌面、near-IK 保护/原失败对象不变、接触 IK 作用域正常/异常恢复、真实模式隔离、固定输入入口、相机格式往返、摘要保留失败与不输出原始轨迹。无 skipped；warning 为既有 trimesh Scene.dump 弃用提示。

## 4. 前端

- 三场景前端/CLI/overlay/请求权限相关离线测试随完整 suite 通过。本轮没有改前端页面，实际浏览器复跑 **NOT_RUN**；不把单测写成现场点击结果。
- PickPlace：新增可信 machine profile 的 `fixed_scene_format=native_world`，实际 WorkcellService → native worker → 原 direct entry 已通过。浏览器仍只能选择原 object_name，不能提交 entrypoint、输入格式或任意 native argv。默认 SAM6D 入口不变。
- Magnetic：前轮完整 21 件导入、round-trip、preview、取景和 GPU 后 Stop 的证据保留；本轮 builder GPU 使用同一原设计。实际 UI 导入/导出、input prompt、Stop 复测 NOT_RUN。
- PushT：前轮页面、preview、CPU surrogate loop、polling、Stop 证据保留；本轮完整 CPU suite 复跑。GPU 验证为独立无运动入口，不把浏览器 CPU SIM 宣称为 GPU。

## 5. PickPlace 回归

### 输入格式门

原 frozen 文件是 `objects/T_world_obj`，不是 SAM6D `results/T_cam_obj`。按原 parser、逐物体参数、真实 ManiSkill robot-base transform、原 bitong 朝向修正及桌面钳制做逆变换/往返审计：

- `pickplace_camera_roundtrip_v1`：9 个物体 **7/9 等价**；笔 z 改变 **+4.230323 mm**、胶棒 **+0.791220 mm**，源于原桌面最低中心约束。
- 严格 1e-6 矩阵差门未通过，**未生成 SAM6D fixture**，没有关掉钳制或伪造感知正例。
- 因而接回已审阅的原 fixed-world direct entry，仅允许 PickPlace SIM + 显式 fixed scene；real/native_world 拒绝。默认相机入口和原坐标/规则不改。

### 实际 GPU 结果

| 运行 | 结果 | 审计 |
| --- | --- | --- |
| `pickplace_workcell_world_v1`，原 gluestick_desk_regression，全部 9 个世界物体保留 | **1/1、final=True**，13.972 s；worker=command_completed_unverified | 61 搬运点、6 payload spheres，5 clearance 点通过；MPLib loaded=[] |
| `pickplace_table_final_v1`，原 current_table_all 七物体及原重试顺序 | **FAIL**，4 个原 cycle=True；没有最终完成标记 | 6 次搬运审计 / 373 点；3 次合格 clearance 为 5/12/3 点；MPLib loaded=[] |

七物体本轮共 6 次已打印 cycle 尝试（4 true、2 false），不是 6 个不同物体。刷子退让 warning 仍保留；后续网球执行前独立 clearance 审计发现第 1 个加密点新增接触 **28.687 µm**（起点 0），超过原 +1 µm 门，直接拒绝。没有把正在执行的第 5 件算成功，没有忽略该异常继续报告完整结束。胶棒/薯片罐原失败尝试仍在日志。

本轮预筛也保留桌面，未减少候选/seed 或改 `lazy_place + primary_only`。因此不把这次 4/7 中止与前轮 5/7、final=False 混成重复成功率；两者严格全链都未通过。候选筛选原始 planning_profile rows 保存在本地 stdout，没有做受控 10× warm timing 对比，性能优化结论 NOT_MEASURED。

所有搬运 `world_exempt_links=[]`、payload 1/5/6 spheres 保留；没有 transport policy rejection。**SAM6D/RRTrack → PickPlace 同场景完整工作台定位链仍 NOT_RUN**；fixed-world 正例不替代它。task_success 仍 null。

## 6. Magnetic / Jimu 回归

同一原 `tag1_standard_three_layer/builder_scene.json`：**21 件 = 9 固定 + 12 可移动**。原 roles/dependencies、同类料位重试、partial/full-open + retreat、候选/seed/几何不重设计。所有本轮完整运行使用原 synthetic anchors，不冒称来自单片 RRTrack 实测。

| 运行 | 原结果 | 搬运审计 |
| --- | --- | --- |
| `jimu_collision_detail_v1` | 8 件后首个完整几何诊断触发取消，165.325 s；不是全链 | 8 次 / 326 点 |
| `jimu_ik_table_v1` | 8/12、final=False，347.225 s | 10 次 / 520 点 |
| `jimu_near_ik_guard_v1` | 0/12、final=False，84.719 s；抓取接触 IK 尚未显式继承已批准规则 | 0 点；960 次 near-IK promotion 被拒绝 |
| `jimu_near_ik_guard_v2` | **8/12、final=False、worker=failed**，266.633 s | **8 次 / 354 点**，无搬运 world-link 豁免 |

没有删除 v1 的退化失败。完整运行均加载 cuRobo 而非 MPLib，`loaded_mplib_modules=[]`；取消的诊断运行没有最终模块 census，不补造为空。Four-wall 独立回归本轮 NOT_RUN；完整设计前四件成功不冒充独立 four-wall 重跑。

### 原始几何证据与修复边界

1. 首个屋顶诊断点由原 candidate label 明确为 **goal**，虽然调用的 native `check_start_state` 状态名含 INVALID_START_STATE。`link_5` sphere 7 与 `virtual_table_plane` 净距 **-25.027076 mm**；高层 cuboid、cuRobo 内部 OBB 缓存和原始 GPU world constraint 一致。这不是夹爪/被抓物接触，不能用已批准的夹爪免检消除。
2. 原 `winner_chain_ik_preselect_grasp{,_contact}` 使用 `include_table=False`，后续搬运恢复桌面。适配器将桌面保留在该预筛阶段，其他原候选、配对、排序不变；table-only 复跑仍给出相同碰撞目标。
3. 原 `_jimu_maybe_accept_near_ik_result` 仅凭位置误差 ≤1e-4 m、旋转误差 ≤1e-3 rad 就把 failed IK 改成 `SuccessNearIK`，没有检查碰撞。现在先在副本上调用原 near helper，再要求当前 cuRobo world/self check 有效；拒绝时保留原 failed result/debug，不变更阈值。
4. v1 揭示旧 near helper 也隐式放过抓取终点的夹爪碰撞。v2 将你已批准的夹爪接触规则显式放到 **唯一的原 grasp-contact IK batch**：只过滤 8 个 finger/pad links 对世界的 cost 输入，不改 shared spheres/self checks，不包含 gripper_base、arm links 或 payload。一次性、线程局部 token；预抓取、其他 paired queries、搬运不会继承该新增 scope，正常/异常都恢复。其他接触阶段沿用前轮已审计策略。
5. v2 拒绝 **2029 次原 near promotion 调用**，不是 2029 个独立物体或独立候选；重复 batch/source retries 保留。屋顶没有剩余可行原 IK 配对，最终仍 8/12。负距离聚合是 ALL-link 几何诊断，不意味着表中每个 finger 接触都是当时未豁免的失败原因；第 5/6 连杆与桌面/结构的碰撞不在任何许可内。

这些是防止碰撞解被误放行的 SIM 安全一致性修复，不声称已优化出合格屋顶路径，也不自动授权 real 接触兼容策略。

## 7. PushT GPU / cuRobo2

本轮新复跑相同 `orthogonal_tool + low_table` 人工仿真 fixture，保留原 10 个 push 候选、3 friction scales、IK seeds、5 mm Cartesian 采样上限、.02 rad edge 加密、3 mm corridor 和碰撞/动力学条件。

| 新 GPU 运行 | 完整五段链 | 审计点 | 轨迹总时长 | 最大 TCP 速度 | 最大关节速度 / 加速度 |
| --- | --- | --- | --- | --- | --- |
| `pusht_final_gpu_v1`，15 mm/s | PASS，规划 17.319 s | 2309 | 76.800 s | 14.035721 mm/s | .128698 rad/s / .494959 rad/s² |
| `pusht_final_slow_v1`，5 mm/s | PASS，规划 19.429 s | 6043 | 201.267 s | 4.687830 mm/s | .063884 rad/s / .449213 rad/s² |

两组都是 approach → descend → contact → push → retreat 完整预规划/审计；joint 限制 .25 rad/s、.5 rad/s² 未放宽。对各自已完成路径再注入无关障碍，**2/2 正确拒绝**。NoMotionArm.execute 会抛错，未调用机械臂执行。

本轮新运行 **2/2**，是同一 fixture 的快慢对照，不是两个不同场景。前轮 6/9（含两个正常输入失败和一个预设阻挡）及更早失败记录保留，不改累计分母。前轮固定工具/T 朝向组合失败本轮未重测，仍不合格。

真实推头/桌面/profile 仍未测量，hardware_profile_qualified=false、integration_qualified=false。stale/replay/drift 等门禁由完整离线测试复跑；物体规划中移动后的真实 re-observe、真相机推动闭环 NOT_RUN。

## 8. Camera / tracking

- 本轮未采集相机、未运行 RRTrack 推理。只做上述固定数据的原坐标映射往返审计，使用仓库既有 camera_extrinsic_opencv.npy，摘要保留校准哈希。
- 前轮单橙色薄片录像 29/30 accepted、坏深度拒绝→下一帧恢复证据保留；不冒称装配精度、12 个实物身份或真实遮挡已测。
- fresh timestamp 在线资格、observe → push → fresh observe：NOT_RUN。

## 9. RealMan no-motion

SDK connection/preflight、真实关节反馈、控制器 Stop API、夹爪 backend：**NOT_RUN**。没有任何机械臂/夹爪运动。仿真 worker 和 NoMotionArm 不等于真实控制器链路资格。

## 10. Physical motion ladder

低速自由空间轨迹、单夹爪、单/多物体 PickPlace、单件/2/4/6+ Jimu、一次 PushT、推后新观测、多步真实闭环：全部 **NOT_RUN**。

## 11. Final summary

- PickPlace：实际 workcell 原世界坐标胶棒正例已闭环；七物体仍失败，刷子/网球释放退让接触和胶棒/薯片罐规划仍待解决；同场景感知输入缺口未掩盖。
- Jimu：完整 builder 仍 8/12；定位到原 near-IK 会放行实际连杆碰撞，保护已补齐，屋顶路径没有资格化。无 Tag 实物精度与多件身份仍缺。
- PushT：本轮补了真实 GPU 快慢两组完整五段链及碰撞负例，均通过；不代表真实场景几何/观测/控制器资格。
- 改动：新仓库 legacy/transport 适配器、workcell 验证工具、相机格式审计/指标汇总工具、单测与本回传；旧仓库和 snapshot 不变。
- 提交：见本回传所在本地 Git commit；没有通过修改成功条件、删除候选或放开 arm/table 碰撞来刷通过。
- **未 push**：先前环境审核因未推送父提交包含原始轨迹/日志及内部路径/硬件标识而拒绝上传；尚未获得对此内容上传到指定远端的补充许可，本轮不重试、不改写历史绕过。需要用户明确是否允许这些现有未推送提交中的数据上传；本地提交不等于远端已更新。
