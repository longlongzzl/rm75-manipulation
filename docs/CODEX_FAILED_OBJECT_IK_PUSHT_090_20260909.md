# 2026-09-09 失败物体 IK 与 PushT 非全闭合续跑

本次从重启前已经落盘的证据恢复。胶棒新增 2 个有效放置端点；红薯片已有四阶段端点解，但原带载抬升仍被碰撞检查拒绝。PushT 的 `.9 rad` 被夹爪规划碰撞球自碰撞拒绝；进一步打开至 `.65 rad` 后，完整机械臂物理仿真实际完成 3 次推进，T 的 x 坐标增加约 **15.155 mm**，第 4 次动作在规划撤退段时失败。上述任务均未报告完整成功。

## 可直接查看的证据

- [胶棒 IK 原图、关节角及全部失败行](../runtime_data/three_scene/ik_failed_objects_pusht09_20260909/failed_object_ik/gluestick.html)
- [红薯片 IK 四阶段对应解及失败行](../runtime_data/three_scene/ik_failed_objects_pusht09_20260909/failed_object_ik/hongshupian.html)
- [关节角与完整行证据 JSON](../runtime_data/three_scene/ik_failed_objects_pusht09_20260909/failed_object_ik/solutions.json)
- [PushT `.65 rad` 首次推进的 8 秒原录像片段](../runtime_data/three_scene/ik_failed_objects_pusht09_20260909/pusht_065_first_push.mp4)
- [PushT `.65 rad` 完整仿真录像](../runtime_data/workcell/jobs/7d48d2b5e3b942738b04e766118c0c21/physics/closed_loop.mp4)

IK 图册来自被重启中断的任务 `afc820337e2648bf8980dd32ad1000ee`。它没有最终 result 或完整 manifest；恢复程序只采用 14 个逐批证据已写完、机器人/规划器/物体注册表状态恢复检查均通过的前台批次。未伪造任务完成状态。核验了 471 个 HTML 链接、368 张可解码原图、所选关节角与配置证据相等、四阶段抓取身份和放置身份一致。

## 胶棒和红薯片

| 物体 | 前台候选行 | 原生 IK 失败行 | 有效 release 行 | 四阶段端点均通过的关系 |
| --- | ---: | ---: | ---: | ---: |
| 胶棒 `gluestick` | 104 | 72 | 2 | 0 |
| 红薯片 `hongshupian` | 264 | 192 | 20 | 20（包含重试） |

行数混合不同阶段，不能当作物体任务成功率；红薯片的 20 条关系含重复尝试，对应 10 个不同的原生关系。

胶棒新增的两条放置解均位于 batch 19：

| 行 | 抓取关系 | 位置误差 | SO(3) 角度误差 |
| --- | --- | ---: | ---: |
| release #19 | `grasp_direct_top_bias_pos2_tilt25_away_axis_2mm` | 4.8623 mm | 1.7069° |
| release #22 | `grasp_direct_top_bias_pos10_tilt15_away_axis_10mm` | 4.6423 mm | 4.1145° |

release #19 的 `joint_1 → joint_7`，单位 rad：

```text
[1.1726078987, 1.4645550251, -1.8961971998, 0.1067562327,
 0.7359566689, 2.2339999676, -0.3462226987]
```

同批 hover #12 也新增了有效解。两条 release 仍没有配套的四阶段全部有效关系，不能直接用于执行完整抓放。

红薯片一组对应端点为 B20/pregrasp #0 → B21/grasp #0 → B23/hover #1 → B23/release #41，原生放置关系包含运输偏航 −45°；另有 +45° 等关系。每个端点的关节角、误差与三视图已在页面逐一列出。

红薯片的实际障碍位于**抓住之后的初始带载抬升**。保存的 `pickplace_failed_lift_ik_diagnostic` 在原目标位姿处解析放置原负载球，得到负载与固定 `base_link` 球的最大线性重叠约 **28.975 mm**。这是原球模型及缓冲下的必要几何条件，不是实体网格穿透深度。该固定目标处的负载/底座相对位置不随机械臂冗余 IK 分支改变；128 种子尝试也没有获得有效抬升解。改变抓取关系或抬升目标需要重新验证整条带载路径。

新搜索入口 `--failed-object-ik-seeds 128` 只在指定、碰撞检查开启的冻结世界 SIM 中启用，只针对这两个物体的前台失败行。恢复后的实现逐个失败目标查询，保存返回缓冲的副本；原先已成功的行保持原结果，新增成功行还需独立配置碰撞检查通过。单目标入口在原生实现中直接进入单目标 IK，不会被固定 CUDA 图批量扩张。该逐目标版本已做隔离 CPU 回归，本次重启后没有再执行全场景大批量搜索。

## PushT：`.9` 与 `.65`

保持原完整 URDF、原球半径/缓冲/忽略对、路径走廊 3 mm 和原 ManiSkill 理想臂重力补偿；规划几何使用实际仿真夹爪关节反馈。没有修改实体几何或增加碰撞豁免。

| 试验 | 结果 |
| --- | --- |
| `.9 rad`，任务 `f012c3baa7b24e4daaf6160b027394e4` | approach 被自碰撞拒绝；T 接触计数 0；物理日志未记录夹爪自接触。 |
| `.65 rad`，180 s 初次限时试验 | 实际推进约 9.04 mm，达到时间上限后结束，不算任务成功。 |
| `.65 rad`，600 s 上限，任务 `7d48d2b5e3b942738b04e766118c0c21` | 189.80 s 后因第 4 次规划的 retreat 被拒绝而结束；不是超时。前 3 次动作全部执行。 |

最后一轮 T 从 `[0.35, -0.18, 0]` 到 `[0.3651545942, -0.1827678233, -0.0935987830]`，目标为 `[0.38, -0.18, 0]`。记录 1185 个与 T 接触的物理子步，提前接触、静态障碍接触、不允许的机械臂链接接触 T 均为 0。最大 TCP 跟踪位置误差 1.352 mm。既有垫片/夹爪底座安装件接触仍出现在物理日志，不能声称全机器人无自接触。

失败位置是 retreat 的第 16/16 个路径点：13 个原生成功 IK 分支均未通过边路径验收；最近分支的路径偏差约 6.414 mm，超过 3 mm 走廊。随后仅重放同一规划请求：1 mm IK 配置仍失败；进一步将笛卡尔采样从 5 mm 加密到 2.5 mm，仍在最后一点失败，最近分支偏差约 6.300 mm。这两次都是静态规划，未重复整段物理执行，也未放宽验收条件。加密采样只用于独立重放，没有改动正式路径算法。

证据：[`.9` 结果](../runtime_data/three_scene/ik_failed_objects_pusht09_20260909/pusht_090/result.json)、[`.65` 物理汇总](../runtime_data/workcell/jobs/7d48d2b5e3b942738b04e766118c0c21/physics/summary.json)、[失败的第 4 次计划](../runtime_data/workcell/jobs/7d48d2b5e3b942738b04e766118c0c21/physics/plan_004.json)、[1 mm 重放](../runtime_data/three_scene/ik_failed_objects_pusht09_20260909/pusht_065_replay_ik001/plan_001.json)、[2.5 mm 采样重放](../runtime_data/three_scene/ik_failed_objects_pusht09_20260909/pusht_065_replay_step0025/plan_001.json)。

## 更正：GPU 自碰撞诊断的距离单位

之前报告将 cuRobo2 `_pair_distance` 直接标为米，再乘 1000 称为毫米，这是错误的。原生 `sphere_squared_distance_fused` 返回的是 `(r1 + r2 + padding1 + padding2)^2 - ||c1-c2||^2`，单位是 m²。

适配器现在从同一对球计算线性重叠，写入 `penetration_m`，并另存 `native_pair_distance_squared_m2` 保留原始值。碰撞正负判定保持原生逻辑。

固定 `.9 rad` 姿态经实际 GPU API 检查，得到 5 组链接自碰撞，最大**带缓冲规划球线性重叠为 25.5947 mm**。CPU 几何扫描与 GPU 原平方分数一致。该值不能解释为实体网格穿透；此前 `.91` 原始 STL 的 0.3702 mm 内部顶点采样结果是另一项独立几何证据，不受此单位修正影响。

原球模型扫描到 `.66 rad` 起不再发生夹爪内部球重叠；本次选 `.65 rad` 留出角度余量，标称左右 pad 参考点距离约 39.315 mm。它显然不是全闭合姿态。

证据：[CPU 角度扫描与核对](../runtime_data/three_scene/ik_failed_objects_pusht09_20260909/closed_angle_cpu_verified.json)、[修正后原生 GPU 检查](../runtime_data/three_scene/ik_failed_objects_pusht09_20260909/self_distance_gpu_verified.json)。历史原始日志保留，不能再把其中旧版 `penetration_m` 自碰撞字段当作线性米值。

## 重启后运行约束与验证

上一启动的内核日志出现 NVIDIA `NV_ERR_NO_MEMORY`，尚不足以断言这就是死机的唯一原因。重启后所有 GPU 工作串行，物理仿真与原生重放使用相同临时 systemd scope：内存高水位 6 GiB、硬上限 7 GiB、禁止交换、CPU 最多 2 核，数值库线程数均为 1。仿真期间观测显存约 1.5 GiB、整个 scope 内存约 2.6 GiB。所有任务现已退出，显存回到约 444 MiB。

所有测试和仿真通过 `tools/run_network_isolated.py` 安装内核网络保护；未连接真机、发送真实夹爪命令或访问串口/USB。原生旧测试脚本只允许文本审查，没有导入或收集。

本轮最终定向回归 **55 passed / 2.33 s**：碰撞距离单位、动态夹爪几何、逐个失败目标重试/缓存复用/候选身份、IK 图册及恢复相关逻辑。此前隔离完整套件为 **1044 passed / 1 warning**；新增改动后没有再次运行完整套件。命令：

```bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 tools/run_network_isolated.py -- python3 -m pytest \
  tests/test_self_collision_distance_units.py tests/test_dynamic_gripper_collision.py \
  tests/three_scene/test_failed_object_ik_search.py tests/three_scene/test_jimu_roof_ik_diagnostics.py \
  tests/three_scene/test_ik_candidate_gallery.py tests/three_scene/test_ik_gallery_export.py -q
```

没有提交或推送代码。
