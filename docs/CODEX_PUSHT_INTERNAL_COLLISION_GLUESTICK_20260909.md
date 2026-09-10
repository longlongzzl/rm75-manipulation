# PushT 夹爪内部碰撞排除与胶棒 IK 解释（2026-09-09）

用户本次明确要求 PushT 忽略夹爪连杆之间的碰撞。已实现并用离线原生规划器、全臂物理仿真验证。所有分析、测试和仿真均经 `tools/run_network_isolated.py` 启动；未连接或操作真机。GPU 工作串行，使用 6 GiB MemoryHigh、7 GiB MemoryMax、禁用 swap、CPUQuota=200% 的资源限制。

## PushT 修改与实测结果

- `pusht_planner_options()` 默认启用 `ignore_gripper_internal_self_collision`，传入正常 PushT worker 和物理规划进程。通用 `Curobo2BackendConfig` 默认仍为 false。
- 原生轨迹规划器和粗筛 IK 求解器创建前，在内存配置中排除夹爪基座、左右指节、支撑连杆和指垫共 9 个 link 之间的所有内部自碰撞对。保留全部碰撞球及其半径；机械臂、外部物体、负载相关检测继续使用原规则。
- `--check-gripper-internal-collisions` 可显式复现旧检测行为。
- 原生 GPU 验证：夹爪内部有效碰撞对为 0，0.9 rad 默认姿态不再报原来的夹爪内部碰撞；已知机械臂自碰撞和夹爪与诊断障碍物碰撞仍能检出。证据：[collision_policy_verified.json](../runtime_data/three_scene/pusht_ignore_internal_glue_20260909/collision_policy_verified.json)。
- 两组有针对性的正式测试共 **82 项通过**，覆盖碰撞对排除范围、模型配置、PushT 链路、IK、物理循环及回放。均显式指定 `tests/` 中的文件，并设置 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`。

0.9 rad、原重力补偿、模拟夹爪关节反馈几何的全臂物理仿真 job：`bffb05b653744b78bf44b6efc471d075`，用时 100.29 s。第一条五阶段动作链规划通过并执行，T 的 x 从 0.349999994 m 变为 0.356301546 m，推进 **6.30155 mm**。

**整项任务仍失败**：第二次重规划的撤退段在第 15/16 个路点拒绝了候选。最近分支的笛卡尔路径偏差为 **7.02119 mm**，超过原有 3 mm 限制；这里没有放宽路径要求。此失败不能继续归因于已排除的夹爪内部碰撞。最终观测为 `[0.356301546, -0.181800231, -0.099091135]`（x、y 单位 m，yaw 单位 rad）。

完整证据：[本轮结果](../runtime_data/three_scene/pusht_ignore_internal_glue_20260909/pusht_090/result.json)、[第二次规划](../runtime_data/workcell/jobs/bffb05b653744b78bf44b6efc471d075/physics/plan_002.json)。

## 胶棒：前五个竖直候选的放置位姿在几何上不可达

分析采用重启前 job `afc820337e2648bf8980dd32ad1000ee` 已保存的第一组胶棒配对查询（batch 19，13 个抓取关系各含 hover/release）。该历史 job 被重启中断，不能记为整体完成。目标、模型、关节范围和原求解器阈值没有改动。

从原始 URDF 得到：肩部关节中心为 `[0, 0, 0.2405]` m；肩到腕的所有连杆平移长度之和是 **466 mm**，这是任何关节姿态均不能超过的上界。腕到 TCP 约有 **352 mm** 固定偏移，因此 TCP 的目标朝向会强制改变腕中心所需位置。

居中竖直候选的 release 目标要求肩到腕距离 **505.336 mm**，比链长上界多 39.336 mm。进一步允许原有的 5 mm 位置偏差，以及姿态指标 0.05 对应的约 **5.732°** SO(3) 角度偏差，腕中心仍至少需要 **474.566 mm**，比上界多 **8.566 mm**。这里还额外减去 10 µm 的几何近似余量。

角度换算依据本地 cuRobo v1 的 `geodesic_distance`：它返回相对四元数向量部分的范数 `sin(theta/2)`，因此旋转阈值 0.05 对应 `theta = 2 asin(0.05)`，不能直接当作 0.05 rad。

令 `A = p_TCP_goal - p_shoulder`、`d` 为腕到 TCP 在工具坐标系的固定偏移、`D = ||d||`，`alpha` 为 `A` 与 `R_goal d` 的夹角。允许整个朝向容差锥和整个位置容差球后，肩到腕距离的下界为：

```text
sqrt(||A||² + D² - 2 ||A|| D cos(max(0, alpha - theta_tol)))
    - position_tol - geometry_allowance
```

这比真实关节约束更宽松，因此下界仍超过 466 mm 时，可以排除在原容差内的解；无需依赖某次数值 IK 是否收敛。

| 原总图候选 | 竖直抓取偏移 | 用足容差后仍超出的距离 |
| --- | --- | --- |
| 01 | 居中 | 8.566 mm |
| 02 | 轴向 2 mm | 8.229 mm |
| 03 | 轴向 6 mm | 7.575 mm |
| 04 | 轴向 10 mm | 6.951 mm |
| 05 | 轴向 14 mm | 6.357 mm |

所以图里“看着差不多”并不表示完整位姿满足 IK。原生居中候选实际仍有 8.926 mm 位置误差、7.344° 朝向误差。第 6 轴触及限位是一个现象；仅在诊断优化中扩大该轴范围仍未得到解，不能把失败单独归因于这一轴限位。

另外 8 个倾斜候选通过上述几何必要条件，**这不证明它们存在满足全部约束的 IK 解**。其中候选 **07、10 的放置 IK 已通过**，但其对应抓取阶段的返回配置被机械臂自碰撞检测拒绝，因此没有形成可用整链。不能把这些倾斜候选也一概称为“超出臂展”，也不能仅由一次返回配置被拒绝断言目标无解。

后续有效方向是调整抓取关系及对应的放置夹爪朝向，并重新检查完整链路；对前五个固定竖直目标继续增加种子无法解决几何不可达。本轮只分析和解释，没有替换胶棒任务目标或放宽容差。

## 一张图与可复现分析

[13 个候选的可达性解释图](../runtime_data/three_scene/pusht_ignore_internal_glue_20260909/glue_reach_explanation.png)（编号对应此前 13 行完整候选渲染图）。

[机器可读分析](../runtime_data/three_scene/pusht_ignore_internal_glue_20260909/glue_reach_analysis.json)保留各目标、原生结果和距离界。分析工具核对 13 个保存配置的 FK/位置误差、100 个关节配置的工具偏移一致性，以及两个原生放置成功结果与距离界的一致性。

```bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
python3 tools/run_network_isolated.py -- python3 tools/analyze_gluestick_wrist_reach.py \
  --job runtime_data/workcell/jobs/afc820337e2648bf8980dd32ad1000ee \
  --output runtime_data/three_scene/pusht_ignore_internal_glue_20260909
```
