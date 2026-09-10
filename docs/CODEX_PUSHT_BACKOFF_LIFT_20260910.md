# PushT 后退再上抬（2026-09-10）

用户要求：“撤退就直接上抬就行，往回一点上抬……往回上抬的过程忽略碰撞。”

实现为推完后沿推力反方向水平后退 **10 mm**，再沿世界 Z 轴上抬原来的 **80 mm**。`retreat_backoff_m` 可在 motion profile 中设置，默认 0.01 m。两段分别求解、检查直线轨迹、按原速度和加速度约束计时，再合并为原有 `retreat` 阶段，保留五阶段执行/回放格式。

撤退两段暂时禁用规划器及 IK 求解器的全部碰撞球，并跳过这两段的碰撞审核。使用原生负半径禁用语义和原地修改，以作用到 CUDA graph 引用的模型；通过 `finally` 恢复进入时的精确半径，保留已有禁用球及模拟夹爪反馈几何。关节范围、IK 位姿要求、3 mm 直线路径限制、速度/加速度限制继续检查。前四个阶段沿用现有碰撞规则。

碰撞豁免是规划策略。全臂物理仿真仍使用实体碰撞模型，记录实际接触，不把穿透产生的运动当作无接触运动。所有验证通过 `tools/run_network_isolated.py`，没有连接或操作真机。

## 验证

- 正式测试 **76 项通过**：撤退两段方向、时间与链路连续性、前四阶段碰撞检测、异常及下一次规划前的恢复、共享/独立模型张量恢复、IK 与回放兼容性、物理循环。均显式指定 `tests/` 文件，禁用 pytest 插件自动加载。
- 原生 GPU 障碍物对照：正常时 16 个接触、0 个成功 IK；豁免时 0 个接触、101 个返回成功 IK 行；恢复后重新检测到 16 个接触、0 个成功 IK。两个模型的球数据完全恢复。见 [native_exemption_verified.json](../runtime_data/three_scene/pusht_backoff_lift_20260910/native_exemption_verified.json)。这项对照只验证豁免开关，不代表障碍物内部的配置可以实际执行。
- GPU 验证与物理仿真串行，使用 MemoryHigh=6 GiB、MemoryMax=7 GiB、MemorySwapMax=0、CPUQuota=200%。

本轮全臂物理 job：`ac073a30e88d4281acd57ebd88f067a2`，闭合关节目标 0.9 rad、原重力补偿、模拟夹爪反馈几何。

```bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 \
systemd-run --user --scope --quiet --unit=rm75-pusht-backoff-lift-20260910 \
  --property=MemoryHigh=6G --property=MemoryMax=7G \
  --property=MemorySwapMax=0 --property=CPUQuota=200% \
  python3 tools/run_network_isolated.py -- python3 tools/run_pusht_physics_validation.py \
  --backend full_arm_physics --case translation --gravity-compensation original-agent \
  --closed-gripper-joint-position .9 --gripper-feedback-geometry \
  --output runtime_data/three_scene/pusht_backoff_lift_20260910/physics_090 --timeout-s 600
```

结果写入 [result.json](../runtime_data/three_scene/pusht_backoff_lift_20260910/physics_090/result.json)，逐轮规划、观测、接触及视频写入该 job 的 `physics/` 目录。各 `push_stage_audited` 事件明确区分前四段 `collision_checks=true` 与撤退 `false`，并保存后退和上抬各自的目标、误差和耗时。


## 本轮物理结果

仿真在 **302.93 s** 后结束：连续完成 **5 次**完整推、后退、上抬，第 6 次在 `approach` 规划失败，整项任务状态仍为 failed。之前第 2 次撤退阻塞已越过。

T 的最终观测为 `[0.374761224, -0.184961766, -0.090875387]`（x/y 单位 m，yaw 单位 rad）。x 从 0.349999994 m 增加 **24.76123 mm**。不可把五次成功推送当作达到完整目标。

第 6 次规划目标端点的 IK 位置误差约 0.000088 mm，但轨迹失败；起点诊断显示 `link_4 / gripper_base_link` 的规划碰撞球线性重叠 **1.69210 mm**。这发生在恢复检测后的重新接近阶段。未将撤退豁免扩展到接近阶段。

物理仿真记录中：提前碰 T、静态障碍物接触、未授权 link 碰 T 均为 0；自接触只记录了原有安装处 `flange_spacer_8mm / gripper_base_link`，未记录 `link_4 / gripper_base_link` 实体接触。后者是规划球模型的判定，不能称为实体网格穿透深度。最大 TCP 跟踪误差为 1.350 mm。

首轮后退和上抬的规划路径误差分别为 0.01150 mm、0.01464 mm。完整视频保留在 job 的 `physics/closed_loop.mp4`；[16 秒后退上抬片段](../runtime_data/three_scene/pusht_backoff_lift_20260910/backoff_then_lift.mp4)来自其中第 57–73 秒，未改变仿真速度。
