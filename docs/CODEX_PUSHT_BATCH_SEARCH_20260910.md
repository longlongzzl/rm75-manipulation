# PushT 多候选与可变推距（2026-09-10）

用户要求比较多个候选、增大每次推进量，减少推送次数。原实现只有 10 个接触点、固定 12 mm 推距和两步前瞻。

当前实现：

- 在原 T 几何上生成 **20 个接触点 × 3 个方向 × 6 个推距 = 360 个组合**。方向为表面内法线及其左右 0.35 rad 偏角；默认推距为 6、12、20、30、40、50 mm。`push_length_m` 作为基准候选保留，`maximum_push_length_m` 控制上限。
- NumPy 批量计算动作、摩擦情形和未来位姿；默认三步、beam width 12，每轮最多评估约 **27000 个摩擦情形下的动作预测**，并检查预测过程中的完整 T 工作区/障碍物约束。
- 代价同时考虑位置、转角、不确定性、中间状态和动作次数。临近目标时，推距受剩余位姿纠正量限制，避免一次很长的“微调”。这是有限候选和有限前瞻内的择优，不证明全局最优。
- 先用完整夹爪几何筛选接触点和方向，并将当前位姿的接触可行性作为前瞻搜索启发式。未来位姿的机械臂可执行性仍需实际重新规划，不能由该筛选证明。
- 最多取 128 个排序后的提案，GPU **每批最多同时规划 4 条接近轨迹**。按候选 ID 精确关联结果，逐个检查完整动作链，选择最先通过的候选；复用其接近轨迹。未创建多个 GPU 仿真进程。
- 推送方向与表面法线分开保存，因此斜推仍以原表面法线构造相切接触；不以斜推方向错误地偏移接触球。
- 每次完成动作并取得新观测后，按接触点拟合推进、侧滑和转角响应。相同内法线的未观测接触点借用最近已观测点的参数作为估计。候选评分和 GPU 预测碰撞审核使用同一份拟合参数。

撤退继续沿推力反方向后退 10 mm，随后竖直上抬；两段规划保留用户授权的碰撞豁免，结束后恢复原检测。原生诊断发现，原 80 mm 上抬的某个终点有 `link_4 / gripper_base_link` 碰撞球重叠，继续抬高会加重；比该终点低 25 mm 起不再检出。因此默认上抬改为 **50 mm**（可用 `retreat_clearance_m` 设置）。接近/下降的 80 mm 高度、3 mm 直线限制、速度、关节范围和成功阈值不变。

## 证据与运行限制

全程通过 `tools/run_network_isolated.py`，没有连接或操作真机。GPU 工作全部串行，MemoryHigh=6 GiB、MemoryMax=7 GiB、禁用 swap、CPUQuota=200%。仿真实体碰撞仍保留，实际观测决定成功。

相关测试覆盖批量/标量预测的一致性、工作区几何、斜推相切、候选身份匹配、完整链路、碰撞豁免恢复、观测拟合、近目标推距限制、物理循环和回放。只运行 `tests/` 中明确指定的文件，并设置 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`。

探索运行保留原始结果：

| job | 目的和结果 |
| --- | --- |
| `7eff378726c34ea5a1039532a9212876` | 未校正长推：首推 40 mm，T 的 X 实际推进 27.946 mm，但转偏约 27.24°。后续纠偏恶化，主动取消仿真。 |
| `111a38a11c18450eafac1570f55ecac3` | 响应校正：完成 40 mm、30 mm 两次推送；随后因撤退终点自碰撞及候选几何限制失败。 |
| `835a8de04cc44be9a7b821e9ff887cb7` | 斜推、较低撤退终点：第二次推完后位置误差约 10 mm、朝向误差约 1.7°；后续过度纠偏，主动取消仿真。 |

这些运行没有记为成功。拟合输入只取已完成动作、且有前后观测的转换，不使用中断动作或预测位姿。最终校准文件 [response_calibration_v3.json](../runtime_data/three_scene/pusht_batch_long_20260910/response_calibration_v3.json)来自上述运行及原 12 mm 基线，共 15 条转换、6 个接触点；它是同一模拟场景的调试校准，不是独立测试集或真机校准。

可用 [fit_pusht_response.py](../tools/fit_pusht_response.py) 重建校准；工具检查 T/接触几何、原 URDF、计划夹爪关节目标一致，并保存来源哈希。

最终运行 job：`8e1396ece8f14deeba8ef32ed0ebccad`。完整结果写入 [result.json](../runtime_data/three_scene/pusht_batch_long_20260910/physics_090_final/result.json)，各轮真实观测、候选排序、GPU 批次、逐段轨迹、接触记录及视频保存在该 job 的 `physics/` 目录。


## 最终结果：2 次推送成功

`8e1396ece8f14deeba8ef32ed0ebccad` 已结束，状态 **succeeded**，`verification=physics_pose`、`task_success=true`、`steps=2`，用时 179.94 s。动作分别为：

1. 30 mm 夹爪斜推行程；T 从 `[0.349999994, -0.180000022, 0]` 到 `[0.374478310, -0.176110670, -0.015737824]`。
2. 6 mm 夹爪微调行程；连续三次观测确认后的终态为 `[0.375463724, -0.176387921, 0.007207535]`。

夹爪行程不等于物体位移。最终位置误差 **5.79870 mm**、朝向误差 **0.41296°**，满足原有 6 mm / 0.1 rad 阈值和稳定性要求，没有放宽成功条件。T 沿 X 实际累计推进 **25.46373 mm**。

本轮无提前碰 T、静态障碍物接触或未批准 link 碰 T；实体自接触记录仅包含原有安装连接 `flange_spacer_8mm / gripper_base_link`。最大 TCP 跟踪误差 1.357 mm。两次完整链路均执行了后退 10 mm、上抬 50 mm；撤退过程豁免规划碰撞，后续恢复检测。

**150 项相关测试通过**（完整相关集合 148 项通过，随后新增两项并复核受影响的 49 项全部通过）。最终运行记录的源码哈希与交付实现一致，`git diff --check` 通过。

[两次成功推送的短视频](../runtime_data/three_scene/pusht_batch_long_20260910/two_pushes_success.mp4)拼接原视频中的两段，保留原速并标注夹爪行程；[片段时间索引](../runtime_data/three_scene/pusht_batch_long_20260910/final_video_clips.json)保留源视频时间。

复现最终验证：

```bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 \
systemd-run --user --scope --quiet --unit=rm75-pusht-batch-final-20260910 \
  --property=MemoryHigh=6G --property=MemoryMax=7G \
  --property=MemorySwapMax=0 --property=CPUQuota=200% \
  python3 tools/run_network_isolated.py -- python3 tools/run_pusht_physics_validation.py \
  --backend full_arm_physics --case translation --gravity-compensation original-agent \
  --closed-gripper-joint-position .9 --gripper-feedback-geometry \
  --maximum-push-length-m .05 \
  --response-calibration runtime_data/three_scene/pusht_batch_long_20260910/response_calibration_v3.json \
  --output runtime_data/three_scene/pusht_batch_long_20260910/physics_090_final --timeout-s 600
```

重新运行时须选择新的输出目录及 scope 名称，保留这次证据。此结果只说明这个经过仿真响应校准的场景已用两次推送完成，不代表任意目标或真机都能两次完成。
