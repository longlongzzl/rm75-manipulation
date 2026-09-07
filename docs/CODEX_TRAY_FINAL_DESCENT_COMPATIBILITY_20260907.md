# 料盘最后下降段兼容例外 — 2026-09-07

## 授权范围与结论

用户本轮明确确认：**只在料盘取件最后下降段，左右夹爪 link 对所有世界物体免检；机械臂其他 link、自碰撞和其他阶段保持检查；仅仿真，无真机动作。**

这条新授权替代该小段的 target-only 要求，但不授权其他阶段或整条程序全局免检，也不等于严格碰撞验收通过。

- 实现/最终 CPU 测试对应代码树：`8c58973031d30790cbd014b58f876989e25ff6a6`（测试后原样提交）。父提交 `dc6f7c6`。
- 新增显式仿真选项 `--tray-final-descent-compatibility`；工作台默认严格模式不变，没有打开真机或整体 compatibility 模式。
- 真实 cuRobo GPU **cost-level** 检查通过：夹爪世界碰撞被过滤，其他 link 世界碰撞和自碰撞保留，退出恢复。
- four-wall **未完成**：仍在第一块更早的 paired-relation IK 阶段被拒绝，不在新授权范围，未擅自扩大豁免。
- 没有机械臂/夹爪动作；PickPlace 去 MPLib 尚未改动。

## 为什么不能直接复用旧 toggle

只读追踪本机 cuRobo1 代码发现：

1. `MotionGen.toggle_link_collision` 调用 `disable_link_spheres`。
2. 后者把对应机器人球半径设为 `-100`。
3. `ArmBase` 的世界碰撞 cost/constraint 和自碰撞 cost/constraint 都使用 `state.robot_spheres`。

因此旧函数虽名为 `set_world_collision_for_links`，并不能保证仅修改世界碰撞。直接启用会违反“自碰撞保留”的确认条件。

新适配器 `rm75_app/workcell/world_only_contact.py` 不调用该 toggle、不改机器人几何，只给 world cost/constraint 传递过滤后的 tensor **副本**；self-collision 仍收到原始数据。

## 实际边界

- 仅识别运行时 cache 中有非负 `jimu_tray_slot_index`、未 placed 的取件对象，且 `execute_real=false`。
- 仅原始候选的 `*_final_approach_gripper_world_relaxed` constrained segment；其他 IK、运输、放置、抬升、返回阶段继续严格拒绝免检请求。
- 段几何限制：向下不超过 0.080m，横向位移不超过 0.001m；非有限、上升、横移或更长运动拒绝。这是例外适用范围，不是放宽原规划成功公差。
- 可豁免 links：左右 `1_Link`、`2_Link`、`Support_Link` 和 `left_pad/right_pad` 共 8 个。**不含 `gripper_base_link`、arm links 或 attached_object**。
- 最终接近 refresh 保留 table 和 active target，让非夹爪 links 仍检查这些物体。
- 已有 disabled link/world object、不能确认启用的 self-collision、无法识别球映射、CUDA graph/replay cache 均拒绝例外，不自动改原配置解决。
- 过滤仅限持有原 native GPU RLock 的调用线程；其他线程仍收到未过滤数据；正常/异常退出都恢复原 forward。
- 未观察到实际 world-filter 调用或者过程中创建 CUDA graph cache 时，不接受该返回路径。
- 以上是兼容模式，不标记 strict acceptance 或物理成功。

## 测试

```bash
PYTHONPATH=. python -m compileall -q rm75_app tools tests/three_scene
PYTHONPATH=. python -m pytest -q tests/three_scene
PYTHONPATH=. python -m pytest -q tests
```

- compileall PASS。
- 三场景 **95 passed / 0 failed / 0 skipped / 7.59s**。
- 全仓 **417 passed / 0 failed / 0 skipped / 27.23s**。
- 新增 15 测涵盖：world/self 输入隔离、基座保留、异常恢复、已有免检状态拒绝、CUDA graph 拒绝、自碰撞关闭拒绝、线程隔离、shape 变化拒绝、合法下降、非料盘/真机/上升/过长/横移拒绝、早期 paired IK 拒绝。

## GPU 无运动检查

环境沿用 foundationpose310：Python 3.10.20、Torch 2.7.1+cu128、CUDA 12.8、RTX 5060 Ti；扩展使用 R2 本机干净编译缓存 `/tmp/rm75_native_curobo_clean`。

```bash
PYTHONPATH=. PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1 timeout 120 \
  /home/zhangzhao/anaconda3/envs/foundationpose310/bin/python \
  tools/check_native_world_only_contact.py \
  --extensions /tmp/rm75_native_curobo_clean \
  --output runtime_data/three_scene/tray_descent_gpu_cost.json
```

使用 native Jimu 同一 `pick_jiaobang/curobo_rm75_config/rm75.yml`，合成 cuboid/球仅用于验证过滤机制，**不是现场几何资格，也不是整条下降轨迹验证**。

| 实际 GPU constraint 输出（非距离单位） | 开启前 | 例外内 | 退出后 |
| --- | --- | --- | --- |
| 选定夹爪球 ↔ 世界 | 221 | 0 | 221 |
| 其他 link 球 ↔ 世界 | 221 | 221 | 未单独重复 |
| 自碰撞重叠输入 | 251 | 251 | 未单独重复 |

枚举并临时绑定 53 个 world-cost 实例；本检查实际调用过滤器 2 次。原始 self 球没有修改。

分母记录：首次诊断脚本遗漏 formal robot YAML，embedded free-space config 拒绝 world update，exit 1；补上 **原有** Jimu collision YAML 后重跑 PASS，exit 0。共 2 次启动，1 次通过，1 次配置失败；没有编造新的碰撞几何配置来降低检查。

## Four-wall 仿真

```bash
PYTHONFAULTHANDLER=1 PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 timeout 180 \
  /home/zhangzhao/anaconda3/envs/foundationpose310/bin/python \
  tools/run_native_contact_audit.py --scene four-wall \
  --extensions /tmp/rm75_native_curobo_clean \
  --output runtime_data/three_scene/tray_descent_four_wall \
  --tray-final-descent-compatibility
```

- 1 次启动，exit 42，`strict_contact_not_supported`，command_success=false，verified_task_success=null。
- `right_wall` 的 `winner_chain_jimu_parallel_grasp_place_paired_relation_ik` 请求关闭左右 Support_Link 世界碰撞；新模式仍在修改前拒绝，changed_links=[]。
- **没有到达最后下降段，整段 native filter 集成尚未验证。** 不把 GPU cost 检查替代 full-chain 成功。
- triangle-roof：NOT_RUN，前置 four-wall 未完成。
- 其他阶段暂时关闭碰撞的历史行为没有因为此次授权被自动恢复。

## 来源保护与回传

- vendored `--verify-only` PASS，固定基线 + 12 文件 overlay 未变，仍为 807 个输入文件。
- 旧 `/home/zhangzhao/Desktop/lerobot` status SHA256 仍为 `7a7759758ac9b3ed67e54374cdcb10f9837bb3c5507c79bae41348deacb60658`，无修改/reset/clean。
- [机器证据与原始日志](../benchmarks/unified_scenarios/tray_final_descent_compatibility_20260907.json) 包含 GPU 两次启动、four-wall result/contact、全部测试日志和源文件 SHA256。
- Camera、SDK/preflight、gripper、所有物理运动阶梯：**NOT_RUN**。
- 新增/修改仅新适配器、显式仿真 runner、GPU 诊断工具和单测。没有修改旧源代码、vendored 算法或安装在外部目录的 cuRobo。
- 后续阻塞明确是**早期配对 IK 的免检依赖**，不是用户刚批准的最后下降段；需单独处理，不能擅自把免检扩展到该阶段。
