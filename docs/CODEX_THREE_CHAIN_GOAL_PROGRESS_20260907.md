# 三条链路目标推进记录 — 2026-09-07

## 0. 目标、结论与环境

用户目标：PickPlace 重新整理并打通、清除无用 MPLib；Jimu 做明白完整 12 块任务；PushT 继续完整验证。

**总目标仍未完成，保持 active。** 本轮三条线都有代码或验证进展，但不能用离线测试全部通过替代完整 GPU/现场链路。

| 链路 | 本轮已证明 | 仍未完成 |
| --- | --- | --- |
| PickPlace | 原 gluestick 固定场景 cuRobo-only 运行；MPLib 加载列表为空；夹爪开合和释放 settle 类型错误已修复 | 释放后退让规划仍有 WORLD_COLLISION 失败；完整回归与物理成功未证明 |
| Jimu | 四墙 4/4；完整两层墙+屋顶 12/12 native 循环和 final success 均成功 | 不升级为真实磁吸/物理成功；工作台默认严格入口没有擅自切成兼容模式 |
| PushT | 抽出完整五阶段无运动规划接口；完整 motion bridge 故障注入、时序、防重放测试 | 实测推头/物体/桌面/相机配置缺失，完整 GPU 短推仍 NOT_RUN |

- Tested code commit: `39ae5450956e5d38700a9de457fd6229338e62a4`；branch `chatgpt/three-scene-software-closeout`。
- CPU Python 3.12.7；native foundationpose310 / Python 3.10.20 / Torch 2.7.1+cu128 / CUDA 12.8 / RTX 5060 Ti / driver 580.173.02。
- PickPlace/Jimu 是迁移的 cuRobo1；PushT 真实后端是 cuRobo2，本轮不混淆二者。
- Native 扩展沿用 R2 本机干净编译 `/tmp/rm75_native_curobo_clean`，无 opaque `.so` 复制。
- 没有连接机械臂或相机，没有机械臂/夹爪真实动作。

## 1. 旧仓库审计与保护

- 旧 `/home/zhangzhao/Desktop/lerobot` 只读，无 reset/clean/覆盖/新增 overlay。
- `git status --porcelain=v1 -z` SHA256 前后一致：`7a7759758ac9b3ed67e54374cdcb10f9837bb3c5507c79bae41348deacb60658`。
- 旧六入口和直接依赖分类沿用已批准清单；本轮没有重新选择 dirty 文件。

## 2. 迁移完整性

- `7aaff9da22486b7d25557b3795dd258f9b65f10d` + 12 个审计 overlay 文件保持不变。
- `tools/migrate_working_sources.py --target-repo . --verify-only` PASS：**807** 个输入文件。
- 原 source archive 未修改。PickPlace 新适配器在隔离子进程中仅对明确的四个 native 模块做内存 AST 依赖/初始化适配，原候选、规划和物体专用放置算法不重写。
- 四模块为 `rm75_jiaobang_pick_move_v10_perpendicular_to_object.py`、`rm75_jiaobang_pick_real_with_foundationpose.py`、`rm75_jiaobang_pick_place_targeted.py`、`rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place.py`。
- 该适配既用于固定场景 runner，也接入 `run_working` 的 PickPlace 工作台入口；没有卸载环境中的 MPLib 包或修改外部安装。

## 3. CPU 与代码完整性

```bash
PYTHONPATH=. python -m compileall -q rm75_app tools tests/three_scene
PYTHONPATH=. python -m pytest -q tests/three_scene
PYTHONPATH=. python -m pytest -q tests
```

- 最终 compileall PASS，包含 vendored snapshot。
- 三场景 **150 passed / 5.50s**；全仓 **472 passed / 23.70s**，0 failed / 0 skipped。
- 中间一次 149 项测试出现 1 failed / 148 passed：旧“新观测正例”给前后两帧相同采集时刻，与新增严格单调要求冲突。修正正例时间，并增加相等时刻也必须拒绝的测试，没有放宽 freshness。
- 更早一轮 144 / 466 通过，不作为最终总数；原始测试输出全部归档。

## 4. 前端与入口

- PickPlace 工作台子进程现已安装 cuRobo-only 适配，原 argv/输入桥/停止桥不变。
- Jimu 本轮完整 12 块验证来自显式 native 仿真 runner，不宣称工作台默认 strict 路径通过。
- PushT 新 `plan_push` 供完整 GPU no-motion 验证使用；原执行入口仍要全链完成、重新观测后才发送动作。
- 本轮三个 tab 的浏览器手工检查均 **NOT_RUN**；有关 CPU 前端/契约测试随全量通过，不能替代浏览器证据。

## 5. PickPlace — 清除运行链 MPLib 依赖

### 实现边界

- 不再构造第二套 MPLib Planner；原 FCL obstacle 记录改成纯数据容器。真实 world 仍由原 `_scene_obstacles_to_curobo_world` 构建。
- 旧 collision query 使用当前 demo 绑定的真实 cuRobo planner，不能确认 world/self/payload 时明确失败；不是 Jimu 旧 shim 那种无条件返回空碰撞列表。
- 移除 MPLib return fallback，原 grasp/place 参数不变；任何旧 IK/规划 fallback 被调用都会 typed failure，而不是返回伪成功。
- 四入口编译后的外部 MPLib import 已全部覆盖，测试遍历实际 native PickPlace 顶层 Python 文件防漏。
- `step_sim` 保留动作值、转为 ManiSkill tensor；失败抛出不能被旧 `except Exception` 当 warning 吞掉的类型。
- 真实异常栈定位到混用 ManiSkill 环境的 `compute_dense_reward`：bridge 将 `obj_xy_shortest_edge_vector` 设为 NumPy，而 reward 用 `torch.linalg.norm`。仅转换该字段表示、device、dtype，不改变几何、reward 数学或成功条件。

### 真实分母：5 次启动

| 次数 | 退出 | 实际结果 |
| --- | ---: | --- |
| v1 | 139 | 已越过旧 Planner 构造，但 direct 场景切换漏掉 FCL import，发生 SIGSEGV；没有 result.json |
| v2 | 0 | 补齐四模块；native 1/1 和 final success，但夹爪/settle 类型 warning 仍在，不能算完整仿真通过 |
| v3 | 0 | 动作 Tensor 转换本身未解决 reward 字段错误，warning 仍在 |
| v4 | 42 | 将 simulator error 升为明确失败后获得完整真实堆栈，定位 reward 几何字段 |
| v5 | 0 | 1/1 和 final success；MPLib modules 为 `[]`；夹爪开合、60 步 settle 实际完成，类型 warning 消失 |

v5 仍出现：

```text
post_place_clearance_replan_after_reverse_collision ... INVALID_START_STATE_WORLD_COLLISION
post-place clearance planning failed after release; settling the object without moving the arm away first
```

因此本轮证明的是**原固定场景主抓放命令链恢复且 MPLib 依赖已从工作链清除**，不是“退让/返回全链已验收”。下一步应检查释放时 world、目标/夹指接触和 reverse-clearance 路径的真实碰撞证据，不关闭碰撞让它通过。

原 dry-run 语义是计划/预览，不是物理轨迹执行证明；`verified_task_success=null` 保留。没有减少候选或改变成功公差。

```bash
PYTHONFAULTHANDLER=1 PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 timeout 420 \
 /home/zhangzhao/anaconda3/envs/foundationpose310/bin/python tools/run_native_pickplace.py \
 --extensions /tmp/rm75_native_curobo_clean --output runtime_data/three_scene/goal_pickplace_v5
```

固定原 `gluestick_desk_regression.json`，无 `execute-real`。其他多物体/frozen 场景本轮 **NOT_RUN**，不将单场景覆盖外推。

## 6. Jimu — 12 块任务

- 上轮第 12 块失败不是已证明的路径碰撞，而是 contact scope 不认识 `scene_obstacle_left_roof_triangle` 这个已附着目标的旧 world cache 别名。
- 本轮通过最近一次 world refresh 绑定当前 source 身份；只有该 refresh 明确排除了 source、source 已附着且球数 > 0、world 中不再存在同名对象时，才认可该禁用缓存副本。
- 不允许邻居、仍在 world 中的对象、无身份、未排除 source、无 payload 或 0 球条件通过；新增 7 个测试覆盖。
- 原顺序执行：**four-wall 4/4**，4 条搬运路径 / 178 点；然后 **triangle-roof 12/12**，两次日志均有 `final success = True`，exit 0。
- Roof 使用原两层完整场景：4 底层墙 + 4 第二层墙 + 4 三角屋顶；本轮 1 次完整 roof 启动，无减块。
- Roof 搬运返回候选/重试累计 **17 条路径 / 816 点**逐点检查通过，不等于 17 个执行循环。
- 其他接触阶段保留用户已批准的兼容 world 例外；搬运世界全部检查、只成对豁免夹指与 payload，附着球保留；不称为零例外 paper strict acceptance。
- 原角色顺序、料槽重试、partial/full-open、retreat、返回值语义未改写；原日志均归档。独立实际磁吸/物理成功 **NOT_RUN**。

```bash
PYTHONFAULTHANDLER=1 PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 timeout 300 \
 /home/zhangzhao/anaconda3/envs/foundationpose310/bin/python tools/run_native_contact_audit.py \
 --scene four-wall --extensions /tmp/rm75_native_curobo_clean \
 --output runtime_data/three_scene/goal_jimu_four_wall_v1 --transport-world-checked-compatibility
```

通过后相同命令改 `timeout 600`、scene `triangle-roof`、output `goal_jimu_roof_v1`，原冻结 SAM6D JSON 不变。

## 7. PushT — 完整链软件验证与 GPU 阻塞

- 新 `plan_push` 规划并审计 **approach → descend → contact → push → retreat** 全部五段，不调用 arm.execute，也不连接 hardware。
- 原 `execute_push` 只有获得整个 `PreparedPush` 后才重新观测、检查目标漂移和关节起点，再依次执行。每个 emitted/timed 路径点仍做 world/self 与 TCP corridor 检查；目标扫掠 ensemble 保留。
- 新增 15 个 motion bridge 测试（Cartesian fake，不是 GPU/物理）：五段全部预规划；任一阶段 planner failure、arm/table/self/neighbor 碰撞、corridor 偏离、旧观测、漂移、时间倒退均禁止全部动作；world 开关恢复；speed_mps 改变轨迹时长并满足 Cartesian 速度上限。
- 新增资格清单与相同/倒退时间戳拒绝测试；序号增加不再能掩盖采集时间倒退。
- 新工具 `tools/check_pusht_chain.py` 可在确认 profile + fresh observation JSON + 7 轴 q JSON + goal 后，用实际 cuRobo2 完整规划五段并导出全部轨迹；只有 read-only joint provider，没有 SDK import/arm connection。
- 实际执行现有 profile 的资格检查 **exit 2 / qualification_incomplete**；完整 GPU cases **0，NOT_RUN**，不造工具姿态或把示例参数升级为实测。

当前缺失字段：

```text
motion.push_tcp_z_m
motion.object_centroid_z_m
motion.object_height_m
motion.tool_quaternion_wxyz
motion.pusher_contact_links
motion.static_collision_objects
motion.tool_collision_geometry_verified
observer.T_base_camera
observer.T_marker_object
observer.marker_size_m
```

现有 `integration_qualified=false`、`hardware_reviewed=false` 未更改。已询问用户实测配置文件路径；当前 profile 仍为空。完整 CPU bridge 测试不替代 GPU / ManiSkill / 实物验证。

```bash
PYTHONPATH=. python tools/check_pusht_chain.py \
 --profile runtime_data/three_scene/machine.json \
 --output runtime_data/three_scene/goal_pusht_qualification
```

输入齐全后的命令还需 `--observation <fresh.json> --joints <q_rad.json> --goal <x y yaw>`；工具不会自动填充或修改 profile，不会发送动作。

## 8. Camera / tracking

真实采集、曝光时刻资格、丢失恢复、RRTrack→PushT 实际接入、observe→push→observe 本轮 **NOT_RUN**。PushT 使用数据契约/故障注入验证 freshness，不捏造真实帧。

## 9. RealMan no-motion

SDK connection/preflight、joint feedback、Stop API、gripper backend 均 **NOT_RUN**。没有机械臂 IP 连接；新 PushT 工具只有读取提供的关节 JSON，不声称关节实测已完成。

## 10. 物理运动阶梯

低速自由空间、单夹爪、单/多 PickPlace、单/多 Magnetic、PushT 单推/重观察/闭环全部 **NOT_RUN**。

## 11. 回传与后续顺序

1. PickPlace：处理上述释放后退让 WORLD_COLLISION，补足更广原固定场景及工作台端到端证据；不把旧 native success 当作退让成功。
2. Jimu：保留本轮 12/12 原始证据；若要工作台兼容仿真入口，显式按可信 profile 接入，不自动放开默认 strict/real gate。
3. PushT：取得确认的 tool/table/T/tracking 配置后运行新 GPU 完整链工具，再继续前端/相机/SDK no-motion。真实动作仍须单独现场授权与资格。

本轮代码 `39ae545`；原始日志、每次失败/成功分母、源文件 SHA256、测试输出、来源核验见 [机器证据 JSON](../benchmarks/unified_scenarios/three_chain_goal_progress_20260907.json)。

**这是一轮可回溯进展，不是三条 demo 全部完成。**
