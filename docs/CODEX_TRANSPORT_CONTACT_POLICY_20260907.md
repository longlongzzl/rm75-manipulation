# Jimu 搬运碰撞策略回传 — 2026-09-07

## 0. 基本信息与授权

- Tested code: `f52a722b2187a3b78eac175cc7b6761decf8cae5`；branch `chatgpt/three-scene-software-closeout`。
- 用户最新边界：其他接触阶段保留旧兼容例外，**搬运完整计算世界碰撞，只对被抓物体与夹指的正常夹持接触作成对豁免**。此范围替代上一轮仅料盘最后下降段的授权；历史结果见 [上一轮报告](CODEX_TRAY_FINAL_DESCENT_COMPATIBILITY_20260907.md)。
- 仅新显式仿真选项 `--transport-world-checked-compatibility` 生效；默认严格入口、真机入口未切换。
- CPU Python 3.12.7；GPU foundationpose310 / Python 3.10.20 / Torch 2.7.1+cu128 / CUDA 12.8 / driver 580.173.02 / RTX 5060 Ti。
- Linux 6.8.0-137-generic；ManiSkill 3.0.0b22；Jimu 使用本机 cuRobo1（不是 cuRobo2），其源码提交 `d64c4b005459db10c5dd867d8b30a87d5bda9bdb`。
- 扩展缓存 `/tmp/rm75_native_curobo_clean`，沿用 R2 本机干净编译的 5 个扩展。SDK 未连接，未使用 Robot IP。

## 1. 旧仓库保护

- `/home/zhangzhao/Desktop/lerobot` 只读；HEAD `36798efbd12814841607951c9af470b309b34fd3`。
- `git status --porcelain=v1 -z` SHA256 仍为 `7a7759758ac9b3ed67e54374cdcb10f9837bb3c5507c79bae41348deacb60658`。文本格式或 `-uall` 的哈希不同，不可混用。
- 六入口/直接依赖分类沿用已批准的逐文件 overlay 清单；本轮没有新增迁移选择，没有修改/覆盖/reset/clean 旧仓库。

## 2. 迁移完整性

- 原固定基线 `7aaff9da22486b7d25557b3795dd258f9b65f10d` + 已审计 12 文件 overlay 不变。
- `PYTHONPATH=. python tools/migrate_working_sources.py --target-repo . --verify-only`：PASS，807 个输入文件；manifest 为 `rm75_app/_vendor/working_snapshot/MIGRATION_MANIFEST.json`。
- 没有修改 vendor、碰撞几何、外部 cuRobo 安装或旧算法；代码仅在新适配器中作进程内包装。

## 3. 代码完整性与 CPU 测试

```bash
PYTHONPATH=. python -m compileall -q rm75_app tools tests/three_scene
PYTHONPATH=. python -m pytest -q tests/three_scene
PYTHONPATH=. python -m pytest -q tests
```

- 最终 compileall PASS（包括 rm75_app 下的 vendored snapshot）。
- 三场景 **114 passed / 4.31s**；完整 tests **436 passed / 22.47s**；0 failed、0 skipped。
- 本轮新增 19 测：成对豁免不改几何/邻接、阶段识别、搬运逐点检查、payload/table/world/self 前置条件、contact mask 泄漏拒绝、CUDA graph 加速关闭、异常恢复、实际碰撞点拒绝、缺席 table 缓存与真实 table 禁用的区分。
- 第一轮完整测试为 113 / 435 通过；增加 absent-table 单测后重跑得到上述最终结果。focused 最终 19 passed / 0.83s。

## 4. 前端

PickPlace / Magnetic / PushT 的前端相关离线单测包含在全量测试中；本轮未额外启动浏览器检查页面、预览、轮询或停止流程，均记 **NOT_RUN**，不以单测替代浏览器证据。

## 5. PickPlace 回归

本轮专注 Jimu 搬运策略，PickPlace frozen/GPU/仿真为 **NOT_RUN**。之前定位的旧 PickPlace MPLib 构造依赖尚未修改；不能宣称 PickPlace 已完成 cuRobo-only 清理。Jimu portable 原有 no-MPLib 路径未改。

## 6. Magnetic 回归与碰撞边界

### 策略

- 旧接触阶段请求的 world 例外保留，包括 paired placement IK、取件最后下降、首次抬升、放置末段和原退让兼容请求。只允许原 gripper/payload links，不允许 arm links。
- 这些例外只过滤 **world cost/constraint 的输入副本**，不再用旧 toggle 修改共享机器人球；自碰撞始终使用原始球。
- 搬运禁用任何 link/world 例外，夹爪、机械臂、附着物体对当前模型中的世界障碍物均检查，强制包含 table；目标原世界副本被附着 payload 替代，避免把同一个物体当作世界障碍和 payload 计算两次。其他物体不得排除。
- `attached_object` 只与 8 个左右 finger/pad link 成对忽略；去掉旧配置中它与 base_link、gripper_base_link 的额外忽略。机械臂原有无关邻接忽略保持不变。
- 搬运前后验证 payload 仍附着（6 球）、检查开启、没有 contact mask 泄漏；对每条返回 `q_path` 的**每个已有路径点**再次检查 world/self。没有下采样，不能据此宣称数学上的连续时间无碰撞证明。
- 新模式关闭 CUDA graph batch IK 加速，避免接触例外被捕获后重放；候选、种子、阈值和成功公差不变。
- 只接入固定已审计 native 仿真 runner，**未接入真机 worker**。

### 两次真实停止与修复

1. four-wall v1：exit 42，首次抬升 `active_self_collision_not_confirmed`，0 个完整循环。本机 cuRobo `MotionGen.check_start_state` 的 WORLD_COLLISION 提前返回分支漏恢复 self constraint。新适配器用 `finally` 恢复进入时 world/self 开关，原失败返回/异常不变，不修改外部 cuRobo。
2. four-wall v2：exit 42，前 2 循环返回成功；第三块 paired IK 因 `preexisting_disabled_world_objects` 停止。原非搬运场景不包含 table，但障碍缓存保留了禁用的 `virtual_table_plane` 名字。仅当非搬运当前 world 中确实没有 table 时允许此缓存元数据；存在但被关闭的 table 仍拒绝，搬运仍强制 table 存在且启用。该次已审计 2 条搬运路径 / 88 点。
3. four-wall v3：exit 0，**4/4 native 循环 success=True、final success=True**；4 条搬运路径 / **178 点**逐点检查通过。

这三次是不同修订下的诊断分母，不能合并称为最终版本多次稳定性测试。最终代码仅完成一次 four-wall 全链仿真回归。

### Triangle-roof

- 使用原冻结场景的完整 12 循环（4 面底层墙、4 面第二层墙、4 面三角屋顶），不是缩减场景。
- 第一次：外层 `timeout 240` 截断，exit 124；日志已完成 **11/12** 循环，无最终 result.json，无 final success；此前 16 条返回候选搬运路径 / 832 点检查通过。
- 第二次：仅外层进程运行预算改为 `timeout 600`，代码、规划超时、种子、候选和碰撞条件均不变；exit 42，仍完成 **11/12**，第 12 块首次抬升前被拒绝。
- 明确原因：`preexisting_disabled_world_objects`，禁用缓存为 `active_target_object` 和 `scene_obstacle_left_roof_triangle`，后者不在当前 world 对象中；step 为 `joint_start_tcp_up_lift_grasp_direct_grasp_tilt_toward_robot_4deg_prez_0_fast_ik_world_z`。当前 contact scope 仅识别 `active_target_object` 这个目标副本名称，未关联此 source-specific mesh 缓存别名。
- 第二次共 17 条返回候选搬运路径 / **880 点**检查通过；含规划候选/重试，**不等于 17 个已执行循环**。
- 因此 **triangle-roof 未完成**，不能报告 12/12 或 final success；这是接触 scope 的禁用对象识别拒绝，不是已证明路径发生碰撞。下一步应关联当前附着目标身份后处理该副本，不能白名单放过任意 disabled mesh；本轮没有继续扩展豁免或关闭检查。
- 两次诊断均保留完整日志。冻结场景中仍有指向旧仓库 `pick_jiaobang/meshs/red_triangle.glb` 的绝对路径，运行时只读使用；该证据不证明屋顶场景已完全摆脱旧仓库路径依赖。

### GPU cost-level 证据

两次诊断启动均 exit 0 / PASS；第二次增加了真实 cuRobo 提前返回恢复检查。

| 合成碰撞输入 | GPU constraint 输出（非距离单位） |
| --- | ---: |
| payload ↔ left_pad | 0 |
| payload ↔ base_link | 136 |
| payload ↔ link_4 | 111 |
| payload ↔ world | 221 |
| finger ↔ world（无接触 mask） | 221 |

接触 scope 内 finger/world 为 0，退出恢复 221；其他 link/world 保持 221；自碰撞重叠输入保持 251。真实起点世界碰撞仍返回 invalid，随后 world/self constraint 均为 enabled=True。

上述合成输入仅检验机制；不替代现场几何、真实抓持、稳定性或物理成功证据。native `main()` 返回 None，因此报告保留 `status=native_return_unverified`、`verified_task_success=null`；命令和日志成功不升级为物理成功或零接触例外的 strict acceptance。

### 复现命令

```bash
PYTHONPATH=. PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1 timeout 120 \
  /home/zhangzhao/anaconda3/envs/foundationpose310/bin/python \
  tools/check_native_world_only_contact.py --extensions /tmp/rm75_native_curobo_clean \
  --output runtime_data/three_scene/transport_payload_gpu_v2.json --transport-payload-pairs

PYTHONFAULTHANDLER=1 PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1 timeout 240 \
  /home/zhangzhao/anaconda3/envs/foundationpose310/bin/python \
  tools/run_native_contact_audit.py --scene four-wall --extensions /tmp/rm75_native_curobo_clean \
  --output runtime_data/three_scene/transport_checked_four_wall_v3 --transport-world-checked-compatibility
```

Triangle 首次使用相同命令，scene 改 `triangle-roof`、output 改 `transport_checked_triangle_roof`；第二次只把外层 timeout 改为 600、output 改为 `transport_checked_triangle_roof_v2`。runner 固定 two layers / triangle profile / 原冻结 SAM6D JSON，禁用 AprilTag 相机输入，未缩减候选。

原始日志、每次 result/contact、源文件 SHA256 与测试输出一并归档于 [机器证据 JSON](../benchmarks/unified_scenarios/transport_contact_policy_20260907.json)。

## 7. PushT GPU / cuRobo2

本轮未改 PushT；GPU cases、tool profile、实际推送链路复验 **NOT_RUN**。相关 CPU 单测属于完整 tests，不替代 GPU 或运动证据。

## 8. Camera / tracking

本轮相机/新鲜时间戳/丢失恢复/observe→push→observe **NOT_RUN**。Jimu 仿真仅读取已有 `assets/calibration/camera_extrinsic_opencv.npy` 和原冻结场景，不连接相机、不生成新标定。

## 9. RealMan no-motion

SDK connection/preflight、joint feedback、Stop API、gripper backend 本轮均 **NOT_RUN**。**没有任何真实机械臂或夹爪动作。**

## 10. Physical motion ladder

低速自由空间、单独夹爪、单 PickPlace、多物体 PickPlace、单 Magnetic、2/4/6+ 实物结构、PushT 单推/重观察/闭环均 **NOT_RUN**。

## 11. 总结与待办

- 已实现用户本轮搬运碰撞边界；显式仿真入口 four-wall 4/4 完成，triangle-roof 11/12 后停止。其他接触兼容例外保留，不宣称屋顶完成或全面严格碰撞验收。
- 屋顶下一步：对 contact scope 的当前附着目标 mesh 缓存副本做身份关联与回归；绝不通过允许所有 disabled objects 跳过检查。
- PickPlace 的 MPLib 清理、真机接入/资格验证、物理成功证据仍未完成；PushT 未改。无新硬件授权时不向运动阶梯推进。
- 本轮代码修改：`transport_contact.py`、`world_only_contact.py`、两个 native 验证工具、`test_transport_contact.py`；代码提交 `f52a722`。上一轮下降段代码提交 `8c58973` 与其历史报告同时 push。
