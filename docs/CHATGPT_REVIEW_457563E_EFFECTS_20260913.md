# 457563e 效果审阅：新观测策略已接入，但完整任务仍未通过

审阅提交：`457563e3c9fc9395da6d3bcfe1f0ba2ad200c295`。对照起点：`f0bb5a833b7d748675bbb0aed84829a07305eeea`。本次读取时共享分支指向审阅提交。

本文件只记录代码审阅、已提交运行证据的核对和下一步验收重点。ChatGPT 本轮没有运行新的 Python 回归、商业模型、GPU/PhysX、相机或机械臂，也没有修改生产算法。后述“通过”有明确归属，不沿用旧成功冒充本轮任务成功。

## 1. 效果总评

| 项目 | 本次可确认的进展 | 仍未证明的效果 |
|---|---|---|
| 抓放观测时机 | `PairedGraspPlace` 已接到笔的正式 runtime；保留两个原子阶段及联合计划；持物期禁止 SWM 对象同步，抬升改用非视觉反馈和预测 attachment 审核 | 最新正式 worker 仍停在规划，未运行到释放退让后的新观测；不能宣称完整原生观测窗口已验收 |
| 笔的规划 | 位置求解容差收紧至 0.0001 m；八个规划抓取末端 FK 残差最大 9.657056180250437e-8 m；两例闭爪预测未拒绝、六例拒绝 | `command_success=false`，`Native atomic phase has no feasible trajectory`；不是 2/8 抓取成功，也不是完整抓放成功 |
| PushT 搜索失败分类 | 明确的 `CandidateInfeasible` 返回 feasible=false；致命异常仍关闭事务；A 失败而 B 获选的替身回归已存在 | 原生实际验证主要是既有正例 1880 个碰撞样本重审；原生 A 拒绝/B 获选与正式连续两段参数消费尚无本批证据 |
| FoundationPose | 已用实际权重及 mustard0 RGB-D/模型运行 `FoundationPoseRefiner.global_register` | 不是 RM75 数据；曝光来源、世界外参缺失，尚未进入 SWM 检查点，也未量化该帧定位误差 |
| 全量回归 | 最新提交携带的完整日志为 1972 passed / 1 failed / 1 warning | 不是全绿；失败需要修正后重跑，不能用之前 1965 passed 替代 |

总状态仍应为 `PARTIAL_DELIVERY`。方向没有重新偏回“持物期必须视觉跟踪”，但功能收尾仍落后于组件验收。

### 证据归属

最新笔和全量测试的记录声明：被测树为 `396bc2e` 加保存的 `source.patch`，附 `source_input_sha256.txt`。这比只写一个过时 HEAD 更好，但下一次须继续记录实际代码、输入和配置摘要；不能笼统说所有结果都直接在 clean 457563e 上运行。

## 2. 新视觉策略：认可当前实现方向，不再恢复旧六视觉检查点

`paired_grasp_place.py` 的正常路径：

```text
before_grasp 新观测
  → 联合 grasp/place 求解
  → 空手时 pre_execute_grasp 刷新、检查是否发生变化
  → 完整联合阶段审核
  → grasp 原子执行（预测 attachment，不重新定位目标物体）
  → 非视觉反馈重审 lift / place 剩余路径
  → place、释放、退让
  → after_release_retreat 新观测
  → 最终目标关系验证
```

空手规划后的刷新不属于被取消的持物期定位，不必为“只能两次调用”而删掉它。初始化探测同样不能计为任务成功证据。

`scene.py` 保留原 measured rows 和时间，将操作期状态放到独立 `execution_state`；期间 world.valid=false，避免一般消费者把最后一次观测当作当前真实状态。`CheckpointSynchronizer` 拒绝在执行中的配对窗口刷新物体。`PredictedLiftAudit` 取代该运行模式下的 `MeasuredLiftAudit`，不调用新的物体定位。`native_execution.py` 在持物期间切换到关节读回，释放退让后再允许对象观测。上述修改应保留。

目前的软件探针证明了无持物期采集、错误落点请求重抓、取消阻止 place 等逻辑，但它明确是 fixture。正式笔任务的可见边界只有初始化和 before_grasp，说明它还没到能验证整个执行窗口的位置。

下一次完整运行后只需保留小而可追踪的事件链：联合计划 ID、两个 actual_action_id、持物期物体采集调用数为零、原 measured rows 未更新、释放退让完成时间、之后的新帧/结果及最终验证。不要把“从未执行，因此零次违规采集”写成完整策略成功。

额外边界：SIM 中的目标级双指接触力仍用于闭爪反馈，它不是视觉，但也不能自动当成实体夹爪已具备的传感能力。SIM 控制器证据与硬件可用性继续分开。

## 3. 笔：精度改动带来了筛选进展，但下一轮不要再盲目收紧 IK 或加候选

最新日志中，以下两个精确规划末端的闭爪预测没有发现需拒绝的接触：

- `grasp_20_+0deg_tilt_+30deg_axis_-48mm__axis_solution_02`
- `grasp_48_+180deg_tilt_-30deg_axis_-48mm`

这是确定的中间进展。它们仍标为 `model_qualified=false` / `skill_qualified=false`。其余六个候选被拒绝，最后没有生成完整原子轨迹，主动作没有执行；清理结果三项均为 true。

求解器内部 FK 残差小于 1e-7 m 只是数值模型与目标的一致性，不代表真机、视觉或夹爪的物理精度达到这个量级。不要继续追求更小浮点误差代替任务效果。也不能把“旧运行五次均拒绝”和“新运行八次有两次未拒绝”当作严格同候选 A/B 成功率提升。

### 3.1 当前缺的是两条存活关系的首个失败阶段

代码里 closure 之后的 lift 和 `_placement_stages` 仍有多个 `None → continue`，最终统一抛 `no feasible trajectory`。现有日志不足以判断每条存活关系到底失败于：

```text
attachment 建立
→ lift_world_z / lift_tool_z
→ placement 目标预筛
→ preplace 路径
→ place 直线
→ 后续阶段审核
```

下一轮只针对这两条真实存活关系，给当前原有函数增加有界结果字段，不另建诊断系统：candidate_id、lift 类型、阶段、原求解 status/返回 None、目标过滤前后数量、位置/角度差、首个碰撞对、输入/attachment 摘要与耗时。每个未执行阶段标 NOT_ATTEMPTED，不能猜它已经失败。保留同输入和原预算。

### 3.2 高优先级代码核查：配对后把 native_relation 当成唯一刚体终点过滤

静态确认的新调用关系：

```text
plan_pair 设置 _paired_place_request
→ grasp lookahead 把这个非空 request 传入 _placement_stages
→ 对每个 planning_target_object_pose 做完整位置与 SO(3) 差值比较
→ 超过 request.position_tolerance_m / rotation_tolerance_rad 则 continue
```

`_placement_stages` 没有按 `request.goal_predicate` 分支。与此同时，笔的 place 是 `native_relation`，最终通过原 `inside` 几何谓词验证；原 builder/关系求解器可以在目标对称性和功能关系允许范围内产生不同姿态。

**这意味着配对接线增加了一个可能排除原合法候选的刚体过滤。** 静态代码可确认过滤存在；当前记录尚未证明上述两个候选是否在这里被全部过滤，不能宣布这就是本轮根因。

最小验证：让这两条关系分别报告候选过滤前后数、各 raw target 与 canonical target 的差，以及原任务关系能否接纳该候选。`rigid_pose` 继续严格比较；`native_relation` 必须按原任务允许的目标集合/对称性和原编译器来源核对，不能把它偷换成一个任意的唯一姿态，也不能直接跳过目标校验。保持物体身份、底板/容器参照、库存和接触规则。

### 3.3 顺带核查：任务目标位姿与松爪位姿不是总相同

原 `FixedSceneAtomTaskBuilder` 同时提供 `planning_target_object_pose` 与 `planning_release_object_pose`，并可能施加原 release clearance；当前 `_placement_stages` 只取前者重建 TCP，且把该姿态同时写作 released_object_pose。

对二者相同的候选不应改变行为；二者不同时必须保留原松爪目标，最终仍按原任务落定目标/关系验证。不要降低 clearance，也不要把已经存在的 release offset 再叠加一遍。此项在旧适配中也存在，并非必然由这五个提交新增；当前笔 inside 场景是否受影响尚未证实，后续迁移其他对象前需要回归。

## 4. PushT：分类修复是正确方向，下一步要原生负例和在线消费

`_NativePushContext.evaluate()` 现在仅捕获明确的 `CandidateInfeasible`，在私有 GPU context 退出后返回该候选/参数的失败结果。原身份损坏、非有限数据、未知异常、超时和取消仍传播并终止事务，不能一律吞掉。这处理了上一轮指出的设计缺口。

实际核对的测试区分了：A 在一个必需假设下不可行、B 全部可行则选 B；全部候选失败则不返回计划；致命异常关闭 factory。该证据是带替身的控制流测试，不是实体或完整 PhysX 搜索成功。

已有实际原生正例 1880 样本审核可以保留，但不能替代“原生 A 真实碰撞拒绝后，B 在同一搜索中继续并被选中”。先完成这个有界原生负例，再完成原 Goal 中两段实际主仿真动作，明确第一段 posterior 的版本/摘要进入第二段正式规划。别继续只重复保存候选评分表。

## 5. FoundationPose：真正运行了权重，但还只通过模型调用层

本次 `offline_fp_01` 的模型、图片、深度、掩码和权重均有摘要，实际调用项目中的 `FoundationPoseRefiner.global_register`，产生了 T_camera_object；这不是 mock，应认可。

需要保留三个事实：

1. 输入是 FoundationPose 的 mustard0 示例，不是工作台对象及 RM75 标定；结果明确 checkpoint_admitted=false。
2. 有矩阵与模型 score，不等于有实际定位精度证据。当前没有该帧误差或独立 render/mask/depth 一致性验收结果，不把 score 77.703125 当成功概率。
3. 记录的约 19.62 秒包含构建 refiner/首次模型调用，不是测得的常驻单帧耗时。分别测加载、首次 register、后续请求耗时，不能直接外推每帧都要这么久。

下一步优先使用项目已经拥有的、带对应模型及相机标定/曝光来源的录制 RGB-D 验证完整 checkpoint。缺 RM75 世界外参时维持 model-only 证据，不伪造时间/外参，也不要为了凑齐数据启动未获许可的实体相机。

非实时定位的计算延迟必须与原 SyncPolicy 的 freshness 合同一起核查：曝光时间不能换成推理完成时间。需要保留真实采集时间、处理时间与验证过的静止/执行前变化检查条件；不能全局增加 max_age 就宣称修复。离线回放使用明确的回放时间域，不把历史图片包装成现场新帧。

## 6. 全量测试：只剩一项可以精确修复的事件消费错误

本批最后完整日志：1972 passed / 1 failed / 1 warning，46.62 s。

失败是 `tests/swm/test_native_relation_continuation.py::test_actual_phase_rescreens_after_closure_veto_preserving_pairing`：新增加 `swm_native_grasp_endpoint` 事件后，旧断言把所有事件都当成 relation_search 读取 `ranked_ids`。

应只筛选 `kind == 'swm_native_relation_search'` 再检查顺序，并保留已有关键断言：high 被拒绝后仍尝试 low、使用 low 对应 place、不混用 high 的配对关系。不要删除该测试、忽略 KeyError、降低断言或只写“属于旧测试所以不处理”。新增末端事件可以有自己的身份与残差断言。修完按 AGENTS 的隔离方式重跑完整 tests/。

## 7. 下一轮交付重点：不新开总体 Goal

继续现有 Goal，按下面顺序推进；本审阅不要求重构新架构或变更用户观测时机：

1. 修正事件类型测试，取得本次精确代码的全量绿灯。
2. 对上述两个闭爪未拒绝候选，定位联合 lift/place 的第一个失败点；重点核查 native_relation 过滤和 release metadata。优先修复真实合同错配，不扩大无差别搜索。
3. 至少完成一次正式笔 worker 的主动作与 after_release_retreat 结果，再验证错误落点与取消处理。代码已接入不能替代这一步。
4. 并行推进可独立完成的离线 FP 检查点数据准备与原生 PushT 负例，但重型任务仍串行。不应所有工作长期等待同一条笔候选。
5. 下一汇总最少显示：原子链是否完整执行、持物期定位调用数、最终独立观测是否完成、最终关系结果、首个失败阶段、端到端耗时、物理版本消费、对应代码/输入摘要。测试总数只作为辅助信息。

报告继续写入现有 `CODEX_SWM_RELEASE_PLAN.md`、`CODEX_SWM_RELEASE_ACCEPTANCE.md` 和 `benchmarks/release/swm_acceptance.json`，不再分裂出另一套验收门槛。未完成 Jimu、五技能、Agent、在线参数与前端部分继续保留，不把它们改成“只差硬件”。

## 8. 审阅依据

- `benchmarks/release/evidence/{paired_observation_01,paired_observation_02,candidate_rejection_01,pen_precision_01}/acceptance.json`
- `benchmarks/release/evidence/pen_precision_01/{events.jsonl,full_tests.log,source_input_sha256.txt,source.patch}`
- `benchmarks/release/evidence/offline_fp_01/{result.json,run.py,console.log}`
- `rm75_app/swm/{paired_grasp_place,scene,native_context,native_execution,native_audit,native_skills,native_push_hypotheses}.py`
- `rm75_app/pickplace/atom_task_builder.py`
- `tests/swm/test_candidate_rejection.py`
- `AGENTS.md`

REVIEW-457563E-EFFECTS-END
