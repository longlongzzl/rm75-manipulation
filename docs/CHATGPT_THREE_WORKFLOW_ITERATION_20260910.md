# 三场景工作流完善：连续抓放、LLM 装配、交互 PushT

审阅/实现基线：`eca8ece3aa6aa86155cd34b2e4b5e59b471a1632`。
协作分支：`chatgpt/three-scene-software-closeout`。

**本轮交付的是实现与本地 CPU/契约验证，不是新的 GPU、LLM 服务或实机通过声明。** 保留 Codex 已有的 Jimu 103 mm 实验 12/12、PushT 整臂两推到位及原完整序列 5/7 的记录；它们是之前版本与输入的证据，不自动归到本提交。没有编辑 vendor、旧目录、底板资产、碰撞球/buffer、已有跟踪/执行保护或原始目标误差阈值。

## 1. 当前交付内容与边界

| 主线 | 实际新增代码 | 必须在本机补的验收 |
|---|---|---|
| PickPlace | 前端多对象选择/建议顺序；同场景 native sequence；逐对象完成/待处理摘要；可选有界 paired-endpoint repair | 正确配置当前 frozen-world 原生环境；基线/修复版逐对象和整序列 GPU 比较 |
| Jimu | 异步 LLM JSON 调用；原 3×3/弧形模板库导入；库存/父子支撑闭包编译；前端预览与原 native manifest 接线；防篡改来源校验 | 只读导入两种旧任务的实际文件；真实模型 API；两种底板生成结构的原生 SIM/全链验证 |
| PushT | 随机位置/yaw/目标；显式不同 T 尺寸；长推参数接线；until-goal 会话；边界暂停/重定位/继续；仿真 T 外部干预；干预不进入响应拟合 | 当前 full-arm physics 上随机矩阵、pause/relocate/resume、真实页面与取消；实机另行资格化 |

**Jimu 生成空间是“原模板槽位的父子依赖完整子结构”，不是 LLM 任意写 6D 位姿。** 同一底板可导入多份原模板来扩大空间。输出保留固定底板、原坐标、局部轴和相对变换，只选择活动件；至多 12 块，具体各类型库存来自该模板实际活动件。只有静态几何/依赖校验通过，绝不自动称为机器人可达或磁吸稳定。

**弧形底板没有被猜一个半径重新画。** 远端固定源里没有找到两份完整原任务数据；接口要求 Codex 从本地旧目录导入真实 manifest/builder/fixed scene。未导入时前端显示未配置，不能换成测试夹具、3×3 或一个假圆弧。

## 2. PickPlace：先连贯完成多数对象，再处理明确失败

### 2.1 前端到原生序列

新页面附加脚本 `rm75_app/web/static/workcell/iteration.js` 与样式由现有 WSGI 注入，保留原 `index.html/app.js`。连续整理传入：

```json
{"task":"pickplace","mode":"sim","parameters":{"object_names":["shuazi","bi","lvmukuai","carriot","tennis","gluestick","hongshupian"],"automatic_order":true}}
```

服务器只接受已配置名称、唯一列表和布尔顺序选项。建议顺序是刷子、笔、木块、胡萝卜、球，再处理胶棒/薯片罐；这是启发式调度，不是成功率保证。关闭 automatic_order 即保留用户顺序。

`iteration_workflows.prepare_request` 在每个作业的 profile 副本设置 `frozen_source_order`，交回原 `legacy.run_working`。原 adapter 已有的 `--cycle-object-names / --cycle-order-targets / --repeat-count` 与完整 frozen-world 验收继续生效，**不是七次独立 reset**。

返回的 `sequence_summary` 区分请求物体、前台 native 已完成、待处理、整体资格和独立物理成功。后台预取/别的物体成功不补分母，部分成功也不把 worker 改成全成功。该新增前端顺序入口先限 native-world SIM；原单对象真实路径不变。

### 2.2 有界配对端点补求

`paired_endpoint_repair.py` 只作用于 SIM 的 gluestick / hongshupian / tennis 配对关系。原阶段先照常运行；某一关系只有 hover 或 release 成功时，用该**相同关系**的合格关节解给另一个不变目标提供 seed。两端都失败不盲目重试，两端都成功不改原解。原种子数、位置/方向目标、碰撞、自碰撞、负载和误差门都保留。

每个源/世界上下文默认最多增加 12 个查询、累计 5 s；5 s 是查询间软预算，不是可抢占 GPU 的硬时限。原成功行/共享 raw buffer 先克隆，防下一次求解覆盖。新候选必须通过 native success 与当前碰撞状态复核，状态指纹变化会中止；每个最终完整链仍由原路径验收。缓存仅记录本作业内已试目标与预算，不会跨作业拿旧解当成功。

显式启用：

```json
"pickplace": {
  "fixed_scene_format": "native_world",
  "simulation_contact_policy": "transport_world_checked_compatibility",
  "paired_endpoint_repair": {"enabled": true, "max_queries": 12, "budget_s": 5.0}
}
```

默认不改既有任务。不要同时叠加其它种子实验后声称这是单变量收益。必须分别保留改前/改后生成数、原查询/额外查询、配对成功、原路径失败、前台物体成功和 wall time。

**这不是已修好 hongshupian。** 已有带载名义抬升与底座球冲突，增加端点 seeds 不能改变那个刚体几何条件。补求仍失败就回传明确首失败阶段；要改抬升出口/抓取关系，应单独保持最终物体目标与负载检查，不隐藏成一次“IK 修复”。

## 3. Jimu：导入真实旧底板 → LLM → 原装配

### 3.1 只读导入，两类底板都恢复

在本地查找并确认实际 `jimu_task_manifest_v1`。已知 3×3 任务曾位于 `Beta_demo-codex-v0.9/jimu_tasks/tag1_standard_three_layer`，弧形按旧 tag2/arc 实际任务确认，**不按名字猜文件内容**。不用清理/stash/reset 旧目录。

```bash
PYTHONPATH=. python tools/import_jimu_design_library.py \
  --grid-task "$GRID_TASK" \
  --arc-task "$ARC_TASK" \
  --output runtime_data/iteration/original_jimu_library.json
```

两个变量由 Codex 只读定位原任务后赋值。可以重复 `--grid-task` / `--arc-task` 导入每类多份原造型。此次用户已要求恢复这两种旧底板，可按导入器的受限闭包处理，不要再次把明确需求卡在一份泛化的“待批准所有旧目录”清单。

导入器只读取 manifest、其目录内 builder 与非空 SAM6D fixed scene，记录原字节 hash；不会复制整目录、运行旧脚本或推测弧形几何。输出必须是新文件且在原任务目录之外。

库里保留**完整原 native manifest**，尤其是旧 `role_target_offsets_builder_m`、tray/triangle slot 信息；原程序的 set-if-not-explicit 逻辑已核对。执行时复制到作业内 `original_base_manifest.json`，只把 builder 指向本次生成的 `builder_scene.json`，fixed scene 指向 hash 已核对的原输入。前端/LLM不能提交 manifest、任意 argv 或相机/机器人配置。基于另一个底板的旧 task-dir / triangle slot 显式参数不再覆盖当前模板。

保留已成功 SIM profile 中的 103 mm、原启动姿态、扩展缓存、环境和其他运行参数。不同底板若原文档有不同启动参数，在本机对照后分别记录到该模板的服务端 recipe；不要全盘复制别的底板的启动值。

### 3.2 新 profile，不覆盖现有配置

选用**已经跑通原生任务与完整 PushT 物理的实际本机配置**，不是直接拿示例里的空 IP/空物理配置运行：

```bash
PYTHONPATH=. python tools/configure_iteration_workflows.py \
  --profile "$CURRENT_MACHINE_JSON" \
  --library runtime_data/iteration/original_jimu_library.json \
  --output runtime_data/iteration/machine.json \
  --enable-paired-repair --add-sim-variants
```

已有 profile 不变。新文件强制 `hardware_reviewed=false`、三项 integration_qualified=false；仅用于此次 SIM。它不安装 cuRobo/SAPIEN、不更换解释器、不制造物理标定。若 PickPlace 原配置不是 native-world 且全搬运检查，工具拒绝自动启用 repair。

### 3.3 LLM 设置与生成流程

使用现有 `RM75_LLM_*` 配置习惯；密钥只在服务端环境变量，不能放浏览器或 Git：

```bash
export RM75_LLM_MODEL='<已有可用模型>'
export RM75_LLM_API_BASE='<已有 OpenAI-compatible /v1 根地址>'
export RM75_LLM_API_KEY_ENV='OPENAI_API_KEY'
# 在本机安全设置 OPENAI_API_KEY；不要把值写进回传/命令日志。
```

也可在 `magnetic.llm` 设置 model/api_base/api_key_env、timeout_s、max_tokens、token_parameter、json_mode；默认超时30 s、最多1600 completion tokens，每次生成至多两次调用。兼容端点若使用 max_tokens，显式设置 token_parameter。HTTPS 或指定 loopback HTTP，拒绝重定向、截断 completion/refusal、超大响应和缺字段。没有默认硬编码付费模型。

```bash
PYTHONPATH=. python -m rm75_app.workcell serve \
  --profile runtime_data/iteration/machine.json --port 7861
```

不加 allow-real。前端选择底板、最多活动件数、描述外形 → LLM 选择模板与角色 → 编译器验证库存/支撑/固定底板和来源 → 原 canvas 显示 → 用户单独预览或运行原 native SIM。生成是异步线程，Stop 不会被外部模型调用占住。

提交设计时再次重编译并核对 design/proof/native recipe digest。编辑器手动改了坐标、底板、角色或 recipe 后不能沿用旧 proof；需重新生成/验证。**设计合法不等于完整动作链通过**，更不等于磁极/支撑稳定已验证。

## 4. PushT：长推与未到位继续

保留最新 NumPy 批量候选、接触方向、响应拟合、GPU 候选筛选、实际 full-arm PD 和原许可策略；不回退到早期12 mm固定推。

新增参数例：

```json
{"task":"pusht","mode":"sim","parameters":{
  "initial_pose":[0.35,-0.18,0],"goal_pose":[0.40,-0.15,0.3],
  "simulation_backend":"full_arm_physics","geometry_id":"original",
  "maximum_push_length_m":0.05,"run_until_goal":true,"max_steps":60,"speed_mps":0.015
}}
```

这里的数值只说明格式，正式案例用本机有效 workspace/profile。最长默认50 mm，SIM参数上限80 mm；保留短推与接近目标的长度限制。没有把长推理解为无碰撞的一段直线，也不保证长推是每次选中的动作。

`run_until_goal=true` 不再因为固定60步自动放弃；到位仍要求原6 mm/0.1 rad及稳定多帧。没有进展或候选预测为空时等待场景变化/操作者恢复，避免原地反复打GPU；碰撞/观测错误/未知执行异常继续停止，不自动吞错重推。可在服务端 `pusht.session_policy.max_wall_s` 设置独立墙钟上限，0表示直到目标/停止/安全异常/等待。

### 4.1 接受中途改变 T：先安全边界，再重新定位

新增会话API：

```text
GET  /api/workcell/iterate/sessions/<job_id>
POST /api/workcell/iterate/sessions/<job_id>/control
     {"action":"pause"}
     {"action":"relocate","pose":[x,y,yaw]}
     {"action":"resume"}
```

仍受原 loopback、same-origin、X-Workcell-Token 保护。控制是有序文件队列，上一条未确认不再堆叠；不能修改其它作业、任意文件或真实机器参数。

前端“本次推完并撤离后暂停”是**动作边界请求**，不是立即物理急停。只有 worker 显示 `paused` 才能用按钮改变仿真 T，之后点击继续。变化被标为 `explicit_external_sim_intervention`，不计一次推送，不进入响应模型拟合。它显式改变SIM刚体位置并清零速度以代表手动干预；报告不会继续声称整个episode从无外部状态修改。

恢复时清掉已有规划缓存，检查并等待新观测稳定，使用原目标继续规划。规划期间观测变化超过原3 mm/0.04 rad门会丢弃计划而不是执行旧路径。无法解释的大位移不进入响应拟合；**小幅未声明的人为位移并不保证都能检测**，正式实验应使用显式干预通道。

现有相机源、序号、会话、时间戳校验不放松。真实机械臂边推边让人伸手不是本轮授权；新增交互接口先SIM-only，原硬件Stop与其许可独立保留。完整物理仍在每步检查原禁止接触/越界，仿真重定位的局部工具包络检查不冒充真实整臂安全认证。

### 4.2 更多 T 与随机实验

随机位置覆盖 workspace，yaw覆盖[-pi,pi]，目标有平移/旋转/混合；初始与目标均检查完整T边界/障碍，非平凡已达目标会拒绝重采样。每例有seed、case_id、状态和几何digest。一个case采样预算耗尽明确报错，不删掉困难例。

`--add-sim-variants` 增加 wide_t、long_stem_t、small_t 三种**作者设置的仿真几何**，不是实际打印件测量。只可改变横杆/竖杆四个尺寸，不能借变体修改碰撞、速度或成功阈值；不同几何清空旧 response_fits。环境与GPU规划器都从同一个Config构建T，当前 PhysicsSession.open 的 model传递链已核对。

只生成矩阵，不运行设备：

```bash
PYTHONPATH=. python tools/run_pusht_random_suite.py \
  --profile runtime_data/iteration/machine.json \
  --seed 42 --count 12 --geometry-id original \
  --output runtime_data/iteration/random_original_42
```

实际串行整臂物理：选择新的输出目录并加 `--run --backend full_arm_physics --timeout-s 600`。每例独立保留成功/失败/超时/NOT_RUN；长时间等待场景的case由矩阵超时取消，不修改任务阈值。分别运行原T和三种变体，不把不同形状拟合数据混在一起。

## 5. 本轮已执行验证

本地是从此前代码包加 GitHub 当前内容恢复的**部分源树**，不是完整GPU安装。关键支持文件（service/model/batch_search/response/observation 等）核对当前Git blob；新 worker以当前原字节为基础修改。未修改远端旧app.js/service.py或vendor。

- Python 新增/修改文件 compileall：PASS。
- 新 `iteration.js`：Node syntax PASS。
- `tests/iteration`：**152 passed，0 failed，0 skipped**。含约束/篡改/库存/依赖/原manifest、LLM模拟响应、CSRF/队列、随机种子、响应污染、暂停恢复、规划中移动T、未知执行错误、预算以及配对buffer等。
- **实际 WorkcellService → 子进程 worker**：连续PickPlace预览、生成Jimu预览、PushT交互surrogate完整循环通过。预览和surrogate不代替 native/GPU/PhysX。
- 原底板数据：测试使用明确的合成schema夹具，**没有宣称旧弧形底板已在此环境导入**。
- LLM传输：mock opener和mock completion，无商业API请求，不声称实际API成功。
- Chromium导航实际尝试被 `net::ERR_BLOCKED_BY_ADMINISTRATOR` 拦截，停在页面加载前；没有通过改策略/换地址绕过。**浏览器点击/截图验收未完成**。
- 新GPU/full-arm随机/实机：NOT_RUN；原全仓测试（不是新增152项）须Codex在当前完整库重跑。没有沿用“980项通过”当本轮全仓结果。

## 6. Codex 下一步，一份回传闭环

1. 拉取此分支，记录精确HEAD和原工作区状态；不覆盖未提交内容。读本MD和新增验证JSON。
2. compileall、新tests/iteration、原完整tests。发现接口断点直接按实际调用关系修，不放宽断言。
3. 从旧目录只读定位3×3和弧形完整任务；运行导入器。对比原全模板再导出的底板/坐标/依赖/manifest和固定场景hash。分别选原启动参数；库里没有弧形时不能用测试夹具凑齐。
4. 用当前成功机器配置创建新的SIM profile，保留原103mm实验与最新PushT实际驱动配置。启动真实前端，验证三条新增工作流、原面板、token/互斥/Stop/输入提示。
5. PickPlace在同一输入/顺序下做repair关/开对照，再完整七物体；原5/7基线独立保留，不能把5个容易物体筛出来叫7/7。报告胶棒/薯片罐具体下一卡点，不再无差别扩大seeds。
6. Jimu两种底板至少各3个不同的父子闭合结构，包括原完整12件和较小子结构；真实LLM生成、前端确认、实际service/worker、原native循环角色数和生成proof逐一对应。要验证未选角色不会仍被调度，原底板不变；先全链预规划/SIM，所有物理资格保留未验证。
7. PushT先一个当前已通过案例验证无回归，再12个随机原T、每变体12个；至少两次pause→外部SIM移动→resume，确认不计push/不更新响应拟合，并从新pose重规划。测中途Stop、目标稳定、未进展等待。报告随机几何有效≠机器人可达。
8. 写 `docs/CODEX_THREE_WORKFLOW_ITERATION_20260910.md`，附一个有界JSON并push：分别列代码/CPU/浏览器/原生SIM/物理/实机状态，不生成十几份重复报告。旧目录和vendor原字节保持不变。无用户现场许可时不启动任何真实运动。

结束标记：THREE-WORKFLOW-ITERATION-20260910-END
