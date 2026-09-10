# 13a5cb8 回传审阅：收敛到同一条真实工作流

审阅提交：`13a5cb84aafd7f90043b5e4a097cb12f8c449dfb`。分支：`chatgpt/three-scene-software-closeout`。
主要证据：`docs/CODEX_THREE_WORKFLOW_ITERATION_20260910.md` 和本次实际调用代码。
本轮修改不触碰 vendor、原底板数据、规划球/缓冲、碰撞豁免、成功误差或真机执行权限。

## 1. 审阅结论：有进展，但三个结论需要收紧

**PickPlace：连续任务实际完成5个不同物体，包括胶棒；薯片罐失败，网球未执行。** 这是5完成+1失败+1未运行，不是6个都已经打通，也不能据此宣布胶棒在任意上下文都修好了。胶棒独立128-seed试验仍失败。对0.12/0.15抬升目标的失败如实保留；固定目标处负载与底座的几何冲突不能靠增加seed消除，但早期28.975 mm证据也不能替代新试验每条关系的首个失败定位。

**PushT：原单例、3个位姿变化案例、1次脚本扰动恢复有合并前物理成功记录。** `RANDOM_CASES` 是3组直接写在代码中的初末位姿，不是一个抽样分布或12例随机套件。称“固定案例泛化检查”更准确。合并后1297项CPU回归不等于合并后完整物理闭环复跑；不能仅凭代码意图声称两个控制器严格等价。

**Jimu：“真实LLM→前端→同一生成设计→原生SIM”还没有闭环证据。** 本轮LLM输出10件（7原locked+2新增locked+1可动）只做了preview；浏览器主要往返的是19件原模板；原生SIM运行的也是19件原模板、12个可动件，完成10/12。这是不同设计的不同测试，不能串成同一任务完成。

回传所说“返程碰撞不属本轮范围”不符合用户希望直接可搭的目标。弧形原模板第11件的返程问题仍属于软件收尾，必须继续处理，但不能关闭相应检查。不要把它仅以“已知问题”排除出验收。

## 2. 本轮已修改：生产Jimu入口接上实际可用的模型协议

### 2.1 原缺口

`iteration_api.py` 的前端配置检查和异步生成固定创建 `magnetic.llm_client.JsonChatClient`，它读OpenAI-compatible配置；实际成功联网的却是独立 `llm/jimu_design_llm.py` 的Anthropic-compatible SDK。机器上现成的 `ANTHROPIC_AUTH_TOKEN / ANTHROPIC_BASE_URL / ANTHROPIC_DEFAULT_OPUS_MODEL` 并没有自动接进前端生产路径。

### 2.2 统一使用的路径

```text
/workcell 的 LLM 按钮
  → IterationAPI._generation
  → completion_provider.completion_client
  → Anthropic-compatible 或原 OpenAI-compatible 文本JSON接口
  → magnetic.generation.generate / compile_selection
  → 原模板、锁定底板、库存、支撑闭包、generation_proof
  → 同一JSON的原生装配SIM
```

新增 `rm75_app/magnetic/completion_provider.py`，并将API的readiness与真正生成两个调用点都改到同一工厂。显式provider优先；无显式provider时，仅在配置不歧义的情况下识别已有协议；绝不因一个提供方报错而换另一个偷偷重试，也不把mock当真实LLM结果。

建议在新的本机SIM profile中明确设置：

```json
"magnetic": {
  "design_library": "<已导入模板库绝对路径>",
  "llm": {
    "provider": "anthropic",
    "auth_token_env": "ANTHROPIC_AUTH_TOKEN",
    "timeout_s": 60,
    "max_tokens": 4096
  }
}
```

合并到原完整section，不覆盖原native_args、固定场景、解释器或103 mm实验配置。实际模型和端点读本机已有环境，不在仓库硬编码一个新付费模型；密钥只引用环境变量名。若本机推理模型确实需要更多thinking预算，可显式调整max_tokens，上限32768，记录实际消耗；截断输出不得当有效JSON接受。

SDK懒加载，使用SDK自身 `DefaultHttpxClient`，单client显式proxy与 `trust_env=False`，不修改多线程Web进程的全局代理环境。禁用自动SDK重试，生成器仍至多两次尝试；读超时和流事件间墙钟预算保留，异常不回显密钥/端点响应。这里只测试了SDK替身，未联网调用。

官方接口参考：Anthropic Python SDK README与Python SDK文档的messages.stream、max_retries、timeout和自定义HTTP client章节；链接见文末。具体机器版本仍需Codex实连核对。

### 2.3 对两份生成实现的取舍已明确，不再交给用户选

生产入口采用上面的受约束编译链。`llm/jimu_design_llm.py` 暂保留为历史探索/preview工具，不据它的schema通过给出“可搭”结论。本轮没有删除其旧实验或修改历史产物，也没有宣称其自由坐标生成已通过物理验收。

在原12块库存与固定底板不变的前提下，LLM不能把活动件改成locked来逃避装配，也不能新增2块固定支撑。新的生产路径仅接收五字段结构选择提案，坐标、locked集合、类型和变换从原模板编译，不让模型自己提供。选子结构仍不证明磁力或机器人可达，需要后续SIM。

**当前生成空间仍是原模板槽位的父子闭合子结构，不是任意自由造型。** 用户要求的更多可搭外形应通过多个原模板或经过几何检验的连接规则扩展，不能把“少搭几块原房屋”宣传为任意新结构生成。

## 3. 本轮已修改：合并后的PushT干预状态

保留现有长推候选、响应模型和完整物理执行器，只修以下控制流：

1. **脚本扰动只跳过拟合，却未更新scene epoch/旧最佳分数。** 移动T以后，即使它从新位置逐渐接近目标，也可能一直比旧位置的最好分数差，继而错误进入无进展等待。现在显式扰动统一增加epoch、清空已有计划、重置best/stagnant，并先取稳定新观测；被扰动转换仍不用于拟合。
2. **成功确认期间暂停/移动后仍可能继续向下规划旧obs。** 旧代码只设置need_stability然后break确认循环，之后仍进入规划。现在直接结束本轮，重新观测和稳定确认，不能把确认窗口前的位姿交给候选规划器。
3. **稳定窗口跨越干预。** 期间发生暂停/恢复，即便最终位姿恰好相同，连续稳定计数也重置，不能拼接两段证据。
4. **脚本disturb钩子明确拒绝real=True。** 这是接口防误用，不是新硬件功能。

这些是真正会改变合并后行为的修复，因此回传的“语义未变、等价”不能替代复跑。旧源码在新增9项控制流测试中5失败/4通过；修复后9项全通过。感知、规划、响应估计与执行用受控替身，不能解释成PhysX结果。

未做：接触中实时制动、任意人手干预识别、完整机械臂避人、新的退让碰撞豁免、真实设备运动。动作边界暂停不是急停，用户不得据本轮软件测试在运动臂旁伸手。

## 4. Codex下一轮：先一个同物产物闭环，再扩大数量

### 4.1 先固定测试版本

pull本提交后记录HEAD、dirty diff和运行时关键文件hash。原GPU证据属于合并前实现，不覆盖这次文件。继续使用AGENTS里的网络隔离和串行GPU资源上限；不得收集或导入vendor内硬件测试。

先新增定向回归，再完整 `tests/`：

```bash
PYTHONPATH=. PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 tools/run_network_isolated.py -- \
  python3 -m pytest tests/test_pusht_merge_epoch_review.py \
  tests/test_jimu_completion_provider_review.py -q

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 tools/run_network_isolated.py -- \
  python3 -m pytest tests -q
```

真实LLM调用只在独立Web/API进程中进行；preview/sim worker保留其内部隔离。不要把整个Web进程放进阻断TCP的测试沙箱后，再通过关闭worker隔离来解决联网问题。

### 4.2 Jimu必须优先补“同一设计”证据

原底板数据现已在当前仓库导入成功，不再要求用户寻找旧路径：

```text
rm75_app/_vendor/jimu_scenes/Beta_demo-codex-v0.9/jimu_tasks/tag1_standard_three_layer
rm75_app/_vendor/jimu_scenes/Beta_demo-codex-v0.9/jimu_tasks/tag2_arc_base
```

用已有 `tools/import_jimu_design_library.py` 从这两份只读bundle生成新的模板库文件，创建新的SIM profile并设置上面的provider。不得改vendor源数据。

从**小而真实**的3～4个可动件结构开始，底板仍保留原9或7个locked件。必须由实际前端生成按钮调用现成API，然后提交同一结果的原生SIM，不要手动换回19件原模板。每个结构记录：

```text
generation_id
canonical design_digest（io.digest计算）
锁定底板digest与可动角色列表
浏览器提交request中的design_digest与generation_proof
作业builder_scene.json的规范化digest
native实际尝试和完成的role序列、执行结果
```

文件原字节SHA256和规范化JSON digest分别保存，不混为同一种哈希。生成、提交、原生输入的规范化digest必须一致；未选角色不得执行。JSON/schema通过与完整路径、仿真装配、物理磁吸结果分开。

先两种底板各一个小结构成功，再每种三个结构。原弧形完整12件仍需修第11件：定位实际role及pre-release/post-release阶段、真实/拟议关节状态、夹爪开度、payload状态、当前世界对象与具体碰撞对。`CuroboOnlyUnsupported` 不是自动等于上次已修复的陈旧缓存错误；也不能因为出现在return就当成可忽略。先做同候选只读trace，区分接触状态未恢复、已放置物体碰撞和真实不可行退让，再最小修改，并复验此前10件不退化。

### 4.3 PickPlace不再只扫抬升高度

先测试实际建议顺序的完整七物体：刷子、笔、木块、胡萝卜、网球、胶棒、薯片罐。仍保留七物体分母和同一场景；先放网球只是调度实验，不能保证交换槽位后胶棒仍通过。原顺序也保留对照。

胶棒先按同一关系做paired-endpoint repair开/关对照，不和128/256 seeds同时开启混淆效果。原独立失败与连续重试成功分开记录，已合格行不覆盖。

薯片罐逐候选记录**第一失败阶段**，分清pregrasp→grasp直线失败、带载退出失败和transport hover失败。只对确实走到带载退出的候选研究出口。先核查原approach-line retreat/world-Z fallback已经尝试了什么；必要时比较有界的、明确记录的退出中间点或已有其他抓取关系，保持最终放置目标和全部负载/底座检查。中间点不是终点IK通过即放行，要验证完整带载路径和执行连接。

不能在实际已抓取且物体去向不明时直接跳过失败源继续下一个；只有状态/持物/碰撞世界确认完整时才能延期。目标先让多数对象顺畅完成，同时如实保留难对象未通过，不再用半个端点或重试次数充当任务成功。

### 4.4 PushT合并后复跑，不立即扩大成大量GPU任务

先用本提交控制器复跑原2推案例与脚本扰动；再实际浏览器执行pause→paused确认→relocate→resume。专门把干预放在“目标已到、正在做多帧确认”的窗口，确认不会提交旧位姿计划。

前述全部通过再运行 `run_pusht_random_suite.py` 生成的12例原T位姿/目标套件；记录seed与完整分母，不把RANDOM_CASES三组常量重复三遍算随机评测。先原T，再不同尺寸；不同几何不混用旧响应拟合。

检查交互额外observe推进物理步后计划起点q不一致问题，不能直接放宽原1e-5检查来掩盖状态变化。对真正无进展的请求保持等待/取消，不反复烧GPU。长会话数组增长仍需流式磁盘证据和有界展示缓存，不能因run_until_goal把内存耗尽。上述物理与长时事项本轮没有完成，须据现场结果逐项落地。

## 5. 本轮执行边界

本地环境无法获取完整Git工作树，使用GitHub已读源码与旧交付中的相同文件重建受限测试树。修改前controller与iteration_api、支持的model/session_control/observation/io/llm_client，以及原网络隔离脚本核对Git blob相同；没有导入vendor或连接设备。

实际执行：新增25项通过，0失败、0跳过；Python编译通过。SDK调用为替身，API接线检查为AST静态检查，不是商业API或浏览器集成成功。原controller对照9项中5失败、4通过，可复现合并控制流缺陷。

未执行：全仓1297项复跑、真实LLM、浏览器点击、GPU、PhysX、相机和真机。本轮不宣称抓放成功率提升或Jimu生成结构已经搭出。

只回传一份 `docs/CODEX_THREE_WORKFLOW_ITERATION_20260910_R2.md` 和一个有界JSON。必须区分同一设计闭环/不同模板独立测试，合并前/合并后源码，预览/native循环/物理成功；不再写“已知返程问题不属范围”。

参考（官方）：https://github.com/anthropics/anthropic-sdk-python ，https://platform.claude.com/docs/en/api/sdks/python 。

结束标记：REVIEW-13A5CB8-WORKFLOW-END
