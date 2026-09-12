# f0dd73a 控制台验收审阅与小范围修正

审阅分支：`chatgpt/three-scene-software-closeout`。审阅提交：`f0dd73abe33dc6dbd96a3d2183041ba1a56c7692`。
依据：`docs/CODEX_OPERATOR_CONSOLE_RESULTS_20260911.md`、提交内的 browser/worker 报告和 `test_suites.log`。

## 1. 接受哪些结果

这次不是只有离线组件或 Python API 替身。回传给出了正常 HTTP 浏览器导航、一次真实 LLM 生成期间刷新后继续查询同一个 generation_id、同一份 proof 的原生 Jimu SIM、共享 Stop，以及从页面操作的 PushT full_arm_physics 暂停/移动/恢复。新控制台的主要使用路径已具有本机验收记录，可以围绕它边用边修，不需要重新做另一套入口。

源版本为 `6dbb6a1`；其后到本轮审阅提交只有验收文档/证据新增，没有运行源码变更。1377 项 Python 与32项 Node 全通过是 Codex 在该版本的本机结果，不是本轮修改后的全仓成绩。

需要纠正回传 §9 的措辞：本轮实际运行了 PushT full_arm_physics 单例和原生 magnetic SIM，不能同时笼统写“PhysX NOT_RUN”或“全量 tests 仍未运行”。应分别写“已运行该单例物理链/维护的全量 tests；未重跑广泛随机物理套件、相机和真机；其他私有代码不在本仓覆盖范围”。正常导航报告、同设计执行报告与验证类型保留原样，不将 command_completed_unverified 当成实物搭建成功。

## 2. 已落实的修正（不改运动算法）

| 回传问题 | 本轮处理 | 边界 |
|---|---|---|
| 1. magnetic.scene 假红项 | Bootstrap 无任务时，检查有效模板库是否可找到原模板场景；可用则显示模板来源提示 | 不带 proof 的手动设计仍要求 fixed_scene；选中生成结构的原场景哈希仍在 preflight 精确校验，不把全部文件存在当作几何资格 |
| 2. favicon 404 | 新控制台使用本地 SVG 图标；独立服务器对 favicon.ico 返回无内容应答 | 保留挂载的旧 Flask 应用自己的 favicon 行为；不引入 CDN |
| 3. Playwright 缺失先占目录 | 依赖/浏览器检查与实际 launch 均在新建证据目录前；提示当前解释器及已有 venv | 不自动 pip install，不覆盖已有证据；输出目录并发被占用也不写入对方 report |
| 4. 找不到真实配置文件 | 按 schema 识别任意 JSON 文件名；支持 --search-root / --profile 精确指定；显示扫描上限、深度与截断 | 有界只读扫描，排除 jobs/vendor/缓存；不自动选择配置，不展示密钥 |
| 5. DOM 夹具写死路径 | 接受 --output / --chromium；优先显式浏览器，其次 Playwright 托管或本机已装浏览器 | 不再默认写 /mnt/data；组件测试仍不等于真实 HTTP/CSP 测试 |
| 6. PickPlace 调试选项误伤 Jimu | 统一 worker 工作流在作业 profile 副本上，移除4个非本任务的 PickPlace 诊断选项并记录告警 | 原共享 profile 不改；PickPlace 本任务选项完全保留，原 legacy 的 SIM/冻结世界/碰撞策略检查未放宽 |
| 7. SDK 构造读取坏 ALL_PROXY | 显式传 transport=None，跳过 SDK v1.4 的环境 proxy-mount 构造分支；底层按 trust_env=False 和明确 proxy 配置构造 | 不修改多线程 Web 进程的全局环境、不换 HTTP 客户端家族、不禁用 TLS/重定向检查；真实 SDK/端点需本机复测 |
| 8. original 重复下拉 | 新控制台 bootstrap 展示层去重并复制返回值，避免修改源配置 | 原兼容页面的 /iterate/features 原始接口未改；不是删除一个几何模型 |

SDK 初始化失败现在归为 `CompletionClientInitError / LLM_CLIENT_INIT`，控制台显示“检查 Web 解释器 SDK 和 magnetic.llm.proxy”，不再将它与已经发出的模型请求超时混为一谈。错误不包含原始 URL、响应或密钥；没有改调用次数、模型、令牌或超时预算。

代理修复依据已核对的官方 v1.4.0 源码：`_DefaultHttpxClient.__init__` 在 **transport 关键字不存在**时，无条件解析环境代理。传入 `transport=None` 让 SDK 保留默认 timeout/limits，但将默认 transport 构造交给底层 HTTP 客户端。此选择不使用该 SDK 分支的自定义 TCP keepalive 设置；长流超时仍由原 timeout/事件间预算管理，须本机真实慢生成复验。

参考：<https://github.com/anthropics/anthropic-sdk-python/blob/v1.4.0/src/anthropic/_base_client.py>（约898–956行）。没有在源码中硬编码凭据，也没有为了测试连接任何设备。

## 3. 一项尚未修复：测试写入生产“最近记录”

回传第9项确实需要处理。本轮**没有**删除旧 jobs、自动清空历史、根据“看起来像测试”的名称隐藏记录，或者将 UI 仅显示新记录伪装成修复。也未修改 WorkcellService 的生产目录语义。

Codex 下一轮在完整仓库中定位具体写入者，将测试的 app_root/profile/runtime_data 放到 pytest 临时目录，必要时仅把只读包/资产路径链接过去。相关测试的工作进程与 robot.lock 也应使用测试目录。不要只把报告目录移到 tmp，但仍把 worker 的 app_root 设成桌面正式仓库。

在租约空闲时，保存正式 `runtime_data/workcell/jobs` 的目录集合，跑完整隔离 tests 后比较集合；应无新增。已有可疑测试记录只列清单，需人工确认才能移动，不能删除真正任务证据。不因这一点阻止用户使用已验收控制台，但不能称“九项全部修复”。

## 4. 本轮实际验证

由于本环境无法解析 GitHub 主机完成 git clone，测试树由前次已交付源码包与当前 connector 读到的源码恢复。变更前 console_api/server/工具与6dbb提交一致；当前 provider、iteration_workflows 及 io/design/网络隔离脚本核对Git blob。未导入或运行 vendor。

- Python：`tests/console` **65 passed / 0 failed / 0 skipped**，其中原43项＋新增22项；主要运行/生成依赖为显式替身。
- JavaScript：原状态测试 **32 passed**。
- 离线 DOM：通过新的 --output/--chromium 入口，**11项 PASS**；空白页组件+内存HTTP替身，无模型/worker/GPU。没有把它标成正常HTTP导航。
- Python编译通过；文件传递按Git blob核对，不能以本机测试树替代完整仓库回归。
- 未执行：修改后的完整1377项、真实 Anthropic 1.4.0/HTTPX2 运行、商业LLM、正常HTTP浏览器导航、原生GPU/SIM、相机、真机。

SDK代理回归模拟了官方构造器的关键字分支，证明旧调用遇到污染环境会失败、新调用不进入该分支；不是宣称本环境安装并运行了真实1.4.0 SDK。测试覆盖参数保留、全局环境不变及初始化错误脱敏。

## 5. 本机接续，不再扩大功能范围

先保留 dirty worktree，fetch后 ff-only pull，不 reset/clean，也不停止用户已有的7861服务。新控制台版本应显示 `2026.09.12-console.2`。

```bash
PYTHONPATH=. PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 tools/run_network_isolated.py -- \
  python3 -m pytest tests/console -q
python3 tools/run_network_isolated.py -- \
  node --test --experimental-default-type=module tests/console/client_state.test.mjs
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 tools/run_network_isolated.py -- \
  python3 -m pytest tests -q
```

配置发现：

```bash
python3 tools/launch_workcell_console.py --list-profiles \
  --search-root runtime_data/console_launch_20260911
python3 tools/launch_workcell_console.py --list-profiles \
  --profile runtime_data/console_launch_20260911/console_profile_r2_nodiag.json
```

继续使用已验收的本机 profile；验证任务隔离时用另一个**新副本**带回 PickPlace 的诊断选项，检查 magnetic 仍能进入原生路径并写入 unrelated_task_diagnostic_ignored，而 PickPlace 自己的诊断前提仍严格要求。不要覆盖现有配置、增加碰撞豁免或资格字段。

浏览器验收选已有 venv 与浏览器：

```bash
~/.venvs/console-acceptance/bin/python tools/check_workcell_console.py \
  --url http://127.0.0.1:7863/workcell/console/ \
  --output runtime_data/console_acceptance_20260912_fix \
  --chromium /home/zhangzhao/.cache/ms-playwright/chromium-1228/chrome-linux64/chrome

~/.venvs/console-acceptance/bin/python tools/run_network_isolated.py -- \
  ~/.venvs/console-acceptance/bin/python tests/console/dom_fixture.py \
  --output runtime_data/console_dom_20260912_fix \
  --chromium /home/zhangzhao/.cache/ms-playwright/chromium-1228/chrome-linux64/chrome
```

以上端口/浏览器路径来自本次回传，启动前确认当前空闲与文件存在；不为抢端口杀进程。缺依赖的默认解释器应给出清晰提示且不创建目录，换正确解释器可以原输出路径重试；真正开始后失败的证据目录必须保留。

代理先做**隔离的构造与close检查**，保留污染的ALL_PROXY，使用本机1.4.0 SDK与明确HTTP proxy，不发送请求；通过后在Web父进程再做一次实际生成。worker网络隔离不关闭。若仍失败记录SDK版本与安全错误类型，不上传密钥，不退回多线程全局环境修改。

回传一份 `docs/CODEX_OPERATOR_CONSOLE_RESULTS_20260912_R2.md`，只列这八项针对性复验、测试历史污染修复、完整回归与NOT_RUN。当前不是新增真机许可，单例SIM通过不自动升级成硬件资格。

CONSOLE-ACCEPTANCE-FOLLOWUP-END
