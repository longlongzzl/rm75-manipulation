# 本地执行约束

- 用户已明确禁止未经单独审批的真机运动和夹爪操作。继续任务、跑测试、跑仿真不构成真机授权；禁止连接机械臂、发送夹爪、回零或停止命令来做验证。
- 执行本地测试和仿真时，必须通过 `python3 tools/run_network_isolated.py -- <command>` 启动。它在 Linux 内核中阻断非 Unix 网络套接字，覆盖原生 SDK 和后代进程。保护安装失败则停止，不能绕过它续跑。
- pytest 始终显式指定 `tests/` 或其中的具体文件，并设置 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`。项目根目录的 `pytest.ini` 和 `conftest.py` 限制收集范围。
- `rm75_app/_vendor/working_snapshot/` 内存在导入即控制硬件的旧 `test_*.py`，只能作为文本审查，禁止导入、执行或收集它们。不能用整个仓库的测试发现命令代替正式 `tests/` 套件。
- 该网络保护不是串口/USB 或文件系统沙箱。仿真和测试不得另行访问真实设备；真实硬件测试需要用户针对具体动作单独批准。
