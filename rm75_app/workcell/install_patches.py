"""Small, hash-gated integration edits; no planner/control algorithm rewrite."""
from __future__ import annotations
import hashlib
from pathlib import Path

EXPECTED={
 'rm75_app/tasks/registry.py':'71a4f713fa74a78f1947fd9aeaa404b09a0c3235',
 'rm75_app/tasks/pickplace.py':'c725178e62b816efda869b72e111f154f81b066c',
 'rm75_app/tasks/jimu.py':'2a0bbba382f05cb957753143f5175aa8c51ae0df',
 'rm75_app/legacy/backends.py':'12317cb7851e7cae3e041373a031fdcc795926b3',
 'rm75_app/web/control_panel.py':'27b8130c838981d1c5babbf61c534390574dae2b',
}

def blob_sha(raw):
    return hashlib.sha1(f'blob {len(raw)}\0'.encode()+raw).hexdigest()


def transform(path,text):
    if path.endswith('tasks/registry.py'):
        text=text.replace('from rm75_app.tasks.pickplace import PickPlaceTask',
                          'from rm75_app.tasks.pickplace import PickPlaceTask\nfrom rm75_app.tasks.pusht import PushTTask')
        text=text.replace('(PickPlaceTask(), JimuTask(), LegoTask())',
                          '(PickPlaceTask(), JimuTask(), LegoTask(), PushTTask())')
    elif path.endswith('tasks/pickplace.py'):
        text=text.replace('modes=("curobo2", "rrtrack", "openworld-geometry", "tabletop-refine"),',
             'modes=("curobo2", "rrtrack", "openworld-geometry", "tabletop-refine", "working-preview", "working-sim", "working-real"),')
        text=text.replace('        return command_for_mode(normalized.mode, normalized.args, python=python)',
             '        if normalized.mode.startswith("working-"):\n'
             '            from rm75_app.workcell.task_command import command\n'
             '            return command(normalized, task="pickplace", mode=normalized.mode.removeprefix("working-"), python=python)\n'
             '        return command_for_mode(normalized.mode, normalized.args, python=python)')
    elif path.endswith('tasks/jimu.py'):
        text=text.replace('modes=("four-wall", "triangle-roof"),',
             'modes=("four-wall", "triangle-roof", "working-preview", "working-sim", "working-real"),')
        text=text.replace('status="compatibility",','status="working_snapshot_adapter",')
        text=text.replace('"当前适配器只负责统一任务契约和旧入口命令；Jimu 运动实现仍在 legacy backend。",',
             '"原磁吸算法迁入仓库内 working_snapshot，隔离进程运行；working-* 模式接入三场景工作台。",')
        text=text.replace('        script = jimu_script(normalized.mode or self.definition.default_mode)',
             '        if normalized.mode.startswith("working-"):\n'
             '            from rm75_app.workcell.task_command import command\n'
             '            return command(normalized, task="magnetic", mode=normalized.mode.removeprefix("working-"), python=python)\n'
             '        script = jimu_script(normalized.mode or self.definition.default_mode)')
    elif path.endswith('legacy/backends.py'):
        text=text.replace('JIMU_ROOT = REPO_ROOT / "Beta_demo-codex-v0.9"',
             'JIMU_ROOT = APP_ROOT / "rm75_app" / "_vendor" / "working_snapshot" / "Beta_demo-codex-v0.9"')
    elif path.endswith('web/control_panel.py'):
        text=text.replace('    args = parser.parse_args()\n    emit("status", f"Web 控制台启动 http://{args.host}:{args.port}")\n    app.run(host=args.host, port=args.port, threaded=True)',
            '    parser.add_argument("--workcell-profile", type=Path, default=RUNTIME_DIR / "workcell" / "machine.json")\n'
            '    parser.add_argument("--allow-workcell-real", action="store_true")\n'
            '    args = parser.parse_args()\n'
            '    service = None\n'
            '    if args.workcell_profile.is_file():\n'
            '        if args.host not in ("127.0.0.1", "localhost"):\n'
            '            raise ValueError("Three-scene real controls require loopback binding; use SSH tunnelling")\n'
            '        from rm75_app.workcell.server import mount\n'
            '        service = mount(app, args.workcell_profile, APP_ROOT, allow_real=args.allow_workcell_real)\n'
            '    elif args.allow_workcell_real:\n'
            '        raise FileNotFoundError("--allow-workcell-real requires a machine profile")\n'
            '    emit("status", f"Web 控制台启动 http://{args.host}:{args.port}")\n'
            '    try:\n'
            '        app.run(host=args.host, port=args.port, threaded=True, use_reloader=False)\n'
            '    finally:\n'
            '        if service is not None:\n'
            '            service.close()')
    return text


def prepare(root):
    changes={}
    for path,expected in EXPECTED.items():
        raw=(Path(root)/path).read_bytes()
        if blob_sha(raw)!=expected:
            backup=Path(root)/'runtime_data/workcell/install_backup'/path
            if backup.is_file() and blob_sha(backup.read_bytes())==expected and raw.decode()==transform(path,backup.read_text()):
                continue
            raise ValueError(f'Target integration file differs from reviewed commit: {path}; reconcile explicitly, no force overwrite')
        updated=transform(path,raw.decode())
        if updated==raw.decode():
            raise RuntimeError(f'No integration edit found: {path}')
        changes[path]=(raw,updated)
    return changes


def apply(root,changes):
    for path,(raw,text) in changes.items():
        backup=Path(root)/'runtime_data/workcell/install_backup'/path
        backup.parent.mkdir(parents=True,exist_ok=True)
        if not backup.exists():
            backup.write_bytes(raw)
        (Path(root)/path).write_text(text,encoding='utf-8')
