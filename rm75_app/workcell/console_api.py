"""Presentation-only API for the operator console.

No robot/solver imports, no arming, and no qualification writes. All accepted
work is still submitted to WorkcellService and its isolated preview/SIM worker.
POSTs are journaled by client-generated identity: an uncertain acknowledgement
must be reconciled, never blindly resubmitted as a second task or LLM call.
"""
from __future__ import annotations

import copy
import os
import re
import sys
import time
from pathlib import Path
from urllib.parse import parse_qs

from .io import atomic_json, read_json, digest, dumps, integer

VERSION = '2026.09.11-console.1'
IDENTITY = re.compile(r'[0-9a-f]{32}\Z')
TASKS = ('pickplace', 'magnetic', 'pusht')
TERMINAL = {'succeeded', 'failed', 'cancelled', 'verification_failed',
            'command_completed_unverified', 'interrupted', 'abandoned'}


def identity(value):
    if not isinstance(value, str) or not IDENTITY.fullmatch(value):
        raise ValueError('Invalid request/job identity')
    return value


def mapping(value, fields):
    if not isinstance(value, dict) or set(value) != set(fields):
        raise ValueError('Unexpected console request fields')
    return value


def redacted(value):
    """Export no profiles, credentials or authorization tokens.

    In addition to sensitive field names, remove configured secret values from
    diagnostic strings; this is not a promise to detect arbitrary unknown secrets.
    """
    secret_names = ('api_key', 'auth_token', 'authorization', 'password', 'secret', 'csrf', 'arm_token')
    secrets = [v for k, v in os.environ.items() if len(v) >= 8 and
               any(t in k.lower() for t in ('api_key', 'auth_token', 'password', 'secret'))]
    def scrub(item, depth=0):
        if depth > 30: return '[depth limit]'
        if isinstance(item, dict):
            return {str(k): ('[redacted]' if any(t in str(k).lower() for t in secret_names)
                             else scrub(v, depth+1)) for k, v in item.items()}
        if isinstance(item, (list, tuple)): return [scrub(x, depth+1) for x in item]
        if isinstance(item, str):
            for secret in secrets: item = item.replace(secret, '[redacted]')
        return item
    return scrub(value)


def same_json_values(a, b):
    """Exact JSON value equality across 0.0 -> 0 browser serialization.

    No coordinate tolerance, key dropping, bool/number coercion or list reorder.
    Canonical execution bytes are restored exclusively from the trusted compiler.
    """
    if type(a) in (int, float) and type(b) in (int, float):
        import math
        try: return math.isfinite(a) and math.isfinite(b) and a == b
        except OverflowError: return False
    if type(a) is not type(b): return False
    if isinstance(a, dict):
        return a.keys() == b.keys() and all(same_json_values(a[k], b[k]) for k in a)
    if isinstance(a, list):
        return len(a) == len(b) and all(same_json_values(x, y) for x, y in zip(a, b))
    return a == b


class ConsoleAPI:
    """One facade, one service, one active worker across all three panels."""
    def __init__(self, service, iteration):
        self.service = service
        self.iteration = iteration
        self.started = time.time()
        self._history_cache = (0., [])

    def _path(self, group, ident):
        ident = identity(ident)
        root = (self.service.root/group).resolve()
        path = root/ident
        # No traversal or symlink substitutions, including a symlinked leaf.
        if path.is_symlink() or not path.resolve().is_relative_to(root):
            raise ValueError('Console data path leaves its root')
        return path

    def _file(self, group, ident):
        path = self._path(group, ident).with_suffix('.json')
        if path.is_symlink() or not path.resolve().is_relative_to((self.service.root/group).resolve()):
            raise ValueError('Console journal path leaves its root')
        return path

    def _safe_feature_state(self):
        # A malformed optional model must not hide console configuration or Stop.
        try:
            return self.iteration.features()
        except (ValueError, OSError, KeyError, TypeError) as exc:
            return {'jimu_catalog': [], 'llm': {'configured': False, 'missing': ['valid configuration']},
                    'geometry_ids': ['original'], 'geometry_models': {},
                    'errors': [f'Feature configuration: {type(exc).__name__}'],
                    'sequence_native_sim_enabled': False}

    def bootstrap(self):
        info = self.service.info()
        public_info = {k: info.get(k) for k in (
            'csrf', 'allow_real', 'real_latched', 'active_job', 'pickplace_objects', 'pusht_model',
            'pusht_simulation_backends', 'pusht_physics_initial_pose', 'snapshot_installed')}
        profile = self.service.profile
        settings = profile.get('magnetic', {}).get('llm', {})
        timeout = settings.get('timeout_s', 60)
        timeout = timeout if type(timeout) in (int, float) and 1 <= timeout <= 600 else 60
        features = self._safe_feature_state()
        return {'schema': 'rm75_operator_console_v1', 'version': VERSION, 'info': public_info,
                'features': features, 'profile_name': Path(self.service.profile_path).name,
                # Bind browser drafts to this profile's content, not a timestamp.
                'profile_id': digest(profile), 'server_started_at': self.started,
                'generation': {'estimated_budget_s': timeout*2+30, 'poll_ms': 1500,
                               'max_attempts': 2, 'browser_deadline': None,
                               'provider_busy': bool(getattr(self.iteration, '_worker', None) and self.iteration._worker.is_alive())},
                'mode_policy': {'allowed': ['preview', 'sim'], 'real_submission_enabled': False,
                                'reason': '当前统一控制台只提交预览/仿真；未通过真机验收，不在此解除限制。'},
                'checks': self.configuration_checks()+[
                    dict(key='magnetic.llm',title='模型生成接口',status='ready' if features.get('llm',{}).get('configured') else 'warning',
                         hint='' if features.get('llm',{}).get('configured') else '检查 magnetic.llm 与本机环境变量；未配置时可载入已有设计。'),
                    dict(key='magnetic.catalog',title='可读取的原底板目录',status='ready' if features.get('jimu_catalog') else 'warning',
                         hint='' if features.get('jimu_catalog') else '库文件需通过内容校验，文件存在本身不代表库有效。')],
                'active_job': self.service.active}

    def configuration_checks(self, task=None, mode='sim', spec=None):
        """Read-only dependency hints; not a camera/GPU/robot or reachability test."""
        profile = self.service.profile
        checks = []
        def add(key, title, okay, hint, level='error'):
            checks.append(dict(key=key, title=title, status='ready' if okay else level,
                               hint='' if okay else hint))
        add('profile', '机器配置格式', profile.get('schema') == 'rm75_workcell_machine_v1',
            '使用 rm75_workcell_machine_v1 配置；示例配置不等于已验收的本机配置。')
        if mode == 'preview': return checks
        selected = (task,) if task else TASKS
        for name in selected:
            section = profile.get(name, {})
            if name in ('pickplace', 'magnetic'):
                native = self.service.app_root/'rm75_app/_vendor/working_snapshot/MIGRATION_MANIFEST.json'
                add(name+'.snapshot', name+' 原版工作源码', native.is_file(),
                    '先按迁移文档安装原版快照；不要复制模型日志或修改旧仓库。')
                python = section.get('python', sys.executable)
                add(name+'.python', name+' 运行解释器', isinstance(python, str) and Path(python).expanduser().is_file(),
                    '在本机配置中设置已安装原生依赖的绝对 python 路径。')
                fixed = section.get('fixed_scene')
                # Generated Jimu selects its own original frozen scene in the recipe.
                generated = name == 'magnetic' and spec and 'generation_proof' in spec.get('parameters', {})
                if not generated:
                    add(name+'.scene', name+' 冻结场景', isinstance(fixed, str) and Path(fixed).expanduser().is_file(),
                        '仿真需要可读取的本机冻结场景。生成结构使用其原模板的场景。')
                if name == 'magnetic':
                    library = section.get('design_library')
                    add('magnetic.library', '原 3×3 / 弧形模板库', isinstance(library, str) and Path(library).expanduser().is_file(),
                        '用已导入的两份原底板 bundle 构建模板库，并设置 magnetic.design_library。', 'warning')
            if name == 'pusht':
                backend = (spec or {}).get('parameters', {}).get('simulation_backend')
                physics = section.get('physics', {})
                if backend and backend != 'surrogate':
                    for field, title in (('simulation_python', '物理仿真解释器'), ('planner_python', 'cuRobo 规划解释器')):
                        value = physics.get(field)
                        add('pusht.'+field, title, isinstance(value, str) and Path(value).expanduser().is_file(),
                            '使用 R2 跑通的物理/规划环境绝对路径；本检查不加载 CUDA。')
                    add('pusht.motion', '物理工具与桌面配置', isinstance(physics.get('motion'), dict) and bool(physics['motion']),
                        '填写 R2 的 motion 配置；不能用硬件资格布尔值代替几何/标定。')
                elif not backend:
                    add('pusht.physics', '整臂物理后端', 'full_arm_physics' in physics.get('enabled_backends', []),
                        '未配置整臂物理时仍可预览或测试 CPU 近似模型。', 'warning')
        return checks

    def lease_busy(self):
        """Observe a cross-process lease without importing or stopping its owner."""
        path = self.service.root/'robot.lock'
        if not path.exists(): return False
        import fcntl
        with path.open('r') as stream:
            try: fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError: return True
            fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
        return False

    def canonical_spec(self, requested):
        """Restore trusted generated design representation, never altered geometry."""
        value = copy.deepcopy(requested)
        if value.get('task') != 'magnetic' or 'generation_proof' not in value.get('parameters', {}):
            return value
        from rm75_app.magnetic.generation import load_library, compile_selection, validate_generated_request
        proof = value['parameters']['generation_proof']
        if not isinstance(proof, dict): raise ValueError('Generated proof must be an object')
        library = load_library(self.service.profile)
        compiled = compile_selection(library, {k: proof[k] for k in
            ('board_id', 'template_id', 'title', 'selected_roles', 'explanation')}, board_id=proof['board_id'])
        if not same_json_values(value['parameters'].get('design'), compiled['design']):
            raise ValueError('Generated design was changed; only lossless numeric JSON round-trips are accepted')
        if not same_json_values(proof, compiled['proof']):
            raise ValueError('Generated proof is stale or changed; generate/validate it again')
        validate_generated_request(library, compiled['design'], compiled['proof'])
        value['parameters']['design'] = compiled['design']
        value['parameters']['generation_proof'] = compiled['proof']
        return value

    def preflight(self, value):
        from .spec import validate_spec
        mapping(value, ('spec',))
        requested = value['spec']
        if not isinstance(requested, dict) or requested.get('mode') not in ('preview', 'sim'):
            raise PermissionError('Console submits preview/SIM only; it does not authorize real execution')
        spec = validate_spec(self.canonical_spec(requested), self.service.profile)
        checks = self.configuration_checks(spec['task'], spec['mode'], spec)
        if spec['task'] == 'magnetic' and 'generation_proof' in spec['parameters']:
            from rm75_app.magnetic.generation import load_library, validate_generated_request
            recipe = validate_generated_request(load_library(self.service.profile), spec['parameters']['design'],
                                                spec['parameters']['generation_proof'])
            if spec['mode'] == 'sim':
                import hashlib
                path = Path(recipe.get('fixed_scene', ''))
                valid = path.is_file() and path.stat().st_size <= 8_000_000
                valid = valid and hashlib.sha256(path.read_bytes()).hexdigest() == recipe.get('fixed_scene_sha256')
                checks.append(dict(key='magnetic.recipe_scene', title='原模板场景与摘要',
                                   status='ready' if valid else 'error',
                                   hint='' if valid else '原模板 fixed_scene 缺失或字节已变，需重新导入/生成；不接受旧 proof。'))
        warnings = ['预检查不调用规划器，不证明轨迹可达、物理稳定或真机安全。']
        if spec['task'] == 'pusht':
            backend = spec['parameters'].get('simulation_backend', 'surrogate')
            if backend == 'surrogate': warnings.append('CPU 近似模型通过不等于物理仿真通过。')
            else: warnings.append('保留当前物理配置的碰撞策略；本页不修改碰撞豁免。随机场景仍可能失败。')
            if spec['parameters'].get('run_until_goal'):
                warnings.append('持续模式遇到无进展会等待；Stop 与所有原安全异常仍有效，不是无限盲推。')
        if spec['task'] == 'magnetic': warnings.append('结构预览不证明磁吸稳定，未通过的原生阶段会停止任务。')
        busy = bool(self.service.active) or self.lease_busy() or bool(getattr(self.service, 'legacy_busy', lambda: False)())
        if busy: warnings.append('工作台或跨进程租约正被占用；未删除锁文件，也未向占用者发送命令。')
        return {'valid': not any(c['status']=='error' for c in checks), 'busy': busy,
                'active_job': self.service.active, 'spec': spec, 'request_digest': digest(spec),
                'checks': checks, 'warnings': warnings, 'hardware_contacted': False}

    def _journal(self, group, ident, request):
        """Called under the service lock. Persist intent before a side effect."""
        path = self._file(group, ident)
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.exists():
            saved = read_json(path)
            if saved.get('request_digest') != digest(request):
                raise ValueError('A request identity cannot be reused for changed input')
            return saved, True
        row = {'request_id': ident, 'request_digest': digest(request), 'status': 'pending',
               'created_at': time.time(), 'server_started_at': self.started}
        try:
            fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        except FileExistsError:
            raise RuntimeError('Request was claimed by another process; reconcile before retrying') from None
        with os.fdopen(fd, 'w') as stream:
            stream.write(dumps(row)+'\n'); stream.flush(); os.fsync(stream.fileno())
        return row, False

    def submit(self, payload):
        mapping(payload, ('request_id', 'spec'))
        ident = identity(payload['request_id'])
        # Resolve an old acknowledgement before rechecking busy/config state.
        with self.service._lock:
            path = self._file('console_submissions', ident)
            if path.exists():
                old = read_json(path)
                if old.get('request_digest') != digest(payload['spec']):
                    raise ValueError('Request id belongs to another input')
                return old
            preflight = self.preflight({'spec': payload['spec']})
            if not preflight['valid']: raise ValueError('预检查未通过；先修复配置项。')
            if preflight['busy']: raise RuntimeError('已有任务占用工作台，请等待或停止当前任务。')
            saved, _ = self._journal('console_submissions', ident, payload['spec'])
            try:
                result = self.service.submit(preflight['spec'])
            except Exception:
                # The launch may have failed before or after process creation;
                # retain ambiguity and never automatically resend this identity.
                saved.update(status='uncertain', message='启动未得到明确确认；请检查任务历史，不要自动重复提交。')
                atomic_json(path, saved)
                raise
            saved.update(status='accepted', job_id=result['job_id'], accepted_at=time.time(),
                         canonical_request_digest=digest(preflight['spec']))
            atomic_json(path, saved)
            self._history_cache = (0., [])
            return saved

    def generation_start(self, payload):
        mapping(payload, ('request_id', 'request'))
        ident = identity(payload['request_id'])
        request = payload['request']
        with self.service._lock:
            saved, exists = self._journal('console_generations', ident, request)
            if exists: return saved
            path = self._file('console_generations', ident)
            try:
                result = self.iteration.post('generate', request)
            except Exception:
                saved.update(status='not_accepted', message='生成请求未获接受；检查配置或等待正在运行的生成结束。')
                atomic_json(path, saved)
                raise
            saved.update(status='accepted', generation_id=result['generation_id'])
            atomic_json(path, saved)
            directory = self._path('design_generations', result['generation_id'])
            atomic_json(directory/'console_meta.json', {'created_at': saved['created_at'],
                'request_id': ident, 'request': request, 'server_started_at': self.started})
            return saved

    def generation_status(self, ident):
        root = self._path('design_generations', ident)
        try: meta = read_json(root/'console_meta.json')
        except FileNotFoundError: meta = {}
        if (root/'console_abandoned.json').is_file():
            return {'generation_id': ident, 'status': 'abandoned', 'created_at': meta.get('created_at'),
                    'message': '本次结果不再采用；已发出的模型请求可能继续运行和计费。'}
        try: status = self.iteration.generation(ident)
        except FileNotFoundError:
            if not (root/'request.json').is_file(): raise
            status = {'generation_id': ident, 'status': 'interrupted',
                      'error': '服务器重启或生成任务已失去管理；未自动重发模型请求。'}
        return {**status, 'created_at': meta.get('created_at', root.stat().st_mtime),
                'request': meta.get('request'), 'poll_ms': 1500, 'auto_execute': False}

    def abandon_generation(self, ident):
        root = self._path('design_generations', ident)
        if not root.is_dir(): raise FileNotFoundError('Unknown generation')
        atomic_json(root/'console_abandoned.json', {'at': time.time(), 'policy': 'discard_result_not_sdk_cancel'})
        return self.generation_status(ident)

    def history(self):
        now = time.monotonic()
        if now-self._history_cache[0] < 2:
            return {'jobs': self._history_cache[1], 'limit': 24}
        import heapq
        root = self.service.root/'jobs'
        paths = heapq.nlargest(48, (p for p in root.iterdir() if IDENTITY.fullmatch(p.name) and
                    p.is_dir() and not p.is_symlink()), key=lambda p:p.stat().st_mtime) if root.is_dir() else []
        rows = []
        for path in paths:
            try:
                req = read_json(path/'request.json', max_bytes=2_000_000)
                outcome = read_json(path/'result.json', max_bytes=2_000_000) if (path/'result.json').is_file() else None
            except (OSError, ValueError): continue
            rows.append({'job_id': path.name, 'task': req.get('task'), 'mode': req.get('mode'),
                         'status': outcome.get('status') if outcome else 'running' if self.service.active == path.name else 'untracked',
                         'created_at': (path/'request.json').stat().st_mtime,
                         'verification': (outcome or {}).get('verification'),
                         'task_success': (outcome or {}).get('task_success')})
            if len(rows)==24: break
        self._history_cache = (now, rows)
        return {'jobs': rows, 'limit': 24}

    def job(self, ident):
        ident = identity(ident)
        row = self.service.job(ident)
        row['managed_active'] = self.service.active == ident
        if row['status'] == 'running' and not row['managed_active']:
            row['status'] = 'untracked'
            row['management_warning'] = '当前服务器未管理此进程，运行状态未知；不要自动重启或删除锁文件。'
        return row

    def report(self, ident):
        row = self.job(ident)
        # Do not include stdout, machine_profile.json, environment or arm tokens.
        return redacted({'schema': 'rm75_console_report_v1', 'console_version': VERSION,
                         'job_id': ident, 'exported_at': time.time(),
                         'request': row.get('request'), 'status': row['status'],
                         'result': row.get('result'), 'progress': row.get('progress'),
                         'events': row.get('events', []),
                         'scope': 'UI/job report; not a hardware safety certificate'})

    def templates(self, board):
        from rm75_app.magnetic.generation import load_library, compile_selection
        from rm75_app.magnetic.design import piece_key
        library = load_library(self.service.profile)
        if board not in library['boards']: raise ValueError('Unknown installed board')
        templates = []
        for template in library['boards'][board]['templates']:
            proposal = dict(board_id=board, template_id=template['id'],
                            title=template.get('title', template['id']),
                            selected_roles=[piece_key(p) for p in template['design']['pieces'] if not p.get('locked', False)],
                            explanation='原模板完整选择，未调用 LLM；仍需原生规划与仿真验收。')
            compiled = compile_selection(library, proposal, board_id=board)
            templates.append({'id': template['id'], 'title': proposal['title'], **compiled,
                              'origin': 'original_template_no_llm'})
        return {'board_id': board, 'templates': templates}

    def get(self, suffix, query=''):
        parts = suffix.strip('/').split('/')
        if parts == ['bootstrap']: return self.bootstrap()
        if parts == ['history']: return self.history()
        if len(parts)==2 and parts[0] == 'jobs': return self.job(parts[1])
        if len(parts)==3 and parts[0]=='jobs' and parts[2]=='report': return self.report(identity(parts[1]))
        if len(parts)==2 and parts[0]=='generations': return self.generation_status(identity(parts[1]))
        if len(parts)==2 and parts[0]=='templates': return self.templates(parts[1])
        if len(parts)==2 and parts[0] in ('submissions', 'generation-requests'):
            group = 'console_submissions' if parts[0]=='submissions' else 'console_generations'
            return read_json(self._file(group, identity(parts[1])))
        raise FileNotFoundError('Unknown console route')

    def post(self, suffix, payload):
        parts = suffix.strip('/').split('/')
        if parts == ['preflight']: return self.preflight(payload)
        if parts == ['jobs']: return self.submit(payload)
        if parts == ['generate']: return self.generation_start(payload)
        if len(parts)==3 and parts[0]=='generations' and parts[2]=='abandon':
            mapping(payload, ()); return self.abandon_generation(identity(parts[1]))
        raise FileNotFoundError('Unknown console route')
