"""Bounded, read-only discovery of arbitrarily named machine profiles."""
from collections import deque
from pathlib import Path
import os
from .io import read_json, integer

PRUNE = frozenset(('.git', '_vendor', '__pycache__', '.venv', 'venv', 'node_modules',
                   'jobs', 'design_generations', 'console_submissions', 'console_generations',
                   'curobo_native_extensions'))


def discover(root, *, search_roots=(), explicit=(), max_entries=20000, max_depth=6):
    integer(max_entries, 'max_entries', 1, 100000)
    integer(max_depth, 'max_depth', 0, 12)
    root = Path(root).resolve()
    roots = ([Path(p).expanduser().resolve() for p in search_roots] or
             [root/'runtime_data', root/'configs/workcell', root/'examples/workcell'])
    seen_files, seen_dirs, rows = set(), set(), []
    stats = dict(entries_scanned=0, files_checked=0, scan_truncated=False, directories_unreadable=0)

    def inspect(path):
        path = Path(path)
        if path.is_symlink() or not path.is_file(): return
        resolved = path.resolve()
        if resolved in seen_files: return
        seen_files.add(resolved)
        if path.suffix.lower() != '.json': return
        stats['files_checked'] += 1
        try:
            value = read_json(path, max_bytes=2_000_000)
            if not isinstance(value, dict) or value.get('schema') != 'rm75_workcell_machine_v1': return
            pick = value.get('pickplace', {})
            mag = value.get('magnetic', {})
            physics = value.get('pusht', {}).get('physics', {})
            library = mag.get('design_library')
            rows.append(dict(path=str(resolved),
                             pickplace_python=pick.get('python'),
                             has_jimu_library=bool(library),
                             physics_backends=physics.get('enabled_backends', []),
                             selected_automatically=False))
        except (OSError, ValueError, KeyError, TypeError, AttributeError):
            return
    for path in explicit: inspect(Path(path).expanduser())
    queue = deque((p, 0) for p in roots)
    while queue:
        directory, depth = queue.popleft()
        if directory.is_symlink(): continue
        if directory.is_file(): inspect(directory); continue
        if directory in seen_dirs: continue
        seen_dirs.add(directory)
        try:
            with os.scandir(directory) as entries:
                for entry in entries:
                    if stats['entries_scanned'] >= max_entries:
                        stats['scan_truncated'] = True
                        queue.clear()
                        break
                    stats['entries_scanned'] += 1
                    if entry.is_symlink(): continue
                    path = Path(entry.path)
                    if entry.is_dir(follow_symlinks=False):
                        if entry.name in PRUNE: continue
                        if depth < max_depth: queue.append((path, depth+1))
                        else: stats['scan_truncated'] = True
                    elif entry.is_file(follow_symlinks=False): inspect(path)
        except (OSError, PermissionError):
            stats['directories_unreadable'] += 1
    # Complete-looking profiles first, without selecting, importing or probing one.
    rows.sort(key=lambda row: (-int(row['has_jimu_library']), -int(bool(row['physics_backends'])), row['path']))
    return dict(profiles=rows, search_roots=list(map(str, roots)), **stats,
                hardware_contacted=False, selected_automatically=False,
                note='只校验 schema；候选不等于已验收。可用 --search-root 缩小扫描，或 --profile 精确指定。')
