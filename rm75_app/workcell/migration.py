"""Deterministic, offline migration from the user's existing Git checkout.

Export only committed blobs from the reviewed revision. The source working tree
is never checked out, reset, cleaned or modified. Algorithms are copied, not
reimplemented. Known absolute project paths are relocated and recorded.
"""
from __future__ import annotations
import ast
import hashlib
import os
import shutil
import subprocess
import tempfile
from pathlib import Path, PurePosixPath
from .io import atomic_json, read_json

SOURCE_COMMIT='7aaff9da22486b7d25557b3795dd258f9b65f10d'
TARGET_COMMIT='263cd7eca41b25cbafd1f365050f022c33285362'
KEY_BLOBS={
 'Beta_demo-codex-v0.9/rm75_jimu_four_wall_portable.py':'3a80846055d8e299d8b1db9e3a7f07a0f08ce9ff',
 'Beta_demo-codex-v0.9/rm75_jimu_triangle_roof_apriltag_portable.py':'fa30d665de334db37099e208085e97cf00a5421c',
 'pick_jiaobang/rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place.py':'71d6bb61c9e43fa14b8655bb0ec88e7e92d3747e',
 'pick_jiaobang/rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place_sam6d.py':'d9368e393176fd5817d9ef3a5b650e73751be339',
 'pick_jiaobang/rm75_jiaobang_pick_real_with_foundationpose.py':'55555ae50700faf932a7e74d34fb6cabd1676122',
 'pick_jiaobang/curobo_rm75_planner.py':'b08a23e28a67101f089ac44261a1e9e908c8f479',
}
ROOTS=('Beta_demo-codex-v0.9/','pick_jiaobang/','RM75_gripper/','Demo_Triangle/',
       'lerobot-sim2real/','lerobot/','src/lerobot/','FoundationPose/assets/',
       'rm75_pick_place_app/assets/','rm75_pick_place_app/rm75_app/web/')
EXTENSIONS={'.py','.json','.yaml','.yml','.toml','.txt','.md','.html','.js','.css',
            '.urdf','.srdf','.obj','.mtl','.stl','.ply','.glb','.gltf','.dae','.npy',
            '.png','.jpg','.jpeg','.svg','.so','.dll','.sh','.npz'}
EXCLUDED={'__pycache__','.git','node_modules','weights','checkpoints','planning_profile_logs',
          'sam6d_grasp_scene_runs','sam6d_groundingdino_runs','sam6d_template_cache',
          'sam6d_pem_feature_cache','failure_renders','localization_debug','runs',
          'repro_runs','post_grasp_diagnostics','sam6d_jimu_direct_runs'}


def git(source,*args):
    return subprocess.run(['git','-C',str(source),*args],check=True,capture_output=True).stdout


def selected(path):
    p=PurePosixPath(path)
    if p.is_absolute() or '..' in p.parts or EXCLUDED.intersection(p.parts):
        return False
    if any('失败' in part or '备份' in part or part.startswith('v6_standard_four_wall_retry_build') for part in p.parts):
        return False
    # Explicitly never copy/share font files.
    return (path.startswith(ROOTS) or path in ('realman_with_gripper.py','pyproject.toml','LICENSE','LICENSE.txt')) and (p.suffix.lower() in EXTENSIONS or p.name.lower().startswith('license'))


def relocate(raw,path,final_root):
    if Path(path).suffix.lower() not in {'.py','.json','.yaml','.yml','.toml','.html','.js','.css','.sh','.urdf','.srdf'}:
        return raw,0
    try:
        text=raw.decode('utf-8')
    except UnicodeDecodeError:
        return raw,0
    original=text
    # Longest prefix first; otherwise the sibling lerobot-sim2real path is corrupted.
    mappings=[('/home/zhangzhao/Desktop/lerobot-sim2real',str(final_root/'lerobot-sim2real')),
              ('/home/zhangzhao/Desktop/lerobot',str(final_root))]
    changes=0
    for old,new in mappings:
        changes+=text.count(old)
        text=text.replace(old,new)
    if path.endswith('.py'):
        ast.parse(text,filename=path)
    return text.encode(),changes


def source_inventory(source,ref):
    ref=git(source,'rev-parse','--verify',f'{ref}^{{commit}}').decode().strip()
    rows=[]
    for record in git(source,'ls-tree','-r','-z',ref).split(b'\0'):
        if not record:
            continue
        meta,raw_path=record.split(b'\t',1)
        mode,kind,sha=meta.decode().split()
        path=raw_path.decode('utf-8')
        if kind=='blob' and selected(path):
            if mode=='120000':
                raise ValueError(f'Symlink requires explicit migration review: {path}')
            rows.append((path,mode,sha))
    return ref,rows


def export_snapshot(source,target,*,ref=SOURCE_COMMIT,expected_blobs=None):
    expected=KEY_BLOBS if expected_blobs is None else expected_blobs
    source=Path(source).resolve();target=Path(target).resolve()
    final=target/'rm75_app'/'_vendor'/'working_snapshot'
    if final.exists():
        raise FileExistsError(f'Refusing to replace an existing snapshot: {final}')
    resolved,rows=source_inventory(source,ref)
    actual={p:sha for p,_,sha in rows}
    for path,sha in expected.items():
        if actual.get(path)!=sha:
            raise ValueError(f'Unreviewed source at {path}; expected {sha}, got {actual.get(path)}')
    if not rows:
        raise ValueError('No committed migration files were selected')
    final.parent.mkdir(parents=True,exist_ok=True)
    staging=Path(tempfile.mkdtemp(prefix='.snapshot-',dir=final.parent))
    report={'source_repository':'longlongzzl/lerobot-realman','source_commit':resolved,
            'target_reviewed_commit':TARGET_COMMIT,'relocated_root':str(final),
            'mode':'copied_working_engine_isolated_process','files':[],
            'algorithm_refactor':False,'runtime_gpu_verified':False,'frontend_candidates':[],
            'dependency_completeness_verified':False}
    try:
        process=subprocess.Popen(['git','-C',str(source),'cat-file','--batch'],stdin=subprocess.PIPE,stdout=subprocess.PIPE)
        try:
            for path,mode,sha in rows:
                process.stdin.write((sha+'\n').encode());process.stdin.flush()
                header=process.stdout.readline().decode().strip().split()
                if len(header)!=3 or header[1]!='blob':
                    raise RuntimeError(f'Git blob unavailable: {path}')
                size=int(header[2])
                if size>80_000_000:
                    raise ValueError(f'Oversized selected source asset requires review: {path}')
                raw=process.stdout.read(size)
                if len(raw)!=size or process.stdout.read(1)!=b'\n':
                    raise RuntimeError('Incomplete Git blob stream')
                blob=hashlib.sha1(f'blob {len(raw)}\0'.encode()+raw).hexdigest()
                if blob!=sha:
                    raise RuntimeError(f'Git blob checksum mismatch: {path}')
                output,changes=relocate(raw,path,final)
                dest=staging/path;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(output)
                if mode=='100755':
                    dest.chmod(0o755)
                report['files'].append({'path':path,'source_blob':sha,'source_bytes':len(raw),
                    'installed_sha256':hashlib.sha256(output).hexdigest(),'relocated_literals':changes})
                if path.endswith(('.html','.js')) and any(s in path.lower() for s in ('jimu','builder','control')):
                    report['frontend_candidates'].append(path)
        finally:
            process.stdin.close()
            if process.poll() is None:
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill();process.wait()
        report['file_count']=len(report['files'])
        atomic_json(staging/'MIGRATION_MANIFEST.json',report)
        os.replace(staging,final)
    except BaseException:
        shutil.rmtree(staging,ignore_errors=True)
        raise
    return report


def verify_snapshot(root):
    root=Path(root).resolve();report=read_json(root/'MIGRATION_MANIFEST.json')
    if report.get('relocated_root')!=str(root):
        raise RuntimeError('Migrated snapshot was moved; reinstall path relocation before running')
    failures=[]
    for item in report['files']:
        path=root/item['path']
        if not path.resolve().is_relative_to(root) or not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=item['installed_sha256']:
            failures.append(item['path'])
    if failures:
        raise RuntimeError(f'Migrated source was changed or is missing: {failures[:8]}')
    return report
