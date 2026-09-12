"""Durable SWM snapshots are evidence, never permission to resume an old motion.

Restart restores geometry identity and parameter beliefs, but forces a new full
checkpoint. Historical measured poses remain in the snapshot file for auditing;
they are not relabelled as current camera measurements or live holding state.
"""
from __future__ import annotations
import copy
import json
import os
from pathlib import Path
import tempfile
from .scene import SceneWorldModel, SceneInvalid, digest


class SnapshotStore:
    def __init__(self,path):
        self.path=Path(path).expanduser()

    def save(self,world):
        snapshot=world.snapshot()
        payload={'schema':'rm75_swm_store_v1','snapshot':snapshot,
                 'automatic_motion_resume':False}
        self.path.parent.mkdir(parents=True,exist_ok=True)
        if self.path.is_symlink():raise ValueError('Refuse symlinked SWM evidence destination')
        fd,tmp=tempfile.mkstemp(prefix='.'+self.path.name+'.',dir=self.path.parent)
        try:
            with os.fdopen(fd,'w',encoding='utf-8') as stream:
                json.dump(payload,stream,ensure_ascii=False,allow_nan=False,sort_keys=True)
                stream.write('\n');stream.flush();os.fsync(stream.fileno())
            os.replace(tmp,self.path)
        finally:
            if os.path.exists(tmp):os.unlink(tmp)
        return snapshot['snapshot_id']

    def restore(self,manifest):
        if self.path.is_symlink() or self.path.stat().st_size>16_000_000:
            raise ValueError('Invalid SWM evidence file')
        data=json.loads(self.path.read_text(encoding='utf-8'))
        if data.get('schema')!='rm75_swm_store_v1' or data.get('automatic_motion_resume') is not False:
            raise ValueError('Unsupported or execution-enabled SWM store')
        snap=data['snapshot'];canonical=dict(snap);sid=canonical.pop('snapshot_id',None)
        if sid!=digest(canonical):raise SceneInvalid('Stored SWM state was modified')
        world=SceneWorldModel(manifest);world.check_assets();new=world.snapshot()
        if (snap['assets']!=new['assets'] or snap['observation_domain']!=world.domain
                or snap['calibration_id']!=world.calibration_id or snap['world_frame']!=world.frame
                or set(snap['objects'])!=set(new['objects'])):
            raise SceneInvalid('Stored world belongs to another model/calibration/domain')
        for oid,obj in new['objects'].items():
            if any(snap['objects'][oid][k]!=obj[k] for k in ('id','name','asset_id','fixed')):
                raise SceneInvalid('Stored instance registry has changed')
        # These are historical parameter hypotheses, not observations. Keep them
        # only with the unchanged support/mesh identities and no motion resume.
        for oid,posterior in snap['physics'].items():
            world.update_physics(oid,posterior,expected_physics_revision=world.physics_revision)
        world.revision=max(world.revision,int(snap['revision']))+1
        world.physics_revision=int(snap['physics_revision'])
        world.invalid_reason='restart_requires_fresh_checkpoint'
        return world
