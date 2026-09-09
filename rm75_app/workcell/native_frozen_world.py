"""Full-scene evidence for the actual workcell native-world SIM boundary.

This is not a planner or a hardware qualification. Original source selection,
candidate generation, contact policy and native return values are untouched.
"""
import hashlib
import math
from pathlib import Path
import re
import functools
import threading

from .io import append_jsonl, loads
from .pickplace_world_coverage import (
    full_chain_observed, install, requested_sources_completed,
)


def read_contract(path, source, source_order=None):
    path=Path(path).resolve()
    with path.open('rb') as stream:
        raw=stream.read(8_000_001)
    if len(raw)>8_000_000:
        raise ValueError('Frozen world JSON exceeds size limit')
    data=loads(raw.decode('utf-8'))
    objects=data.get('objects') if isinstance(data,dict) else None
    if not isinstance(objects,dict) or not objects or source not in objects:
        raise ValueError('Frozen world must contain nonempty objects and the requested source')
    for name,entry in objects.items():
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,80}',name) or name.startswith('-'):
            raise ValueError('Invalid frozen world object name')
        pose=entry.get('T_world_obj') if isinstance(entry,dict) else None
        if (not isinstance(pose,list) or len(pose)!=4 or any(
                not isinstance(row,list) or len(row)!=4 or any(
                    type(x) not in (float,int) or not math.isfinite(x) for x in row)
                for row in pose)):
            raise ValueError('Each native-world object requires a finite 4x4 T_world_obj')
    if source_order is not None and (not isinstance(source_order,list) or len(source_order)<2
            or len(set(source_order))!=len(source_order) or source_order[0]!=source
            or any(name not in objects for name in source_order)):
        raise ValueError('Frozen sequence must start at requested source and contain distinct existing objects')
    return dict(path=path,names=tuple(sorted(objects)),source=source,source_order=tuple(source_order or [source]),
                sha256=hashlib.sha256(raw).hexdigest())


class FrozenWorldValidation:
    def __init__(self, contract, run_dir, events):
        self.contract=contract
        self.run_dir=Path(run_dir)
        self.events=events
        self.worlds=[]
        self.outcomes=[]
        self.foreground_demo_ids=set()

    def install(self, direct):
        def world(row):
            self.worlds.append(row)
            append_jsonl(self.run_dir/'frozen_world_coverage.jsonl',row)
            self.events.emit('frozen_world_coverage',evidence=row)
        def episode(row):
            self.outcomes.append(row)
            self.events.emit('frozen_world_source_outcome',evidence=row)
        install(direct,self.contract['names'],world,episode)
        if len(self.contract.get('source_order',()))>1:
            original=direct.run_targeted_place_episode_curobo_direct;owner=threading.get_ident()
            @functools.wraps(original)
            def sequence_episode(demo,bridge,real_exec,args,*pos,**kwargs):
                if threading.get_ident()==owner and not getattr(args,'_planning_prefetch_capture_only',False):
                    self.foreground_demo_ids.add(id(demo))
                    self.events.emit('frozen_sequence_episode',source=direct._current_source_object_name(args),
                        same_demo=len(self.foreground_demo_ids)==1)
                    if len(self.foreground_demo_ids)!=1:
                        from .pickplace_world_coverage import FrozenWorldIncomplete
                        raise FrozenWorldIncomplete('Frozen sequence unexpectedly reset its scene')
                return original(demo,bridge,real_exec,args,*pos,**kwargs)
            direct.run_targeted_place_episode_curobo_direct=sequence_episode

    def result(self, native_outcome, clearance_audits):
        sources=self.contract.get('source_order',(self.contract['source'],))
        try:
            unchanged=hashlib.sha256(self.contract['path'].read_bytes()).hexdigest()==self.contract['sha256']
        except OSError:
            unchanged=False
        source_done=requested_sources_completed(self.outcomes,sources)
        transport_seen=full_chain_observed(self.worlds,sources)
        clearance=bool(len(clearance_audits)==len(sources) and all(
            a.get('all_valid') is True and a.get('samples',0)>0 for a in clearance_audits))
        passed=bool(native_outcome.get('native_full_chain_passed') is True and unchanged
                    and source_done and transport_seen and clearance)
        return dict(command_success=passed,task_success=None,
            verification='frozen_world_sim_only' if passed else 'frozen_world_validation_failed',
            frozen_world_validation=dict(passed=passed,
                frozen_input_sha256=self.contract['sha256'],input_unchanged=unchanged,
                frozen_names=list(self.contract['names']),requested_sources=list(sources),
                requested_sources_completed=source_done,all_sources_transport_observed=transport_seen,
                independent_clearance_passed=clearance,
                refresh_count=len(self.worlds),
                foreground_refresh_count=sum(w.get('foreground') is True for w in self.worlds),
                foreground_transport_refresh_count=sum(w.get('foreground') is True and w['transport_scope']
                                                       for w in self.worlds),
                rejected_refreshes=[w for w in self.worlds if not w['complete']],
                source_outcomes=list(self.outcomes),physical_geometry_qualified=False))
