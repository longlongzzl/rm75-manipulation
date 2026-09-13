"""Frozen-distribution prediction scoring, explicitly not parameter fitting."""
import copy
import math
import numpy as np
from .scene import digest,pose_error
from .identification import PhysicsParameters,validate_transition


def frozen_hypotheses(report,transition,geometry):
    data=validate_transition(transition);belief=report['posterior']
    if report.get('native_tool_geometry_qualified') is not True:
        raise ValueError('Qualified original-tool training replay required')
    if report['action_digest']==data['action_digest'] or report['transition_digest']==data['transition_digest']:
        raise ValueError('Holdout must be a different measured action')
    for key in ('object_id','support_id','domain','mesh_sha256'):
        if belief[key]!=data[key]:raise ValueError('Holdout object/support identity changed')
    asset=data['initial_snapshot']['assets'][data['initial_snapshot']['objects'][data['support_id']]['asset_id']]
    if belief['support_mesh_sha256']!=asset['mesh_sha256']:
        raise ValueError('Holdout support geometry changed')
    if not report.get('native_geometry_readback') or any(
            row['native_tool_geometry_digest']!=digest(geometry) for row in report['native_geometry_readback']):
        raise ValueError('Holdout native tool geometry changed')
    rows=copy.deepcopy(belief['particles'])
    weights=[r['weight'] for r in rows]
    if (not 2<=len(rows)<=128 or len({r['id'] for r in rows})!=len(rows) or
            not all(math.isfinite(w) and w>=0 for w in weights) or not math.isclose(sum(weights),1.,abs_tol=1e-9)):
        raise ValueError('Invalid frozen distribution')
    for row in rows:PhysicsParameters(**row['parameters'])
    return [dict(id=r['id'],parameters=r['parameters']) for r in rows],rows


def score_frozen_prediction(transition,particles,rollouts):
    data=validate_transition(transition);results={r['hypothesis_id']:r for r in rollouts}
    if len(results)!=len(rollouts) or set(results)!={p['id'] for p in particles}:
        raise ValueError('One holdout result per frozen particle required')
    scores=[]
    for particle in particles:
        row=results[particle['id']];loss=None
        if row.get('valid') is True:
            if (row['action_digest']!=data['action_digest'] or row['transition_digest']!=data['transition_digest'] or
                    row['parameters']!=particle['parameters'] or row['initial_snapshot_id']!=data['initial_snapshot']['snapshot_id'] or
                    row.get('engine_domain')!='physics' or not np.array_equal(row['time_s'],data['object_time_s']) or
                    len(row['T_world_object'])!=len(data['T_world_object'])):
                raise ValueError('Holdout replay identity or sample coverage changed')
            errors=[pose_error(a,b) for a,b in zip(row['T_world_object'],data['T_world_object'])][1:]
            loss=float(np.mean([(p/.003)**2+(r/.04)**2 for p,r in errors]))
        scores.append(dict(id=particle['id'],weight=particle['weight'],loss=loss))
    complete=all(r['loss'] is not None for r in scores)
    return dict(schema='rm75_frozen_prediction_v1',refitted=False,weights_updated=False,
                complete=complete,particles=scores,
                weighted_loss=sum(r['weight']*r['loss'] for r in scores) if complete else None,
                position_noise_m=.003,rotation_noise_rad=.04)
