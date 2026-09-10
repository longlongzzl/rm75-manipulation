"""Reproducible diverse T states; geometry-valid is not robot-reachable."""
from __future__ import annotations
from dataclasses import asdict
import numpy as np
from .model import Config, valid_pose, reached, wrap
from rm75_app.workcell.io import integer, digest

GEOMETRY_FIELDS = frozenset(('bar_width_m', 'bar_height_m', 'stem_width_m', 'stem_height_m'))


def configured_model(profile, *, geometry_id='original', maximum_push_length_m=None):
    data = dict(profile.get('pusht', {}).get('model', {}))
    if geometry_id != 'original':
        variants = profile.get('pusht', {}).get('physics', {}).get('geometry_variants', {})
        if geometry_id not in variants: raise ValueError('Unknown server-configured simulation T geometry')
        override = variants[geometry_id]
        if not isinstance(override, dict) or set(override) - GEOMETRY_FIELDS:
            raise ValueError('Geometry variants cannot change dynamics, thresholds or collision rules')
        data.update(override)
        # Calibration from a different shape must NOT leak into this experiment.
        data['response_fits'] = ()
    if maximum_push_length_m is not None:
        data['maximum_push_length_m'] = maximum_push_length_m
    return Config.from_dict(data)


def sample_scenarios(profile, *, seed=0, count=12, geometry_id='original'):
    integer(seed, 'seed', 0, 2**32-1); integer(count, 'count', 1, 64)
    config = configured_model(profile, geometry_id=geometry_id)
    rng = np.random.default_rng(seed)
    bounds = np.asarray(profile.get('pusht', {}).get('physics', {}).get('randomization_bounds', config.workspace), dtype=float)
    if (bounds.shape != (4,) or not np.isfinite(bounds).all() or bounds[0] >= bounds[1] or bounds[2] >= bounds[3]
            or bounds[0] < config.workspace[0] or bounds[1] > config.workspace[1]
            or bounds[2] < config.workspace[2] or bounds[3] > config.workspace[3]):
        raise ValueError('Randomization bounds must be finite and inside configured workspace')
    cases = []; identities = set()
    for index in range(count):
        mode = ('translation', 'rotation', 'mixed')[index % 3]
        for _ in range(4000):
            initial = np.array([rng.uniform(*bounds[:2]), rng.uniform(*bounds[2:]), rng.uniform(-np.pi, np.pi)])
            goal = initial.copy()
            if mode != 'rotation':
                angle = rng.uniform(-np.pi, np.pi); distance = rng.uniform(.02, .12)
                goal[:2] += distance * np.array([np.cos(angle), np.sin(angle)])
            if mode != 'translation': goal[2] = wrap(goal[2] + rng.choice([-1, 1]) * rng.uniform(.2, 1.2))
            if not valid_pose(initial, config) or not valid_pose(goal, config) or reached(initial, goal, config): continue
            identity = digest(dict(initial=initial.tolist(), goal=goal.tolist(), geometry=geometry_id))
            if identity in identities: continue
            identities.add(identity)
            cases.append(dict(case_id=f'{seed:08x}_{index:03d}', mode=mode, seed=seed, geometry_id=geometry_id,
                initial_pose=initial.tolist(), goal_pose=goal.tolist(), state_digest=identity,
                model_geometry_digest=digest({k: getattr(config, k) for k in sorted(GEOMETRY_FIELDS)}),
                geometry_valid=True, robot_planning_verified=False,
                calibrated_response_reused=(geometry_id == 'original')))
            break
        else: raise ValueError(f'Could not sample case {index}; constraints too tight, no cases silently dropped')
    return dict(schema='rm75_pusht_random_suite_v1', seed=seed, requested_count=count,
                cases=cases, geometry_id=geometry_id, hardware_connected=False,
                success_thresholds=dict(position_m=config.position_tolerance_m, yaw_rad=config.yaw_tolerance_rad))
