#!/usr/bin/env python3
"""GPU cost-level diagnostic with synthetic spheres; no trajectory or hardware."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rm75_app.workcell.io import atomic_json
from rm75_app.workcell.migration import verify_snapshot
from rm75_app.workcell.world_only_contact import world_only_fingers, FINGER_LINKS


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--extensions', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--transport-payload-pairs', action='store_true')
    args = parser.parse_args()
    root = ROOT / 'rm75_app/_vendor/working_snapshot'
    verify_snapshot(root)
    sys.path.insert(0, str(root / 'pick_jiaobang'))
    from curobo_rm75_planner import RM75CuRoboPlanner, RM75CuRoboPlannerConfig
    if args.transport_payload_pairs:
        from rm75_app.workcell.transport_contact import install_payload_pairs, install_start_check_restoration
        install_payload_pairs(RM75CuRoboPlanner)
        install_start_check_restoration(RM75CuRoboPlanner)
    import torch
    robot_cfg = root / 'pick_jiaobang/curobo_rm75_config/rm75.yml'
    planner = RM75CuRoboPlanner(RM75CuRoboPlannerConfig(
        torch_extensions_dir=args.extensions.resolve(), robot_cfg_path=robot_cfg))
    planner.set_world_from_cuboids([{'name': 'synthetic_table', 'pose': [0, 0, 0, 1, 0, 0, 0], 'dims': [0.2, 0.2, 0.2]}])
    rollout = planner.ik_solver.rollout_fn
    config = rollout.kinematics.kinematics_config
    mapping = config.link_sphere_idx_map
    ids = config.link_name_to_idx_map
    fingers = [i for i, link_id in enumerate(mapping.tolist()) if link_id in {ids[n] for n in FINGER_LINKS if n in ids}]
    others = [i for i, link_id in enumerate(mapping.tolist()) if link_id not in {ids[n] for n in FINGER_LINKS if n in ids}]
    assert fingers and others
    world = rollout.primitive_collision_constraint
    self_check = rollout.robot_self_collision_constraint
    def spheres_at(index):
        value = torch.full((1, 1, len(mapping), 4), 10., device=mapping.device)
        value[..., 3] = 0.01
        value[0, 0, index, :3] = 0
        return value
    finger = spheres_at(fingers[0])
    arm = spheres_at(others[0])
    overlap = torch.zeros_like(finger)
    overlap[..., 3] = 0.1
    before = {'finger_world': world.forward(finger).clone(), 'arm_world': world.forward(arm).clone(),
              'self_overlap': self_check.forward(overlap).clone()}
    with world_only_fingers(planner) as evidence:
        during = {'finger_world': world.forward(finger).clone(), 'arm_world': world.forward(arm).clone(),
                  'self_overlap': self_check.forward(overlap).clone()}
    after = world.forward(finger).clone()
    assert bool((before['finger_world'] > 0).any()) and bool((during['finger_world'] == 0).all())
    assert bool((during['arm_world'] > 0).any()) and torch.equal(before['arm_world'], during['arm_world'])
    assert bool((before['self_overlap'] > 0).any()) and torch.equal(before['self_overlap'], during['self_overlap'])
    assert torch.equal(after, before['finger_world'])
    report = {'status': 'PASS', 'kind': 'synthetic_gpu_cost_only', 'motion': False,
              'before': {k: v.tolist() for k, v in before.items()},
              'during': {k: v.tolist() for k, v in during.items()}, 'restored_finger_world': after.tolist(),
              'evidence': evidence, 'full_chain_validated': False}
    if args.transport_payload_pairs:
        def first(name):
            return next(i for i, value in enumerate(mapping.tolist()) if value == ids[name])
        payload = first('attached_object')
        finger_index = first('left_pad')
        base_index = first('base_link')
        arm_index = first('link_4')
        def pair_cost(a, b):
            spheres = torch.zeros_like(finger)
            spheres[..., 3] = -100.
            spheres[0, 0, [a, b], 3] = 0.05
            return self_check.forward(spheres).clone()
        costs = {'payload_finger': pair_cost(payload, finger_index),
                 'payload_base': pair_cost(payload, base_index),
                 'payload_arm': pair_cost(payload, arm_index),
                 'payload_world': world.forward(spheres_at(payload)).clone(),
                 'finger_world': world.forward(spheres_at(finger_index)).clone()}
        assert bool((costs['payload_finger'] == 0).all())
        assert all(bool((value > 0).any()) for name, value in costs.items() if name != 'payload_finger')
        report['transport_pair_checks'] = {k: v.tolist() for k, v in costs.items()}
        report['payload_contact_ignore_pairs'] = sorted(planner._self_collision_ignore_pairs())
        planner.set_world_from_cuboids([{'name': 'synthetic_blocking_world',
            'pose': [0, 0, 0, 1, 0, 0, 0], 'dims': [10., 10., 10.]}])
        valid, status = planner.check_start_state([0.] * 7)
        restored = {name: bool(getattr(planner.motion_gen.rollout_fn, name).enabled)
                    for name in ('primitive_collision_constraint', 'robot_self_collision_constraint')}
        assert not valid and str(status).endswith('INVALID_START_STATE_WORLD_COLLISION')
        assert all(restored.values())
        report['native_start_check_restoration'] = {'valid': valid, 'status': str(status), 'restored': restored}
    atomic_json(args.output.resolve(), report)
    print(json.dumps(report))


if __name__ == '__main__':
    main()
