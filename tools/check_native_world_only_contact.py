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
    args = parser.parse_args()
    root = ROOT / 'rm75_app/_vendor/working_snapshot'
    verify_snapshot(root)
    sys.path.insert(0, str(root / 'pick_jiaobang'))
    from curobo_rm75_planner import RM75CuRoboPlanner, RM75CuRoboPlannerConfig
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
    atomic_json(args.output.resolve(), report)
    print(json.dumps(report))


if __name__ == '__main__':
    main()
