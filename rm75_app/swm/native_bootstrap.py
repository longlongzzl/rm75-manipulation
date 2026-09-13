"""Owned original frozen-world initialization for the atomic worker runtime.

This module never invokes an episode. Initialization-only pose writes stay in
the original create_demo; subsequent reads do not synchronize into the primary
world. Installing a complete runtime factory remains a separate requirement.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from contextlib import nullcontext
import threading
import time
from pathlib import Path
import shutil
from unittest.mock import patch

import numpy as np

from .scene import SceneInvalid


def _array(value):
    if hasattr(value, 'detach'):
        value = value.detach().cpu().numpy()
    result = np.asarray(value, dtype=float)
    if not np.isfinite(result).all():
        raise SceneInvalid('Nonfinite primary simulation readback')
    return result


@dataclass
class FrozenPrimaryWorld:
    env: object
    demo: object
    args: object
    actors: dict
    contract: dict
    stop: object
    sequence: int = 0
    closed: bool = False
    construction_recipes: dict = field(default_factory=dict)
    constraint_recipes: dict = field(default_factory=dict)
    velocity_readback: object = None

    def read_state(self):
        """Fresh actual actor/robot reads; no setters and no command-cache data.

        This is a physical-state exposure interval, not a camera frame or an
        inferred holding claim. SWM conversion must retain that distinction.
        """
        from rm75_app.execution.maniskill_task_bridge import _pose_matrix

        if self.closed:
            raise SceneInvalid('Primary simulation is closed')
        self.stop.check()
        started = time.monotonic()
        robot = self.demo.robot
        names = tuple(joint.get_name() for joint in robot.get_active_joints())
        q = _array(robot.get_qpos()).reshape(-1)
        qdot = _array(robot.get_qvel()).reshape(-1)
        if len(names) != len(q) or len(qdot) != len(q) or len(set(names)) != len(names):
            raise SceneInvalid('Primary robot joint readback identity mismatch')
        velocity_evidence = None
        if self.velocity_readback is not None:
            qdot, velocity_evidence = self.velocity_readback(names, q, qdot)
        objects = {oid: _pose_matrix(actor).tolist() for oid, actor in self.actors.items()}
        ended = time.monotonic()
        self.sequence += 1
        return dict(schema='rm75.swm_primary_readback_v1', domain='physics',
            source='native_actor_and_joint_readback', sequence=self.sequence,
            capture_started_at=started, capture_finished_at=ended,
            joint_names=list(names), positions=q.tolist(), velocities=qdot.tolist(),
            velocity_evidence=velocity_evidence,
            objects=objects, coordinate_frame='native_world',
            initialization_input_sha256=self.contract['sha256'],
            hardware_connected=False, holding_verified=False)

    def close(self):
        if not self.closed:
            self.closed = True
            self.construction_recipes.clear()
            self.constraint_recipes.clear()
            self.env.close()


def initialize_frozen_primary(resources, direct, base_args, contract, *, stop, events, artifact_directory, capture_builders=False):
    """Reuse the existing full-scene constructor under immediate resource ownership.

    Called only in a dedicated, network-isolated simulation worker. The caller
    already owns the reviewed source_adapter and original imported modules.
    Register gym's returned environment BEFORE reset, asset loading, scene
    registration or demo construction can fail. No live localization fallback.
    """
    if (getattr(base_args, 'execute_real', False)
            or getattr(base_args, 'skip_foundationpose', False) is not True
            or getattr(base_args, 'foundationpose_refine_after_render', False)):
        raise PermissionError('Frozen primary initialization is offline simulation only')
    artifact_directory = Path(artifact_directory).absolute()
    # Refuse pre-existing paths (including symlinks), so constructor-generated
    # artifacts cannot replace any user-owned or earlier-run file.
    artifact_directory.mkdir(parents=False, exist_ok=False)
    source = contract['source']
    base = direct.targeted.base
    args, _ = base.make_cycle_args(base_args, source)
    args.selected_obstacle_object_names = [oid for oid in contract['names'] if oid != source]
    args.required_scene_object_names = list(args.selected_obstacle_object_names)
    args.fixed_scene_strict = True
    args.freeze_active_object_before_grasp = False
    args.next_cycle_plan_prefetch = False
    args.foundationpose_refine_after_render = False
    args._targeted_place_state_cache = {'used_slots_by_target': {}}
    bridge = base.load_module_from_path('swm_original_pick_bridge', args.bridge_script_path)
    planner = base.load_module_from_path('swm_original_demo_setup', args.pick_script_path)
    planning_urdf = artifact_directory / 'robot.planning.tiny.urdf'
    planning_srdf = artifact_directory / 'robot.permissive.srdf'
    source_srdf = getattr(args, 'srdf_path', None)
    if source_srdf is not None:
        shutil.copyfile(Path(source_srdf).expanduser().resolve(), planning_srdf)
        args.srdf_path = str(planning_srdf)
    def artifact_paths(sim_urdf_path, native_args):
        return str(planning_urdf), str(planning_srdf)
    cache = {}
    owner = threading.get_ident()
    original_make = base.gym.make
    acquired = []
    lifetime = {'world': None, 'closed': False}

    def close_environment(env):
        if lifetime['closed']:
            return
        lifetime['closed'] = True
        if lifetime['world'] is not None:
            lifetime['world'].close()
        else:
            env.close()
        events.emit('swm_primary_closed', hardware_connected=False)

    def acquire(*pos, **kw):
        if threading.get_ident() != owner or acquired:
            raise SceneInvalid('Primary initialization cannot create extra/background worlds')
        stop.check()
        env = original_make(*pos, **kw)
        acquired.append(env)
        resources.callback(close_environment, env)
        events.emit('swm_primary_acquired', hardware_connected=False)
        return env

    stop.check()
    if capture_builders:
        import sapien
        from mani_skill.utils.building.actor_builder import ActorBuilder
        from .native_construction import capture_actor_construction, capture_drive_construction
        construction_context = capture_actor_construction(ActorBuilder)
        drive_context = capture_drive_construction(sapien.Scene, sapien.physx.PhysxDriveComponent)
    else:
        construction_context = nullcontext({})
        drive_context = nullcontext({})
    with construction_context as recipes, drive_context as drive_recipes, patch.object(base.gym, 'make', acquire), patch.object(
            planner, 'resolve_planning_artifact_paths', artifact_paths):
        env, demo = base.create_demo(args, bridge, planner, scene_capture_cache=cache)
    if len(acquired) != 1 or env is not acquired[0]:
        raise SceneInvalid('Original initialization returned another environment')
    if getattr(demo, 'foundationpose_runtime', None) is not None:
        raise SceneInvalid('Frozen primary unexpectedly created a live localization runtime')
    if getattr(demo, '_freeze_active_object_before_grasp', False):
        raise SceneInvalid('Primary world must not teleport a frozen target after initialization')
    registry = direct._single_scene_build_actor_registry(demo, base_args, cache, source, args)
    if set(registry) != set(contract['names']) or any(row.get('actor') is None for row in registry.values()):
        raise SceneInvalid('Original primary world omitted a frozen scene actor')
    world = FrozenPrimaryWorld(env, demo, args, {oid: row['actor'] for oid, row in registry.items()},
                               contract, stop, construction_recipes=recipes, constraint_recipes=drive_recipes)
    lifetime['world'] = world
    events.emit('swm_primary_initialized', source=source, object_ids=sorted(world.actors),
                input_sha256=contract['sha256'], primary_world=True, hardware_connected=False)
    return world
