"""cuRobo-only boundary for the archived native PickPlace implementation.

Four verified source modules are adapted in memory; the source archive remains
reproducible. Remove the second planner/FCL dependency, retain scene bookkeeping,
and route collision queries to the *actual* cuRobo world. Never fake clear checks.
"""
import ast
import copy
from contextlib import contextmanager
from dataclasses import dataclass
import functools
from importlib.machinery import SourceFileLoader
from importlib.abc import MetaPathFinder
from pathlib import Path
import sys
from types import SimpleNamespace


class CuroboOnlyUnsupported(BaseException):
    """Fail closed across the native broad except-Exception fallback ladder."""


@dataclass
class Box:
    size: object


@dataclass
class CollisionObject:
    geometry: object
    position: object = None
    quaternion: object = None


# Data containers only: the real collision world is built by the original
# _scene_obstacles_to_curobo_world, not by these obsolete FCL records.
collision_detection = SimpleNamespace(fcl=SimpleNamespace(Box=Box, CollisionObject=CollisionObject))
pymp = None


def as_sim_action(demo, action):
    """ManiSkill 3 consumes tensors; preserve action values and device."""
    import torch
    return torch.as_tensor(action,device=getattr(demo.base_env,'device',None),dtype=torch.float32)


def step_sim(demo, action):
    try:
        # Original bridge stores this reset-time geometry as NumPy, but the
        # installed ManiSkill dense reward consumes it with torch.linalg.
        # Convert representation only: no reward/geometry/success change.
        import torch
        edge=getattr(demo.base_env,'obj_xy_shortest_edge_vector',None)
        if edge is not None:
            demo.base_env.obj_xy_shortest_edge_vector=torch.as_tensor(
                edge,device=getattr(demo.base_env,'device',None),dtype=torch.float32)
        return demo.env.step(as_sim_action(demo,action))
    except Exception as exc:
        # Native sync/settle helpers catch Exception and merely print a warning.
        # A failed simulator step must invalidate this run, not report success.
        raise CuroboOnlyUnsupported('simulation step failed') from exc


class CuroboDemoPlanner:
    def __init__(self, demo):
        self.demo = demo
        self.robot = demo.robot
        self.normal_objects = {}
        self.joint_types = []
        self.joint_limits = []
        self.native = None
        self.lock = None

    def set_base_pose(self, pose):
        self.base_pose = pose

    def set_normal_object(self, name, value):
        self.normal_objects[name] = value

    def remove_normal_object(self, name):
        self.normal_objects.pop(name, None)

    def update_attached_box(self, *args, **kwargs):
        self.attached_box_record = (args, kwargs)

    def _collisions(self, qpos, kind, use_attach=False):
        native = self.native
        if native is None or self.lock is None:
            raise CuroboOnlyUnsupported('cuRobo world has not been bound to this demo')
        with self.lock:
            if not native.collision_enabled or not native.config.self_collision_check:
                raise CuroboOnlyUnsupported('cuRobo collision checks disabled')
            if native._disabled_collision_links:
                raise CuroboOnlyUnsupported('shared collision spheres disabled')
            if use_attach and not native.attached_object_active:
                raise CuroboOnlyUnsupported('attached collision model missing')
            valid, status = native.check_start_state(qpos)
            if valid:
                return []
            # Native reports the first invalid category, not every overlapping
            # pair. Unknown failures are conservative for both legacy queries.
            status = str(status)
            if kind == 'self' and 'WORLD_COLLISION' in status:
                return []
            if kind == 'world' and 'SELF_COLLISION' in status:
                return []
            return [SimpleNamespace(link_name1='cuRobo', object_name1=kind,
                    link_name2='', object_name2=status)]

    def check_for_self_collision(self, qpos, **kwargs):
        return self._collisions(qpos, 'self')

    def check_for_env_collision(self, qpos, *, with_point_cloud=False, use_attach=False):
        if with_point_cloud:
            raise CuroboOnlyUnsupported('unqualified legacy point cloud request')
        return self._collisions(qpos, 'world', use_attach)

    def plan_qpos_to_pose(self, *args, **kwargs):
        raise CuroboOnlyUnsupported('legacy pose fallback removed; use cuRobo')

    plan_qpos_to_qpos = plan_qpos_to_pose
    IK = plan_qpos_to_pose


def transform_source(source, filename, *, snapshot_root=None):
    """Adapt dependency plumbing and one verified reverse-path reuse predicate.

    Original candidate generation, solver calls and fallback bodies stay intact.
    The reuse hook is installed by install_release_contact before native main.
    """
    direct_entry=Path(filename).name=='rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place.py'
    reverse_hooks=[]
    class Rewrite(ast.NodeTransformer):
        def visit_If(self,node):
            self.generic_visit(node)
            if direct_entry and isinstance(node.test,ast.Name) and node.test.id=='reverse_endpoint_valid':
                reverse_hooks.append(node)
                check=ast.parse('_rm75_clearance_reverse_path_reusable(planner, demo, args, reverse_clearance_path)',mode='eval').body
                node.test=ast.copy_location(ast.BoolOp(op=ast.And(),values=[node.test,check]),node.test)
            return node

        def visit_Constant(self,node):
            # Fixed migration rewrote absolute paths, but these 24 original
            # mesh assets use a literal tilde prefix. Their vendored bytes were
            # audited equal; resolve only this exact known dependency subtree.
            prefix='~/Desktop/lerobot/pick_jiaobang/meshs/'
            if snapshot_root is not None and isinstance(node.value,str) and node.value.startswith(prefix):
                target=Path(snapshot_root)/'pick_jiaobang/meshs'/node.value[len(prefix):]
                if not target.is_file():raise FileNotFoundError('Missing vendored object asset: '+str(target))
                node.value=str(target.resolve())
            return node

        def visit_FunctionDef(self, node):
            self.generic_visit(node)
            if node.name == 'step_and_render':
                first=node.body[0]
                if not isinstance(first,ast.Assign):
                    raise RuntimeError('Unexpected native simulator step layout')
                first.value=ast.copy_location(ast.parse('mplib.step_sim(self, action)',mode='eval').body,first.value)
            return node

        def visit_Import(self, node):
            if any(alias.name == 'mplib' for alias in node.names):
                if len(node.names) != 1:
                    raise RuntimeError('Unexpected combined mplib import')
                return ast.copy_location(ast.ImportFrom(module='rm75_app.workcell',
                    names=[ast.alias(name='pickplace_curobo_only', asname='mplib')], level=0), node)
            return node

        def visit_ImportFrom(self, node):
            if node.module == 'mplib':
                if [(a.name,a.asname) for a in node.names] != [('collision_detection','mplib_cd')]:
                    raise RuntimeError('Unexpected legacy mplib import')
                node.module = 'rm75_app.workcell.pickplace_curobo_only'
            return node

        def visit_Call(self, node):
            self.generic_visit(node)
            if (isinstance(node.func,ast.Attribute) and node.func.attr=='Planner'
                    and isinstance(node.func.value,ast.Name) and node.func.value.id=='mplib'):
                node.func.attr = 'CuroboDemoPlanner'
                node.args = [ast.Name(id='self',ctx=ast.Load())]
                node.keywords = []
            return node
    tree=Rewrite().visit(ast.parse(source,filename))
    if direct_entry and len(reverse_hooks)!=1:
        raise RuntimeError('Reviewed reverse-clearance reuse boundary changed')
    return ast.fix_missing_locations(tree)


@contextmanager
def source_adapter(root):
    """Process-local loader for the four audited modules using MPLib/FCL."""
    root = Path(root).resolve()
    targets = {root/'pick_jiaobang'/name for name in (
        'rm75_jiaobang_pick_move_v10_perpendicular_to_object.py',
        'rm75_jiaobang_pick_real_with_foundationpose.py',
        'rm75_jiaobang_pick_place_targeted.py',
        'rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place.py','object_specs.py')}
    original = SourceFileLoader.get_code
    class NoMplib(MetaPathFinder):
        def find_spec(self, fullname, path=None, target=None):
            if fullname=='mplib' or fullname.startswith('mplib.'):
                raise CuroboOnlyUnsupported('external MPLib import forbidden: '+fullname)
    if any(name=='mplib' or name.startswith('mplib.') for name in sys.modules):
        raise CuroboOnlyUnsupported('MPLib already loaded before isolated native adapter')
    guard=NoMplib()
    def get_code(loader, fullname):
        path = Path(loader.path).resolve()
        if path in targets:
            # Avoid reusing a pyc that still imports/constructs MPLib.
            return compile(transform_source(path.read_text(),str(path),snapshot_root=root),str(path),'exec')
        return original(loader,fullname)
    SourceFileLoader.get_code = get_code
    sys.meta_path.insert(0,guard)
    try:
        yield
    finally:
        SourceFileLoader.get_code = original
        sys.meta_path.remove(guard)


def install_jimu_binding(portable):
    """Keep native demo setup but replace its always-clear diagnostic planner.

    The four source adapters already supply data-only FCL records. Therefore
    Jimu's import-and-monkeypatch of external MPLib is obsolete; no global
    MPLib module is created, and every collision query uses the bound cuRobo.
    """
    for name in ('_JimuNoopMplibPlanner','_install_jimu_no_mplib_collision_detection',
                 '_restore_jimu_no_mplib_collision_detection'):
        if not hasattr(portable,name):raise RuntimeError('Missing reviewed Jimu boundary: '+name)
    portable._JimuNoopMplibPlanner=CuroboDemoPlanner
    portable._install_jimu_no_mplib_collision_detection=lambda args=None:None
    portable._restore_jimu_no_mplib_collision_detection=lambda:None


def install(direct):
    from .transport_contact import install_start_check_restoration
    from curobo_rm75_planner import RM75CuRoboPlanner
    install_start_check_restoration(RM75CuRoboPlanner)
    preserve_diagnostic_world_state(RM75CuRoboPlanner,direct._CUROBO_GPU_LOCK)
    isolate_print_only_diagnostics(direct)
    serialize_return_planning(direct)
    from .pickplace_release_contact import install_release_contact
    install_release_contact(direct)
    original_refresh = direct._refresh_curobo_world
    @functools.wraps(original_refresh)
    def refresh(planner, demo, args, **kwargs):
        with direct._CUROBO_GPU_LOCK:
            from .pickplace_gripper_state import update_from_demo
            update_from_demo(planner,demo,args)
            result = original_refresh(planner,demo,args,**kwargs)
            if isinstance(demo.planner,CuroboDemoPlanner):
                demo.planner.native = planner
                demo.planner.lock = direct._CUROBO_GPU_LOCK
            return result
    direct._refresh_curobo_world = refresh
    original_parse = direct.parse_args
    @functools.wraps(original_parse)
    def parse(*args,**kwargs):
        parsed = original_parse(*args,**kwargs)
        # Removed backend must never be a fallback (especially after a failed
        # cuRobo return). All primary grasp/place parameters remain untouched.
        parsed.return_to_start_mplib_fallback = False
        return parsed
    direct.parse_args = parse
    clearance_audits=[]
    original_wrappers=direct._install_dry_run_motion_window_wrappers
    @functools.wraps(original_wrappers)
    def install_wrappers():
        original_wrappers()
        # Install OUTSIDE native wrappers, including their early dry-run branch.
        from .pickplace_clearance_audit import install_execution_guards
        install_execution_guards(direct.targeted.base,direct._CUROBO_GPU_LOCK,clearance_audits,
                                 released_source=direct._current_source_object_name)
    direct._install_dry_run_motion_window_wrappers=install_wrappers
    return clearance_audits


def isolate_print_only_diagnostics(direct):
    """An unavailable failure printout must not abort original source retries.

    Only two void PRINT helpers are wrapped. Planner queries and execution
    guards still raise CuroboOnlyUnsupported, never reporting false clear space.
    """
    records=[];direct._curobo_diagnostic_rejections=records
    for owner,name in ((direct,'_print_transport_motiongen_failure_diagnostics'),
                       (direct.targeted.base,'print_failure_diagnostics')):
        original=getattr(owner,name,None)
        if not callable(original):continue
        def wrap(function,name):
            @functools.wraps(function)
            def diagnostic(*args,**kwargs):
                try:return function(*args,**kwargs)
                except CuroboOnlyUnsupported as exc:
                    row={'function':name,'error':str(exc),'collision_state_qualified':False}
                    records.append(row)
                    print(f'[curobo diagnostic unavailable] {row}')
                    return None
            return diagnostic
        setattr(owner,name,wrap(original,name))


def serialize_return_planning(direct):
    """Keep temporary return-preplan masks inside the existing GPU RLock.

    The old worker shares its planner with foreground release/clearance. Locking
    only solve_ik left the disable/restore scope observable by geometry refresh.
    Do not lock worker creation or consumption (the latter may join the worker).
    Native try/finally still owns mask restoration; no collision policy changes.
    """
    for name in ('_plan_return_to_start_joint_curobo',
                 '_plan_return_to_start_prelift_rescue_curobo'):
        original = getattr(direct, name)
        def wrap(function):
            @functools.wraps(function)
            def transaction(*args, **kwargs):
                with direct._CUROBO_GPU_LOCK:
                    planner = args[0] if args else kwargs.get('planner')
                    if planner is None:
                        return function(*args, **kwargs)
                    with preserve_planner_world(planner):
                        return function(*args, **kwargs)
            return transaction
        setattr(direct, name, wrap(original))


@contextmanager
def preserve_planner_world(planner):
    """Restore the foreground world after a shared return-planning transaction.

    Called under the GPU RLock. WorldConfig contains CPU obstacle descriptions;
    the native supported update_world APIs rebind every solver/cache owner.
    """
    world = copy.deepcopy(planner._world)
    disabled = set(planner._disabled_world_obstacles)
    fields = ('_persistent_world_signature', '_last_world_changed',
              '_last_world_cache_hit', '_last_world_cache_forced_refresh')
    previous = {name: copy.deepcopy(getattr(planner, name))
                for name in fields if hasattr(planner, name)}
    try:
        yield
    finally:
        planner.motion_gen.update_world(world)
        planner.ik_solver.update_world(world)
        planner._world = world
        planner._update_cuda_graph_batch_ik_world(world)
        names = set(planner.world_collision_checker_obstacle_names())
        present = {item.name for item in world.objects}
        # Cached rows absent from the restored scene must remain disabled.
        enable = present - disabled
        disable = (names - present) | disabled
        planner.set_world_obstacles_enabled(sorted(enable), enabled=True)
        planner.set_world_obstacles_enabled(sorted(disable), enabled=False)
        for name in fields:
            if name in previous:
                setattr(planner, name, previous[name])
            elif hasattr(planner, name):
                delattr(planner, name)


def preserve_diagnostic_world_state(planner_class, lock):
    """Native obstacle ablation must not enable an initially disabled cache."""
    original=planner_class.diagnose_start_state_world_collision
    @functools.wraps(original)
    def diagnose(planner,*args,**kwargs):
        with lock:
            before=set(planner._disabled_world_obstacles)
            setter=planner.set_world_obstacles_enabled
            had_override='set_world_obstacles_enabled' in vars(planner)
            previous_override=vars(planner).get('set_world_obstacles_enabled')
            def scoped_set(names,*,enabled):
                if not enabled:return setter(names,enabled=False)
                restore_disabled=[name for name in names if name in before]
                restore_enabled=[name for name in names if name not in before]
                return (setter(restore_disabled,enabled=False) if restore_disabled else []) + (
                    setter(restore_enabled,enabled=True) if restore_enabled else [])
            planner.set_world_obstacles_enabled=scoped_set
            try:return original(planner,*args,**kwargs)
            finally:
                if had_override:planner.set_world_obstacles_enabled=previous_override
                else:del planner.set_world_obstacles_enabled
                current=set(planner._disabled_world_obstacles)
                if current-before:setter(sorted(current-before),enabled=True)
                if before-current:setter(sorted(before-current),enabled=False)
    planner_class.diagnose_start_state_world_collision=diagnose
