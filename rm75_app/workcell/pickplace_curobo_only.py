"""cuRobo-only boundary for the archived native PickPlace implementation.

Four verified source modules are adapted in memory; the source archive remains
reproducible. Remove the second planner/FCL dependency, retain scene bookkeeping,
and route collision queries to the *actual* cuRobo world. Never fake clear checks.
"""
import ast
from contextlib import contextmanager
from dataclasses import dataclass
import functools
from importlib.machinery import SourceFileLoader
from pathlib import Path
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


def transform_source(source, filename):
    """Only remove dependency/init plumbing, never candidate or solver logic."""
    class Rewrite(ast.NodeTransformer):
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
    return ast.fix_missing_locations(Rewrite().visit(ast.parse(source,filename)))


@contextmanager
def source_adapter(root):
    """Process-local loader for the four audited modules using MPLib/FCL."""
    root = Path(root).resolve()
    targets = {root/'pick_jiaobang'/name for name in (
        'rm75_jiaobang_pick_move_v10_perpendicular_to_object.py',
        'rm75_jiaobang_pick_real_with_foundationpose.py',
        'rm75_jiaobang_pick_place_targeted.py',
        'rm75_jiaobang_pick_place_targeted_curobo_direct_pre_place.py')}
    original = SourceFileLoader.get_code
    def get_code(loader, fullname):
        path = Path(loader.path).resolve()
        if path in targets:
            # Avoid reusing a pyc that still imports/constructs MPLib.
            return compile(transform_source(path.read_text(),str(path)),str(path),'exec')
        return original(loader,fullname)
    SourceFileLoader.get_code = get_code
    try:
        yield
    finally:
        SourceFileLoader.get_code = original


def install(direct):
    from .transport_contact import install_start_check_restoration
    from curobo_rm75_planner import RM75CuRoboPlanner
    install_start_check_restoration(RM75CuRoboPlanner)
    preserve_diagnostic_world_state(RM75CuRoboPlanner,direct._CUROBO_GPU_LOCK)
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
        install_execution_guards(direct.targeted.base,direct._CUROBO_GPU_LOCK,clearance_audits)
    direct._install_dry_run_motion_window_wrappers=install_wrappers
    return clearance_audits


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
