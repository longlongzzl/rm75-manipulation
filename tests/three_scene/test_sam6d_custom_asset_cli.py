"""Exercise the CLI guard without importing GPU perception dependencies."""
import ast
from pathlib import Path

import pytest


def guard():
    path = Path(__file__).resolve().parents[2] / 'rm75_app/perception/sam6d_pose_provider.py'
    tree = ast.parse(path.read_text())
    node = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                and n.name == 'validate_object_name_args')
    namespace = {'list_object_spec_names': lambda: ['gluestick'],
                 'normalize_object_name': lambda name: name}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace['validate_object_name_args'], tree


@pytest.mark.parametrize('names,mesh,valid', [
    (['gluestick'], None, True),
    (['orange_jimu_plate'], '/explicit/plate.glb', True),
    (['orange_jimu_plate'], None, False),
    (['orange_jimu_plate', 'gluestick'], '/explicit/plate.glb', False),
    (['orange--mask-mode'], '/explicit/plate.glb', False),
])
def test_only_explicit_single_custom_cad_bypasses_known_asset_lookup(names, mesh, valid):
    validate, _ = guard()
    if valid:
        validate(names, mesh_file=mesh)
    else:
        with pytest.raises(ValueError):
            validate(names, mesh_file=mesh)


def test_main_passes_explicit_mesh_into_guard():
    _, tree = guard()
    main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
    call = next(n for n in ast.walk(main) if isinstance(n, ast.Call)
                and isinstance(n.func, ast.Name) and n.func.id == 'validate_object_name_args')
    mesh = next(k.value for k in call.keywords if k.arg == 'mesh_file')
    assert ast.unparse(mesh) == 'args.mesh_file'
