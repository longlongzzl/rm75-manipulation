from pathlib import Path

from rm75_app.orchestration.multi_object_executor import load_task_scene
from rm75_app.scenarios.sorting import SortingPlanCompiler
from rm75_app.scenarios.sorting_io import load_sorting_request


def test_three_object_fixture_preserves_all_assignments_and_supports():
    root = Path(__file__).resolve().parents[1]
    request = load_sorting_request(root / "benchmarks/unified_scenarios/fixtures/sorting_historical_three_objects.json")
    scene = load_task_scene(str(root / request.scene_file))
    plan = SortingPlanCompiler().compile(request, scene)
    assert [atom.object_id for atom in plan.atoms] == ["tennis", "gluestick", "carriot"]
    assert [atom.support_object_id for atom in plan.atoms] == ["bitong", "lvmukuai", "shuazi"]
    assert [atom.success.relation for atom in plan.atoms] == ["inside", "target_pose", "target_pose"]
    assert all(atom.success.position_tolerance_m == 0.02 for atom in plan.atoms)
    assert set(scene.objects) >= {"tennis", "gluestick", "carriot", "bitong", "lvmukuai", "shuazi"}
    assert request.metadata["not_u1_real_snapshot_suite"] is True
