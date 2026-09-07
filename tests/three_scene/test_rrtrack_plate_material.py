from pathlib import Path

import numpy as np
import trimesh

from rm75_app.perception.rrtrack.foundationpose_adapter import foundationpose_compatible_mesh


def test_actual_plate_uniform_pbr_expands_color_without_changing_mesh():
    root = Path(__file__).resolve().parents[2]
    path = root / ('rm75_app/_vendor/working_snapshot/Beta_demo-codex-v0.9/'
                   'jimu_portable_repro/assets/red_jimu_plate_74x6p5x74.glb')
    mesh = trimesh.load(path, force='scene').dump(concatenate=True)
    original_vertices, original_faces = mesh.vertices.copy(), mesh.faces.copy()
    expected_color = mesh.visual.material.main_color.copy()
    compatible = foundationpose_compatible_mesh(mesh)
    copied = compatible.copy()  # This exact FoundationPose operation failed.
    assert copied.visual.vertex_colors.shape == (len(mesh.vertices), 4)
    np.testing.assert_array_equal(copied.visual.vertex_colors,
                                  np.tile(expected_color, (len(mesh.vertices), 1)))
    np.testing.assert_array_equal(copied.vertices, original_vertices)
    np.testing.assert_array_equal(copied.faces, original_faces)
    np.testing.assert_array_equal(mesh.vertices, original_vertices)
    assert isinstance(mesh.visual, trimesh.visual.texture.TextureVisuals)


def test_native_diameter_keeps_value_without_numpy_double_promotion(monkeypatch, tmp_path):
    from types import SimpleNamespace
    from rm75_app.perception.rrtrack import foundationpose_adapter as adapter
    (tmp_path / 'estimater.py').write_text('# fixture')
    diameter = np.float64(0.10485347053880018)
    fake = SimpleNamespace(
        dr=SimpleNamespace(RasterizeCudaContext=lambda: None),
        FoundationPose=lambda **kwargs: SimpleNamespace(diameter=diameter))
    monkeypatch.setattr(adapter, '_load_module', lambda *args: fake)
    refiner = adapter.FoundationPoseRefiner(trimesh.creation.box(), foundationpose_root=tmp_path)
    assert type(refiner.estimator.diameter) is float
    assert refiner.estimator.diameter == diameter
