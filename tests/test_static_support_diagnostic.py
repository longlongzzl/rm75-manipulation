import copy
from pathlib import Path

import numpy as np
import pytest


def test_static_scene_changes_only_two_initial_poses(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "tools"))
    from diagnose_static_support import make_static_scene
    scene = {"objects": [{"object_id": name, "pose": np.eye(4).tolist(),
                          "physics": {"static_friction": 0.4}, "fixed": False}
                         for name in ("carrot", "brush", "obstacle")]}
    original = copy.deepcopy(scene)
    pose = np.eye(4); pose[0, 3] = 0.2
    result = make_static_scene(scene, {"object_pose": pose, "support_pose": np.eye(4)},
                               "carrot", "brush")
    assert scene == original
    assert result["objects"][0]["pose"][0][3] == 0.2
    assert result["objects"][0]["physics"] == original["objects"][0]["physics"]
    assert result["objects"][1:] == original["objects"][1:]


def test_static_scene_rejects_missing_support(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1] / "tools"))
    from diagnose_static_support import make_static_scene
    with pytest.raises(ValueError, match="exactly once"):
        make_static_scene({"objects": []}, {"object_pose": np.eye(4), "support_pose": np.eye(4)},
                          "carrot", "brush")
