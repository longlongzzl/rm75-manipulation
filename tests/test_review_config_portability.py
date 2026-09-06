"""The checked-in collision sphere file follows the robot config, not cwd."""
from pathlib import Path
import yaml
from rm75_app.planning.backends.curobo2 import load_curobo2_robot_config


def test_collision_spheres_resolve_from_config_directory(monkeypatch, tmp_path):
    config = Path(__file__).resolve().parents[1] / "assets/curobo_rm75_config/rm75.yml"
    original = config.read_text()
    sphere_path = yaml.safe_load(original)["robot_cfg"]["kinematics"]["collision_spheres"]
    assert not Path(sphere_path).is_absolute()
    expected = yaml.safe_load((config.parent / sphere_path).read_text())["collision_spheres"]
    monkeypatch.chdir(tmp_path)
    converted = load_curobo2_robot_config(config)
    assert converted["robot_cfg"]["kinematics"]["collision_spheres"] == expected
    assert config.read_text() == original
