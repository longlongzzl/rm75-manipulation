"""Benchmark evidence must preserve backend config paths in JSON."""
from dataclasses import asdict
import importlib.util
import json
from pathlib import Path

from rm75_app.planning.backends.curobo2 import Curobo2BackendConfig


def test_curobo_config_serializes_with_unified_summary_writer(tmp_path):
    path = Path(__file__).resolve().parents[1] / "tools/run_unified_scenario.py"
    spec = importlib.util.spec_from_file_location("unified_cli_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    target = tmp_path / "summary.json"
    config = asdict(Curobo2BackendConfig())
    module._write_json(target, {"backend_config": config})
    saved = json.loads(target.read_text())["backend_config"]
    for key, value in config.items():
        if isinstance(value, Path):
            assert saved[key] == str(value)
