from pathlib import Path
import pytest
from tools.run_native_contact_audit import build_native_argv


@pytest.mark.parametrize('scene,layers',[('four-wall','first'),('triangle-roof','two')])
def test_original_contact_runner_uses_fixed_scene_without_real_or_live_camera(scene,layers):
    root=Path(__file__).resolve().parents[2]/'rm75_app/_vendor/working_snapshot'
    path,argv=build_native_argv(scene,root,Path('/tmp/example_extensions'),transport_world_checked=True)
    assert path.is_file() and argv[argv.index('--jimu-build-layers')+1]==layers
    fixed=Path(argv[argv.index('--sam6d-fixed-scene-result-file')+1]);assert fixed.is_file()
    assert '--no-fast-chain-cuda-graph-ik' in argv and '--execute-real' not in argv
    assert not any(arg in argv for arg in ('--jimu-live-sam6d','--jimu-apriltag-anchor-localization'))


def test_missing_fixed_scene_does_not_fall_back_to_live_camera(tmp_path):
    with pytest.raises(FileNotFoundError):build_native_argv('four-wall',tmp_path,Path('/tmp/extensions'))


def test_unknown_scene_cannot_select_an_unreviewed_entry(tmp_path):
    with pytest.raises(ValueError):build_native_argv('another-task',tmp_path,Path('/tmp/extensions'))
