from types import SimpleNamespace

from rm75_app.runtime import rrtrack_pose_tracking as entry


def test_custom_mesh_scale_and_prompt_reach_initialization(monkeypatch, tmp_path):
    result = tmp_path / 'sam6d_pose_result.json'
    result.write_text('{}')
    calls = []

    def run(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=0, stderr='',
                               stdout=f'[sam6d-gdino] result: {result}')

    monkeypatch.setattr(entry.subprocess, 'run', run)
    args = entry.parse_args(['--object-name', 'orange_jimu_plate',
                             '--mesh-file', str(tmp_path / 'plate.glb'),
                             '--mesh-scale', '1', '--prompt', 'orange square plastic panel.'])
    assert entry._run_sam6d_initialization(args) == result
    command = calls[0]
    for flag, value in [('--mesh-file', str(tmp_path / 'plate.glb')),
                        ('--mesh-scale', '1.0'),
                        ('--prompt', 'orange square plastic panel.')]:
        assert command[command.index(flag) + 1] == value
    assert '--execute-real' not in command


def test_named_asset_keeps_default_initialization_arguments(monkeypatch, tmp_path):
    result = tmp_path / 'sam6d_pose_result.json'
    result.write_text('{}')
    calls = []
    def run(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=0, stderr='',
                               stdout=f'[sam6d-gdino] result: {result}')
    monkeypatch.setattr(entry.subprocess, 'run', run)
    entry._run_sam6d_initialization(entry.parse_args(['--object-name', 'gluestick']))
    assert not {'--mesh-file', '--mesh-scale', '--prompt'}.intersection(calls[0])


def test_sam3_recovery_uses_native_model_resolution():
    assert entry.parse_args(['--object-name', 'gluestick']).sam3_recovery_resolution == 1008
