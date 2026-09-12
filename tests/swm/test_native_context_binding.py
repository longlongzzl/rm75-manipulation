"""Trusted factory gates only; real worker qualification is a native run."""
import pytest
from rm75_app.swm import integration, native_context


@pytest.fixture
def profile():
    return {'swm': {'enabled': True}, 'pickplace': {
        'fixed_scene_format': 'native_world', 'fixed_scene': 'original.json'}}


def test_installation_is_worker_local_idempotent_and_not_qualification(monkeypatch, profile):
    monkeypatch.setattr(integration, '_FACTORIES', {})
    native_context.install_native_factories(profile)
    native_context.install_native_factories(profile)
    assert integration._FACTORIES == {'pickplace': native_context.pen_runtime_context}
    status = integration.contract_status(profile)
    assert not status['native_atomic_integration_verified']
    assert not status['hardware_qualified']


@pytest.mark.parametrize('mode', ['real', 'preview'])
def test_non_sim_is_rejected_before_native_imports(profile, mode):
    with pytest.raises(PermissionError, match='offline simulation only'):
        native_context.pen_runtime_context({'task': 'pickplace', 'mode': mode,
            'parameters': {'object_name': 'bi'}}, profile, None, None, None, None)


@pytest.mark.parametrize('parameters', [{'object_name': 'tennis'}, {'object_names': ['bi']},
    {'object_name': 'bi', 'callback': 'arbitrary.module'}])
def test_unadapted_programs_remain_unavailable(profile, parameters):
    with pytest.raises(NotImplementedError, match='SWM_ATOMIC_ADAPTER_REQUIRED'):
        native_context.pen_runtime_context({'task': 'pickplace', 'mode': 'sim',
            'parameters': parameters}, profile, None, None, None, None)


def test_missing_profile_does_not_remove_missing_adapter_check(monkeypatch):
    monkeypatch.setattr(integration, '_FACTORIES', {})
    native_context.install_native_factories({'swm': {'enabled': True}})
    with pytest.raises(RuntimeError, match='SWM_ATOMIC_ADAPTER_REQUIRED'):
        integration.dispatch_if_enabled({'task': 'pickplace', 'mode': 'sim'},
            {'swm': {'enabled': True}}, None, None, None, None)


def test_existing_factory_is_not_overwritten(monkeypatch, profile):
    factory = lambda *args: None
    monkeypatch.setattr(integration, '_FACTORIES', {'pickplace': factory})
    with pytest.raises(RuntimeError, match='replace'):
        native_context.install_native_factories(profile)
    assert integration._FACTORIES['pickplace'] is factory
