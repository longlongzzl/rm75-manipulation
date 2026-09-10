"""Keep provider HTTP in the UI process and deny it in offline task workers.

Called inside worker.main before importing a native engine or physics backend.
No profile/browser flag can disable isolation for preview/sim. Real jobs retain
the existing separate authorization checks; this function does not authorize.
"""
def isolate_task_worker(spec, events):
    mode = spec.get('mode')
    if mode == 'real':
        return False
    if mode not in ('preview', 'sim'):
        raise ValueError('Unknown task mode at offline worker boundary')
    block_network = _load_network_filter()
    # Installation/self-test failure propagates to the worker's failed result.
    block_network()
    events.emit('worker_network_isolation_active', mode=mode,
                non_unix_network_denied=True, kernel_filter_inherited_by_children=True,
                serial_usb_isolated=False, hardware_authorized=False)
    return True


def _load_network_filter():
    """Load the existing seccomp filter, keeping the normal import when possible.

    The worker may run with an app-root that is not this repository (service
    test fixtures symlink rm75_app only), where `import tools` fails. In that
    case resolve this module's own file through symlinks to the real repository
    root, where tools/run_network_isolated.py lives. The plain import stays
    primary so tests and callers can patch tools.run_network_isolated directly.
    """
    try:
        from tools.run_network_isolated import block_network
        return block_network
    except ImportError:
        pass
    import importlib.util
    from pathlib import Path
    module_path = Path(__file__).resolve().parents[2] / 'tools' / 'run_network_isolated.py'
    if not module_path.is_file():
        raise RuntimeError('Network isolation filter module is missing; refusing to dispatch')
    spec = importlib.util.spec_from_file_location('_workcell_network_filter', module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.block_network
