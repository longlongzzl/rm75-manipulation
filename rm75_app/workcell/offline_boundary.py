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
    from tools.run_network_isolated import block_network
    # Installation/self-test failure propagates to the worker's failed result.
    block_network()
    events.emit('worker_network_isolation_active', mode=mode,
                non_unix_network_denied=True, kernel_filter_inherited_by_children=True,
                serial_usb_isolated=False, hardware_authorized=False)
    return True
