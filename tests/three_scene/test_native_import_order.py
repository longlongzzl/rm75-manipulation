import sys

from rm75_app.workcell.legacy import import_working_entry


def test_portable_can_prepend_its_own_pickplace_dependency(tmp_path, monkeypatch):
    pick = tmp_path / 'pick_jiaobang'
    jimu = tmp_path / 'Beta_demo-codex-v0.9'
    pick.mkdir()
    jimu.mkdir()
    (pick / 'audit_order_dependency.py').write_text("SOURCE = 'pickplace'\n")
    (jimu / 'audit_order_dependency.py').write_text("SOURCE = 'wrong_same_name'\n")
    (jimu / 'rm75_jimu_triangle_roof_apriltag_portable.py').write_text(
        'import sys\nfrom pathlib import Path\n'
        'pick = str(Path(__file__).parent.parent / "pick_jiaobang")\n'
        'if pick not in sys.path: sys.path.insert(0, pick)\n'
        'import audit_order_dependency\nSOURCE = audit_order_dependency.SOURCE\n'
    )
    monkeypatch.setattr(sys, 'path', list(sys.path))
    monkeypatch.setenv('LEROBOT_ROOT', '')
    monkeypatch.delitem(sys.modules, 'audit_order_dependency', raising=False)
    monkeypatch.delitem(sys.modules, '_rm75_working_entry', raising=False)
    try:
        assert import_working_entry(tmp_path, 'magnetic').SOURCE == 'pickplace'
    finally:
        sys.modules.pop('audit_order_dependency', None)
        sys.modules.pop('_rm75_working_entry', None)
