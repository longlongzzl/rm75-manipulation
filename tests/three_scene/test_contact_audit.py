from types import SimpleNamespace as NS
import pytest
from rm75_app.workcell.contact_audit import install_contact_audit, scene_evidence, StrictContactNotSupported


def fixture(*, strict=True, omit_table=False):
    rows, mutations = [], []
    planner = NS(_world=NS(objects=[NS(name="table", pose=[0, 0, 0]), NS(name="unrelated", pose=[1, 0, 0])]),
                 _disabled_collision_links=set(), _disabled_world_obstacles=set())
    direct = NS(_current_source_object_name=lambda args: args.object_name,
                _normalize_disabled_world_collision_links=lambda p, links: list(links))
    def toggle(p, links, *, enabled, label):
        mutations.append((list(links), enabled))
        if enabled:
            p._disabled_collision_links.difference_update(links)
        else:
            p._disabled_collision_links.update(links)
        return list(links)
    def final(p, demo, args, choice, lookup):
        direct._refresh_curobo_world(p, demo, args, include_table=not omit_table, label="final")
        links = direct._set_world_collision_for_links(p, ["gripper"], enabled=False, label="final_gripper_world_relaxed")
        try:
            return choice
        finally:
            direct._set_world_collision_for_links(p, links, enabled=True, label="final_gripper_world_relaxed")
    direct._set_world_collision_for_links = toggle
    direct._refresh_curobo_world = lambda *a, **kw: mutations.append("refresh")
    direct._apply_deferred_two_step_final_approach = final
    direct.run_targeted_place_episode_curobo_direct = lambda *a, **kw: True
    install_contact_audit(direct, rows.append, strict=strict)
    return direct, planner, rows, mutations


def run(direct, planner):
    return direct._apply_deferred_two_step_final_approach(planner, None, NS(object_name="right_wall", curobo_table_collision=True), {"label": "candidate_1"}, {})


def test_strict_blocks_before_broad_toggle():
    direct, planner, rows, changes = fixture()
    with pytest.raises(StrictContactNotSupported) as caught:
        run(direct, planner)
    assert caught.value.code == "strict_contact_not_supported"
    assert changes == ["refresh"]
    assert not planner._disabled_collision_links
    assert rows[-1]["relaxed_branch_required"] is True
    assert rows[-1]["changed_links"] == []
    assert rows[-1]["piece_id"] == "right_wall"
    assert rows[-1]["permitted_contact_support"] is None
    assert [o["name"] for o in rows[-1]["world_objects"]] == ["table", "unrelated"]


def test_strict_blocks_table_omission_before_world_mutation():
    direct, planner, rows, changes = fixture(omit_table=True)
    with pytest.raises(StrictContactNotSupported):
        run(direct, planner)
    assert changes == []
    assert rows[-1]["reason"] == "final_contact_table_omission"
    assert rows[-1]["relaxed_branch_required"] is None


def test_compatibility_records_disable_and_restore_without_success_claim():
    direct, planner, rows, changes = fixture(strict=False, omit_table=True)
    assert run(direct, planner) == {"label": "candidate_1"}
    assert changes == ["refresh", (["gripper"], False), (["gripper"], True)]
    assert not planner._disabled_collision_links
    applied = [r for r in rows if r["event"] == "native_contact_toggle_applied"]
    assert len(applied) == 2
    assert all(r["changed_links"] == ["gripper"] for r in applied)
    assert all(r["verified_task_success"] is None for r in rows)


def test_fingerprint_tracks_geometry_not_only_names():
    _, planner, _, _ = fixture()
    before = scene_evidence(planner)
    assert before == scene_evidence(planner)
    planner._world.objects[1].pose[0] = 2
    assert before["scene_fingerprint"] != scene_evidence(planner)["scene_fingerprint"]


def test_native_exception_fallback_cannot_swallow_strict_abort():
    direct, planner, _, _ = fixture()
    with pytest.raises(StrictContactNotSupported):
        try:
            run(direct, planner)
        except Exception:
            pytest.fail("strict rejection reached the native retry ladder")


def test_context_restored_after_abort():
    direct, planner, rows, _ = fixture()
    with pytest.raises(StrictContactNotSupported):
        run(direct, planner)
    direct._set_world_collision_for_links(planner, [], enabled=True, label="outside")
    assert rows[-1]["piece_id"] is None


def test_worker_preserves_strict_failure_and_nonzero_exit(tmp_path, monkeypatch, profile, design):
    from rm75_app.workcell import worker, legacy
    from rm75_app.workcell.io import atomic_json, read_json
    run_dir = tmp_path / 'run'
    run_dir.mkdir()
    atomic_json(run_dir / 'request.json', {'task': 'magnetic', 'mode': 'sim', 'parameters': {'design': design}})
    atomic_json(tmp_path / 'profile.json', profile)
    monkeypatch.setattr(legacy, 'run_working', lambda *a: {
        'command_success': False, 'task_success': None, 'status': 'strict_contact_not_supported'})
    code = worker.main(['--run-dir', str(run_dir), '--profile', str(tmp_path / 'profile.json'), '--app-root', str(tmp_path)])
    result = read_json(run_dir / 'result.json')
    assert code == 1
    assert result['status'] == 'failed'
    assert result['failure_code'] == 'strict_contact_not_supported'
    assert result['command_success'] is False
    assert result['task_success'] is None
