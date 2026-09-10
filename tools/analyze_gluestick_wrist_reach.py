#!/usr/bin/env python3
"""Explain the saved glue release failures with a URDF reach upper bound.

Pure CPU geometry, no IK, physics, native SDK, or task-target modifications.
"""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rm75_app.pusht.physics_replay import TcpFK
from rm75_app.planning.gripper_collision import _axis_angle_matrix
from rm75_app.workcell.transforms import quaternion_matrix


def wrist_and_tcp(fk, q):
    transform = np.eye(4)
    for origin, index, axis in fk.chain:
        transform = transform @ origin
        if index == 1:
            shoulder = transform[:3, 3].copy()
        if index == 5:
            wrist = transform[:3, 3].copy()
        if index is not None:
            rotation = np.eye(4)
            rotation[:3, :3] = _axis_angle_matrix(axis.copy(), q[index])
            transform = transform @ rotation
    return shoulder, wrist, transform


def analyze(job, output):
    evidence = json.loads((job / 'ik_gallery/batch_019/evidence.json').read_text())
    assert evidence['source'] == 'gluestick' and not evidence['prefetch']
    events = [json.loads(line).get('evidence', {}) for line in (job / 'events.jsonl').read_text().splitlines()]
    query = next(e for e in events if e.get('event') == 'failed_object_ik_search'
                 and e.get('source') == 'gluestick' and e.get('goal_count') == 26)
    assert query['state_unchanged'] and query['goals_changed'] is False
    urdf = ROOT / 'rm75_app/_vendor/working_snapshot/RM75_gripper/RM75-B/urdf/RM75-B.urdf'
    fk = TcpFK(urdf)
    maximum_length = sum(np.linalg.norm(origin[:3, 3]) for origin, index, _ in fk.chain
                         if index is not None and 2 <= index <= 5)
    offsets = []
    for q in np.random.default_rng(11).uniform(-2, 2, (100, 7)):
        shoulder, wrist, tcp = wrist_and_tcp(fk, q)
        assert np.linalg.norm(wrist - shoulder) <= maximum_length + 1e-9
        offsets.append(tcp[:3, :3].T @ (tcp[:3, 3] - wrist))
    offset = np.mean(offsets, axis=0)
    variation = float(np.max(np.linalg.norm(np.asarray(offsets) - offset, axis=1)))
    geometry_allowance = 1e-5
    assert variation < geometry_allowance
    tool_length = float(np.linalg.norm(offset))
    # v1 native geodesic_distance returns ||vector(q_goal*q_actual^-1)||,
    # hence its 0.05 limit is sin(theta/2), not theta in radians.
    orientation_limit = 2 * math.asin(.05)
    rows = []
    for index in range(13, 26):
        saved = evidence['rows'][index]
        goal = query['rows'][index]['goal']
        tcp = fk(saved['rendered_q'])
        assert np.linalg.norm(tcp[:3, 3] - saved['configuration']['fk']['position']) < 3e-5
        assert abs(np.linalg.norm(tcp[:3, 3] - goal['position']) - saved['position_error_m']) < 3e-5
        rotation = quaternion_matrix(goal['quaternion'])
        a = np.asarray(goal['position']) - shoulder
        length_a = float(np.linalg.norm(a))
        direction = rotation @ offset / tool_length
        alpha = math.acos(float(np.clip(np.dot(a, direction) / length_a, -1, 1)))
        exact_distance = float(np.linalg.norm(a - rotation @ offset))
        # Allow the entire orientation cone, then the entire position ball.
        # This deliberately gives more freedom than the actual joint chain.
        best_angle = max(0., alpha - orientation_limit)
        lower_bound = math.sqrt(length_a**2 + tool_length**2 -
                               2 * length_a * tool_length * math.cos(best_angle))
        lower_bound -= .005 + geometry_allowance
        if saved['native_success']:
            assert lower_bound <= maximum_length
        rows.append(dict(candidate=index - 12, grasp_label=saved['grasp_label'],
            goal=goal, exact_wrist_distance_m=exact_distance,
            minimum_wrist_distance_with_pose_tolerances_m=lower_bound,
            minimum_reach_deficit_m=lower_bound - maximum_length,
            native_release_success=saved['native_success'],
            native_grasp_accepted=saved['grasp_approach_accepted'],
            rendered_position_error_m=saved['position_error_m'],
            rendered_so3_error_rad=saved['so3_error_rad']))
    report = dict(source_job=job.name, source_batch=19, urdf=str(urdf),
        execute_real=False, hardware_connected=False, physics_steps=0, new_ik_queries=0,
        shoulder_position=shoulder.tolist(), shoulder_wrist_chain_upper_bound_m=float(maximum_length),
        wrist_to_tcp_offset=offset.tolist(), offset_variation_m=variation,
        geometry_allowance_m=geometry_allowance, position_tolerance_m=.005,
        native_rotation_tolerance=.05, equivalent_so3_tolerance_rad=orientation_limit, rows=rows,
        interpretation='Positive deficit proves this release is outside even the relaxed geometric necessary condition. Nonpositive deficit does not prove IK, collision, or path success.')
    output.mkdir(parents=True, exist_ok=True)
    (output / 'glue_reach_analysis.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    return report


def plot(report, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    font = FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    rows = report['rows']
    labels = [f'{r["candidate"]:02d}  ' + ('居中竖直' if r['candidate'] == 1 else
              r['grasp_label'].removeprefix('grasp_direct_top_bias_')) for r in rows]
    minimum = np.array([r['minimum_wrist_distance_with_pose_tolerances_m'] * 1000 for r in rows])
    exact = np.array([r['exact_wrist_distance_m'] * 1000 for r in rows])
    limit = report['shoulder_wrist_chain_upper_bound_m'] * 1000
    fig, ax = plt.subplots(figsize=(18, 9))
    fig.patch.set_facecolor('#f4f6f8')
    y = np.arange(13)
    colors = ['#c33d4e' if d > limit else '#6a8daf' for d in minimum]
    ax.barh(y, minimum, color=colors, height=.65)
    ax.scatter(exact, y, color='#24394f', marker='|', s=130, label='严格目标要求的肩到腕距离')
    ax.axvline(limit, color='#111a24', linewidth=2, linestyle='--', label='URDF 肩到腕链长上限：466 mm')
    ax.set_xlim(400, max(exact) + 45)
    ax.set_yticks(y, labels, fontproperties=font, fontsize=13)
    ax.invert_yaxis()
    for index, row in enumerate(rows):
        status = '放置 IK 通过 / 抓取未通过' if row['native_release_success'] else '放置 IK 未通过'
        ax.text(max(minimum[index], exact[index]) + 2, index, status, va='center',
                fontproperties=font, fontsize=11,
                color='#147548' if row['native_release_success'] else '#923343')
    ax.set_xlabel('肩到腕距离（mm）；横条已允许位置偏差 5 mm、姿态偏差约 5.732°', fontproperties=font, fontsize=15)
    ax.grid(axis='x', alpha=.22)
    ax.set_axisbelow(True)
    ax.legend(prop=font, loc='lower right')
    fig.suptitle('胶棒为什么“看着接近”却没有可用整链？', fontproperties=font, fontsize=23, y=.97)
    fig.text(.02, .905,
        '红条：用足现有位姿容差，腕中心仍超出链长。蓝条：只满足这一个必要条件，仍须检查关节范围、碰撞和其余阶段。',
        fontproperties=font, fontsize=14)
    fig.text(.02, .02,
        '候选编号对应之前的 13 行总图。居中竖直：精确目标需要 505.3 mm；用足容差仍至少需要 474.6 mm，比上限多 8.6 mm。',
        fontproperties=font, fontsize=14)
    fig.subplots_adjust(left=.29, right=.97, top=.87, bottom=.13)
    fig.savefig(output / 'glue_reach_explanation.png', dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--job', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = analyze(args.job.resolve(), args.output.resolve())
    plot(report, args.output.resolve())
    print(json.dumps(dict(geometrically_excluded=[r['candidate'] for r in report['rows']
                                                if r['minimum_reach_deficit_m'] > 0])))


if __name__ == '__main__':
    main()
