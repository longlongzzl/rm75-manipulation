#!/usr/bin/env python3
"""Two data-driven still sheets from ONE recorded failure per object.

No IK solve, physics step, SDK import, or hardware command. Glue uses original
captured images. Red renders each saved joint row using the original URDF.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rm75_app.workcell.transforms import quaternion_matrix, rotation_error

FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
BG = '#f0f3f7'
INK = '#172536'
GREEN = '#167547'
RED = '#b62b3b'


def font(size):
    return ImageFont.truetype(FONT, size)


def text(draw, xy, value, size=25, fill=INK):
    draw.text(xy, str(value), font=font(size), fill=fill)


def batch(job, number):
    path = job / f'ik_gallery/batch_{number:03d}/evidence.json'
    data = json.loads(path.read_text())
    assert not data['prefetch']
    assert all(data[key] for key in ('robot_state_restored', 'planner_state_restored',
                                    'actor_registry_restored', 'returned_rows_unchanged'))
    assert len(data['rows']) == data['goal_count']
    return data


def short_label(label):
    value = label.removeprefix('grasp_direct_top_bias_')
    if value == 'center_vertical':
        return '居中 / 竖直'
    import re
    match = re.fullmatch(r'pos(\d+)_(vertical|tilt(\d+)_away)_axis_\d+mm', value)
    if match:
        return f'偏移 {match[1]} mm / ' + (f'外倾 {match[3]}°' if match[3] else '竖直')
    return value


def glue_sheet(job, out):
    batches = [batch(job, number) for number in (17, 18, 19)]
    assert all(b['source'] == 'gluestick' for b in batches)
    phases = ('pregrasp', 'grasp', 'hover', 'release')
    mappings = {phase: {r['grasp_label']: r for b in batches for r in b['rows']
                        if r['phase'] == phase} for phase in phases}
    labels = [r['grasp_label'] for r in batches[0]['rows']]
    assert len(labels) == 13 and all(set(mappings[p]) == set(labels) for p in phases)
    # One row = one grasp relation, four columns = its queried chain endpoints.
    width, label_w, cell_w, cell_h, top = 2460, 300, 532, 330, 292
    canvas = Image.new('RGB', (width, top + 13 * cell_h + 70), BG)
    draw = ImageDraw.Draw(canvas)
    text(draw, (28, 18), '胶棒：同一次失败链路筛选的全部 13 个候选', 42)
    text(draw, (30, 83), '按行看：同一抓取关系 → 预抓取 → 抓取 → 放置上方 → 放置端点。四步全通过的候选：0 / 13。', 28)
    text(draw, (30, 130), '紫色夹爪＝要求的目标；黑色机械臂＝返回的关节姿态。红色＝IK 未通过；绿色＝该端点通过。', 27)
    text(draw, (30, 173), '两条放置解虽通过，但它们的抓取未通过；这次实际停在链路筛选，尚未执行抓取。', 28, RED)
    text(draw, (30, 215), f'原任务 {job.name[:12]} · 第一轮前台 B17–B19 · 图中物体保持原位置 · 静态证据', 23)
    titles = ('① 预抓取', '② 抓取', '③ 放置上方', '④ 放置端点')
    for i, title in enumerate(titles):
        count = sum(r['native_success'] for r in mappings[phases[i]].values())
        text(draw, (label_w + i * cell_w + 10, 254), f'{title}   {count}/13 通过', 25)
    manifest = []
    for row_index, label in enumerate(labels):
        y = top + row_index * cell_h
        draw.rectangle((18, y, width - 18, y + cell_h - 8), fill='white')
        text(draw, (34, y + 26), f'候选 {row_index + 1:02d}', 31)
        value = short_label(label).split(' / ')
        for line, part in enumerate(value):
            text(draw, (34, y + 82 + line * 38), part, 27)
        valid = all(mappings[p][label]['native_success'] for p in phases)
        text(draw, (34, y + 216), '整链通过' if valid else '整链未通过', 25, GREEN if valid else RED)
        for column, phase in enumerate(phases):
            r = mappings[phase][label]
            x = label_w + column * cell_w
            color = GREEN if r['native_success'] else RED
            draw.rectangle((x + 3, y + 4, x + cell_w - 9, y + cell_h - 12), outline=color, width=3)
            text(draw, (x + 13, y + 9), 'IK 通过' if r['native_success'] else 'IK 未通过', 25, color)
            # Preserve both the actual robot overview and the target/tool detail.
            source = job / 'ik_gallery' / r['image']
            with Image.open(source) as original:
                for k, crop in enumerate(((0, 108, 512, 620), (512, 108, 1024, 620))):
                    canvas.paste(original.crop(crop).resize((252, 252)), (x + 10 + k * 252, y + 44))
            text(draw, (x + 13, y + 294), f'{r["position_error_m"]*1000:.2f} mm / {math.degrees(r["so3_error_rad"]):.2f}°', 19)
            manifest.append(dict(candidate=row_index + 1, phase=phase, source_image=str(source),
                                 native_success=r['native_success'], q=r['rendered_q']))
    text(draw, (28, canvas.height - 55), '每格左半是机械臂全景、右半是末端近景。误差对应图中实际渲染的关节姿态；端点通过不能代替整条路径通过。', 24)
    canvas.save(out / 'gluestick_one_chain.jpg', quality=94)
    return manifest


def red_sheet(job, out):
    import sapien
    import yaml
    from mani_skill.utils.sapien_utils import look_at
    events = [json.loads(line) for line in (job / 'events.jsonl').read_text().splitlines()]
    matches = [(i, e['evidence']) for i, e in enumerate(events)
               if e.get('evidence', {}).get('event') == 'pickplace_failed_lift_ik_diagnostic'
               and e['evidence'].get('source') == 'hongshupian']
    assert len(matches) == 1
    event_index, evidence = matches[0]
    assert evidence['state_unchanged'] and evidence['diagnostic_complete'] and evidence['attached']
    rows = evidence['returned_rows']
    assert len(rows) == 64 and all(r['finite'] for r in rows)
    retries = [e['evidence'] for e in events
               if e.get('evidence', {}).get('event') == 'failed_object_ik_search'
               and e['evidence'].get('source') == 'hongshupian'
               and e['evidence'].get('goal_count') == 1
               and e['evidence']['rows'][0]['goal'] == evidence['goal_pose']
               and e['evidence']['rows'][0]['explicit_start_q'] == evidence['start']['joints']]
    assert len(retries) == 1 and retries[0]['state_unchanged']
    retry_rows = retries[0]['rows'][0]['raw_rows']
    assert len(retry_rows) == 128 and all(r['finite'] for r in retry_rows)
    all_rows = [dict(r, group='原始', q=r['configuration']['joints'], expected_fk=r['configuration']['fk'])
                for r in rows]
    all_rows += [dict(r, group='重试', index=r['seed_index'], q=r['joints'], expected_fk=None)
                 for r in retry_rows]
    prior = next(e['evidence'] for e in reversed(events[:event_index])
                 if e.get('evidence', {}).get('event') == 'planner_phase_state'
                 and e['evidence'].get('source') == 'hongshupian')
    cfg_path = ROOT / 'rm75_app/_vendor/working_snapshot/pick_jiaobang/curobo_rm75_config/rm75.yml'
    cfg = yaml.safe_load(cfg_path.read_text())['robot_cfg']['kinematics']
    urdf = Path(cfg['urdf_path'])
    engine = sapien.Engine()
    renderer = sapien.SapienRenderer()
    engine.set_renderer(renderer)
    scene = engine.create_scene(sapien.SceneConfig())
    scene.set_ambient_light([.8, .8, .8])
    scene.add_directional_light([-.4, .2, -1], [.9, .9, .9], shadow=False)
    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    robot = loader.load(str(urdf))
    assert robot is not None
    names = [j.name for j in robot.get_active_joints()]
    indices = [names.index(f'joint_{i}') for i in range(1, 8)]
    tcp = next(link for link in robot.get_links() if link.name == 'gripper_tcp')
    locked = np.zeros(len(names), dtype=np.float32)
    for name, value in prior['gripper_locks'].items():
        locked[names.index(name)] = value
    # Render world obstacles as the saved planner envelopes, with virtual walls
    # omitted from view only. No planner or collision state is constructed.
    for obj in evidence['world_objects']:
        if obj['name'] in evidence['disabled_objects'] or 'virtual_' in obj['name']:
            continue
        builder = scene.create_actor_builder()
        material = renderer.create_material()
        material.set_base_color([.46, .54, .60, .32])
        if obj['dims'] is not None:
            builder.add_box_visual(half_size=np.asarray(obj['dims']) / 2, material=material)
        elif obj['file_path']:
            builder.add_visual_from_file(obj['file_path'], scale=obj['scale'], material=material)
        else:
            raise ValueError('Unsupported saved world geometry')
        actor = builder.build_static(name=obj['name'])
        actor.set_pose(sapien.Pose(obj['pose'][:3], obj['pose'][3:]))
    # Purple cross = the SAME requested TCP goal for all returned seeds.
    goal = evidence['goal_pose']
    builder = scene.create_actor_builder()
    material = renderer.create_material()
    material.set_base_color([.95, .10, .80, 1])
    for axis in range(3):
        half = [.0016, .0016, .0016]
        half[axis] = .020
        builder.add_box_visual(half_size=half, material=material)
    actor = builder.build_static(name='saved_tcp_goal')
    actor.set_pose(sapien.Pose(goal['position'], goal['quaternion']))
    camera = scene.add_camera('recorded_ik', 480, 380, .76, .005, 6)
    aim = np.array([.15, -.01, .28])
    camera.set_pose(look_at(aim + np.array([.80, -.90, .55]), aim).sp)
    detail = scene.add_camera('recorded_ik_detail', 480, 380, .85, .005, 6)
    detail_aim = np.array([.10, -.025, .15])
    detail.set_pose(look_at(detail_aim + np.array([.32, -.37, .20]), detail_aim).sp)
    def render(q, expected=None, cam=camera):
        values = locked.copy()
        values[indices] = q
        robot.set_qpos(values)
        actual = tcp.pose
        if expected is not None:
            delta = float(np.linalg.norm(np.asarray(actual.p) - expected['position']))
            angle = rotation_error(quaternion_matrix(actual.q), quaternion_matrix(expected['quaternion']))
            if delta > 3e-5 or angle > .001:
                raise ValueError(f'Render FK differs from recorded FK: {delta} m / {angle} rad')
        scene.update_render()
        cam.take_picture()
        image = Image.fromarray((np.clip(cam.get_picture('Color')[:, :, :3], 0, 1) * 255).astype(np.uint8))
        return image, np.asarray(actual.p).copy(), np.asarray(actual.q).copy()
    w, cell_w, cell_h, top = 3840, 320, 318, 752
    canvas = Image.new('RGB', (w, top + 16 * cell_h + 100), BG)
    draw = ImageDraw.Draw(canvas)
    text(draw, (30, 16), '红薯片：同一条已选链路，抬升失败的原始 64 ＋ 重试 128 个返回姿态', 42)
    text(draw, (32, 81), '选中：沿轴偏移 6 mm / 竖直抓取 / 放置偏航 +45° → 抓取后上抬 80 mm → IK 被拒绝。', 32)
    text(draw, (32, 130), '下面每格是同一个抬升目标的一个原生种子返回值；不是 192 个不同抓取目标，也不是执行轨迹。', 30)
    text(draw, (32, 181), '紫色十字＝要求的 TCP 位置；红框＝原生 IK 未通过。近目标但碰撞的姿态也全部保留。', 30)
    start = evidence['start']
    start_img, _, _ = render(start['joints'], start['fk'])
    canvas.paste(start_img, (35, 280))
    text(draw, (40, 239), '抬升前：原有效起点', 28, GREEN)
    nearest = min(rows, key=lambda r: r['position_error_m'])
    img, _, _ = render(nearest['configuration']['joints'], nearest['configuration']['fk'], detail)
    canvas.paste(img, (555, 280))
    text(draw, (560, 239), f'近景：种子 #{nearest["index"]:02d}，末端已到目标附近', 26)
    near_count = sum(r['position_error_m'] < .005 for r in rows)
    lines = [f'原生 IK 通过：原始 0 / 64；同目标、同起点重试 0 / 128',
             f'原始结果中，{near_count} / 64 的位置误差 < 5 mm，配置碰撞检查仍全部失败',
             '日志主要报：attached_object（带载包络）↔ base_link（底座）',
             '原目标位置处的包络重叠约 28.975 mm；不是“机械臂完全够不到”。',
             '图中画原 URDF 机械臂及保存的环境包络。',
             '附着负载球的坐标没有落盘，因此不凭空补画；碰撞对按原日志标注。',
             '先读“原始 #00–63”，再读“重试 #00–127”。两轮返回完整保留。']
    for line, value in enumerate(lines):
        text(draw, (1090, 267 + line * 55), value, 30, RED if line in (0, 2, 3) else INK)
    text(draw, (32, 694), f'原任务 {job.name[:12]} · 同一个 lift 查询 · 0 次新求解 / 0 次物理步进 · 192 格：同目标、同起点的两次查询', 27)
    manifest = []
    for index, row in enumerate(all_rows):
        config = row.get('configuration')
        img, position, quat = render(row['q'], row['expected_fk'])
        error = float(np.linalg.norm(position - goal['position']))
        if abs(error - row['position_error_m']) > 3e-5:
            raise ValueError('Rendered position error differs from the native saved error')
        x, y = (index % 12) * cell_w, top + (index // 12) * cell_h
        color = GREEN if row['native_success'] else RED
        draw.rectangle((x + 5, y + 4, x + cell_w - 6, y + cell_h - 7), fill='white', outline=color, width=3)
        text(draw, (x + 16, y + 8), f'{row["group"]} #{row["index"]:02d}   未通过', 24, color)
        canvas.paste(img.resize((304, 241)), (x + 8, y + 43))
        pos_error = float(np.linalg.norm(position - goal['position']))
        angle = rotation_error(quaternion_matrix(quat), quaternion_matrix(goal['quaternion']))
        text(draw, (x + 14, y + 289), f'{pos_error*1000:.3f} mm / {math.degrees(angle):.2f}°', 21)
        manifest.append(dict(group=row['group'], seed_index=row['index'], q=row['q'], native_success=row['native_success'],
            render_fk_position=position.tolist(), position_error_m=pos_error, so3_error_rad=angle,
            collision_pairs=config['self_collision'].get('link_pairs', []) if config is not None else None))
        if index % 12 == 11:
            print(f'Rendered red seeds {index + 1}/192', flush=True)
    text(draw, (30, canvas.height - 75), '静态渲染仅用于检查 IK 返回姿态。原64行FK及全部192行位置误差已与原日志核对；未连接真机。虚拟墙未显示，负载碰撞球未虚构。', 28)
    canvas.save(out / 'hongshupian_failed_lift_all.jpg', quality=94)
    (out / 'hongshupian_lift_evidence.json').write_text(json.dumps(evidence, ensure_ascii=False, indent=2) + '\n')
    (out / 'hongshupian_lift_retry_evidence.json').write_text(json.dumps(retries[0], ensure_ascii=False, indent=2) + '\n')
    return dict(rows=manifest, urdf=str(urdf), urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),
                event_index=event_index, same_goal=goal, near_goal_rows=near_count)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--job', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    job, out = args.job.resolve(), args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    metadata = dict(source_job=job.name, execute_real=False, hardware_connected=False,
                    new_ik_queries=0, physics_steps=0)
    metadata['gluestick'] = glue_sheet(job, out)
    metadata['hongshupian'] = red_sheet(job, out)
    (out / 'manifest.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(output=str(out),glue_rows=13,red_seeds=192)))


if __name__ == '__main__':
    main()
