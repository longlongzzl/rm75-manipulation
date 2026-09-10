#!/usr/bin/env python3
"""Recover completed IK evidence batches, including from an interrupted SIM."""
import argparse
import html
import json
import math
import os
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from tools.export_ik_candidate_gallery import aligned_relations


def export(job,output):
    job=Path(job).resolve();output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    manifest_path=job/'ik_gallery/manifest.json'
    complete=manifest_path.exists() and json.loads(manifest_path.read_text()).get('complete') is True
    batches=[];sources={name:[] for name in ('gluestick','hongshupian')}
    for path in sorted((job/'ik_gallery').glob('batch_*/evidence.json')):
        data=json.loads(path.read_text())
        if data['source'] not in sources or data['prefetch']:continue
        if not all(data.get(k) for k in ('robot_state_restored','planner_state_restored',
                                        'actor_registry_restored','returned_rows_unchanged')):
            raise ValueError('Unrestored or unfinished evidence batch: '+str(path))
        if len(data['rows'])!=data['goal_count']:raise ValueError('Incomplete evidence row count')
        batches.append(str(path.relative_to(job)))
        for row in data['rows']:
            image=job/'ik_gallery'/row['image']
            image.resolve().relative_to(job)
            if not image.is_file():raise ValueError('Missing candidate image')
            sources[data['source']].append(dict(row,batch=data['batch'],prefetch=False,
                source=data['source'],image=os.path.relpath(image,output)))
    retries=[]
    for line in (job/'events.jsonl').read_text().splitlines():
        try:event=json.loads(line).get('evidence',{})
        except ValueError:continue
        if event.get('event')=='failed_object_ik_search':retries.append(event)
    new_q={tuple(r['configuration']['joints']) for event in retries for r in event['rows']
           if r['newly_accepted'] and r['configuration'] is not None}
    summary=dict(source_job=job.name,source_job_completed=(job/'result.json').exists(),
        source_gallery_complete=complete,recovered_batch_evidence=batches,
        execute_real=False,hardware_connected=False,full_chain_success=False,sources={})
    style='body{font:16px system-ui;max-width:1280px;margin:24px auto;padding:0 18px;background:#f5f6f8;color:#18222e}a{color:#175b9e}article{background:white;padding:16px;margin:18px 0;border:1px solid #ddd}img{width:100%;height:auto}table{border-collapse:collapse;width:100%;background:white}th,td{border:1px solid #ccc;padding:8px;text-align:left}pre{white-space:pre-wrap;overflow-wrap:anywhere}.pass{color:#147434}.fail{color:#ab2634}nav{display:flex;gap:20px}.note{background:#fff4da;padding:14px}.row{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}@media(max-width:700px){.row{display:block}}'
    def page(title,body):return '<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>'+html.escape(title)+'</title><style>'+style+'</style><h1>'+html.escape(title)+'</h1>'+body+'</html>'
    caveat='<p class="note">原任务被重启中断；这里只恢复逐批已经写完、状态恢复检查通过的证据，不将整项任务标为完成。以下是静态 IK，未连接真机，端点解不能代替完整路径通过。</p>' if not complete else '<p class="note">静态 IK 端点图；完整路径必须另行验证。</p>'
    overview=[caveat,'<nav><a href="gluestick.html">胶棒 IK</a><a href="hongshupian.html">红薯片 IK</a><a href="solutions.json">关节角与完整证据 JSON</a></nav>']
    all_solutions={}
    for name,rows in sources.items():
        title={'gluestick':'胶棒','hongshupian':'红薯片'}[name]
        relations=aligned_relations(rows)
        valid_relations=[rel for rel in relations if all(r is not None and r['native_success'] and r['configuration']['valid'] for r in rel)]
        new_rows=[r for r in rows if tuple(r['rendered_q']) in new_q and r['native_success']]
        releases=[r for r in rows if r['phase']=='release' and r['native_success'] and r['configuration']['valid']]
        def key(r):return (r['batch'],r['index'])
        selected=[];seen=set()
        # Show newly found place endpoints, then one complete endpoint relation.
        for row in [*sorted(new_rows,key=lambda r:r['phase']!='release'),
                    *(valid_relations[0] if valid_relations else []),*releases[:2]]:
            if key(row) not in seen:selected.append(row);seen.add(key(row))
        stats=dict(foreground_rows=len(rows),failed_rows=sum(not r['native_success'] for r in rows),
            valid_release_rows=len(releases),all_four_endpoints_valid_relations=len(valid_relations),
            newly_found_rendered_rows=[{'batch':r['batch'],'index':r['index'],'phase':r['phase']} for r in new_rows])
        summary['sources'][name]=stats
        bodies=[f'<a href="index.html">返回总览</a>',caveat,
            f'<p>前台候选 {len(rows)} 行；IK 失败 {stats["failed_rows"]} 行；有效 place 端点 {len(releases)} 行；四阶段端点均通过 {len(valid_relations)} 条关系（包含重试）。</p>']
        if name=='gluestick':
            explanation='新增了放置端点解；对应抓取关系仍须通过预抓取、抓取与完整路径，不能仅凭放置端点宣布胶棒任务成功。'
        else:
            explanation='已存在多条四阶段端点均通过的关系。失败发生在带载抬升：原冻结目标处的负载包络与底座球重叠约 28.975 mm，后续短直线动作仍被拒绝。'
        bodies.append('<p>'+explanation+'</p><h2>定位到的 IK 解</h2>')
        selected_json=[]
        for row in selected:
            q=row['rendered_q'];degrees=[math.degrees(v) for v in q]
            record={**row,'joints_rad':q,'joints_deg':degrees,'joint_names':[f'joint_{i}' for i in range(1,8)]}
            selected_json.append(record)
            bodies.append(f'<article><h3>B{row["batch"]} / #{row["index"]} / {row["phase"]}</h3><p>{html.escape(row["grasp_label"])} / {html.escape(row["place_label"] or "")}</p><p>位置误差 {row["position_error_m"]*1000:.3f} mm；姿态误差 {math.degrees(row["so3_error_rad"]):.3f}°；原世界碰撞检查通过。</p><pre>joint_1 → joint_7（rad）\n{json.dumps([round(v,7) for v in q])}\njoint_1 → joint_7（deg）\n{json.dumps([round(v,4) for v in degrees])}</pre><a href="{row["image"]}"><img loading="lazy" src="{row["image"]}" alt="{title} {row["phase"]} IK 三视图"></a></article>')
        bodies.append('<h2>四阶段均通过的原关系</h2><table><tr><th>抓取 / 放置关系</th><th>预抓取 → 抓取 → preplace → place</th></tr>')
        for rel in valid_relations:
            links=' → '.join(f'<a href="{r["image"]}">B{r["batch"]} #{r["index"]}</a>' for r in rel)
            bodies.append(f'<tr><td>{html.escape(rel[-1]["grasp_label"])}<br>{html.escape(rel[-1]["place_label"])}</td><td>{links}</td></tr>')
        bodies.append('</table><h2>全部前台候选索引</h2><p>失败行保留。点击“原图”查看三视图；关节向量与碰撞对见 JSON。</p><table><tr><th>批次 / 行</th><th>阶段 / 关系</th><th>IK</th><th>位置 / 姿态误差</th><th>图</th></tr>')
        for r in rows:
            bodies.append(f'<tr><td>B{r["batch"]} #{r["index"]}</td><td>{r["phase"]}<br>{html.escape(r["grasp_label"])}<br>{html.escape(r["place_label"] or "")}</td><td class="{"pass" if r["native_success"] else "fail"}">{"通过" if r["native_success"] else "失败"}</td><td>{r["position_error_m"]*1000:.3f} mm / {math.degrees(r["so3_error_rad"]):.3f}°</td><td><a href="{r["image"]}">原图</a></td></tr>')
        bodies.append('</table>');(output/f'{name}.html').write_text(page(title+' IK 定位', ''.join(bodies)),encoding='utf-8')
        all_solutions[name]=dict(selected=selected_json,all_foreground_rows=rows,
            complete_endpoint_relations=[[dict(batch=r['batch'],index=r['index'],image=r['image']) for r in rel] for rel in valid_relations])
        overview.append(f'<article><h2><a href="{name}.html">{title}</a></h2><p>{explanation}</p><p>有效 place 端点 {len(releases)} 行；四阶段均通过 {len(valid_relations)} 条关系。</p></article>')
    (output/'index.html').write_text(page('失败物体 IK：胶棒与红薯片', ''.join(overview)),encoding='utf-8')
    (output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    (output/'solutions.json').write_text(json.dumps(dict(summary=summary,objects=all_solutions),ensure_ascii=False,indent=2)+'\n')
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--job',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    args=p.parse_args();print(json.dumps(export(args.job,args.output),ensure_ascii=False))
