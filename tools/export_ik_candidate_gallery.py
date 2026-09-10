#!/usr/bin/env python3
"""Package ALL captured original IK rows as browsable images and contact sheets."""
import argparse
from collections import defaultdict
import html
import json
from pathlib import Path
import shutil
import sys

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from rm75_app.workcell.io import atomic_json


def aligned_relations(rows):
    """Match named original relations, never borrow a success from another grasp."""
    output=[];pre={};grasp={}
    for number in dict.fromkeys(r['batch'] for r in rows if not r['prefetch']):
        batch=[r for r in rows if r['batch']==number and not r['prefetch']]
        if batch[0]['phase']=='pregrasp':pre={r['grasp_label']:r for r in batch};grasp={};continue
        if batch[0]['phase']=='grasp':grasp={r['grasp_label']:r for r in batch};continue
        hover=[r for r in batch if r['phase']=='hover'];release=[r for r in batch if r['phase']=='release']
        if len(hover)!=len(release):raise ValueError('Unpaired render rows')
        for h,r in zip(hover,release):
            if (h['grasp_label'],h['place_label'])!=(r['grasp_label'],r['place_label']):
                raise ValueError('Original hover/release relation mismatch')
            output.append((pre.get(h['grasp_label']),grasp.get(h['grasp_label']),h,r))
    return output


def export(source,output):
    from PIL import Image,ImageDraw,ImageFont
    source=Path(source).resolve();output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    manifest=json.loads((source/'manifest.json').read_text())
    if not manifest['complete']:raise ValueError('Cannot publish incomplete candidate gallery')
    groups=defaultdict(list);all_rows=[]
    for batch in manifest['batches']:
        number=batch['batch'];record=json.loads((source/f'batch_{number:03d}/evidence.json').read_text())
        if not all(record.get(k) for k in ('robot_state_restored','planner_state_restored','actor_registry_restored','returned_rows_unchanged')):
            raise ValueError('Missing exact restoration evidence')
        for row in record['rows']:
            image=source/row['image'];image.relative_to(source)
            with Image.open(image) as check:check.verify()
            destination=output/row['image'];destination.parent.mkdir(exist_ok=True)
            shutil.copy2(image,destination)
            row=dict(row,batch=number,prefetch=record['prefetch'],source=record['source'])
            groups[record['source']].append(row);all_rows.append(row)
    style='body{font:16px system-ui;margin:24px;background:#f3f4f6;color:#18222e}a{color:#125eac}article{background:white;padding:12px;margin:18px 0}img{max-width:100%;height:auto}table{border-collapse:collapse;width:100%;background:white}td,th{padding:8px;border:1px solid #d1d5db;text-align:left}code{overflow-wrap:anywhere}.pass{color:#167337}.fail{color:#a92222}nav{display:flex;gap:20px;flex-wrap:wrap}.small{font-size:14px}'
    legend='<p>紫色夹爪：原模型摆到 IK 目标位姿；黑白机械臂：求解器返回的关节姿态。每张图含全景、近景、对侧观察。</p><p>这些是静态候选，不是执行视频。场景物体保持当时位置，未伪造抓持；IK 通过不代表完整路径或带载碰撞通过。预取与前台分开标明，原生 32 内部种子不冒充 32 个姿态候选。</p>'
    def page(title,body):return '<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>'+html.escape(title)+'</title><style>'+style+'</style><h1>'+html.escape(title)+'</h1>'+body+'</html>'
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
    index=['<p>原 current-table 同一场景顺序：shuazi → bi → lvmukuai → carriot → gluestick → hongshupian → tennis。</p>',legend,'<nav>']
    summaries={}
    for name,rows in groups.items():
        index.append(f'<a href="{name}.html">{name}（{len(rows)} 张）</a>')
        parts=['<a href="index.html">返回所有物体</a>',legend]
        matrices=[]
        aligned=aligned_relations(rows)
        if aligned:parts.append('<h2>前台四阶段逐关系对照</h2><p>从左到右：预抓取 → 抓取 → preplace → place。每行严格对应同一原生关系；同名关系在不同批次重试仍分别展示。绿／红只表示 IK 是否通过，不代替直线进给与完整路径验收。</p>')
        for offset in range(0,len(aligned),4):
            chunk=aligned[offset:offset+4];canvas=Image.new('RGB',(1440,50+400*len(chunk)),(245,246,248));draw=ImageDraw.Draw(canvas)
            for k,phase in enumerate(('PREGRASP','GRASP','PREPLACE / HOVER','PLACE / RELEASE')):
                draw.text((k*360+10,10),phase,font=font,fill=(30,40,50))
            for j,relation in enumerate(chunk):
                for k,row in enumerate(relation):
                    x=k*360;y=50+j*400
                    if row is None:
                        draw.text((x+8,y+15),'No matching queried grasp row',font=font,fill=(80,80,80));continue
                    with Image.open(output/row['image']) as original:
                        canvas.paste(original.crop((512,108,1024,620)).resize((352,352)),(x+4,y+45))
                    color=(18,114,54) if row['native_success'] else (173,30,34)
                    draw.text((x+8,y+3),f'B{row["batch"]} #{row["index"]} '+('IK PASS' if row['native_success'] else 'IK FAIL'),font=font,fill=color)
                    draw.text((x+8,y+24),f'{row["position_error_m"]*1000:.1f} mm / {row["so3_error_rad"]*180/3.14159265:.1f} deg',font=font,fill=(30,40,50))
            filename=f'{name}_four_graphs_{offset//4+1:02d}.jpg';canvas.save(output/filename,quality=90);matrices.append(filename)
            parts.append(f'<a href="{filename}"><img loading="lazy" src="{filename}" alt="{name} 四阶段逐关系对照 {offset//4+1}"></a>')
        parts.append('<h2>按阶段索引</h2><table><tr><th>批次 / 执行上下文</th><th>阶段</th><th>通过 / 全部</th><th>逐张图</th></tr>')
        for key in dict.fromkeys((r['batch'],r['phase']) for r in rows):
            subset=[r for r in rows if (r['batch'],r['phase'])==key]
            links=' '.join(f'<a class="{"pass" if r["native_success"] else "fail"}" href="#b{r["batch"]}r{r["index"]}">{r["index"]}: {"通过" if r["native_success"] else "失败"}</a>' for r in subset)
            parts.append(f'<tr><td>{key[0]} / {"后台预取" if subset[0]["prefetch"] else "前台"}</td><td>{key[1]}</td><td>{sum(r["native_success"] for r in subset)} / {len(subset)}</td><td>{links}</td></tr>')
        parts.append('</table><h2>全部候选联系表（点击原图）</h2>')
        sheets=[]
        # Every captured row appears, no cherry-picking or hidden failed rows.
        for offset in range(0,len(rows),16):
            chunk=rows[offset:offset+16];canvas=Image.new('RGB',(1440,400*((len(chunk)+3)//4)),(245,246,248));draw=ImageDraw.Draw(canvas)
            for j,row in enumerate(chunk):
                x=(j%4)*360;y=(j//4)*400
                with Image.open(output/row['image']) as original:
                    crop=original.crop((512,108,1024,620)).resize((352,352));canvas.paste(crop,(x+4,y+45))
                color=(18,114,54) if row['native_success'] else (173,30,34)
                draw.text((x+8,y+3),f'B{row["batch"]} #{row["index"]} {row["phase"]} '+('PASS' if row['native_success'] else 'FAIL'),font=font,fill=color)
                draw.text((x+8,y+24),f'{row["position_error_m"]*1000:.1f} mm / {row["so3_error_rad"]*180/3.14159265:.1f} deg',font=font,fill=(30,40,50))
            sheet=f'{name}_sheet_{offset//16+1:02d}.jpg';canvas.save(output/sheet,quality=90);sheets.append(sheet)
            parts.append(f'<a href="{sheet}"><img loading="lazy" src="{sheet}" alt="{name} 全候选联系表 {offset//16+1}"></a>')
        for r in rows:
            label=html.escape(str(r['grasp_label'])+' / '+str(r['place_label'] or ''))
            parts.append(f'<article id="b{r["batch"]}r{r["index"]}"><h3>批次 {r["batch"]} / {r["phase"]} #{r["index"]} / {"IK 通过" if r["native_success"] else "IK 失败"} / {"后台预取" if r["prefetch"] else "前台"}</h3><code>{label}</code><p><a href="{r["image"]}">打开原尺寸三视图</a></p><a href="{r["image"]}"><img loading="lazy" src="{r["image"]}" alt="{label}"></a></article>')
        (output/f'{name}.html').write_text(page(name+' IK 全候选', ''.join(parts)),encoding='utf-8')
        summaries[name]=dict(images=len(rows),ik_pass=sum(r['native_success'] for r in rows),sheets=sheets,
            four_graph_sheets=matrices,foreground_relations=len(aligned))
    index.append('</nav>');(output/'index.html').write_text(page('PickPlace 原生 IK 候选图册', ''.join(index)),encoding='utf-8')
    summary=dict(source_job=source.parent.name,images=len(all_rows),batches=len(manifest['batches']),sources=summaries,
        all_original_observed_rows_included=True,execute_real=False,hardware_connected=False,
        physical_or_full_chain_success=False)
    atomic_json(output/'summary.json',summary)
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    args=p.parse_args();print(json.dumps(export(args.source,args.output),ensure_ascii=False))
