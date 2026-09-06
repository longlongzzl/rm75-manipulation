'use strict';
const $=id=>document.getElementById(id);
let task='pickplace', info=null, active=null, polling=null, busy=false, observed=null;
let design={schema:'jimu_builder_scene_v1',pieces:[
 {id:'floor',role:'floor',type:'square',locked:true,center:[0,.00325,0],u:[1,0,0],n:[0,1,0],v:[0,0,1]},
 {id:'right_wall',role:'right_wall',type:'square',locked:false,parentRole:'floor',center:[.037,.0435,0],u:[0,0,1],n:[1,0,0],v:[0,1,0]}]};
let selected='right_wall', pendingNonce=null;
function fail(e){$('error').textContent=e.message||String(e);}
async function api(path,data){
 const response=await fetch('/api/workcell/'+path,{method:data===undefined?'GET':'POST',headers:data===undefined?{}:{'Content-Type':'application/json','X-Workcell-Token':info.csrf},body:data===undefined?undefined:JSON.stringify(data)});
 const value=await response.json();if(!response.ok)throw new Error(value.error||response.statusText);return value;
}
function key(p){return p.role||p.id;}
function number(id){const value=Number($(id).value);if(!Number.isFinite(value))throw new Error(id+'必须为有限数字');return value;}
function spec(mode){let parameters;if(task==='pickplace')parameters={object_name:$('object-name').value};
 else if(task==='magnetic')parameters={design:JSON.parse(JSON.stringify(design))};
 else parameters={initial_pose:[number('push-x'),number('push-y'),number('push-yaw')*Math.PI/180],goal_pose:[number('goal-x'),number('goal-y'),number('goal-yaw')*Math.PI/180],speed_mps:number('push-speed'),max_steps:number('push-steps')};
 return {task,mode,parameters};}
function stateButtons(){for(const id of ['preview','simulate','real'])$(id).disabled=!info||busy||!!active||(id==='real'&&(!info?.allow_real||info?.real_latched));$('stop').disabled=!active;}
async function start(mode){if(active||busy)return;busy=true;stateButtons();$('error').textContent='';try{
 const request=spec(mode);let token;
 if(mode==='real'){
  if(!window.confirm('请确认相机标定、推头 / 夹爪、工作区与现场急停均已检查。本次将使真实机械臂运动。继续？'))return;
  token=(await api('arm',{spec:request,confirmation:'我确认现场安全并允许本次真机运行'})).arm_token;
 }
 const result=await api('jobs',{spec:request,arm_token:token});active=result.job_id;observed=null;
 $('run-status').textContent='运行中 · '+mode;$('result').textContent='任务 '+active;$('logs').textContent='等待工作进程…';
 clearInterval(polling);polling=setInterval(poll,350);await poll();
 }catch(e){fail(e);}finally{busy=false;stateButtons();}}
async function poll(){if(!active)return;try{const value=await api('jobs/'+active);$('logs').textContent=value.log||'等待日志…';
 const progress=value.progress,prompt=value.input_request;pendingNonce=prompt?.nonce||null;$('native-prompt').classList.toggle('hidden',!pendingNonce);if(pendingNonce)$('prompt-text').textContent=prompt.prompt;if(progress){$('result').textContent=JSON.stringify(progress,null,2);if(progress.kind==='observation'){observed=progress.observation.pose;drawPush();}}
 if(value.events)for(const e of value.events)if(e.kind==='observation'){observed=e.observation.pose;drawPush();}
 if(value.result){const r=value.result;$('result').textContent=JSON.stringify(r,null,2);
 $('run-status').textContent=r.status==='succeeded'?(r.mode==='sim'?'模型内达到目标（非真机验证）':'任务已通过观测验证'):r.status==='command_completed_unverified'?'命令结束 · 任务结果未验证':r.status==='cancelled'?'任务已停止':'任务失败';
 if(r.error)fail(r.error);if(r.final_observation){observed=r.final_observation.pose;drawPush();}
 clearInterval(polling);active=null;info=await api('info');stateButtons();}
 }catch(e){fail(e);}}
function fillPieces(){const list=$('piece-select');list.replaceChildren();for(const p of design.pieces){const option=document.createElement('option');option.value=key(p);option.textContent=(p.locked?'🔒 ':'')+key(p);list.appendChild(option);}if(!design.pieces.some(p=>key(p)===selected))selected=key(design.pieces[0]);list.value=selected;
 const p=design.pieces.find(p=>key(p)===selected);if(!p)return;
 $('piece-role').value=key(p);$('piece-type').value=p.type;['x','y','z'].forEach((a,i)=>$('piece-'+a).value=p.center[i]);
 const parent=$('piece-parent');parent.replaceChildren();const none=document.createElement('option');none.value='';none.textContent='未指定（保留原版规则）';parent.appendChild(none);
 for(const other of design.pieces)if(key(other)!==key(p)){const o=document.createElement('option');o.value=key(other);o.textContent=key(other);parent.appendChild(o);}parent.value=p.parentRole||p.parent_role||p.parentId||p.parent_id||p.parent||'';
 for(const id of ['apply-piece','rotate-piece','delete-piece'])$(id).disabled=!!p.locked;
 $('piece-count').textContent=design.pieces.filter(p=>!p.locked).length+' / 12 活动件';$('design-json').value=JSON.stringify(design,null,2);drawMagnetic();}
function applyPiece(){try{const p=design.pieces.find(p=>key(p)===selected);if(p.locked)return;
 const next=$('piece-role').value.trim();if(!/^[A-Za-z0-9_-]{1,100}$/.test(next)||design.pieces.some(o=>o!==p&&key(o)===next))throw new Error('角色名必须唯一且不包含路径');
 const old=key(p);p.role=next;p.id=p.id===old?next:p.id;p.type=$('piece-type').value;p.center=['x','y','z'].map(a=>number('piece-'+a));
 // Changing a parent requires invalidating its old relative transform; leaving it
 // would silently contradict the absolute exported pose.
 for(const field of ['parentRole','parent_role','parentId','parent_id','parent','parentRelativeTransform','parent_relative_transform','T_parent_piece'])delete p[field];
 if($('piece-parent').value)p.parentRole=$('piece-parent').value;
 if(old!==next)for(const child of design.pieces)for(const field of ['parentRole','parent_role','parentId','parent_id','parent'])if(child[field]===old)child[field]=next;
 selected=next;fillPieces();}catch(e){fail(e);}}
function rotatePiece(){try{const p=design.pieces.find(p=>key(p)===selected);if(p.locked)return;const a=number('piece-angle')*Math.PI/180,c=Math.cos(a),s=Math.sin(a);
 for(const name of ['u','n','v']){const [x,y,z]=p[name];p[name]=[c*x+s*z,y,-s*x+c*z];}
 for(const field of ['parentRelativeTransform','parent_relative_transform','T_parent_piece'])delete p[field];fillPieces();}catch(e){fail(e);}}
function loadDesign(value){if(value.schema!=='jimu_builder_scene_v1'||!Array.isArray(value.pieces)||!value.pieces.length)throw new Error('需要原始 jimu_builder_scene_v1 JSON');for(const p of value.pieces){for(const a of ['center','u','n','v'])if(!Array.isArray(p[a])||p[a].length!==3||!p[a].every(Number.isFinite))throw new Error('无效的积木坐标或轴');}
 design=value;selected=key(design.pieces[0]);fillPieces();}
function dims(type){return type==='half_square'?[.037,.0065,.074]:type==='triangle'?[.074,.0065,.135]:[.074,.0065,.074];}
function drawMagnetic(){const canvas=$('magnetic-canvas'),ctx=canvas.getContext('2d');ctx.clearRect(0,0,800,440);const proj=p=>[400+(p[0]-p[2]) *1500,365-p[1]*1500+(p[0]+p[2])*350];
 ctx.strokeStyle='#e0e8e4';ctx.lineWidth=1;for(let i=-5;i<=5;i++){let a=proj([i*.03,0,-.15]),b=proj([i*.03,0,.15]);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();a=proj([-.15,0,i*.03]);b=proj([.15,0,i*.03]);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();}
 for(const p of [...design.pieces].sort((a,b)=>a.center[1]-b.center[1])){const [w,th,h]=dims(p.type);const local=p.type==='triangle'?[[-w/2,-h/2],[w/2,-h/2],[0,h/2]]:[[-w/2,-h/2],[w/2,-h/2],[w/2,h/2],[-w/2,h/2]];
 const points=local.map(([x,z])=>proj(p.center.map((v,i)=>v+p.u[i]*x+p.v[i]*z)));ctx.beginPath();points.forEach((q,i)=>i?ctx.lineTo(...q):ctx.moveTo(...q));ctx.closePath();ctx.fillStyle=p.locked?'#c6d5d0':key(p)===selected?'#38887b':'#e6b777';ctx.fill();ctx.strokeStyle='#294c48';ctx.lineWidth=1.5;ctx.stroke();ctx.fillStyle='#263f42';ctx.font='12px system-ui';const at=proj(p.center);ctx.fillText(key(p),at[0]+8,at[1]-8);}
 ctx.fillStyle='#687f79';ctx.font='12px system-ui';ctx.fillText('Builder 坐标 · Y ↑ · 米 · 目标投影（非物理仿真）',18,24);}
function model(){return Object.assign({workspace:[.15,.65,-.3,.3],bar_width_m:.1,bar_height_m:.03,stem_width_m:.03,stem_height_m:.07,obstacles:[]},info?.pusht_model||{});}
function drawPush(){const ctx=$('pusht-canvas').getContext('2d'),m=model(),w=m.workspace;ctx.clearRect(0,0,800,440);const proj=p=>[45+(p[0]-w[0])/(w[1]-w[0])*710,405-(p[1]-w[2])/(w[3]-w[2])*365];ctx.strokeStyle='#d3dfd9';ctx.strokeRect(45,40,710,365);
 const draw=(pose,goal)=>{const [x,y,a]=pose,c=Math.cos(a),s=Math.sin(a),bw=m.bar_width_m,bh=m.bar_height_m,sw=m.stem_width_m,sh=m.stem_height_m,com=-(sw*sh)*(bh/2+sh/2)/(bw*bh+sw*sh);ctx.strokeStyle=goal?'#b78542':'#176b66';ctx.fillStyle='#a6c9bf';ctx.lineWidth=2;ctx.setLineDash(goal?[7,5]:[]);
 for(const [rx,ry,rw,rh] of [[0,-com,bw,bh],[0,-bh/2-sh/2-com,sw,sh]]){let corners=[[-rw/2,-rh/2],[rw/2,-rh/2],[rw/2,rh/2],[-rw/2,rh/2]].map(([u,v])=>[u+rx,v+ry]).map(([u,v])=>proj([x+c*u-s*v,y+s*u+c*v]));ctx.beginPath();corners.forEach((p,i)=>i?ctx.lineTo(...p):ctx.moveTo(...p));ctx.closePath();if(!goal)ctx.fill();ctx.stroke();}ctx.setLineDash([]);};
 try{draw([number('goal-x'),number('goal-y'),number('goal-yaw')*Math.PI/180],true);draw(observed||[number('push-x'),number('push-y'),number('push-yaw')*Math.PI/180],false);}catch(e){}
 ctx.fillStyle='#687f79';ctx.font='12px system-ui';ctx.fillText('base_link · X → / Y ↑ · 点击设置目标',45,23);ctx.fillText(observed?'实线：最新状态':'实线：手工初始状态（真机不使用）',45,429);}
for(const button of document.querySelectorAll('.tab'))button.addEventListener('click',()=>{task=button.dataset.task;for(const b of document.querySelectorAll('.tab'))b.classList.toggle('active',b===button);for(const t of ['pickplace','magnetic','pusht'])$(t+'-panel').classList.toggle('hidden',t!==task);$('task-label').textContent={pickplace:'PickPlace',magnetic:'磁吸积木',pusht:'PushT'}[task];});
$('preview').addEventListener('click',()=>start('preview'));$('simulate').addEventListener('click',()=>start('sim'));$('real').addEventListener('click',()=>start('real'));
$('stop').addEventListener('click',async()=>{try{const value=await api('jobs/'+active+'/stop',{});$('result').textContent=JSON.stringify(value,null,2);if(value.requires_physical_estop)fail('控制器未确认停止：请立即使用现场物理急停');}catch(e){fail(e);}});
$('piece-select').addEventListener('change',()=>{selected=$('piece-select').value;fillPieces();});$('apply-piece').addEventListener('click',applyPiece);$('rotate-piece').addEventListener('click',rotatePiece);
$('add-piece').addEventListener('click',()=>{if(design.pieces.filter(p=>!p.locked).length>=12)return fail('最多 12 个活动件');let n=1;while(design.pieces.some(p=>key(p)==='piece_'+n))n++;selected='piece_'+n;design.pieces.push({id:selected,role:selected,type:'square',locked:false,center:[-.037,.0435+.08*(n-1),0],u:[0,0,-1],n:[-1,0,0],v:[0,1,0]});fillPieces();});
$('delete-piece').addEventListener('click',()=>{const p=design.pieces.find(p=>key(p)===selected);if(p&&!p.locked){design.pieces=design.pieces.filter(p=>key(p)!==selected);fillPieces();}});
$('import-design').addEventListener('click',()=>$('design-file').click());$('design-file').addEventListener('change',async()=>{try{const f=$('design-file').files[0];if(f.size>2e6)throw new Error('文件超过 2 MB');loadDesign(JSON.parse(await f.text()));}catch(e){fail(e);}});
$('apply-json').addEventListener('click',()=>{try{loadDesign(JSON.parse($('design-json').value));}catch(e){fail(e);}});
$('export-design').addEventListener('click',()=>{const blob=new Blob([JSON.stringify(design,null,2)],{type:'application/json'}),url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download='jimu_builder_scene_v1.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);});
for(const id of ['push-x','push-y','push-yaw','goal-x','goal-y','goal-yaw'])$(id).addEventListener('input',()=>{observed=null;drawPush();});
$('pusht-canvas').addEventListener('click',event=>{if(active)return;const rect=event.target.getBoundingClientRect(),x=(event.clientX-rect.left)*800/rect.width,y=(event.clientY-rect.top)*440/rect.height,w=model().workspace;$('goal-x').value=(w[0]+(x-45)/710*(w[1]-w[0])).toFixed(3);$('goal-y').value=(w[2]+(405-y)/365*(w[3]-w[2])).toFixed(3);drawPush();});
for(const [id,value] of [['confirm-input',''],['retry-input','r'],['quit-input','q']]){
 $(id).addEventListener('click',async()=>{
  if(!active||!pendingNonce)return;
  const nonce=pendingNonce;
  for(const name of ['confirm-input','retry-input','quit-input'])$(name).disabled=true;
  try{await api('jobs/'+active+'/input',{nonce,value});pendingNonce=null;$('native-prompt').classList.add('hidden');}
  catch(e){fail(e);}finally{for(const name of ['confirm-input','retry-input','quit-input'])$(name).disabled=false;}
 });
}
stateButtons();
(async()=>{
 try{
  info=await api('info');
  $('connection').textContent=info.real_latched?'真机待现场复核':info.allow_real?'真机许可已启用 · 仍需单次确认':'默认不连接真机';
  $('migration-state').textContent=info.snapshot_installed?'已安装快照（不代表 GPU / 真机验收通过）':'尚未安装工作快照';
  for(const name of info.pickplace_objects){const option=document.createElement('option');option.value=name;option.textContent=name;$('object-name').appendChild(option);}
  fillPieces();drawPush();
  if(info.active_job){active=info.active_job;polling=setInterval(poll,350);await poll();}
 }catch(e){fail(e);$('connection').textContent='连接失败 · 不启用真机';}
 finally{stateButtons();}
})();
