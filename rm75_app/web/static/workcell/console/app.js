import {VERSION,tasks,statusNames,phaseNames,verifyNames,uid,canonical,finiteNumber,integer,safeStorage,
  outcome,sessionButtons,parseDesign,specFor,validateRestore,formatTime,terminal} from './state.js';
import {DesignView,drawPush,pushTransform} from './views.js';

const $=id=>document.getElementById(id);
const store=safeStorage((()=>{try{return window.localStorage;}catch{return null;}})()),names={shuazi:'刷子',bi:'笔',lvmukuai:'木块',carriot:'胡萝卜',tennis:'网球',gluestick:'胶棒',hongshupian:'薯片罐'};
const preferred=['shuazi','bi','lvmukuai','carriot','tennis','gluestick','hongshupian'];
const state={initialized:false,errorIsTransport:false,showJobScene:false,fullPushView:false,pushViewBounds:null,discarded:new Set(),boot:null,connected:false,task:'pickplace',busy:false,activeId:null,job:null,jobId:null,
  generationId:null,generationRunning:false,generation:null,pending:null,controlPending:null,
  design:null,proof:null,designText:'',origin:'',drafts:{},observed:null,trail:[],storageKey:null};
const designView=new DesignView($('design-canvas'));
let jobTimer=null,genTimer=null,heartTimer=null,saveTimer=null,pollingJob=false,pollingGeneration=false;
function node(tag,text='',className=''){const e=document.createElement(tag);e.textContent=text;if(className)e.className=className;return e;}
function text(id,value){$(id).textContent=value;}
function notice(id,message){text(id,message||'');$(id).hidden=!message;}
function error(exc){state.errorIsTransport=!!exc?.isTransport;notice('error',exc?.message||String(exc));}
function setConnected(ok){state.connected=ok;$('connection').className='connection '+(ok?'online':'offline');$('connection').replaceChildren(node('i'),document.createTextNode(ok?'服务器已连接':'连接中断'));if(ok){notice('connection-warning','');if(state.errorIsTransport){notice('error','');state.errorIsTransport=false;}}else notice('connection-warning','服务器暂时不可达。已经提交的任务可能仍在运行；不会自动重发请求。停止按钮可重试，但不替代现场急停。');buttons();}
async function api(path,body,timeoutMs=12000){
  const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),timeoutMs);
  try{
    const res=await fetch(path,{method:body===undefined?'GET':'POST',cache:'no-store',credentials:'same-origin',
      headers:body===undefined?{}:{'Content-Type':'application/json','X-Workcell-Token':state.boot?.info.csrf||''},
      ...(body===undefined?{}:{body:JSON.stringify(body)}),signal:controller.signal});
    const data=await res.json();if(!res.ok){const e=new Error(data.error||`服务器返回 ${res.status}`);e.status=res.status;throw e;}return data;
  }catch(e){if(e.name==='AbortError'){const err=new Error('请求超时。正在核对是否已提交，不会自动重新发起任务。');err.isTransport=true;throw err;}if(e instanceof TypeError&&!e.status)e.isTransport=true;throw e;}
  finally{clearTimeout(timer);}
}
const base='/api/workcell/console/';
function snapshotDrafts(){
  state.drafts.pickplace={objects:[...document.querySelectorAll('#object-grid input:checked')].map(e=>e.value),automatic:$('pp-auto').checked};
  state.drafts.magnetic={board:$('mg-board').value,budget:$('mg-budget').value,prompt:$('mg-prompt').value,
    designText:state.designText,rawText:$('design-json').value,proof:state.proof,origin:state.origin};
  state.drafts.pusht={initial:[$('pt-x').value,$('pt-y').value,$('pt-yaw').value],goal:[$('pt-gx').value,$('pt-gy').value,$('pt-gyaw').value],
    speed:$('pt-speed').value,length:$('pt-length').value,steps:$('pt-steps').value,seed:$('pt-seed').value,
    backend:$('pt-backend').value,geometry:$('pt-geometry').value,continuous:$('pt-continuous').checked};
}
function save(){
  if(!state.storageKey)return;snapshotDrafts();const ok=store.set(state.storageKey,{schema:1,task:state.task,drafts:state.drafts,
    jobId:state.jobId,generationId:state.generationId,pending:state.pending});
  text('draft-status',ok?'草稿已保存 · 刷新不重发任务':'浏览器存储不可用 · 刷新前请导出设计');
}
function changed(){clearTimeout(saveTimer);saveTimer=setTimeout(save,250);$('preflight-result').hidden=true;
  text('action-caption','草稿已更新，运行前将重新检查');buttons();}
function chooseTask(task){if(!tasks[task])return;state.task=task;for(const e of document.querySelectorAll('[data-task]')){
  const yes=e.dataset.task===task;e.classList.toggle('active',yes);if(yes)e.setAttribute('aria-current','page');else e.removeAttribute('aria-current');}
  for(const t of Object.keys(tasks))$('panel-'+t).hidden=t!==task;
  text('task-title',tasks[task].title);text('task-en',`${String(Object.keys(tasks).indexOf(task)+1).padStart(2,'0')} / ${tasks[task].en.toUpperCase()}`);
  text('breadcrumb-task',tasks[task].title);text('task-subtitle',tasks[task].subtitle);$('preflight-result').hidden=true;
  draw();save();buttons();
}
function currentSpec(mode){snapshotDrafts();if(state.task==='magnetic' && $('design-json').value!==state.designText)throw new Error('JSON 中有未应用的编辑。先“校验并应用手工编辑”，或重新载入当前设计。');return specFor(state.task,state.drafts[state.task],mode);}
function buttons(){
  const idle=state.connected&&!state.busy&&!state.activeId&&!state.pending;
  let ready=idle;
  if(state.task==='pickplace')ready&&=[...document.querySelectorAll('#object-grid input:checked')].length>0;
  if(state.task==='magnetic')ready&&=!!state.design&&!state.generationRunning&&$('design-json').value===state.designText;
  if(state.task==='pusht')ready&&=!!$('pt-backend').value;
  for(const id of ['preflight','preview','simulate'])$(id).disabled=!ready;
  const feature=state.boot?.features;
  $('generate').disabled=!(idle&&!state.generationRunning&&!state.boot?.generation?.provider_busy&&feature?.llm?.configured&&$('mg-board').value);
  $('load-template').disabled=!(idle&&!state.generationRunning&&$('mg-board').value);
  $('abandon-generation').hidden=!state.generationRunning;
  for(const id of ['mg-board','mg-template','mg-budget','mg-prompt'])$(id).disabled=state.generationRunning;
  $('export-design').disabled=!state.design;
  $('stop').disabled=!state.activeId; // Keep Stop reachable even if polling failed.
  for(const id of ['select-all','select-none','randomize','import-design','apply-design'])$(id).disabled=!idle||state.generationRunning;
  const b=sessionButtons(state.job,state.job?.session,state.controlPending||!state.connected);
  $('pause').disabled=!b.pause;$('resume').disabled=!b.resume;$('relocate').disabled=!b.relocate;
  const prompt=state.job?.input_request;
  for(const id of ['native-enter','native-retry','native-quit'])$(id).disabled=!(state.connected&&state.job?.managed_active&&prompt?.nonce&&!state.busy);
  $('pt-steps').disabled=$('pt-continuous').checked;
}
function populateOptions(select,options,value){select.replaceChildren();for(const o of options){const opt=node('option',o.title??o.id);opt.value=o.id;select.append(opt);}if([...select.options].some(o=>o.value===value))select.value=value;}
function objectCards(values,selected){
  const grid=$('object-grid');grid.replaceChildren();for(const [index,value]of values.entries()){
    const label=node('label','','object-card');const input=document.createElement('input');input.type='checkbox';input.value=value;input.checked=selected.includes(value);
    const caption=node('span');caption.append(node('strong',names[value]||value),node('small',value));label.append(input,node('span',String(index+1).padStart(2,'0'),'object-icon'),caption);grid.append(label);
    input.addEventListener('change',()=>{renderOrder();changed();});
  }renderOrder();
}
function renderOrder(){
  const namesSelected=[...document.querySelectorAll('#object-grid input:checked')].map(e=>e.value);
  for(const card of document.querySelectorAll('.object-card'))card.classList.toggle('selected',card.querySelector('input').checked);
  if($('pp-auto').checked)namesSelected.sort((a,b)=>(preferred.includes(a)?preferred.indexOf(a):99)-(preferred.includes(b)?preferred.indexOf(b):99)||a.localeCompare(b));
  const box=$('object-order');box.replaceChildren();namesSelected.forEach((n,i)=>{if(i)box.append(node('i','→'));box.append(node('span',names[n]||n));});if(!namesSelected.length)box.append(node('p','请选择至少一个物体。','micro'));
}
function updateTemplates(){const options=(state.boot?.features?.jimu_catalog||[]).filter(t=>t.board_id===$('mg-board').value).map(t=>({id:t.template_id,title:t.title}));populateOptions($('mg-template'),options);}
function applyLoadedDesign(design,proof=null,origin='手工设计'){
  // Never project raw unvalidated vectors into the canvas.
  const value=parseDesign(JSON.stringify(design));state.design=value;state.proof=proof;state.origin=origin;state.designText=JSON.stringify(value,null,2);
  $('design-json').value=state.designText;designView.update(value);
  const moving=value.pieces.filter(p=>!p.locked),fixed=value.pieces.length-moving.length;
  text('design-summary',`${origin} · 固定 ${fixed} 件 / 活动 ${moving.length} 件`);
  $('piece-list').replaceChildren(...moving.map(p=>node('span',p.role||p.id)));
  changed();save();
}
function draw(){
  if(state.task==='magnetic')designView.draw();
  if(state.task!=='pusht')return;
  try{
    snapshotDrafts();const d=state.drafts.pusht,pose=a=>[Number(a[0]),Number(a[1]),Number(a[2])*Math.PI/180];
    const active=state.job?.request?.task==='pusht'&&(state.job?.managed_active||state.showJobScene),p=active?state.job.request.parameters:null;
    const geometry=p?.geometry_id||d.geometry,model={...state.boot?.info.pusht_model,...state.boot?.features.geometry_models?.[geometry]};
    const initial=p?.initial_pose||pose(d.initial),target=p?.goal_pose||pose(d.goal),workspace=model.workspace||[.15,.65,-.3,.3];
    state.pushViewBounds=null;
    if(!state.fullPushView){
      const points=[initial,target,...(state.observed?[state.observed]:[])],pad=Math.max(model.bar_width_m||.1,(model.bar_height_m||.03)+(model.stem_height_m||.07))*.8;
      const b=[Math.max(workspace[0],Math.min(...points.map(p=>p[0]))-pad),Math.min(workspace[1],Math.max(...points.map(p=>p[0]))+pad),
               Math.max(workspace[2],Math.min(...points.map(p=>p[1]))-pad),Math.min(workspace[3],Math.max(...points.map(p=>p[1]))+pad)];
      if(b.every(Number.isFinite)&&b[1]>b[0]&&b[3]>b[2])state.pushViewBounds=b;
    }
    drawPush($('push-canvas'),{model,initial,target,observed:state.observed,trail:state.trail,bounds:state.pushViewBounds});
    text('push-caption',active?'显示所选任务的目标；运行中表单改动仅用于下次草稿。':'点击画布设置草稿目标；角度可在上方调整。');
  }catch{/* Incomplete numeric edit is validated before submission. */}
}
function renderChecks(container,checks){container.replaceChildren();for(const c of checks){const row=node('div','','config-row '+(c.status==='ready'?'':c.status));const description=node('div');description.append(node('strong',c.title));if(c.hint)description.append(node('p',c.hint));row.append(node('span',c.status==='ready'?'✓':c.status==='warning'?'!':'×','config-dot'),description);container.append(row);}}
function configView(){const b=state.boot;if(!b)return;renderChecks($('config-summary'),b.checks.filter(c=>c.key==='profile'||c.key.startsWith(state.task+'.')));
  renderChecks($('config-detail'),b.checks);text('config-profile',`配置：${b.profile_name} · 摘要 ${b.profile_id.slice(0,12)} · ${b.mode_policy.reason}`);
}
function setJob(row){
  state.job=row;state.jobId=row.job_id;state.showJobScene=row.request?.task==='pusht';if(row.managed_active)state.activeId=row.job_id;
  else if(state.activeId===row.job_id&&row.status!=='running')state.activeId=null;
  const o=outcome(row);$('job-empty').hidden=true;$('job-content').hidden=false;
  text('job-state',o.label);$('job-state').className='pill '+o.tone;
  text('job-task',tasks[row.request?.task]?.title||row.request?.task||'任务');text('job-mode',row.request?.mode==='sim'?'仿真':row.request?.mode==='real'?'历史真机记录':'预览');
  const phase=row.session?.phase||row.progress?.kind;const running=row.status==='running';
  text('job-phase',running?(phaseNames[phase]||'任务执行中'):o.label);text('job-verification',o.verification);text('job-id',row.job_id);
  const r=row.result||{},seq=r.sequence_summary,pushSteps=r.steps??row.session?.step;
  $('job-summary').replaceChildren();const metric=(value,label)=>{const box=node('div','','metric');box.append(node('b',value),node('span',label));$('job-summary').append(box);};
  if(seq){metric(`${seq.completed_count??0} / ${seq.total??seq.requested?.length??'?'}`,'前台原生完成 / 请求');metric(String(seq.pending?.length??'?'),'待完成（不计入成功）');}
  else if(row.request?.task==='pusht'){metric(String(pushSteps??'—'),'已执行推段');metric(String(row.session?.epoch??r.external_scene_epochs??'—'),'外部场景版本');}
  else if(row.request?.task==='magnetic'){const pieces=row.request.parameters.design?.pieces||[];metric(String(pieces.filter(p=>!p.locked).length),'本次活动件');metric(row.request.parameters.generation_proof?'一致性受检':'手工输入','设计来源');}
  notice('job-error',row.management_warning||r.error||(row.status==='verification_failed'?'动作指令已结束，但结果验证未通过。':''));
  const prompt=row.managed_active&&running?row.input_request:null;$('native-prompt').hidden=!prompt?.nonce;text('native-prompt-text',prompt?.prompt||'原程序等待输入。');
  const events=row.events||[],timeline=$('timeline');timeline.replaceChildren();
  const labels={task_started:'任务启动',task_finished:'任务结束',observation:'获得新观测',goal_confirmation:'目标稳定确认',push_command_finished:'本段推移结束',physics_replan:'物理重规划',push_plan_invalidated:'旧计划已失效',frozen_world_source_outcome:'原生物体结果',pusht_session_state:'会话状态',contact_audit:'路径 / 接触检查'};
  for(const e of events.filter(e=>Object.hasOwn(labels,e.kind)).slice(-6)){
    let label=labels[e.kind];if(e.kind==='pusht_session_state')label=phaseNames[e.phase]||label;
    const li=node('li',label);const at=e.at||e.timestamp||e.ts;if(typeof at==='number'&&at>1000000000)li.append(node('time',new Date(at*1000).toLocaleTimeString('zh-CN',{hour12:false})));timeline.append(li);
  }
  const obs=r.final_observation?.pose||row.session?.last_pose||[...events].reverse().find(e=>e.observation?.pose)?.observation.pose;
  if(row.request?.task==='pusht'&&obs&&obs.length===3){if(canonical(obs)!==canonical(state.observed)){state.observed=obs;state.trail.push(obs);state.trail=state.trail.slice(-100);}draw();}
  if(row.log!==undefined){text('logs',row.log||'尚无日志输出。');if($('follow-log').checked)$('logs').scrollTop=$('logs').scrollHeight;}
  text('raw-result',JSON.stringify({request:row.request,status:row.status,result:row.result,session:row.session},null,2));
  $('export-report').disabled=false;buttons();save();
}
async function pollJob(){
  clearTimeout(jobTimer);if(pollingJob||!state.jobId)return;pollingJob=true;
  try{
    const row=await api(base+'jobs/'+state.jobId);
    if(row.request.mode==='sim'&&row.request.task==='pusht'&&Object.hasOwn(row.request.parameters,'run_until_goal')){
      try{row.session=await api(`/api/workcell/iterate/sessions/${row.job_id}`);}catch{row.session=null;}
      if(state.controlPending&&row.session?.last_command>=state.controlPending.sequence){text('control-status',`指令 ${state.controlPending.sequence} 已确认 · ${phaseNames[row.session.phase]||row.session.phase}`);state.controlPending=null;}
    }
    setConnected(true);setJob(row);if(row.status==='running')jobTimer=setTimeout(pollJob,1000);else await history();
  }catch(e){setConnected(false);error(e);jobTimer=setTimeout(pollJob,2500);}
  finally{pollingJob=false;}
}
async function history(){
  try{const result=await api(base+'history');$('history').replaceChildren();for(const row of result.jobs){const b=node('button','','history-item');b.type='button';b.append(node('strong',`${tasks[row.task]?.title||row.task} · ${statusNames[row.status]||row.status}`),node('small',`${new Date(row.created_at*1000).toLocaleString('zh-CN',{hour12:false})} · ${row.mode}`));b.addEventListener('click',()=>{
      if(state.activeId&&state.activeId!==row.job_id){error('有任务正在运行，请先等待或停止当前任务。');return;}state.jobId=row.job_id;state.observed=null;state.trail=[];pollJob();});$('history').append(b);}if(!result.jobs.length)$('history').append(node('p','暂无任务记录','muted'));
  }catch(e){error(e);}
}
async function preflight(mode='sim'){
  const spec=currentSpec(mode),result=await api(base+'preflight',{spec});
  const box=$('preflight-result');box.hidden=false;box.replaceChildren(node('h3',result.valid?'配置与请求检查通过':'配置或请求需要修正'));
  const list=node('div');renderChecks(list,result.checks);box.append(list);
  for(const w of result.warnings)box.append(node('p',w,'micro'));
  if(result.busy)box.append(node('p','工作台当前已有任务，请先等待。','micro'));
  return result;
}
async function modal(message,summary,title='确认运行仿真',button='确认本次仿真'){
  const d=$('confirm-dialog');d.querySelector('h2').textContent=title;d.querySelector('[value=run]').textContent=button;
  text('confirm-text',message);text('confirm-spec',summary);d.returnValue='cancel';
  return new Promise(resolve=>{d.addEventListener('close',()=>resolve(d.returnValue==='run'),{once:true});d.showModal();});
}
async function reconcile(){
  const p=state.pending;if(!p)return;const route=p.kind==='task'?'submissions/':'generation-requests/';
  let row;try{row=await api(base+route+p.id);}catch(e){if(e.status!==404)throw e;
    notice('restore-note','此提交尚未在服务器找到记录。不会自动重发；请刷新任务历史后，再决定放弃这条本地记录。');offerForget();return;}
  if(row.status==='accepted'){
    state.pending=null;notice('restore-note','已找回服务器接收记录，没有重复提交。');
    if(p.kind==='task'){state.activeId=row.job_id;state.jobId=row.job_id;state.observed=null;state.trail=[];await pollJob();}
    else{state.generationId=row.generation_id;state.generationRunning=true;await pollGeneration();}
  }else if(row.status==='not_accepted'){
    state.pending=null;notice('restore-note',row.message||'此前生成请求未获接受，可检查配置后重新发起。');
  }else{notice('restore-note',row.message||'提交已登记但结果不确定。先检查历史和进程，不会自动重发。');offerForget();}
  save();buttons();
}
function offerForget(){const b=node('button','我已核对历史，清除此本地待确认记录','link-button');b.addEventListener('click',async()=>{if(await modal('仅清除浏览器记录，不会取消任何进程。确定已经检查任务历史？','','清除待确认记录','仅清除记录')){state.pending=null;notice('restore-note','已清除本地待确认记录，未操作工作进程。');save();buttons();}});$('restore-note').append(document.createElement('br'),b);}
async function launch(mode){
  if(state.activeId||state.pending||state.busy)throw new Error('先处理当前任务或待确认提交。');state.busy=true;buttons();notice('error','');
  try{const check=await preflight(mode);if(!check.valid||check.busy)throw new Error('尚不能提交：请查看预检查结果。');
    const spec=check.spec;
    if(mode==='sim'&&!await modal('将启动配置中的仿真工作进程。保留原算法与检查，不运行真机。',JSON.stringify(spec.task==='magnetic'?{task:spec.task,mode:spec.mode,board:spec.parameters.generation_proof?.board_id||'手工设计，请核对底板配置',movable:spec.parameters.design.pieces.filter(p=>!p.locked).length,roles:spec.parameters.generation_proof?.selected_roles||'手工设计',design_digest:spec.parameters.generation_proof?.design_digest}:spec,null,2)))return;
    const id=uid();state.pending={id,kind:'task'};save();
    try{await api(base+'jobs',{request_id:id,spec});}catch(e){error(e);}
    await reconcile();
  }finally{state.busy=false;buttons();}
}
async function pollGeneration(){
  clearTimeout(genTimer);if(pollingGeneration||!state.generationId)return;pollingGeneration=true;const ident=state.generationId;
  try{
    const result=await api(base+'generations/'+ident);if(ident!==state.generationId||state.discarded.has(ident))return;state.generation=result;state.generationRunning=result.status==='running';
    const box=$('generation-status');box.className='generation-status '+(state.generationRunning?'running':result.status==='succeeded'?'':'bad');
    if(state.generationRunning){const elapsed=result.created_at?(Date.now()/1000-result.created_at):0,budget=state.boot?.generation.estimated_budget_s||150;
      text('generation-status',`生成中 · 已等待 ${formatTime(elapsed)} · ${elapsed>budget?'超过预计时长，服务器仍在处理。可放弃结果，不会重新调用模型。':'最多两次模型调用。刷新后可恢复；前端不再按 135 秒提前报失败。'}`);
      genTimer=setTimeout(pollGeneration,result.poll_ms||1500);
    }else if(result.status==='succeeded'){
      if([...$('mg-board').options].some(o=>o.value===result.proof.board_id)){$('mg-board').value=result.proof.board_id;updateTemplates();}
      applyLoadedDesign(result.design,result.proof,'LLM 生成');
      text('generation-status',`${result.proof.title} · ${result.proof.movable_count} 块活动件。${result.proof.explanation} 生成不代表已通过原生仿真。`);
      state.generationId=null;save();
    }else{state.generationId=null;text('generation-status',result.error||result.message||'生成已结束，没有采用结果。');save();}
  }catch(e){state.generationRunning=true;error(e);text('generation-status','生成状态暂时不可达，任务可能仍在运行。正在重连，不会重新发起模型调用。');genTimer=setTimeout(pollGeneration,3000);}
  finally{pollingGeneration=false;buttons();}
}
async function startGeneration(){
  if(state.generationRunning||state.pending||state.activeId)throw new Error('先等待或处理当前任务。');
  const prompt=$('mg-prompt').value.trim();if(!prompt)throw new Error('请输入结构描述。');
  const request={board_id:$('mg-board').value,piece_budget:integer($('mg-budget').value,'活动件数量',1,12),prompt};
  if(prompt.length>1500)throw new Error('描述不能超过1500字符。');state.busy=true;buttons();notice('error','');
  try{const id=uid();state.pending={id,kind:'generation'};save();try{await api(base+'generate',{request_id:id,request});}catch(e){error(e);}await reconcile();}
  finally{state.busy=false;buttons();}
}
function download(value,filename){const blob=new Blob([JSON.stringify(value,null,2)+'\n'],{type:'application/json'}),url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download=filename;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
async function control(action){
  if(!state.activeId||!state.job?.managed_active||state.controlPending)throw new Error('等待当前会话指令确认。');
  const payload={action};if(action==='relocate')payload.pose=[finiteNumber($('move-x').value,'新 X',-3,3),finiteNumber($('move-y').value,'新 Y',-3,3),finiteNumber($('move-yaw').value,'新角度',-360,360)*Math.PI/180];
  state.busy=true;buttons();
  try{const r=await api(`/api/workcell/iterate/sessions/${state.activeId}/control`,payload);state.controlPending={sequence:r.sequence,action};text('control-status',`指令 ${r.sequence} 已排队，等待服务器确认；尚未允许修改 T。`);}
  catch(e){text('control-status','未得到明确回执，请刷新会话状态。不会自动重发控制指令。');throw e;}
  finally{state.busy=false;buttons();await pollJob();}
}
async function refresh(){if(!state.initialized)return initialize();const boot=await api(base+'bootstrap');state.boot=boot;setConnected(true);configView();
  if(state.storageKey&&state.storageKey!=='rm75.console.v1.'+boot.profile_id.slice(0,16)){notice('restore-note','服务器配置已变化。当前草稿仍可查看，但运行前会按新配置重新检查。');}
  if(boot.active_job&&!state.activeId){state.activeId=boot.active_job;state.jobId=boot.active_job;state.observed=null;state.trail=[];await pollJob();}
  return boot;
}
async function heartbeat(){clearTimeout(heartTimer);if(!state.initialized){await initialize();return;}try{await refresh();if(state.pending)await reconcile();}catch(e){setConnected(false);}heartTimer=setTimeout(heartbeat,state.connected?7000:3000);}
function on(id,event,fn){$(id).addEventListener(event,e=>Promise.resolve().then(()=>fn(e)).catch(error));}
for(const b of document.querySelectorAll('[data-task]'))b.addEventListener('click',()=>{chooseTask(b.dataset.task);configView();});
on('select-all','click',()=>{for(const e of document.querySelectorAll('#object-grid input'))e.checked=true;renderOrder();changed();});
on('select-none','click',()=>{for(const e of document.querySelectorAll('#object-grid input'))e.checked=false;renderOrder();changed();});
on('pp-auto','change',()=>{renderOrder();changed();});
for(const id of ['mg-prompt','mg-budget','design-json','pt-x','pt-y','pt-yaw','pt-gx','pt-gy','pt-gyaw','pt-speed','pt-steps','pt-length','pt-seed'])on(id,'input',()=>{if(id.startsWith('pt-')){if(!state.activeId){state.observed=null;state.trail=[];state.showJobScene=false;}draw();}changed();});
for(const id of ['pt-continuous','pt-backend','pt-geometry'])on(id,'change',()=>{if(!state.activeId){state.observed=null;state.trail=[];state.showJobScene=false;}draw();changed();});
on('mg-board','change',()=>{updateTemplates();changed();});
on('generate','click',startGeneration);
on('abandon-generation','click',async()=>{const ident=state.generationId;if(!ident)return;
  if(await modal('已发出的模型请求可能继续运行和计费。本次返回将不再载入或执行。','','放弃本次生成结果','不再采用')){
    state.discarded.add(ident);clearTimeout(genTimer);
    try{const result=await api(base+`generations/${ident}/abandon`,{});state.generationRunning=false;state.generationId=null;text('generation-status',result.message);save();buttons();}
    catch(e){state.discarded.delete(ident);genTimer=setTimeout(pollGeneration,1500);throw e;}
  }});
on('load-template','click',async()=>{const result=await api(base+'templates/'+encodeURIComponent($('mg-board').value));const t=result.templates.find(t=>t.id===$('mg-template').value)||result.templates[0];if(!t)throw new Error('该底板没有可用模板');applyLoadedDesign(t.design,t.proof,'原模板（未调用 LLM）');text('generation-status','已载入原模板，保留对应底板与运行配置；这不是本次 LLM 生成结果。');});
on('view-iso','click',()=>designView.view('iso'));on('view-top','click',()=>designView.view('top'));on('view-front','click',()=>designView.view('front'));on('fit-design','click',()=>designView.view('iso'));
on('apply-design','click',async()=>{const design=parseDesign($('design-json').value);if(state.proof&&canonical(design)!==canonical(state.design)&&!await modal('手工修改不再使用原生成证明。需要重新通过服务器校验，底板和结构稳定性并未因此得到验证。','','转为手工设计','确认转为手工设计'))return;
  const same=canonical(design)===canonical(state.design),proof=same?state.proof:null;
  const result=await api(base+'preflight',{spec:{task:'magnetic',mode:'preview',parameters:{design,...(proof?{generation_proof:proof}:{})}}});
  if(!result.valid)throw new Error('设计未通过校验');applyLoadedDesign(design,proof,same?state.origin:'手工设计');});
on('import-design','click',()=>$('design-file').click());
on('design-file','change',async e=>{const file=e.target.files?.[0];if(!file)return;if(file.size>1500000)throw new Error('文件超过1.5 MB');const value=JSON.parse(await file.text());const bundle=value.schema==='rm75_console_design_bundle_v1';const design=parseDesign(JSON.stringify(bundle?value.design:value)),proof=bundle?value.generation_proof:null;
  await api(base+'preflight',{spec:{task:'magnetic',mode:'preview',parameters:{design,...(proof?{generation_proof:proof}:{})}}});
  applyLoadedDesign(design,proof,proof?'导入设计（来源已校验）':'导入手工设计');e.target.value='';});
on('export-design','click',()=>download({schema:'rm75_console_design_bundle_v1',design:state.design,generation_proof:state.proof,origin:state.origin},'jimu_design_bundle.json'));
on('randomize','click',async()=>{const seed=integer($('pt-seed').value,'种子',0,4294967295),result=await api('/api/workcell/iterate/randomize',{seed,count:3,geometry_id:$('pt-geometry').value});const c=result.cases[seed%3];
  ['pt-x','pt-y','pt-yaw'].forEach((id,i)=>$(id).value=String(c.initial_pose[i]*(i===2?180/Math.PI:1)));
  ['pt-gx','pt-gy','pt-gyaw'].forEach((id,i)=>$(id).value=String(c.goal_pose[i]*(i===2?180/Math.PI:1)));
  text('random-case',`${c.case_id} · ${c.mode} · 仅几何边界有效`);$('pt-seed').value=String((seed+1)%4294967296);state.observed=null;state.trail=[];state.showJobScene=false;draw();changed();});
on('pt-fit','click',()=>{state.fullPushView=false;draw();});on('pt-world','click',()=>{state.fullPushView=true;draw();});
on('push-canvas','click',e=>{if(state.activeId)return;const c=$('push-canvas'),r=c.getBoundingClientRect(),coords=[(e.clientX-r.left)*c.width/r.width,(e.clientY-r.top)*c.height/r.height];const xy=pushTransform(c,state.pushViewBounds||state.boot?.info.pusht_model?.workspace).world(coords);$('pt-gx').value=xy[0].toFixed(4);$('pt-gy').value=xy[1].toFixed(4);state.observed=null;state.trail=[];state.showJobScene=false;draw();changed();});
on('preflight','click',()=>preflight('sim'));on('preview','click',()=>launch('preview'));on('simulate','click',()=>launch('sim'));
on('stop','click',async()=>{if(!state.activeId)return;const id=state.activeId;try{const r=await api(`/api/workcell/jobs/${id}/stop`,{});notice('restore-note',r.requires_physical_estop?'控制器停止未确认，需立即按现场安全流程处理。':'停止已请求，等待工作进程返回。不会将“已请求”当成“已停止”。');}finally{await pollJob();}});
for(const [id,action]of [['pause','pause'],['resume','resume'],['relocate','relocate']])on(id,'click',()=>control(action));
for(const [id,value]of [['native-enter',''],['native-retry','r'],['native-quit','q']])on(id,'click',async()=>{const nonce=state.job?.input_request?.nonce;if(!nonce||!state.activeId)return;state.busy=true;buttons();try{await api(`/api/workcell/jobs/${state.activeId}/input`,{nonce,value});}finally{state.busy=false;await pollJob();buttons();}});
on('export-report','click',async()=>{if(state.jobId)download(await api(base+`jobs/${state.jobId}/report`),`rm75_${state.jobId.slice(0,8)}_report.json`);});
on('copy-log','click',async()=>{try{await navigator.clipboard.writeText($('logs').textContent);text('copy-log','已复制');setTimeout(()=>text('copy-log','复制可见日志'),1500);}catch{throw new Error('浏览器禁止剪贴板访问，可直接选择日志文字复制。');}});
on('refresh','click',async()=>{await refresh();if(state.pending)await reconcile();if(state.jobId)await pollJob();if(state.generationId)await pollGeneration();await history();});
on('history-refresh','click',history);on('config-refresh','click',refresh);on('open-config','click',()=>{configView();$('config-dialog').showModal();});
window.addEventListener('online',()=>{refresh().then(()=>state.pending?reconcile():null).catch(error);});window.addEventListener('offline',()=>setConnected(false));
window.addEventListener('beforeunload',save);
async function initialize(){
  try{const boot=await api(base+'bootstrap');state.boot=boot;setConnected(true);state.storageKey='rm75.console.v1.'+boot.profile_id.slice(0,16);const saved=validateRestore(store.get(state.storageKey));
    const drafts=saved?.drafts||{};
    objectCards(boot.info.pickplace_objects||[],drafts.pickplace?.objects?.filter(n=>(boot.info.pickplace_objects||[]).includes(n))||[]);
    if(drafts.pickplace)$('pp-auto').checked=!!drafts.pickplace.automatic;
    const boards=[...new Map((boot.features.jimu_catalog||[]).map(t=>[t.board_id,{id:t.board_id,title:t.board_title}])).values()];
    populateOptions($('mg-board'),boards,drafts.magnetic?.board);updateTemplates();
    const m=drafts.magnetic;if(m){$('mg-prompt').value=String(m.prompt||'');$('mg-budget').value=String(m.budget||3);if(m.designText){try{applyLoadedDesign(parseDesign(m.designText),m.proof,m.origin||'恢复的草稿');if(m.rawText!==undefined)$('design-json').value=m.rawText;}catch(e){error(e);}}}
    populateOptions($('pt-backend'),[...new Set(boot.info.pusht_simulation_backends||['surrogate'])].map(id=>({id,title:{surrogate:'CPU 近似模型（非物理）',tool_only_physics:'工具接触物理仿真',full_arm_physics:'完整 RM75 物理仿真'}[id]||id})),drafts.pusht?.backend||'full_arm_physics');
    populateOptions($('pt-geometry'),[...new Set(boot.features.geometry_ids||['original'])].map(id=>({id,title:id==='original'?'原 T 几何':id})),drafts.pusht?.geometry||'original');
    const p=drafts.pusht;if(p){['pt-x','pt-y','pt-yaw'].forEach((id,i)=>$(id).value=String(p.initial?.[i]??0));['pt-gx','pt-gy','pt-gyaw'].forEach((id,i)=>$(id).value=String(p.goal?.[i]??0));
      for(const[id,k]of [['pt-speed','speed'],['pt-length','length'],['pt-steps','steps'],['pt-seed','seed']])if(p[k]!==undefined)$(id).value=String(p[k]);$('pt-continuous').checked=!!p.continuous;
    }else if(boot.info.pusht_physics_initial_pose){const a=boot.info.pusht_physics_initial_pose;['pt-x','pt-y','pt-yaw'].forEach((id,i)=>$(id).value=String(a[i]*(i===2?180/Math.PI:1)));}
    state.pending=saved?.pending||null;state.generationId=saved?.generationId||null;state.generationRunning=!!state.generationId;
    state.jobId=boot.active_job||saved?.jobId||null;state.activeId=boot.active_job||null;
    text('version',boot.version||VERSION);text('generation-status',boot.features.llm?.configured?'模型已配置。生成后仍需单独预览和运行仿真。':'模型或原模板库尚未配置。可在“本机配置”查看提示，或导入已有设计进行预览。');
    if(saved)notice('restore-note','已恢复此机器配置的浏览器草稿。没有自动发起任务、重试或模型请求。');
    state.initialized=true;chooseTask(saved?.task||'pickplace');renderOrder();configView();
    if(state.pending)await reconcile();if(state.generationId)await pollGeneration();if(state.jobId)await pollJob();await history();
  }catch(e){setConnected(false);error(e);}finally{buttons();heartTimer=setTimeout(heartbeat,5000);}
}
initialize();
