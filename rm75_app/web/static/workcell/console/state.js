/* Pure client contracts, also exercised by Node tests. No DOM or network. */
export const VERSION = '2026.09.11-console.1';
export const terminal = new Set(['succeeded','failed','cancelled','verification_failed','command_completed_unverified','interrupted','abandoned']);
export const tasks = {
  pickplace: {title:'桌面整理', en:'PickPlace', subtitle:'选择物体，核对顺序，再预览或运行原生仿真。'},
  magnetic: {title:'磁吸搭建', en:'Jimu', subtitle:'沿用原 3×3 与弧形底板，生成、预览和执行同一份设计。'},
  pusht: {title:'闭环推移', en:'PushT', subtitle:'规划一段，推动一段，再观察。恢复时不沿用旧计划。'},
};
export const statusNames = {running:'运行中', succeeded:'已验证完成', failed:'任务失败',cancelled:'已停止',
  verification_failed:'结果未通过验证',command_completed_unverified:'指令结束 · 未验证任务成功',
  untracked:'进程状态待确认',interrupted:'生成已中断',abandoned:'已放弃本次结果',idle:'待命'};
export const phaseNames = {observing:'重新观察',planning:'规划中',executing:'执行仿真',paused:'已在动作边界暂停',
  waiting_for_stability:'等待稳定观测',waiting_for_scene_change:'等待场景变化',succeeded:'目标确认完成',
  stopped_or_failed:'已停止或异常',starting:'启动中',finished:'会话结束'};
export const verifyNames = {preview_only:'预览 / 不执行动作',surrogate_pose:'CPU 近似模型 / 非物理',
  physics_pose:'物理仿真状态',live_pose:'实时观测',frozen_world_sim_only:'原生冻结场景仿真',
  frozen_world_validation_failed:'冻结场景校验失败',legacy_command_only:'仅原生命令完成'};
export function uid() { const bytes=new Uint8Array(16);crypto.getRandomValues(bytes);return Array.from(bytes,x=>x.toString(16).padStart(2,'0')).join(''); }
export function canonical(value) {
  if(Array.isArray(value))return '['+value.map(canonical).join(',')+']';
  if(value && typeof value==='object')return '{'+Object.keys(value).sort().map(k=>JSON.stringify(k)+':'+canonical(value[k])).join(',')+'}';
  return JSON.stringify(value);
}
export function finiteNumber(raw,name,low,high) {
  if(raw===null || raw===undefined || String(raw).trim()==='')throw new Error(`${name}不能为空`);
  const n=Number(raw); if(!Number.isFinite(n)||n<low||n>high)throw new Error(`${name}须在 ${low} 到 ${high} 之间`);
  return n;
}
export function integer(raw,name,low,high){ const n=finiteNumber(raw,name,low,high);if(!Number.isInteger(n))throw new Error(`${name}须是整数`);return n; }
export function safeStorage(storage) {
  return {get(key){try{return JSON.parse(storage.getItem(key)||'null');}catch{return null;}},
    set(key,value){try{storage.setItem(key,JSON.stringify(value));return true;}catch{return false;}},
    remove(key){try{storage.removeItem(key);}catch{}}};
}
export function outcome(job) {
  const r=job?.result || {},status=job?.status||'idle';
  const verified=status==='succeeded'&&r.task_success===true;
  let label=statusNames[status]||status;
  if(r.verification==='preview_only'&&!['failed','verification_failed','cancelled'].includes(status))label='预览完成（未执行动作）';
  if(status==='succeeded'&&!verified)label='状态不一致 · 未确认成功';
  if(verified&&r.verification==='surrogate_pose')label='近似模型内到位（非物理）';
  if(verified&&r.verification==='physics_pose')label='仿真目标已验证';
  return {label,tone:verified?(r.verification==='surrogate_pose'?'neutral':'good'):status==='failed'||status==='verification_failed'?'bad':
    status==='untracked'||status==='command_completed_unverified'?'warn':'neutral',
    verification:verifyNames[r.verification]||r.verification||'尚无结果验证',verified};
}
export function sessionButtons(job,session,pending) {
  const active=job?.managed_active===true && job?.status==='running';
  const interactive=active && job.request?.mode==='sim' && job.request?.task==='pusht' && Object.hasOwn(job.request?.parameters||{},'run_until_goal');
  return {stop:active,pause:interactive&&!pending&&session?.phase!=='paused',
    resume:interactive&&!pending&&['paused','waiting_for_scene_change'].includes(session?.phase),
    relocate:interactive&&!pending&&session?.phase==='paused'&&session?.safe_to_adjust===true&&
      ['full_arm_physics','tool_only_physics'].includes(job.request.parameters.simulation_backend)};
}
export function parseDesign(text){
  if(typeof text!=='string'||new TextEncoder().encode(text).length>1500000)throw new Error('设计文件超过 1.5 MB');
  const d=JSON.parse(text);if(!d||d.schema!=='jimu_builder_scene_v1'||!Array.isArray(d.pieces)||d.pieces.length>64)throw new Error('需要原 jimu_builder_scene_v1 设计');
  let moving=0;const seen=new Set();
  for(const p of d.pieces){
    if(typeof p?.locked!=='undefined'&&typeof p.locked!=='boolean')throw new Error('locked 必须是布尔值');
    const key=p?.role||p?.id;if(typeof key!=='string'||!key||seen.has(key))throw new Error('积木角色不能为空或重复');seen.add(key);if(!p.locked)moving++;
    if(!p||!['square','half_square','triangle'].includes(p.type))throw new Error('包含未知积木类型');
    for(const f of ['center','u','n','v'])if(!Array.isArray(p[f])||p[f].length!==3||p[f].some(x=>typeof x!=='number'||!Number.isFinite(x)))throw new Error(`积木 ${p.role||p.id||'?'} 的 ${f} 无效`);
    if(p.center.some(x=>Math.abs(x)>3))throw new Error('设计坐标应以米为单位');
    const axes=[p.u,p.n,p.v];for(let i=0;i<3;i++)for(let j=0;j<3;j++){const dot=axes[i].reduce((s,x,k)=>s+x*axes[j][k],0);if(Math.abs(dot-(i===j?1:0))>.02)throw new Error('设计的轴向量无效');}
    const [u,n,v]=axes,det=(u[1]*n[2]-u[2]*n[1])*v[0]+(u[2]*n[0]-u[0]*n[2])*v[1]+(u[0]*n[1]-u[1]*n[0])*v[2];if(det<.98)throw new Error('设计含反射坐标系');
  }
  if(moving<1||moving>12)throw new Error('活动件须为1到12块');
  return d; // Server performs full axes, parents, inventory and provenance checks.
}
export function specFor(task,draft,mode){
  if(!['preview','sim'].includes(mode))throw new Error('此控制台不提交真机请求');
  if(task==='pickplace'){
    const names=draft.objects||[];if(!names.length)throw new Error('至少选择一个物体');
    return {task,mode,parameters:names.length===1?{object_name:names[0]}:{object_names:[...names],automatic_order:!!draft.automatic}};
  }
  if(task==='magnetic'){
    const design=parseDesign(draft.designText||'');return {task,mode,parameters:{design,...(draft.proof?{generation_proof:draft.proof}:{})}};
  }
  if(task==='pusht'){
    const pose=(a)=>[finiteNumber(a[0],'X',-3,3),finiteNumber(a[1],'Y',-3,3),finiteNumber(a[2],'角度',-360,360)*Math.PI/180];
    return {task,mode,parameters:{initial_pose:pose(draft.initial),goal_pose:pose(draft.goal),
      speed_mps:finiteNumber(draft.speed,'速度',.001,.05),max_steps:integer(draft.steps,'步数上限',1,500),
      maximum_push_length_m:finiteNumber(draft.length,'最长推距',.012,.08),
      simulation_backend:draft.backend,geometry_id:draft.geometry,run_until_goal:!!draft.continuous}};
  }
  throw new Error('未知任务');
}
export function validateRestore(value){
  if(!value||value.schema!==1||!tasks[value.task])return null;
  // Restore only drafts. No automatic execution, arm token, provider key or stale CSRF.
  return {task:value.task,drafts:value.drafts||{},jobId:/^[0-9a-f]{32}$/.test(value.jobId||'')?value.jobId:null,
    generationId:/^[0-9a-f]{32}$/.test(value.generationId||'')?value.generationId:null,
    pending:value.pending&&/^[0-9a-f]{32}$/.test(value.pending.id||'')&&['task','generation'].includes(value.pending.kind)?value.pending:null};
}
export function formatTime(seconds){seconds=Math.max(0,Math.floor(seconds||0));return `${Math.floor(seconds/60)}:${String(seconds%60).padStart(2,'0')}`;}
