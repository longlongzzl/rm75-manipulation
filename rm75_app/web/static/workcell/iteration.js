/* Three workflows, mounted beside the preserved original editor. No model HTML. */
(() => {
  'use strict';
  const $ = (id) => document.getElementById(id);
  const el = (tag, text, attrs = {}) => {
    const node = document.createElement(tag);
    if (text !== null) node.textContent = text;
    for (const [key, value] of Object.entries(attrs)) {
      if (key === 'class') node.className = value;
      else node.setAttribute(key, value);
    }
    return node;
  };
  const field = (parent, title, input) => { const label = el('label', title); label.append(input); parent.append(label); return input; };
  const button = (parent, text, fn) => {
    const node = el('button', text, {type: 'button'}); parent.append(node);
    node.addEventListener('click', () => Promise.resolve().then(fn).catch(showError)); return node;
  };
  const section = (parent, title, note) => {
    const node = el('section', null, {class: 'iteration-section'});
    node.append(el('h3', title), el('p', note)); $(parent).append(node); return node;
  };
  let info, features, currentJob = null, currentRequest = null, proof = null, generatedDesign = null, timer = null;
  let pendingNonce = null, lastCase = null;
  const jobPanel = section('pickplace-panel', '本轮工作流状态', '该状态栏随任务切换共用；停止与“等待本次推完后暂停”是两个操作。');
  // Move shared job controls outside the tab panels, without altering legacy listeners.
  document.querySelector('aside').prepend(jobPanel);
  const status = el('p', '待命'), details = el('pre', ''), error = el('p', '', {role: 'alert', class: 'iteration-error'});
  jobPanel.append(status, error, details);
  function showError(exc) { error.textContent = exc && exc.message ? exc.message : String(exc); }
  async function api(path, body) {
    const response = await fetch(path, body === undefined ? {cache: 'no-store'} : {
      method: 'POST', headers: {'Content-Type': 'application/json', 'X-Workcell-Token': info.csrf}, body: JSON.stringify(body)
    });
    const value = await response.json();
    if (!response.ok) throw new Error(value.error || `HTTP ${response.status}`);
    return value;
  }
  const stop = button(jobPanel, '停止本轮任务', async () => {
    if (!currentJob) throw new Error('没有本轮任务');
    await api(`/api/workcell/jobs/${currentJob}/stop`, {}); status.textContent = '已请求停止，等待工作进程确认';
  }); stop.disabled = true;
  const pause = button(jobPanel, '本次推完并撤离后暂停', async () => control('pause')); pause.disabled = true;
  const resume = button(jobPanel, '重新定位并继续', async () => control('resume')); resume.disabled = true;
  const prompt = el('p', ''); jobPanel.append(prompt);
  const confirm = button(jobPanel, '回应原程序 Enter 确认', async () => {
    if (!pendingNonce) throw new Error('没有待确认的原生提示');
    await api(`/api/workcell/jobs/${currentJob}/input`, {nonce: pendingNonce, value: ''});
  }); confirm.disabled = true;
  async function control(action, pose) {
    if (!currentJob) throw new Error('没有运行中的 PushT');
    await api(`/api/workcell/iterate/sessions/${currentJob}/control`, {action, ...(pose ? {pose} : {})});
    status.textContent = '指令已排队。只有状态显示“paused”后，才允许在仿真中修改 T。';
    pause.disabled = resume.disabled = relocate.disabled = true;
  }
  async function poll() {
    if (!currentJob) return;
    try {
      const job = await api(`/api/workcell/jobs/${currentJob}`);
      const terminal = job.status !== 'running';
      status.textContent = `${job.request.task} / ${job.status}`;
      const result = job.result || job.progress || {};
      details.textContent = JSON.stringify(result, null, 2);
      if ($('logs') && job.log) $('logs').textContent = job.log;
      pendingNonce = !terminal && job.input_request && job.input_request.nonce;
      prompt.textContent = pendingNonce ? String(job.input_request.prompt || '原程序等待 Enter 确认') : '';
      confirm.disabled = !pendingNonce; stop.disabled = terminal;
      const interactive = !terminal && job.request.task === 'pusht' && Object.hasOwn(job.request.parameters, 'run_until_goal');
      if (interactive) {
        const session = await api(`/api/workcell/iterate/sessions/${currentJob}`);
        status.textContent += ` / ${session.phase} / 已推 ${session.step || 0} 次`;
        if (session.last_pose) {
          lastPose = session.last_pose; drawIterationT();
          status.textContent += ` / T=${lastPose.map(x => x.toFixed(4)).join(', ')}`;
        }
        pause.disabled = session.phase === 'paused';
        resume.disabled = !['paused','waiting_for_scene_change'].includes(session.phase);
        relocate.disabled = !(session.phase === 'paused' && session.safe_to_adjust &&
          ['tool_only_physics','full_arm_physics'].includes(job.request.parameters.simulation_backend));
      } else { pause.disabled = resume.disabled = relocate.disabled = true; }
      if (terminal && result.final_observation) { lastPose=result.final_observation.pose; drawIterationT(); }
      if (!terminal) timer = setTimeout(poll, 600);
      else {
        // Five of seven completed is shown as partial evidence, never a green all-success badge.
        if (result.sequence_summary) details.textContent = JSON.stringify({sequence: result.sequence_summary, result}, null, 2);
        currentJob = null;
      }
    } catch (exc) { showError(exc); timer = setTimeout(poll, 1500); }
  }
  async function launch(spec) {
    error.textContent = '';
    if (currentJob) throw new Error('先停止或等待本轮任务结束');
    const fresh = await api('/api/workcell/info'); info = fresh;
    if (fresh.active_job) throw new Error('已有任务占用工作台');
    const result = await api('/api/workcell/jobs', {spec});
    currentJob = result.job_id; currentRequest = spec;
    if (spec.task === 'pusht') { lastPose=null; drawIterationT(); }
    if (timer) clearTimeout(timer); await poll();
  }

  const pp = section('pickplace-panel', '连续整理：同一场景，多物体', '保留原抓放规则。先完成容易的对象，再处理困难对象；已放置物体留在碰撞世界，部分完成不会被记成全成功。当前新增顺序入口先限于 native-world 仿真。');
  const picks = field(pp, '请求物体（可多选）', el('select', null, {multiple: '', size: '7'}));
  const autoOrder = field(pp, '采用建议顺序（刷子、笔、木块、胡萝卜、球，再处理困难物体）', el('input', null, {type: 'checkbox', checked: ''}));
  const sequenceSpec = mode => ({task: 'pickplace', mode, parameters: {
    object_names: [...picks.selectedOptions].map(o => o.value), automatic_order: autoOrder.checked
  }});
  const ppPreview = button(pp, '预览连续任务', () => launch(sequenceSpec('preview')));
  const ppRun = button(pp, '运行连续抓放仿真', () => launch(sequenceSpec('sim')));
  ppPreview.disabled = ppRun.disabled = true;

  const mg = section('magnetic-panel', '用自然语言生成装配设计', 'LLM 从旧版底板模板的有效槽位中组合外形，固定底板、原坐标与父子关系不改。先验证并运行原装配仿真；LLM 输出不是物理可搭性的保证。');
  const board = field(mg, '原底板', el('select', null));
  const budget = field(mg, '最多使用活动件数', el('input', null, {type: 'number', min: '1', max: '12', value: '12'}));
  const description = field(mg, '想搭什么', el('textarea', '搭一个两层、有三角屋顶的小房子。尽量左右对称，使用不超过12块。', {rows: '3', maxlength: '1500'}));
  const mgMessage = el('p', '读取原底板和 LLM 配置…'); mg.append(mgMessage);
  const generateButton = button(mg, 'LLM 生成并校验', async () => {
    generateButton.disabled = true; generatedDesign = proof = null;
    try {
      const job = await api('/api/workcell/iterate/generate', {board_id: board.value, prompt: description.value, piece_budget: Number(budget.value)});
      mgMessage.textContent = '正在生成，最多两次模型调用；不会自动执行装配。';
      let output;
      for (let attempts=0; attempts<180; attempts++) {
        await new Promise(resolve => setTimeout(resolve, 750));
        output = await api(`/api/workcell/iterate/generations/${job.generation_id}`);
        if (output.status !== 'running') break;
      }
      if (!output || output.status !== 'succeeded') throw new Error(output && output.error || '生成尚未完成或失败');
      generatedDesign = output.design; proof = output.proof;
      $('design-json').value = JSON.stringify(output.design, null, 2);
      $('apply-json').click(); // Preserved editor performs its own redraw/round-trip.
      mgMessage.textContent = `${proof.title}：${proof.movable_count}块，${proof.explanation}（原几何/依赖已校验，尚未做该设计的整链仿真）`;
      mgPreview.disabled = mgRun.disabled = false;
    } finally { generateButton.disabled = !(features.llm.configured && board.options.length); }
  });
  generateButton.disabled = true;
  function generatedSpec(mode) {
    if (!generatedDesign || !proof) throw new Error('先生成设计');
    const current = JSON.parse($('design-json').value);
    // Compare on server; altered legacy editor content cannot reuse the old proof.
    return {task: 'magnetic', mode, parameters: {design: current, generation_proof: proof}};
  }
  const mgPreview = button(mg, '预览生成的结构', () => launch(generatedSpec('preview'))); mgPreview.disabled = true;
  const mgRun = button(mg, '使用原装配程序仿真', () => launch(generatedSpec('sim'))); mgRun.disabled = true;

  const pt = section('pusht-panel', '长推、随机初态与交互恢复', '远处使用长推候选，接近目标时保留短推微调。每次推完重新定位。无进展时等待场景变化而非空转；碰撞/跟踪等异常仍停止，不会盲推。');
  const backend = field(pt, '仿真后端', el('select', null));
  const geometry = field(pt, 'T 几何（服务端已配置）', el('select', null));
  const seed = field(pt, '随机种子', el('input', null, {type:'number',min:'0',max:'4294967295',value:'42'}));
  const maximum = field(pt, '最长推移 / 米', el('input', null, {type:'number',min:'.012',max:'.08',step:'.005',value:'.05'}));
  const continuous = field(pt, '未到位则继续；停止按钮、碰撞异常与无进展等待仍有效', el('input', null, {type:'checkbox',checked:''}));
  const tCanvas=el('canvas',null,{width:'800',height:'400','aria-label':'本轮随机T几何与实际观测',class:'iteration-t-canvas'}); pt.append(tCanvas);
  const randomInfo = el('p', '随机位置、方向和目标；不同尺寸只来自服务端几何配置。'); pt.append(randomInfo);
  button(pt, '随机一个 T 初态和目标', async () => {
    const result = await api('/api/workcell/iterate/randomize', {seed:Number(seed.value),count:3,geometry_id:geometry.value});
    const index = Number(seed.value) % 3; lastCase = result.cases[index];
    const a=lastCase.initial_pose,b=lastCase.goal_pose;
    [['push-x',a[0]],['push-y',a[1]],['push-yaw',a[2]*180/Math.PI],['goal-x',b[0]],['goal-y',b[1]],['goal-yaw',b[2]*180/Math.PI]].forEach(([id,value]) => {
      $(id).value=String(value); $(id).dispatchEvent(new Event('input',{bubbles:true}));
    });
    randomInfo.textContent = `案例 ${lastCase.case_id} / ${lastCase.mode}：几何边界通过，机械臂可达性尚未验证。`;
    lastPose=null; drawIterationT();
    seed.value=String((Number(seed.value)+1)%4294967296);
  });
  function pushSpec(mode) {
    return {task:'pusht',mode,parameters:{initial_pose:[Number($('push-x').value),Number($('push-y').value),Number($('push-yaw').value)*Math.PI/180],
      goal_pose:[Number($('goal-x').value),Number($('goal-y').value),Number($('goal-yaw').value)*Math.PI/180],
      speed_mps:Number($('push-speed').value),max_steps:Number($('push-steps').value),
      maximum_push_length_m:Number(maximum.value),geometry_id:geometry.value,
      simulation_backend:backend.value,run_until_goal:continuous.checked}};
  }
  button(pt, '预览长推请求', () => launch(pushSpec('preview')));
  button(pt, '启动交互推移仿真', () => launch(pushSpec('sim')));
  let lastPose = null;
  function drawIterationT() {
    if (!features || !info) return;
    const m=features.geometry_models[geometry.value], w=info.pusht_model.workspace || [.15,.65,-.3,.3];
    if (!m) return;
    const ctx=tCanvas.getContext('2d'), xy=p=>[40+(p[0]-w[0])/(w[1]-w[0])*720,360-(p[1]-w[2])/(w[3]-w[2])*320];
    ctx.clearRect(0,0,800,400); ctx.strokeStyle='#cad6dd'; ctx.strokeRect(40,40,720,320);
    const bw=m.bar_width_m,bh=m.bar_height_m,sw=m.stem_width_m,sh=m.stem_height_m,com=-(sw*sh)*(bh/2+sh/2)/(bw*bh+sw*sh);
    const draw=(pose,goal)=>{
      const c=Math.cos(pose[2]),s=Math.sin(pose[2]); ctx.setLineDash(goal?[6,5]:[]);
      ctx.strokeStyle=goal?'#b17b31':'#216579';ctx.fillStyle='#b9d7df';
      for (const [x,y,width,height] of [[0,-com,bw,bh],[0,-bh/2-sh/2-com,sw,sh]]) {
        const points=[[-1,-1],[1,-1],[1,1],[-1,1]].map(([u,v])=>[x+u*width/2,y+v*height/2]).map(([u,v])=>xy([pose[0]+c*u-s*v,pose[1]+s*u+c*v]));
        ctx.beginPath();points.forEach((p,i)=>i?ctx.lineTo(...p):ctx.moveTo(...p));ctx.closePath();if(!goal)ctx.fill();ctx.stroke();
      }
      ctx.setLineDash([]);
    };
    const p=pushSpec('preview').parameters;draw(p.goal_pose,true);draw(lastPose || p.initial_pose,false);
    ctx.fillStyle='#334f5b';ctx.font='14px sans-serif';ctx.fillText(`${geometry.value} / 实线：${lastPose?'本轮观测':'待运行初态'}，虚线：目标`,40,22);
  }
  geometry.addEventListener('change',()=>{lastPose=null;drawIterationT();});
  for (const id of ['push-x','push-y','push-yaw','goal-x','goal-y','goal-yaw']) $(id).addEventListener('input',drawIterationT);

  const newX = field(pt, '暂停后 T 的新 X', el('input', null, {type:'number',value:'.36',step:'.005'}));
  const newY = field(pt, '暂停后 T 的新 Y', el('input', null, {type:'number',value:'-.18',step:'.005'}));
  const newYaw = field(pt, '暂停后 T 的新 yaw / 度', el('input', null, {type:'number',value:'20',step:'5'}));
  const relocate = button(pt, '模拟手动移动 T（仅暂停后）', () => control('relocate',[Number(newX.value),Number(newY.value),Number(newYaw.value)*Math.PI/180]));
  relocate.disabled = true;
  pt.append(el('p','先点“本次推完并撤离后暂停”，等待 paused 确认，再修改仿真 T；改完点“重新定位并继续”。这个按钮不控制真实机械臂，也不是物理急停。'));

  async function initialize() {
    info=await api('/api/workcell/info'); features=await api('/api/workcell/iterate/features');
    for (const name of info.pickplace_objects) { const o=el('option',name,{value:name}); o.selected=true; picks.append(o); }
    ppRun.disabled = ppPreview.disabled = !features.sequence_native_sim_enabled;
    const bases=new Map(features.jimu_catalog.map(t => [t.board_id,t.board_title]));
    for (const [key,title] of bases) board.append(el('option',title,{value:key}));
    generateButton.disabled=!(features.llm.configured && board.options.length);
    mgMessage.textContent = generateButton.disabled ? '尚需配置原版底板库或 LLM。请按交接文档导入3×3与弧形任务；不会用猜测的弧形底板替代。' : '底板库和 LLM 已配置。生成后再单独提交仿真。';
    for (const name of info.pusht_simulation_backends || ['surrogate']) backend.append(el('option',name,{value:name}));
    if ([...backend.options].some(o => o.value === 'full_arm_physics')) backend.value='full_arm_physics';
    for (const name of features.geometry_ids) geometry.append(el('option',name,{value:name}));
    drawIterationT();
    if (info.active_job) { currentJob=info.active_job; await poll(); }
  }
  initialize().catch(showError);
})();
