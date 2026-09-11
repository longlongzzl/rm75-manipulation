/* Local canvas previews. Schematic geometry only; never a planning input. */
const dims={square:[.074,.0065,.074],half_square:[.037,.0065,.074],triangle:[.074,.0065,.135]};
const norm=(x)=>{const n=Math.hypot(...x);return x.map(v=>v/n);};
const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
export function pieceFaces(piece){
  const d=dims[piece.type];if(!d)return [];
  const [w,t,h]=d;
  // Match the existing viewer's [u,n,v] builder convention; lightly normalize
  // export rounding for rendering only. Stored design bytes are never altered.
  const u=norm(piece.u),nd=dot(piece.n,u),n=norm(piece.n.map((x,i)=>x-nd*u[i])),v=cross(u,n);
  const ring=piece.type==='triangle'?[[-w/2,-h/2],[w/2,-h/2],[0,h/2]]:
      [[-w/2,-h/2],[w/2,-h/2],[w/2,h/2],[-w/2,h/2]];
  const at=(x,y,z)=>piece.center.map((c,i)=>c+x*u[i]+y*n[i]+z*v[i]);
  const a=ring.map(([x,z])=>at(x,-t/2,z)),b=ring.map(([x,z])=>at(x,t/2,z));
  const faces=[a,b];for(let i=0;i<a.length;i++){const j=(i+1)%a.length;faces.push([a[i],a[j],b[j],b[i]]);}return faces;
}
export class DesignView {
  constructor(canvas){
    this.canvas=canvas;this.design=null;this.yaw=.7;this.elevation=.5;this.zoom=1;this.pointer=null;
    canvas.addEventListener('pointerdown',e=>{this.pointer=[e.clientX,e.clientY,this.yaw,this.elevation];canvas.setPointerCapture(e.pointerId);});
    canvas.addEventListener('pointermove',e=>{if(!this.pointer)return;this.yaw=this.pointer[2]+(e.clientX-this.pointer[0])*.008;this.elevation=Math.max(-.15,Math.min(1.55,this.pointer[3]+(e.clientY-this.pointer[1])*.008));this.draw();});
    const done=()=>this.pointer=null;canvas.addEventListener('pointerup',done);canvas.addEventListener('pointercancel',done);
    canvas.addEventListener('wheel',e=>{e.preventDefault();this.zoom=Math.max(.45,Math.min(3,this.zoom*Math.exp(-e.deltaY*.001)));this.draw();},{passive:false});
    canvas.addEventListener('keydown',e=>{if(!['ArrowLeft','ArrowRight','ArrowUp','ArrowDown','+','-'].includes(e.key))return;e.preventDefault();
      if(e.key==='ArrowLeft')this.yaw-=.12;if(e.key==='ArrowRight')this.yaw+=.12;if(e.key==='ArrowUp')this.elevation=Math.min(1.55,this.elevation+.12);if(e.key==='ArrowDown')this.elevation=Math.max(-.15,this.elevation-.12);
      if(e.key==='+')this.zoom=Math.min(3,this.zoom*1.1);if(e.key==='-')this.zoom=Math.max(.45,this.zoom/1.1);this.draw();});
    this.draw();
  }
  update(d){this.design=d;this.draw();}
  view(mode){this.zoom=1;if(mode==='top'){this.yaw=0;this.elevation=1.55;}else if(mode==='front'){this.yaw=0;this.elevation=0;}else {this.yaw=.7;this.elevation=.5;}this.draw();}
  draw(){
    const c=this.canvas,ctx=c.getContext('2d'),W=c.width,H=c.height;ctx.clearRect(0,0,W,H);
    ctx.fillStyle='#f7f9fc';ctx.fillRect(0,0,W,H);
    if(!this.design){ctx.textAlign='center';ctx.fillStyle='#879aac';ctx.font='18px system-ui';ctx.fillText('生成或载入原模板后，在这里查看结构',W/2,H/2);ctx.font='13px system-ui';ctx.fillText('固定底板保持原样 · 最多 12 块活动积木',W/2,H/2+32);return;}
    const ca=Math.cos(this.yaw),sa=Math.sin(this.yaw),ce=Math.cos(this.elevation),se=Math.sin(this.elevation);
    const project=p=>[ca*p[0]-sa*p[2],-ce*p[1]+se*(sa*p[0]+ca*p[2]),se*p[1]+ce*(sa*p[0]+ca*p[2])];
    let faces=[];this.design.pieces.forEach((p,i)=>{for(const f of pieceFaces(p))faces.push({points:f.map(project),locked:!!p.locked,index:i});});
    if(!faces.length)return;
    const all=faces.flatMap(f=>f.points),min=[Infinity,Infinity],max=[-Infinity,-Infinity];for(const p of all)for(let j=0;j<2;j++){min[j]=Math.min(min[j],p[j]);max[j]=Math.max(max[j],p[j]);}
    const scale=Math.min((W-180)/Math.max(max[0]-min[0],.05),(H-130)/Math.max(max[1]-min[1],.05))*this.zoom;
    const screen=p=>[W/2+(p[0]-(min[0]+max[0])/2)*scale,H/2+15+(p[1]-(min[1]+max[1])/2)*scale];
    ctx.strokeStyle='#e5ebf1';ctx.lineWidth=1;for(let k=-5;k<=5;k++){for(const [a,b]of [[[-.37,0,k*.074],[.37,0,k*.074]],[[k*.074,0,-.37],[k*.074,0,.37]]]){ctx.beginPath();ctx.moveTo(...screen(project(a)));ctx.lineTo(...screen(project(b)));ctx.stroke();}}
    faces.sort((a,b)=>a.points.reduce((s,p)=>s+p[2],0)/a.points.length-b.points.reduce((s,p)=>s+p[2],0)/b.points.length);
    for(const f of faces){const pts=f.points.map(screen);ctx.beginPath();pts.forEach((p,i)=>i?ctx.lineTo(...p):ctx.moveTo(...p));ctx.closePath();
      ctx.fillStyle=f.locked?'#b8c8d7':['#71a0e6','#93b5ea','#5487d3'][f.index%3];ctx.strokeStyle=f.locked?'#829caf':'#3b6daa';ctx.lineWidth=1.3;ctx.fill();ctx.stroke();}
    ctx.textAlign='left';ctx.fillStyle='#8296a8';ctx.font='12px system-ui';ctx.fillText('Builder 坐标 · Y 向上 · 示意几何，不是碰撞网格',24,30);
  }
}
export function pushTransform(canvas,workspace){
  const W=canvas.width,H=canvas.height,w=workspace||[.15,.65,-.3,.3],scale=Math.min((W-120)/(w[1]-w[0]),(H-80)/(w[3]-w[2]));
  const ox=W/2,oy=H/2+3,cx=(w[0]+w[1])/2,cy=(w[2]+w[3])/2;
  return {scale,xy:p=>[ox+(p[0]-cx)*scale,oy-(p[1]-cy)*scale],world:p=>[cx+(p[0]-ox)/scale,cy-(p[1]-oy)/scale]};
}
export function drawPush(canvas,{model={},initial,target,observed,trail=[],bounds=null}){
  const ctx=canvas.getContext('2d'),W=canvas.width,H=canvas.height,w=bounds||model.workspace||[.15,.65,-.3,.3],t=pushTransform(canvas,w);
  ctx.clearRect(0,0,W,H);ctx.fillStyle='#f7f9fc';ctx.fillRect(0,0,W,H);
  const a=t.xy([w[0],w[3]]),b=t.xy([w[1],w[2]]);ctx.strokeStyle='#d9e3ec';ctx.lineWidth=1;ctx.strokeRect(a[0],a[1],b[0]-a[0],b[1]-a[1]);
  ctx.font='11px system-ui';ctx.textAlign='center';ctx.fillStyle='#8fa0af';for(let i=0;i<=5;i++){let x=w[0]+i*(w[1]-w[0])/5,y=w[2]+i*(w[3]-w[2])/5;ctx.strokeStyle='#e8eef4';ctx.beginPath();ctx.moveTo(...t.xy([x,w[2]]));ctx.lineTo(...t.xy([x,w[3]]));ctx.stroke();ctx.fillText(x.toFixed(2),...t.xy([x,w[2]-.025]));ctx.beginPath();ctx.moveTo(...t.xy([w[0],y]));ctx.lineTo(...t.xy([w[1],y]));ctx.stroke();}
  for(const [x,y,r] of model.obstacles||[]){ctx.fillStyle='#e7ccd0';ctx.beginPath();ctx.arc(...t.xy([x,y]),r*t.scale,0,2*Math.PI);ctx.fill();}
  if(trail.length){ctx.strokeStyle='#7eabc8';ctx.setLineDash([3,5]);ctx.beginPath();trail.forEach((p,i)=>i?ctx.lineTo(...t.xy(p)):ctx.moveTo(...t.xy(p)));ctx.stroke();ctx.setLineDash([]);}
  const bw=model.bar_width_m??.1,bh=model.bar_height_m??.03,sw=model.stem_width_m??.03,sh=model.stem_height_m??.07,com=-(sw*sh)*(bh/2+sh/2)/(bw*bh+sw*sh);
  const shape=(p,goal)=>{
    if(!p||p.some(x=>!Number.isFinite(x)))return;
    ctx.strokeStyle=goal?'#af8a44':'#286cac';ctx.fillStyle='#7daee177';ctx.lineWidth=2;ctx.setLineDash(goal?[7,5]:[]);
    const ca=Math.cos(p[2]),sa=Math.sin(p[2]);for(const[x,y,width,height]of [[0,-com,bw,bh],[0,-bh/2-sh/2-com,sw,sh]]){
      const pts=[[-1,-1],[1,-1],[1,1],[-1,1]].map(([u,v])=>[x+u*width/2,y+v*height/2]).map(([u,v])=>t.xy([p[0]+ca*u-sa*v,p[1]+sa*u+ca*v]));
      ctx.beginPath();pts.forEach((q,i)=>i?ctx.lineTo(...q):ctx.moveTo(...q));ctx.closePath();if(!goal)ctx.fill();ctx.stroke();}
    ctx.setLineDash([]);const center=t.xy(p);ctx.fillStyle=goal?'#af8a44':'#286cac';ctx.beginPath();ctx.arc(...center,3,0,7);ctx.fill();
  };
  shape(target,true);shape(observed||initial,false);ctx.textAlign='left';ctx.fillStyle='#64829b';ctx.font='12px system-ui';ctx.fillText(observed?'显示最近一帧任务观测（非相机画面）':'草稿初态 / 目标；点击可设目标 X、Y',20,25);
}
