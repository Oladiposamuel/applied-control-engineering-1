const $ = id => document.getElementById(id);
const ids = ['a1','a2','k12','k2','h1','h2','target','kp','ki','bias','maxFlow','demand','disturbanceAt','duration','dt'];
const units = {a1:' m²',a2:' m²',k12:' m²/s',k2:' m²/s',h1:' m',h2:' m',target:' m',kp:' m²/s',ki:' m/s²',bias:' m³/s',maxFlow:' m³/s',demand:' m³/s',disturbanceAt:' s',duration:' s',dt:' s'};
const defaults = Object.fromEntries(ids.map(id=>[id,$(id).value]));
defaults.mode = $('mode').value;
let latest = null;
let playing = true;
let lastFrameTime = performance.now();

function parameters(){return {...Object.fromEntries(ids.map(id=>[id,Number($(id).value)])),mode:$('mode').value}}

function evaluateStep(p,time,h1,h2,integral,step){
  const error=p.target-h2;
  const pTerm=p.mode==='open_loop'?0:p.kp*error;
  const iTerm=p.mode==='pi'?p.ki*integral:0;
  const rawPump=p.mode==='open_loop'?p.bias:p.bias+pTerm+iTerm;
  const pump=Math.min(p.maxFlow,Math.max(0,rawPump));
  const demandRequested=time>=p.disturbanceAt-1e-9?p.demand:0;
  const headDifference=h1-h2;
  const q12Requested=Math.max(0,p.k12*headDifference);
  const qoutRequested=Math.max(0,p.k2*h2);

  const volume1=p.a1*Math.max(0,h1);
  const available1=volume1+pump*step;
  const transferVolume=Math.min(q12Requested*step,available1);
  const q12=transferVolume/step;
  const nextVolume1=available1-transferVolume;

  const available2=p.a2*Math.max(0,h2)+transferVolume;
  const outletRequestVolume=qoutRequested*step;
  const demandRequestVolume=demandRequested*step;
  const totalRequest=outletRequestVolume+demandRequestVolume;
  const serviceScale=totalRequest>0?Math.min(1,available2/totalRequest):1;
  const outletVolume=outletRequestVolume*serviceScale;
  const demandVolume=demandRequestVolume*serviceScale;
  const qout=outletVolume/step;
  const demandServed=demandVolume/step;
  const demandUnmet=demandRequested-demandServed;
  const nextVolume2=available2-outletVolume-demandVolume;
  const nextH1=nextVolume1/p.a1;
  const nextH2=nextVolume2/p.a2;
  const rate1=(nextH1-h1)/step;
  const rate2=(nextH2-h2)/step;
  const integralRate=p.mode==='pi'?error:0;
  const totalVolume=p.a1*h1+p.a2*h2;
  const nextTotal=nextVolume1+nextVolume2;
  const residual=nextTotal-totalVolume-step*(pump-qout-demandServed);
  return {time,h1,h2,integral,error,pTerm,iTerm,rawPump,pump,headDifference,q12Requested,q12,qoutRequested,qout,demandRequested,demandServed,demandUnmet,rate1,rate2,integralRate,totalVolume,residual,nextH1,nextH2,nextIntegral:integral+step*integralRate};
}

function simulate(p){
  const frames=[];let time=0,h1=p.h1,h2=p.h2,integral=0,maxResidual=0;
  while(true){
    const atEnd=time>=p.duration-1e-10;
    const step=atEnd?p.dt:Math.min(p.dt,p.duration-time);
    const frame=evaluateStep(p,time,h1,h2,integral,step);
    frames.push(frame);maxResidual=Math.max(maxResidual,Math.abs(frame.residual));
    if(atEnd)break;
    h1=frame.nextH1;h2=frame.nextH2;integral=frame.nextIntegral;time=Math.min(p.duration,time+step);
  }
  return {frames,final:frames.at(-1),maxResidual};
}

function plotFrames(frames,limit=2800){if(frames.length<=limit)return frames;const stride=Math.ceil((frames.length-1)/(limit-1));const out=frames.filter((_,i)=>i%stride===0);if(out.at(-1)!==frames.at(-1))out.push(frames.at(-1));return out}
function range(frames,keys,extra=[]){const data=plotFrames(frames),values=data.flatMap(d=>keys.map(k=>d[k])).concat(extra);const low=Math.min(...values),high=Math.max(...values),pad=(high-low||1)*.1;return[Math.min(0,low-pad),high+pad]}

function draw(canvas,frames,keys,p,target=false){
  const data=plotFrames(frames),ratio=devicePixelRatio||1,width=canvas.clientWidth,height=canvas.clientHeight;canvas.width=width*ratio;canvas.height=height*ratio;const c=canvas.getContext('2d');c.scale(ratio,ratio);c.clearRect(0,0,width,height);
  const pad={l:58,r:18,t:10,b:34},[y0,y1]=range(frames,keys,target?[p.target]:[]),x=t=>pad.l+t/p.duration*(width-pad.l-pad.r),y=v=>height-pad.b-(v-y0)/(y1-y0)*(height-pad.t-pad.b),colors={h1:'#7f8ea0',h2:'#0ebbbb',pump:'#16c7c7',q12:'#8f79c6',qout:'#d4ae17',demandServed:'#ff6b35'};
  c.font='10px ui-monospace,monospace';c.fillStyle='#778087';c.strokeStyle='#dedbd1';c.lineWidth=1;
  for(let i=0;i<=4;i++){const v=y0+(y1-y0)*i/4,py=y(v);c.beginPath();c.moveTo(pad.l,py);c.lineTo(width-pad.r,py);c.stroke();c.fillText(v.toFixed(2),8,py+3)}
  for(let i=0;i<=5;i++){const t=p.duration*i/5;c.fillText(Math.round(t)+'s',x(t)-10,height-10)}
  if(target){c.strokeStyle='#ff6b35';c.setLineDash([6,5]);c.beginPath();c.moveTo(pad.l,y(p.target));c.lineTo(width-pad.r,y(p.target));c.stroke();c.setLineDash([])}
  if(p.demand>0&&p.disturbanceAt<p.duration){c.strokeStyle='#07131f55';c.setLineDash([3,4]);c.beginPath();c.moveTo(x(p.disturbanceAt),pad.t);c.lineTo(x(p.disturbanceAt),height-pad.b);c.stroke();c.setLineDash([])}
  keys.forEach(key=>{c.strokeStyle=colors[key];c.lineWidth=key==='h2'?2.7:1.8;c.beginPath();data.forEach((d,i)=>i?c.lineTo(x(d.time),y(d[key])):c.moveTo(x(d.time),y(d[key])));c.stroke()});
}

function setFlow(element,value,maximum){
  const active=value>1e-9;
  const n=active?Math.max(0,Math.min(1,value/Math.max(maximum,1e-9))):0;
  element.style.setProperty('--flow-opacity',active?String(.28+.72*n):'0');
  element.style.setProperty('--flow-thickness',`${3+9*n}px`);
  element.style.setProperty('--flow-duration',`${(1.55-1.2*n).toFixed(2)}s`);
  element.classList.toggle('flow-off',!active);
}
function currentFrame(){return latest?latest.run.frames[Number($('animationTime').value)]:null}

function updateAnimation(){
  const d=currentFrame();if(!d)return;const{p,heightScale,flowScale}=latest;
  $('water1').style.height=`${Math.max(0,Math.min(100,d.h1/heightScale*100))}%`;$('water2').style.height=`${Math.max(0,Math.min(100,d.h2/heightScale*100))}%`;$('target2').style.bottom=`${Math.max(0,Math.min(100,p.target/heightScale*100))}%`;
  const tank1Width=Math.max(105,Math.min(220,155*Math.sqrt(p.a1/2))),tank2Width=Math.max(105,Math.min(220,155*Math.sqrt(p.a2/1.5)));
  $('tank1Shell').closest('.tank-unit').style.setProperty('--tank-width',`${tank1Width}px`);$('tank2Shell').closest('.tank-unit').style.setProperty('--tank-width',`${tank2Width}px`);$('twoScaleTop').textContent=`${heightScale.toFixed(1)} m`;
  setFlow($('pumpStream'),d.pump,flowScale);setFlow($('linkStream'),d.q12,flowScale);setFlow($('outletStream'),d.qout,flowScale);setFlow($('demandStream'),d.demandServed,flowScale);
  $('height1Live').textContent=`h₁ = ${d.h1.toFixed(3)} m`;$('height2Live').textContent=`h₂ = ${d.h2.toFixed(3)} m`;$('trend1').textContent=d.rate1>1e-6?'↑ rising':d.rate1<-1e-6?'↓ falling':'≈ steady';$('trend2').textContent=d.rate2>1e-6?'↑ rising':d.rate2<-1e-6?'↓ falling':'≈ steady';
  $('pumpLive').textContent=`pump u = ${d.pump.toFixed(4)} m³/s`;$('linkLabel').textContent=`q₁₂ = ${d.q12.toFixed(4)} m³/s`;$('outletLive').textContent=`qout = ${d.qout.toFixed(4)} m³/s`;$('demandLive').textContent=d.demandUnmet>1e-9?`d = ${d.demandServed.toFixed(4)} of ${d.demandRequested.toFixed(4)} m³/s`:`d served = ${d.demandServed.toFixed(4)} m³/s`;
  $('controlError').textContent=`${p.target.toFixed(3)} − ${d.h2.toFixed(3)} = ${d.error.toFixed(3)} m`;
  const commandTerms=p.mode==='open_loop'?`${p.bias.toFixed(4)}`:p.mode==='p'?`${p.bias.toFixed(4)} + ${d.pTerm.toFixed(4)}`:`${p.bias.toFixed(4)} + ${d.pTerm.toFixed(4)} + ${d.iTerm.toFixed(4)}`;
  $('controllerLive').textContent=`clip(${commandTerms}) = ${d.pump.toFixed(4)} m³/s`;
  const linkBase=`max(0, ${p.k12.toFixed(3)} × ${d.headDifference.toFixed(3)}) = ${d.q12Requested.toFixed(4)}`;
  $('linkEquation').textContent=Math.abs(d.q12-d.q12Requested)>1e-9?`${linkBase}; available volume → ${d.q12.toFixed(4)} m³/s`:`${linkBase} m³/s`;
  $('withdrawalLive').textContent=`${d.qout.toFixed(4)} + ${d.demandServed.toFixed(4)} = ${(d.qout+d.demandServed).toFixed(4)} m³/s`;
  $('rate1Live').textContent=`(${d.pump.toFixed(4)} − ${d.q12.toFixed(4)}) / ${p.a1.toFixed(1)} = ${d.rate1.toFixed(6)} m/s`;$('rate2Live').textContent=`(${d.q12.toFixed(4)} − ${d.qout.toFixed(4)} − ${d.demandServed.toFixed(4)}) / ${p.a2.toFixed(1)} = ${d.rate2.toFixed(6)} m/s`;
  $('animationClock').textContent=`${d.time.toFixed(1)} s`;
}

function format(){ids.forEach(id=>{const digits=['a1','a2','h1','h2','target'].includes(id)?1:['duration','disturbanceAt'].includes(id)?0:id==='ki'?5:3;document.querySelector(`[data-for="${id}"]`).textContent=Number($(id).value).toFixed(digits)+units[id]})}

function render(){
  const priorProgress=latest?Number($('animationTime').value)/Math.max(1,latest.run.frames.length-1):0;
  const p=parameters();if(p.disturbanceAt>p.duration){$('disturbanceAt').value=p.duration;p.disturbanceAt=p.duration}format();
  const run=simulate(p);let maxHeight=p.target,flowScale=0;for(const d of run.frames){maxHeight=Math.max(maxHeight,Math.max(0,d.h1),Math.max(0,d.h2));flowScale=Math.max(flowScale,d.pump,d.q12,d.qout,d.demandServed)}latest={p,run,heightScale:Math.max(1,Math.ceil(maxHeight*1.1*2)/2),flowScale:Math.max(flowScale,1e-9)};
  draw($('heightChart'),run.frames,['h1','h2'],p,true);draw($('flowChart'),run.frames,['pump','q12','qout','demandServed'],p,false);
  $('animationTime').max=String(run.frames.length-1);$('animationTime').value=String(Math.round(priorProgress*(run.frames.length-1)));const end=run.final,error=p.target-end.h2;
  $('finalH1').textContent=`${end.h1.toFixed(3)} m`;$('finalH2').textContent=`${end.h2.toFixed(3)} m`;$('tank2Error').textContent=`${error>=0?'+':''}${error.toFixed(3)} m error`;$('finalPump').textContent=`${end.pump.toFixed(4)} m³/s`;$('finalQ12').textContent=`${end.q12.toFixed(4)} m³/s`;
  $('equation').textContent=`Final flow chain: u ${end.pump.toFixed(4)} → q₁₂ ${end.q12.toFixed(4)} → qout + d ${(end.qout+end.demandServed).toFixed(4)} m³/s`;$('insightText').textContent=p.mode==='open_loop'?'The fixed pump does not react to Tank 2; the levels move until inflow equals the executed outlet and demand.':p.mode==='p'?'P reads Tank 2 but acts through Tank 1. Permanent demand requires a persistent Tank 2 error to sustain extra pump flow.':'PI stores Tank 2 error. Its pump changes Tank 1 first; the head difference then changes q₁₂ and restores Tank 2.';
  updateAnimation();
}

function animate(now){if(playing&&latest){const count=latest.run.frames.length;let next=Number($('animationTime').value)+(now-lastFrameTime)/20000*count;if(next>=count-1)next=0;$('animationTime').value=String(Math.floor(next));updateAnimation()}lastFrameTime=now;requestAnimationFrame(animate)}
let renderRequest;function schedule(){cancelAnimationFrame(renderRequest);renderRequest=requestAnimationFrame(render)}
ids.forEach(id=>$(id).addEventListener('input',schedule));$('mode').addEventListener('change',schedule);$('animationTime').addEventListener('input',()=>{playing=false;$('playAnimation').textContent='Play';updateAnimation()});$('playAnimation').onclick=()=>{playing=!playing;$('playAnimation').textContent=playing?'Pause':'Play'};window.addEventListener('resize',schedule);$('reset').onclick=()=>{ids.forEach(id=>$(id).value=defaults[id]);$('mode').value=defaults.mode;playing=true;latest=null;$('animationTime').value='0';$('playAnimation').textContent='Pause';render()};render();requestAnimationFrame(animate);
