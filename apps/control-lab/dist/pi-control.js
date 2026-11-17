const $ = id => document.getElementById(id);
const ids = ['area','initial','outlet','leak','target','kp','ki','bias','maxFlow','extraLeak','disturbanceAt','duration','dt'];
const units = {area:' m²',initial:' m',outlet:' m²/s',leak:' m³/s',target:' m',kp:' m²/s',ki:' m/s²',bias:' m³/s',maxFlow:' m³/s',extraLeak:' m³/s',disturbanceAt:' s',duration:' s',dt:' s'};
const defaults = Object.fromEntries(ids.map(id => [id,$(id).value]));
const colors = {p:'#d4ae17',pi:'#0ebbbb'};
let latest = null;
let playing = true;
let lastFrameTime = performance.now();

function parameters(){return Object.fromEntries(ids.map(id=>[id,Number($(id).value)]))}

function evaluateFrame(p,time,height,integral,mode){
  const error=p.target-height;
  const pTerm=p.kp*error;
  const iTerm=mode==='pi'?p.ki*integral:0;
  const rawFlow=p.bias+pTerm+iTerm;
  const flow=Math.min(p.maxFlow,Math.max(0,rawFlow));
  const leak=p.leak+(time>=p.disturbanceAt-1e-9?p.extraLeak:0);
  const outlet=p.outlet*height;
  const heightRate=(flow-outlet-leak)/p.area;
  const integralRate=mode==='pi'?error:0;
  return {time,height,integral,error,pTerm,iTerm,rawFlow,flow,outlet,leak,heightRate,integralRate};
}

function simulate(p,mode){
  const frames=[];let time=0,height=p.initial,integral=0;
  while(true){
    const frame=evaluateFrame(p,time,height,integral,mode);frames.push(frame);
    if(time>=p.duration-1e-10)break;
    const step=Math.min(p.dt,p.duration-time);
    height+=step*frame.heightRate;
    integral+=step*frame.integralRate;
    time=Math.min(p.duration,time+step);
  }
  return {frames,final:frames.at(-1)};
}

function plotFrames(frames,limit=2600){if(frames.length<=limit)return frames;const stride=Math.ceil((frames.length-1)/(limit-1));const out=frames.filter((_,i)=>i%stride===0);if(out.at(-1)!==frames.at(-1))out.push(frames.at(-1));return out}
function selectedRuns(){const runs=[];if($('showBiased').checked)runs.push(['p',latest.runs.p]);if($('showPi').checked)runs.push(['pi',latest.runs.pi]);return runs.length?runs:[['pi',latest.runs.pi]]}
function range(runs,key,extra=[]){const values=runs.flatMap(([,run])=>plotFrames(run.frames).map(d=>d[key])).concat(extra);const low=Math.min(...values),high=Math.max(...values),pad=(high-low||1)*.1;return[Math.min(0,low-pad),high+pad]}

function draw(canvas,key,p,showTarget=false){
  const runs=selectedRuns(),ratio=devicePixelRatio||1,width=canvas.clientWidth,height=canvas.clientHeight;canvas.width=width*ratio;canvas.height=height*ratio;const c=canvas.getContext('2d');c.scale(ratio,ratio);c.clearRect(0,0,width,height);
  const pad={l:58,r:18,t:10,b:34},[y0,y1]=range(runs,key,showTarget?[p.target]:[]),x=t=>pad.l+t/p.duration*(width-pad.l-pad.r),y=v=>height-pad.b-(v-y0)/(y1-y0)*(height-pad.t-pad.b);
  c.font='10px ui-monospace,monospace';c.fillStyle='#778087';c.strokeStyle='#dedbd1';c.lineWidth=1;
  for(let i=0;i<=4;i++){const v=y0+(y1-y0)*i/4,py=y(v);c.beginPath();c.moveTo(pad.l,py);c.lineTo(width-pad.r,py);c.stroke();c.fillText(v.toFixed(2),8,py+3)}
  for(let i=0;i<=5;i++){const t=p.duration*i/5;c.fillText(Math.round(t)+'s',x(t)-10,height-10)}
  if(showTarget){c.strokeStyle='#ff6b35';c.setLineDash([6,5]);c.beginPath();c.moveTo(pad.l,y(p.target));c.lineTo(width-pad.r,y(p.target));c.stroke();c.setLineDash([])}
  if(p.extraLeak>0&&p.disturbanceAt<p.duration){c.strokeStyle='#07131f55';c.setLineDash([3,4]);c.beginPath();c.moveTo(x(p.disturbanceAt),pad.t);c.lineTo(x(p.disturbanceAt),height-pad.b);c.stroke();c.setLineDash([])}
  runs.forEach(([name,run])=>{c.strokeStyle=colors[name];c.lineWidth=name==='pi'?2.7:1.8;c.beginPath();plotFrames(run.frames).forEach((d,i)=>i?c.lineTo(x(d.time),y(d[key])):c.moveTo(x(d.time),y(d[key])));c.stroke()});
}

function setFlowMark(element,value,maximum,vertical=false){const n=Math.max(0,Math.min(1,value/Math.max(maximum,1e-9)));element.style.transform=vertical?`scaleY(${.08+.92*n})`:`scaleX(${.08+.92*n})`;element.style.opacity=String(.25+.75*n);if(vertical)element.style.width=`${3+11*n}px`;else element.style.height=`${3+11*n}px`}
function currentFrame(){return latest?latest.runs.pi.frames[Number($('animationTime').value)]:null}

function updateAnimation(){
  const d=currentFrame();if(!d)return;const{p,heightScale}=latest;
  $('animatedWater').style.height=`${Math.max(0,Math.min(100,d.height/heightScale*100))}%`;
  $('animatedTarget').style.bottom=`${Math.max(0,Math.min(100,p.target/heightScale*100))}%`;
  $('processTank').style.width=`${Math.max(115,Math.min(230,170*Math.sqrt(p.area/2)))}px`;
  $('scaleTop').textContent=`${heightScale.toFixed(1)} m`;
  setFlowMark($('inletStream'),d.flow,p.maxFlow,true);setFlowMark($('outletStream'),Math.max(0,d.outlet),p.maxFlow);setFlowMark($('leakStream'),d.leak,p.maxFlow,true);
  $('pumpLabel').textContent=`u = ${d.flow.toFixed(4)} m³/s`;$('outletLabel').textContent=`k·h = ${d.outlet.toFixed(4)} m³/s`;$('leakLabel').textContent=`q_leak = ${d.leak.toFixed(4)} m³/s`;$('animatedHeight').textContent=`h = ${d.height.toFixed(3)} m`;
  $('levelTrend').textContent=d.heightRate>1e-6?'↑ rising':d.heightRate<-1e-6?'↓ falling':'≈ steady';$('modelWarning').textContent=d.height<0?'computed below 0 m · ideal model is outside its physical range':'';
  $('liveError').textContent=`e = ${d.error.toFixed(3)} m`;$('animatedIntegral').textContent=`z = ${d.integral.toFixed(3)} m·s`;
  $('liveCommand').textContent=`${p.bias.toFixed(3)} + ${d.pTerm.toFixed(4)} + ${d.iTerm.toFixed(4)} → u = ${d.flow.toFixed(4)} m³/s`;
  $('liveRates').textContent=`dh/dt = ${d.heightRate.toFixed(6)} m/s · dz/dt = ${d.integralRate.toFixed(3)} m`;
  $('balanceIn').textContent=`IN ${d.flow.toFixed(4)}`;$('balanceOut').textContent=`OUT ${(d.outlet+d.leak).toFixed(4)} m³/s`;$('balanceRelation').textContent=d.heightRate>1e-6?'>':d.heightRate<-1e-6?'<':'≈';$('animationClock').textContent=`${d.time.toFixed(1)} s`;
}

function render(){
  const p=parameters();if(p.disturbanceAt>p.duration){$('disturbanceAt').value=p.duration;p.disturbanceAt=p.duration}
  ids.forEach(id=>{const digits=['area','initial','target'].includes(id)?1:['disturbanceAt','duration'].includes(id)?0:id==='ki'?4:3;document.querySelector(`[data-for="${id}"]`).textContent=Number($(id).value).toFixed(digits)+units[id]});
  const runs={p:simulate(p,'p'),pi:simulate(p,'pi')};let maxHeight=p.target;for(const run of Object.values(runs))for(const d of run.frames)maxHeight=Math.max(maxHeight,Math.max(0,d.height));latest={p,runs,heightScale:Math.max(1,Math.ceil(maxHeight*1.1*2)/2)};
  draw($('heightChart'),'height',p,true);draw($('flowChart'),'flow',p,false);
  $('animationTime').max=String(runs.pi.frames.length-1);$('animationTime').value='0';
  const pi=runs.pi.final,pOnly=runs.p.final,piError=p.target-pi.height,pError=p.target-pOnly.height;
  $('finalHeight').textContent=`${pi.height.toFixed(3)} m`;$('finalError').textContent=`${piError>=0?'+':''}${piError.toFixed(3)} m error`;$('pOffset').textContent=`${pError>=0?'+':''}${pError.toFixed(3)} m`;$('integralFinal').textContent=`${pi.integral.toFixed(2)} m·s`;$('integralFlow').textContent=`${pi.iTerm.toFixed(4)} m³/s`;
  $('equation').textContent=`Ki × z = ${pi.iTerm.toFixed(4)} m³/s`;$('insightText').textContent=p.extraLeak>0&&p.disturbanceAt<p.duration?`P retains ${Math.abs(pError).toFixed(3)} m of error to request extra flow. PI stores past error in z, so Ki·z supplies ${pi.iTerm.toFixed(4)} m³/s while the current error returns toward zero.`:`With the nominal bias, both controllers can hold the known operating point. Integral memory matters when a persistent unknown disturbance changes the required balancing flow.`;
  updateAnimation();
}

function animate(now){if(playing&&latest){const count=latest.runs.pi.frames.length;let next=Number($('animationTime').value)+(now-lastFrameTime)/18000*count;if(next>=count-1)next=0;$('animationTime').value=String(Math.floor(next));updateAnimation()}lastFrameTime=now;requestAnimationFrame(animate)}
let renderRequest;function schedule(){cancelAnimationFrame(renderRequest);renderRequest=requestAnimationFrame(render)}
ids.forEach(id=>$(id).addEventListener('input',schedule));['showBiased','showPi'].forEach(id=>$(id).addEventListener('change',()=>{draw($('heightChart'),'height',latest.p,true);draw($('flowChart'),'flow',latest.p,false)}));
$('animationTime').addEventListener('input',()=>{playing=false;$('playAnimation').textContent='Play';updateAnimation()});$('playAnimation').onclick=()=>{playing=!playing;$('playAnimation').textContent=playing?'Pause':'Play'};
$('reset').onclick=()=>{ids.forEach(id=>$(id).value=defaults[id]);playing=true;$('playAnimation').textContent='Pause';render()};window.addEventListener('resize',schedule);
$('download').onclick=()=>{const{runs}=latest;let csv='time_s,p_height_m,p_flow_m3_s,pi_height_m,pi_integral_m_s,pi_p_term_m3_s,pi_i_term_m3_s,pi_raw_flow_m3_s,pi_actual_flow_m3_s\n';for(let i=0;i<runs.pi.frames.length;i++){const p=runs.p.frames[i],q=runs.pi.frames[i];csv += [q.time,p.height,p.flow,q.height,q.integral,q.pTerm,q.iTerm,q.rawFlow,q.flow].join(',')+'\n'}const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([csv],{type:'text/csv'}));a.download='water-tank-pi-simulation.csv';a.click();URL.revokeObjectURL(a.href)};
render();requestAnimationFrame(animate);
