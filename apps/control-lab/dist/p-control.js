const $ = id => document.getElementById(id);
const ids = ['area','initial','outlet','leak','target','kp','bias','maxFlow','extraLeak','disturbanceAt','duration','dt'];
const units = {area:' m²',initial:' m',outlet:' m²/s',leak:' m³/s',target:' m',kp:' m²/s',bias:' m³/s',maxFlow:' m³/s',extraLeak:' m³/s',disturbanceAt:' s',duration:' s',dt:' s'};
const defaults = Object.fromEntries(ids.map(id => [id, $(id).value]));
let latest = null;
let playing = true;
let lastFrameTime = performance.now();

function parameters() {
  return Object.fromEntries(ids.map(id => [id, Number($(id).value)]));
}

function evaluateFrame(p, time, height) {
  const error = p.target - height;
  const pTerm = p.kp * error;
  const rawFlow = p.bias + pTerm;
  const flow = Math.min(p.maxFlow, Math.max(0, rawFlow));
  const leak = p.leak + (time >= p.disturbanceAt - 1e-9 ? p.extraLeak : 0);
  const outlet = p.outlet * height;
  const rate = (flow - outlet - leak) / p.area;
  return {time, height, error, pTerm, rawFlow, flow, outlet, leak, rate};
}

function simulate(p) {
  const frames = [];
  let time = 0;
  let height = p.initial;
  while (true) {
    const frame = evaluateFrame(p, time, height);
    frames.push(frame);
    if (time >= p.duration - 1e-10) break;
    const step = Math.min(p.dt, p.duration - time);
    height += step * frame.rate;
    time = Math.min(p.duration, time + step);
  }
  return {frames, final: frames.at(-1)};
}

function plotFrames(frames, limit=2600) {
  if (frames.length <= limit) return frames;
  const stride = Math.ceil((frames.length - 1) / (limit - 1));
  const sampled = frames.filter((_, i) => i % stride === 0);
  if (sampled.at(-1) !== frames.at(-1)) sampled.push(frames.at(-1));
  return sampled;
}

function valueRange(frames, key, extra=[]) {
  const values = frames.map(d => d[key]).concat(extra);
  const low = Math.min(...values), high = Math.max(...values);
  const pad = (high - low || 1) * .1;
  return [Math.min(0, low - pad), high + pad];
}

function draw(canvas, frames, key, p, showTarget=false) {
  const series = plotFrames(frames);
  const ratio = devicePixelRatio || 1;
  const width = canvas.clientWidth, height = canvas.clientHeight;
  canvas.width = width * ratio; canvas.height = height * ratio;
  const c = canvas.getContext('2d'); c.scale(ratio, ratio); c.clearRect(0,0,width,height);
  const pad = {l:58,r:18,t:10,b:34};
  const [y0,y1] = valueRange(series,key,showTarget?[p.target]:[]);
  const x = t => pad.l + t/p.duration*(width-pad.l-pad.r);
  const y = v => height-pad.b-(v-y0)/(y1-y0)*(height-pad.t-pad.b);
  c.font='10px ui-monospace,monospace'; c.fillStyle='#778087'; c.strokeStyle='#dedbd1'; c.lineWidth=1;
  for(let i=0;i<=4;i++){const v=y0+(y1-y0)*i/4,py=y(v);c.beginPath();c.moveTo(pad.l,py);c.lineTo(width-pad.r,py);c.stroke();c.fillText(v.toFixed(2),8,py+3)}
  for(let i=0;i<=5;i++){const t=p.duration*i/5;c.fillText(Math.round(t)+'s',x(t)-10,height-10)}
  if(showTarget){c.strokeStyle='#ff6b35';c.setLineDash([6,5]);c.beginPath();c.moveTo(pad.l,y(p.target));c.lineTo(width-pad.r,y(p.target));c.stroke();c.setLineDash([])}
  if(p.extraLeak>0&&p.disturbanceAt<p.duration){c.strokeStyle='#07131f55';c.setLineDash([3,4]);c.beginPath();c.moveTo(x(p.disturbanceAt),pad.t);c.lineTo(x(p.disturbanceAt),height-pad.b);c.stroke();c.setLineDash([])}
  c.strokeStyle='#d4ae17';c.lineWidth=2.4;c.beginPath();series.forEach((d,i)=>i?c.lineTo(x(d.time),y(d[key])):c.moveTo(x(d.time),y(d[key])));c.stroke();
}

function setFlowMark(element, value, maximum, vertical=false) {
  const normalized = Math.max(0, Math.min(1, value / Math.max(maximum, 1e-9)));
  if (vertical) element.style.transform = `scaleY(${.08 + .92*normalized})`;
  else element.style.transform = `scaleX(${.08 + .92*normalized})`;
  element.style.opacity = String(.25 + .75*normalized);
  element.style.width = vertical ? `${3 + 11*normalized}px` : element.style.width;
  element.style.height = vertical ? element.style.height : `${3 + 11*normalized}px`;
}

function animationFrame() {
  if (!latest) return null;
  return latest.run.frames[Number($('animationTime').value)];
}

function updateAnimation() {
  const d = animationFrame();
  if (!d) return;
  const {p, heightScale} = latest;
  const waterPercent = Math.max(0, Math.min(100, d.height/heightScale*100));
  $('animatedWater').style.height = `${waterPercent}%`;
  $('animatedTarget').style.bottom = `${Math.max(0,Math.min(100,p.target/heightScale*100))}%`;
  $('processTank').style.width = `${Math.max(115,Math.min(230,170*Math.sqrt(p.area/2)))}px`;
  $('scaleTop').textContent = `${heightScale.toFixed(1)} m`;
  setFlowMark($('inletStream'),d.flow,p.maxFlow,true);
  setFlowMark($('outletStream'),Math.max(0,d.outlet),p.maxFlow,false);
  setFlowMark($('leakStream'),d.leak,p.maxFlow,true);
  $('pumpLabel').textContent = `u = ${d.flow.toFixed(4)} m³/s`;
  $('outletLabel').textContent = `k·h = ${d.outlet.toFixed(4)} m³/s`;
  $('leakLabel').textContent = `q_leak = ${d.leak.toFixed(4)} m³/s`;
  $('animatedHeight').textContent = `h = ${d.height.toFixed(3)} m`;
  const trend = d.rate > 1e-6 ? '↑ rising' : d.rate < -1e-6 ? '↓ falling' : '≈ steady';
  $('levelTrend').textContent = trend;
  $('modelWarning').textContent = d.height < 0 ? 'computed below 0 m · ideal model is outside its physical range' : '';
  $('liveError').textContent = `e = r − h = ${d.error.toFixed(3)} m`;
  $('liveCommand').textContent = `u = clip(${p.bias.toFixed(3)} + ${d.pTerm.toFixed(4)}) = ${d.flow.toFixed(4)} m³/s`;
  $('liveRate').textContent = `dh/dt = (${d.flow.toFixed(4)} − ${d.outlet.toFixed(4)} − ${d.leak.toFixed(4)}) / ${p.area.toFixed(1)} = ${d.rate.toFixed(6)} m/s`;
  $('balanceIn').textContent = `IN ${d.flow.toFixed(4)}`;
  $('balanceOut').textContent = `OUT ${(d.outlet+d.leak).toFixed(4)} m³/s`;
  $('balanceRelation').textContent = d.rate > 1e-6 ? '>' : d.rate < -1e-6 ? '<' : '≈';
  $('animationClock').textContent = `${d.time.toFixed(1)} s`;
}

function render() {
  const p = parameters();
  if (p.disturbanceAt > p.duration) { $('disturbanceAt').value=p.duration; p.disturbanceAt=p.duration; }
  ids.forEach(id=>{const digits=['area','initial','target'].includes(id)?1:['disturbanceAt','duration'].includes(id)?0:3;document.querySelector(`[data-for="${id}"]`).textContent=Number($(id).value).toFixed(digits)+units[id]});
  const run = simulate(p);
  let maxHeight=p.target;for(const d of run.frames)maxHeight=Math.max(maxHeight,Math.max(0,d.height));
  latest = {p,run,heightScale:Math.max(1,Math.ceil(maxHeight*1.1*2)/2)};
  draw($('heightChart'),run.frames,'height',p,true);
  draw($('flowChart'),run.frames,'flow',p,false);
  $('animationTime').max = String(run.frames.length-1);
  $('animationTime').value = '0';
  const end=run.final,error=p.target-end.height;
  $('finalHeight').textContent=`${end.height.toFixed(3)} m`;
  $('finalError').textContent=`${error>=0?'+':''}${error.toFixed(3)} m`;
  $('finalFlow').textContent=`${end.flow.toFixed(4)} m³/s`;
  $('finalLoss').textContent=`${(end.outlet+end.leak).toFixed(4)} m³/s`;
  $('explanation').textContent=`The added leak requires ${p.extraLeak.toFixed(3)} m³/s more inlet flow. P creates that correction only through Kp × error, so it balances the actual losses at ${end.height.toFixed(3)} m rather than the ${p.target.toFixed(1)} m reference.`;
  updateAnimation();
}

function animate(now) {
  if (playing && latest) {
    const elapsed = now-lastFrameTime;
    const count = latest.run.frames.length;
    let next = Number($('animationTime').value) + elapsed/18000*count;
    if (next >= count-1) next = 0;
    $('animationTime').value = String(Math.floor(next));
    updateAnimation();
  }
  lastFrameTime=now;
  requestAnimationFrame(animate);
}

let renderRequest;
function schedule(){cancelAnimationFrame(renderRequest);renderRequest=requestAnimationFrame(render)}
ids.forEach(id=>$(id).addEventListener('input',schedule));
$('animationTime').addEventListener('input',()=>{playing=false;$('playAnimation').textContent='Play';updateAnimation()});
$('playAnimation').onclick=()=>{playing=!playing;$('playAnimation').textContent=playing?'Pause':'Play'};
$('reset').onclick=()=>{ids.forEach(id=>$(id).value=defaults[id]);playing=true;$('playAnimation').textContent='Pause';render()};
window.addEventListener('resize',schedule);
render();requestAnimationFrame(animate);
