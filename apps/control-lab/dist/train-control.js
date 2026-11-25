const $ = id => document.getElementById(id);
const { groups, examples } = TrainExamples;
const units = {
  fixedForce: 'N', mass: 'kg', rollingResistance: 'N', velocityResistance: 'N·s/m',
  initialPosition: 'm', initialSpeed: 'm/s', speedTarget: 'm/s', positionTarget: 'm',
  initialIntegral: '', kpSpeed: 'N·s/m', kiSpeed: 'N/m', kpPosition: 'N/m',
  kiPosition: 'N/(m·s)', kdPosition: 'N·s/m', tractionMax: 'N', brakeMax: 'N',
  disturbance: 'N', disturbanceAt: 's', duration: 's', dt: 's',
  availableDeceleration: 'm/s²', safetyMargin: 'm',
};
const inputs = Object.keys(units);
const groupButtons = new Map();
const lastByGroup = new Map();
let selected = examples[0], run, rail, reference, edited, frameIndex, sampleRows, forceScale;
let playing = false, elapsed = 0, lastTick = null, pendingRender;

const fmt = (value, digits = 2) => (Math.abs(value) < .5 * 10 ** -digits ? 0 : value).toFixed(digits).replace('-', '−');
const kn = value => fmt(value / 1000) + ' kN';
const signed = (value, digits) => (value >= .5 * 10 ** -digits ? '+' : '') + fmt(value, digits);
const integralMode = mode => mode === 'speed_pi' || mode.startsWith('position_pid');
const stationMode = mode => mode.startsWith('position_');

function parameters() {
  const p = { ...selected.p, mode: $('mode').value };
  for (const id of inputs) p[id] = Number($(id).value);
  // All other step choices divide any whole-second duration.
  if (p.dt === 2 && p.duration % 2) $('duration').value = p.duration = p.duration + 1;
  return p;
}

function labelInputs(p) {
  for (const id of inputs) {
    const unit = id === 'initialIntegral' ? (p.mode === 'speed_pi' ? 'm' : 'm·s') : units[id];
    document.querySelector('[data-for="' + id + '"]').textContent =
      p[id].toLocaleString(undefined, { maximumFractionDigits: 2 }) + (unit ? ' ' + unit : '');
  }
  for (const id of inputs) {
    const relevant = {
      fixedForce: p.mode === 'open_loop',
      speedTarget: p.mode.startsWith('speed_'), kpSpeed: p.mode.startsWith('speed_'),
      kiSpeed: p.mode === 'speed_pi', initialIntegral: integralMode(p.mode),
      kpPosition: stationMode(p.mode), kiPosition: p.mode.startsWith('position_pid'),
      kdPosition: stationMode(p.mode) && p.mode !== 'position_p',
      positionTarget: stationMode(p.mode) || reference,
      availableDeceleration: stationMode(p.mode) || reference, safetyMargin: stationMode(p.mode) && !reference,
    }[id] ?? true;
    $(id).previousElementSibling.hidden = $(id).hidden = !relevant;
  }
}

function selectExample(example) {
  cancelAnimationFrame(pendingRender);
  selected = example;
  edited = false;
  const group = groups.find(item => item.examples.includes(example));
  lastByGroup.set(group, example);
  for (const [item, button] of groupButtons) button.setAttribute('aria-pressed', String(item === group));
  $('exampleChoice').replaceChildren(...group.examples.map(item => new Option(item.title, item.id)));
  $('exampleChoice').value = example.id;
  const index = examples.indexOf(example);
  $('previousExample').disabled = index === 0;
  $('nextExample').disabled = index === examples.length - 1;
  $('exampleNotes').href = example.notes;
  $('mode').value = example.p.mode;
  for (const id of inputs) $(id).value = example.p[id];
  $('playbackSpeed').value = example.p.duration <= 20 ? '1' : example.p.duration <= 60 ? '5' : '20';
  render();
}

function play(active) {
  playing = active;
  lastTick = null;
  $('playAnimation').textContent = active ? 'Pause' : frameIndex === run.frames.length - 1 ? 'Replay' : 'Play';
}

function chart(canvas, series, target) {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (!w || !h) return;
  const ratio = devicePixelRatio || 1, end = run.final.time;
  canvas.width = w * ratio;
  canvas.height = h * ratio;
  const ctx = canvas.getContext('2d');
  ctx.scale(ratio, ratio);
  const pad = { l: 49, r: 19, t: 12, b: 25 };
  const data = run.frames.filter((_, i) => i % Math.ceil(run.frames.length / 1500) === 0);
  if (data.at(-1) !== run.final) data.push(run.final);
  const values = data.flatMap(frame => series.map(s => s.value(frame)));
  if (target !== undefined) values.push(target);
  const min = Math.min(...values), max = Math.max(...values), gap = (max - min || 1) * .12;
  const low = min - gap, high = max + gap;
  const x = t => pad.l + t / end * (w - pad.l - pad.r);
  const y = v => h - pad.b - (v - low) / (high - low) * (h - pad.t - pad.b);
  ctx.font = '10px ui-monospace,monospace';
  ctx.fillStyle = '#5d6872';
  const line = (color, a, b) => {
    ctx.strokeStyle = color;
    ctx.beginPath();
    ctx.moveTo(...a);
    ctx.lineTo(...b);
    ctx.stroke();
  };
  for (let i = 0; i <= 4; i++) {
    const t = end * i / 4, v = low + (high - low) * i / 4;
    line('#dedbd1', [pad.l, y(v)], [w - pad.r, y(v)]);
    ctx.fillText(Math.abs(v) >= 1000 ? fmt(v / 1000, 1) + 'k' : fmt(v, 1), 4, y(v) + 3);
    ctx.fillText(fmt(t, end < 10 ? 2 : end % 4 ? 1 : 0) + 's', x(t) - 12, h - 5);
  }
  if (target !== undefined) {
    ctx.setLineDash([6, 5]);
    line('#d87821', [pad.l, y(target)], [w - pad.r, y(target)]);
  }
  for (const s of series) {
    ctx.strokeStyle = s.color;
    ctx.lineWidth = 2;
    ctx.setLineDash(s.dash ?? []);
    ctx.beginPath();
    data.forEach((frame, i) => i ? ctx.lineTo(x(frame.time), y(s.value(frame))) : ctx.moveTo(x(frame.time), y(s.value(frame))));
    ctx.stroke();
  }
}

function drawPlots() {
  if (!run || !$('plots').open) return;
  const p = run.p;
  chart($('positionChart'), [{ value: f => f.x, color: '#087f83' }], stationMode(p.mode) || reference ? p.positionTarget : undefined);
  chart($('speedChart'), [{ value: f => f.v, color: '#2479bf' }], p.mode.startsWith('speed_') ? p.speedTarget : undefined);
  chart($('forceChart'), [{ value: f => f.request / 1000, color: '#d87821', dash: [5, 4] }, { value: f => f.applied / 1000, color: '#10263f' }]);
}

function requestText(f, p) {
  const c = f.control;
  const terms = p.mode === 'open_loop' ? [['Fixed', c.raw]] :
    p.mode.startsWith('speed_') ? [['Bias', c.bias], ['P', c.proportional]] : [['P', c.proportional]];
  if (integralMode(p.mode)) terms.push(['I', c.integral]);
  if (stationMode(p.mode) && p.mode !== 'position_p') terms.push(['D', c.derivative]);
  let text = terms.map(([name, force], i) => (i === 0 ? name + ' ' + kn(force) :
    (force < 0 ? ' − ' : ' + ') + name + ' ' + kn(Math.abs(force)))).join('');
  if (terms.length > 1) text += ' = ' + kn(c.raw);
  if (f.supervisor) text += ' · supervisor set ' + kn(f.applied);
  else if (f.saturated) text += ' · limited to ' + kn(f.applied);
  return text;
}

function liveDetail(f, next, p) {
  if (reference || p.mode === 'position_pid_supervisor') {
    const { remaining, stopDistance, requiredDeceleration, feasible } = f.feasibility;
    $('detailLabel').textContent = 'STOPPING CHECK';
    if (f.v <= 1e-12) return Math.abs(remaining) <= .005 ? 'Stopped at the station' :
      'Stopped ' + fmt(Math.abs(remaining)) + ' m ' + (remaining < 0 ? 'past' : 'before') + ' the station';
    if (remaining <= 0) return (remaining < 0 ? fmt(-remaining) + ' m past the station' : 'At the station') + ' · still moving';
    return fmt(remaining) + ' m left · ' + fmt(stopDistance) + ' m to stop' +
      (p.safetyMargin ? ' + ' + fmt(p.safetyMargin) + ' m buffer' : '') + ' · ' +
      (feasible ? 'fits' : 'does not fit') + ' at ' + fmt(p.availableDeceleration) +
      ' m/s² · need ' + fmt(requiredDeceleration) + ' m/s²';
  }
  if (integralMode(p.mode) && (selected.id.includes('windup') || p.mode === 'speed_pi')) {
    const unit = p.mode === 'speed_pi' ? 'm' : 'm·s';
    const gain = p.mode === 'speed_pi' ? p.kiSpeed : p.kiPosition;
    $('detailLabel').textContent = 'INTEGRAL MEMORY';
    return 'z = ' + fmt(f.z) + ' ' + unit + (next ? ' → ' + fmt(next.z) + ' ' + unit +
      ' · next I = ' + kn(gain * next.z) : ' · I = ' + kn(gain * f.z)) +
      (f.antiWindup && next ? ' · held by anti-windup' : '');
  }
  $('detailLabel').textContent = 'NEXT EULER STEP';
  if (!next) return 'End of the selected calculation';
  if (f.stopEvent) return 'Stop within this step at x = ' + fmt(next.x) + ' m · v = 0.00 m/s';
  return 'x → ' + fmt(next.x) + ' m · v → ' + fmt(next.v) + ' m/s';
}

function arrow(element, force) {
  element.hidden = Math.abs(force) < 1e-8;
  element.classList.toggle('points-left', force < 0);
  element.style.setProperty('--arrow-width', Math.max(1.4, 10 * Math.abs(force) / forceScale) + '%');
}

function showFrame(index) {
  frameIndex = index;
  const f = run.frames[index], next = run.frames[index + 1], p = run.p;
  const resistance = f.standstillConstraint ? 0 : f.rolling + f.velocity + f.disturbance;
  $('animationTime').value = index;
  $('animationTime').setAttribute('aria-valuetext', fmt(f.time) + ' seconds');
  $('animationClock').textContent = fmt(f.time) + ' s';
  $('stepAnimation').disabled = !next;
  $('animatedPosition').textContent = 'x = ' + fmt(f.x) + ' m';
  $('animatedSpeed').textContent = 'v = ' + fmt(f.v) + ' m/s';
  $('motionState').textContent = f.v < 1e-8 ? (f.acceleration > 1e-8 ? 'Starting forward' : 'Stopped') :
    f.acceleration > 1e-8 ? 'Accelerating' : f.acceleration < -1e-8 ? 'Decelerating' : 'Constant speed';
  const stage = $('trainStage');
  stage.style.setProperty('--train-left', rail.map(f.x) + '%');
  $('plots').style.setProperty('--cursor-time', String(f.time / run.final.time));
  arrow($('actuatorArrow'), f.applied);
  arrow($('resistanceArrow'), -resistance);
  $('actuatorKey').textContent = f.applied === 0 ? 'Actuator off' : (f.applied < 0 ? 'Brake ' : 'Traction ') + kn(Math.abs(f.applied));
  $('actuatorKey').dataset.braking = f.applied < 0;
  $('resistanceKey').textContent = f.standstillConstraint ? 'Standstill constraint' : 'Resistance ' + kn(resistance);
  $('liveControl').textContent = requestText(f, p);
  $('liveDynamics').textContent = f.standstillConstraint ? 'Held at rest · a = 0.00 m/s²' :
    kn(f.applied) + (resistance ? ' − ' + kn(resistance) : '') + ' → ' + kn(f.net) +
    ' net · a = ' + signed(f.acceleration, 2) + ' m/s²';
  $('liveDetail').textContent = liveDetail(f, next, p);
  sampleRows?.forEach((row, i) => row.classList.toggle('current-sample', i === index));
}

function render(keepTime = false) {
  const time = keepTime ? run.frames[frameIndex].time : 0;
  const wasPlaying = keepTime && playing;
  reference = selected.reference && !edited;
  const p = parameters();
  labelInputs(p);
  run = reference ? TrainModel.simulateConstantStop(p) : TrainModel.simulate(p);
  const showStation = reference || stationMode(p.mode);
  const min = Math.min(p.initialPosition, showStation ? p.positionTarget : p.initialPosition);
  let max = Math.max(run.final.x, showStation ? p.positionTarget : p.initialPosition);
  max = Math.max(max, min + 1);
  rail = { map: x => 14 + 72 * (x - min) / (max - min) };
  forceScale = Math.max(1, ...run.frames.map(f => Math.max(Math.abs(f.applied), f.rolling + f.velocity + f.disturbance)));
  $('scaleStart').textContent = fmt(min) + ' m';
  $('scaleEnd').textContent = fmt(max) + ' m';
  $('station').hidden = !showStation;
  $('trainStage').style.setProperty('--station-left', rail.map(p.positionTarget) + '%');
  $('stationLabel').textContent = fmt(p.positionTarget, 0) + ' m';
  $('animationTime').max = run.frames.length - 1;
  $('solverLabel').textContent = reference ? 'EXACT KINEMATIC REFERENCE' : 'FORWARD EULER · Δt = ' + fmt(p.dt) + ' s';
  $('solverExplanation').textContent = reference ?
    'The reference holds net deceleration constant until zero speed. It is separate from the resistance-based train simulation.' :
    'Each step uses the current speed to update position and the current net force to update speed.';
  $('exampleDescription').textContent = edited ? 'Custom run from “' + selected.title +
    '”. Reset the example to restore the handwritten inputs.' : selected.description;
  const rows = $('sampleRows');
  rows.replaceChildren();
  $('sampleTable').hidden = run.frames.length > 8;
  sampleRows = $('sampleTable').hidden ? null : run.frames.map(f => {
    const row = rows.insertRow();
    for (const value of [f.time, f.x, f.v]) row.insertCell().textContent = fmt(value);
    return row;
  });
  const index = Math.min(run.frames.length - 1, Math.round(time / p.dt));
  elapsed = run.frames[index].time;
  showFrame(index);
  play(wasPlaying && index < run.frames.length - 1);
  drawPlots();
}

function tick(now) {
  if (playing) {
    if (lastTick !== null) elapsed += Math.min(100, now - lastTick) / 1000 * Number($('playbackSpeed').value);
    lastTick = now;
    const last = run.frames.length - 1;
    const index = Math.min(last, Math.floor((elapsed + 1e-9) / run.p.dt));
    if (index !== frameIndex) showFrame(index);
    if (index === last) play(false);
  }
  requestAnimationFrame(tick);
}

for (const group of groups) {
  const button = document.createElement('button');
  button.type = 'button';
  button.textContent = group.title;
  button.addEventListener('click', () => selectExample(lastByGroup.get(group) ?? group.examples[0]));
  $('exampleGroups').append(button);
  groupButtons.set(group, button);
}
$('exampleChoice').addEventListener('change', event => selectExample(examples.find(item => item.id === event.target.value)));
$('previousExample').onclick = () => selectExample(examples[examples.indexOf(selected) - 1]);
$('nextExample').onclick = () => selectExample(examples[examples.indexOf(selected) + 1]);
$('reset').onclick = () => selectExample(selected);
function seek(index) {
  showFrame(index);
  elapsed = run.frames[index].time;
  play(false);
}
$('animationTime').addEventListener('input', event => seek(Number(event.target.value)));
$('rewindAnimation').onclick = () => seek(0);
$('stepAnimation').onclick = () => seek(Math.min(frameIndex + 1, run.frames.length - 1));
$('playAnimation').onclick = () => {
  if (playing) return play(false);
  if (frameIndex === run.frames.length - 1) seek(0);
  play(true);
};
function edit() {
  edited = true;
  cancelAnimationFrame(pendingRender);
  pendingRender = requestAnimationFrame(() => render(true));
}
for (const id of inputs) $(id).addEventListener('input', edit);
$('mode').addEventListener('change', edit);
$('plots').addEventListener('toggle', drawPlots);
window.addEventListener('resize', drawPlots);
selectExample(selected);
requestAnimationFrame(tick);
