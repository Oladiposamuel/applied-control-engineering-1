import assert from 'node:assert/strict';
import '../dist/train-model.js';
import '../dist/train-examples.js';

const { examples } = globalThis.TrainExamples;
const { simulate, simulateConstantStop } = globalThis.TrainModel;
const runs = new Map(examples.map(example => [
  example.id, (example.reference ? simulateConstantStop : simulate)(example.p),
]));
assert.equal(runs.size, examples.length, 'Example IDs must be unique');
const near = (actual, expected) => assert.ok(Math.abs(actual - expected) < 1e-8, `${actual} ≠ ${expected}`);
const at = (id, i = 0) => runs.get(id).frames[i];

for (const run of runs.values()) {
  const { frames, p } = run;
  assert.equal(frames.length, Math.round(p.duration / p.dt) + 1);
  for (const [i, f] of frames.entries()) {
    near(f.time, i * p.dt);
    assert.ok(f.v >= 0 && (!i || f.x >= frames[i - 1].x));
  }
}

const braking = runs.get('euler-braking').frames;
near(braking[0].net, -22000);
near(braking[0].acceleration, -1.1);
[[0, 20], [20, 18.9], [38.9, 17.8275], [56.7275, 16.7818125]].forEach(([x, v], i) => {
  near(braking[i].x, x);
  near(braking[i].v, v);
});
near(at('euler-braking-100', 1).x, 120);
near(at('euler-traction').acceleration, 1.15);
near(at('euler-traction', 1).x, 20);
near(at('euler-traction', 1).v, 12.3);
near(at('traction-equilibrium').net, -2000);
const cruise = runs.get('traction-equilibrium').p;
near((cruise.fixedForce - cruise.rollingResistance) / cruise.velocityResistance, 56);
near(at('speed-p-nominal').control.bias, 14500);
near(at('speed-p-nominal').applied, 24500);
near(at('speed-p-offset').acceleration, -.15);
assert.ok(Math.abs(runs.get('speed-p-offset').final.v - 23) < 1e-4);
assert.ok(Math.abs(runs.get('speed-pi-recovery').final.v - 25) < .02);
near(at('speed-pi-equilibrium').z, 30);
near(at('speed-pi-equilibrium').applied, 17500);
near(runs.get('speed-pi-equilibrium').final.v, 25);
for (const [id, force, acceleration] of [
  ['station-p', 10000, .5], ['station-pd', -6000, -.3], ['station-pid', -5000, -.25],
  ['station-crossing', -3000, -.15],
]) {
  near(at(id).control.raw, force);
  near(at(id).acceleration, acceleration);
}
for (const [id, memory, integralForce] of [['windup', 500, 2500], ['anti-windup', 0, 0]]) {
  near(at(id).control.raw, 50000);
  near(at(id).applied, 40000);
  near(at(id, 1).z, memory);
  near(at(id, 1).control.integral, integralForce);
}
for (const [id, x, d] of [['stop-1', 600, 200], ['stop-2', 500, 100]]) {
  near(at(id).feasibility.remaining, 100);
  near(at(id).feasibility.requiredDeceleration, 2);
  near(at(id).feasibility.stopDistance, d);
  near(runs.get(id).final.x, x);
  near(runs.get(id).final.v, 0);
}

console.log(`Validated ${runs.size} selectable train examples against the handwritten anchors.`);
