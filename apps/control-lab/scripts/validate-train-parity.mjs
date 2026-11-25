import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';

const root = resolve(import.meta.dirname, '..');
await import(resolve(root, 'dist/train-model.js'));
const fixture = JSON.parse(readFileSync(resolve(root, 'dist/assets/train-control/python-fixture.json'), 'utf8'));
const {plant,controller,scenario} = fixture;
const params = {
  mode: controller.mode, mass: plant.mass, rollingResistance: plant.rolling_resistance,
  velocityResistance: plant.velocity_resistance, tractionMax: plant.traction_max,
  brakeMax: plant.brake_max, fixedForce: controller.fixed_force,
  speedTarget: controller.speed_target, positionTarget: controller.position_target,
  kpSpeed: controller.kp_speed, kiSpeed: controller.ki_speed,
  kpPosition: controller.kp_position, kiPosition: controller.ki_position,
  kdPosition: controller.kd_position, nominalDisturbance: controller.nominal_disturbance,
  duration: scenario.duration, dt: scenario.dt, initialPosition: scenario.initial_position,
  initialSpeed: scenario.initial_speed, initialIntegral: scenario.initial_integral,
  disturbance: scenario.disturbance, disturbanceAt: scenario.disturbance_at,
  availableDeceleration: scenario.available_deceleration, safetyMargin: scenario.safety_margin,
  responseDelay: scenario.response_delay,
};
const run = globalThis.TrainModel.simulate(params);
const fields = {x_m:'x',v_mps:'v',z_integral:'z',force_requested_N:'request',force_applied_N:'applied',a_mps2:'acceleration'};
for (const sample of fixture.samples) {
  const actual=run.frames[sample.index];
  for (const [pythonField,browserField] of Object.entries(fields)) {
    const error=Math.abs(actual[browserField]-sample[pythonField]);
    if (error>fixture.tolerance) throw new Error(`Train parity failed at step ${sample.index}, ${pythonField}: error ${error}`);
  }
}
console.log(`Validated train browser parity at ${fixture.samples.length} Python fixture samples.`);
