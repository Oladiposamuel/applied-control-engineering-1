(function (root) {
  'use strict';

  const base = {
    mode: 'open_loop', mass: 20000, rollingResistance: 2000, velocityResistance: 500,
    initialPosition: 0, initialSpeed: 20, initialIntegral: 0, fixedForce: -10000,
    speedTarget: 25, positionTarget: 500, kpSpeed: 1000, kiSpeed: 100,
    kpPosition: 100, kiPosition: 5, kdPosition: 800, tractionMax: 40000, brakeMax: 40000,
    disturbance: 0, disturbanceAt: 0, nominalDisturbance: 0, responseDelay: 0,
    duration: 3, dt: 1, availableDeceleration: 1, safetyMargin: 0,
  };
  const ideal = { rollingResistance: 0, velocityResistance: 0 };
  const station = { ...ideal, initialPosition: 400, duration: 36, dt: 0.2 };
  const speed = { initialSpeed: 25, disturbance: 3000, duration: 180, dt: 0.2 };
  const stop = { ...station, duration: 20, dt: 1 };
  const source = name => 'assets/train-control/source-notes/' + name;
  const sheets = {
    braking: source('08_braking_three_euler_steps.png'),
    speed: source('11_p_controller_steady_state_offset.png'),
    pd: source('14_pd_position_control.png'),
    windup: source('17_pid_integral_windup.png'),
    notes: source('handwriting_contact_sheet.jpg'),
  };

  function example(id, title, description, notes, overrides, reference = false) {
    return { id, title, description, notes, p: { ...base, ...overrides }, reference };
  }

  const groups = [
    { id: 'motion', title: 'Force & motion', examples: [
      example('euler-braking', 'Braking · three Euler steps', 'Start at 0 m and 20 m/s. Apply −10 kN of braking and use Step to reproduce each 1 s calculation. Initial net force is −22 kN, giving −1.10 m/s².', sheets.braking, {}),
      example('euler-braking-100', 'Braking · start at 100 m', 'Same braking force and initial speed, but the position starts at 100 m. One second later the train is at 120 m and 18.90 m/s.', sheets.notes, { initialPosition: 100, duration: 1 }),
      example('euler-traction', 'Traction · first Euler step', 'Apply 30 kN starting at 10 m/s. Net force is 23 kN and acceleration is +1.15 m/s². The first 2 s step reaches 20 m and 12.30 m/s.', sheets.notes, { fixedForce: 30000, initialSpeed: 10, duration: 2, dt: 2 }),
      example('traction-equilibrium', 'Traction · approach equilibrium', 'With constant 30 kN traction the force balance gives 56 m/s. Starting above it at 60 m/s gives 32 kN of resistance and −0.10 m/s², despite the motor pulling forward.', sheets.notes, { fixedForce: 30000, initialSpeed: 60, duration: 240, dt: 1 }),
    ] },
    { id: 'speed', title: 'Speed P / PI', examples: [
      example('speed-p-nominal', 'Speed P · initial force', 'With a 25 m/s target and 15 m/s initial speed, the nominal bias is 14.5 kN and the proportional term adds 10 kN.', sheets.notes, { mode: 'speed_p', initialSpeed: 15, duration: 60, dt: 0.2 }),
      example('speed-p-offset', 'Speed P · disturbance and offset', 'Apply 3 kN extra resistance at the 25 m/s target. Initial acceleration is −0.15 m/s². The analytical equilibrium is 23 m/s with 16.5 kN applied.', sheets.speed, { ...speed, mode: 'speed_p' }),
      example('speed-pi-recovery', 'Speed PI · recovery after a disturbance', 'Apply the same 3 kN disturbance at 25 m/s. The integral term builds up and restores the target, with analytical equilibrium memory of 30 m.', sheets.notes, { ...speed, mode: 'speed_pi' }),
      example('speed-pi-equilibrium', 'Speed PI · exact equilibrium check', 'Start at 25 m/s with 30 m of integral memory and 3 kN of extra resistance. The applied force is 17.5 kN and the net force is zero.', sheets.notes, { ...speed, mode: 'speed_pi', initialIntegral: 30, duration: 20 }),
    ] },
    { id: 'station', title: 'Position P / PD / PID', examples: [
      example('station-p', 'Position P · 100 m to go', 'This ideal calculation omits passive resistance. Starting at 400 m and 20 m/s, P asks for +10 kN, giving +0.50 m/s² even though the station is close.', sheets.notes, { ...station, mode: 'position_p' }),
      example('station-pd', 'Position PD · braking before the mark', 'This ideal calculation omits passive resistance. At 400 m and 20 m/s, +10 kN from position and −16 kN from velocity give −6 kN and −0.30 m/s².', sheets.pd, { ...station, mode: 'position_pd' }),
      example('station-pid', 'Position PID · stored error', 'Use the same ideal approach with 200 m·s of initial memory. The +1 kN integral term brings the request to −5 kN and acceleration to −0.25 m/s².', sheets.notes, { ...station, mode: 'position_pid', initialIntegral: 200 }),
      example('station-crossing', 'Position PID · crossing snapshot', 'Start exactly at 500 m, moving at 5 m/s with 200 m·s of memory. Position error is zero, but +1 kN integral and −4 kN derivative still request −3 kN.', sheets.notes, { ...station, mode: 'position_pid', initialPosition: 500, initialSpeed: 5, initialIntegral: 200, duration: 10 }),
      example('windup', 'PID · one second of windup', 'From rest at 0 m the controller asks for 50 kN, but only 40 kN can be applied. After one 1 s update, memory is 500 m·s and contributes +2.5 kN.', sheets.windup, { mode: 'position_pid', initialSpeed: 0 }),
      example('anti-windup', 'PID · repeat with anti-windup', 'The same initial request is clipped to 40 kN. Conditional integration keeps the first memory update at zero instead of adding 500 m·s.', sheets.windup, { mode: 'position_pid_aw', initialSpeed: 0 }),
    ] },
    { id: 'stop', title: 'Stopping distance', examples: [
      example('stop-1', 'Stopping check · 1 m/s²', 'The handwritten constant-deceleration estimate needs 200 m but the station is only 100 m ahead. The exact reference passes the station and stops at 600 m.', sheets.notes, { ...stop, fixedForce: -20000 }, true),
      example('stop-2', 'Stopping check · 2 m/s²', 'The same 20 m/s approach needs 100 m when net deceleration is 2 m/s². The exact reference stops at the 500 m mark.', sheets.notes, { ...stop, fixedForce: -40000, availableDeceleration: 2, duration: 10 }, true),
      example('supervisor', 'Compare · PID with brake supervisor', 'This is a separate simulation, not the constant-deceleration hand calculation. It uses rolling and speed-dependent resistance and can override the signed PID request.', sheets.notes, { mode: 'position_pid_supervisor', initialPosition: 400, duration: 60, dt: 0.1 }),
    ] },
  ];

  root.TrainExamples = { groups, examples: groups.flatMap(group => group.examples) };
})(globalThis);
