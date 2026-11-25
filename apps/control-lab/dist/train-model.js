(function (root) {
  'use strict';
  const MODES = new Set(['open_loop','speed_p','speed_pi','position_p','position_pd','position_pid','position_pid_aw','position_pid_supervisor']);
  const clamp = (value, low, high) => Math.min(high, Math.max(low, value));

  function validate(p) {
    const positive = ['mass','tractionMax','brakeMax','dt','duration','availableDeceleration'];
    const nonnegative = ['rollingResistance','velocityResistance','initialSpeed','disturbance','disturbanceAt','safetyMargin','responseDelay','kpSpeed','kiSpeed','kpPosition','kiPosition','kdPosition'];
    if (!MODES.has(p.mode)) throw new Error('Unknown controller mode');
    for (const key of positive) if (!(Number.isFinite(p[key]) && p[key] > 0)) throw new Error(`${key} must be positive`);
    for (const key of nonnegative) if (!(Number.isFinite(p[key]) && p[key] >= 0)) throw new Error(`${key} must be nonnegative`);
    if (Math.abs(p.duration / p.dt - Math.round(p.duration / p.dt)) > 1e-8) throw new Error('dt must divide duration');
  }

  function controller(state, z, p) {
    const speedError = p.speedTarget - state.v;
    const positionError = p.positionTarget - state.x;
    let bias = 0, proportional = 0, integral = 0, derivative = 0, integralRate = 0, raw;
    if (p.mode === 'open_loop') raw = p.fixedForce;
    else if (p.mode.startsWith('speed_')) {
      bias = p.rollingResistance + p.velocityResistance * p.speedTarget + p.nominalDisturbance;
      proportional = p.kpSpeed * speedError;
      if (p.mode === 'speed_pi') { integral = p.kiSpeed * z; integralRate = speedError; }
      raw = bias + proportional + integral;
    } else {
      proportional = p.kpPosition * positionError;
      if (['position_pd','position_pid','position_pid_aw','position_pid_supervisor'].includes(p.mode)) derivative = -p.kdPosition * state.v;
      if (['position_pid','position_pid_aw','position_pid_supervisor'].includes(p.mode)) { integral = p.kiPosition * z; integralRate = positionError; }
      raw = proportional + integral + derivative;
    }
    const applied = clamp(raw, -p.brakeMax, p.tractionMax);
    const saturated = Math.abs(applied - raw) > 1e-12;
    let antiWindup = false;
    const conditional = ['speed_pi','position_pid_aw','position_pid_supervisor'].includes(p.mode);
    const gain = p.mode === 'speed_pi' ? p.kiSpeed : p.kiPosition;
    if (conditional && gain !== 0 && ((raw >= p.tractionMax && integralRate > 0) || (raw <= -p.brakeMax && integralRate < 0))) {
      integralRate = 0; antiWindup = true;
    }
    return {speedError,positionError,bias,proportional,integral,derivative,raw,applied,saturated,integralRate,antiWindup};
  }

  function feasibility(state, p) {
    const remaining = p.positionTarget - state.x;
    const stopDistance = state.v * state.v / (2 * p.availableDeceleration) + state.v * p.responseDelay;
    const requiredDeceleration = remaining > 0 ? state.v * state.v / (2 * remaining) : (state.v === 0 ? 0 : null);
    const feasible = state.v === 0 || (remaining > 0 && remaining >= stopDistance + p.safetyMargin);
    return {remaining,stopDistance,requiredDeceleration,margin:remaining-stopDistance,feasible};
  }

  function forceBalance(speed, requested, disturbance, p) {
    const applied = clamp(requested, -p.brakeMax, p.tractionMax);
    const saturated = Math.abs(applied-requested)>1e-12;
    const traction = Math.max(0,applied), braking = Math.max(0,-applied);
    if (speed <= 1e-12) {
      const threshold=p.rollingResistance+disturbance;
      if (applied<=threshold) return {requested,applied,traction,braking,rolling:0,velocity:0,disturbance,net:0,acceleration:0,saturated,standstillConstraint:true};
      const net=applied-p.rollingResistance-disturbance;
      return {requested,applied,traction,braking,rolling:p.rollingResistance,velocity:0,disturbance,net,acceleration:net/p.mass,saturated,standstillConstraint:false};
    }
    const rolling=p.rollingResistance, velocity=p.velocityResistance*speed;
    const net=applied-rolling-velocity-disturbance;
    return {requested,applied,traction,braking,rolling,velocity,disturbance,net,acceleration:net/p.mass,saturated,standstillConstraint:false};
  }

  function stopEvent(state, forces, disturbance, dt, p) {
    const base=forces.applied-p.rollingResistance-disturbance, c=p.velocityResistance;
    if (c>0) {
      const asymptote=base/c, denominator=state.v-asymptote, ratio=denominator!==0?-asymptote/denominator:-1;
      if (ratio>0&&ratio<=1) {
        const eventTime=-Math.log(ratio)*p.mass/c, rate=c/p.mass;
        const distance=asymptote*eventTime+(state.v-asymptote)*(1-Math.exp(-rate*eventTime))/rate;
        return {time:clamp(eventTime,0,dt),distance:Math.max(0,distance)};
      }
    }
    const eventTime=forces.acceleration<0?clamp(-state.v/forces.acceleration,0,dt):dt;
    return {time:eventTime,distance:Math.max(0,state.v*eventTime+.5*forces.acceleration*eventTime*eventTime)};
  }

  function step(state, requested, disturbance, p) {
    const forces=forceBalance(state.v,requested,disturbance,p), predicted=state.v+p.dt*forces.acceleration;
    if (state.v>0&&predicted<0) {
      const event=stopEvent(state,forces,disturbance,p.dt,p);
      return {state:{x:state.x+event.distance,v:0},forces,stopEvent:true,stopTimeOffset:event.time,distance:event.distance};
    }
    return {state:{x:state.x+p.dt*state.v,v:Math.max(0,predicted)},forces,stopEvent:false,stopTimeOffset:-1,distance:p.dt*state.v};
  }

  function evaluate(state,z,time,p,advance=true,supervisorLatched=false) {
    const control=controller(state,z,p), disturbance=time>=p.disturbanceAt-1e-12?p.disturbance:0;
    const stop=feasibility(state,p);
    let request=control.raw, supervisor=false;
    if (p.mode==='position_pid_supervisor') {
      if (state.v<=1e-12) supervisorLatched=false;
      const holdAtTarget=state.v<=1e-12&&Math.abs(p.positionTarget-state.x)<=.5;
      if (holdAtTarget) { request=0; supervisor=true; }
      else if (!supervisorLatched&&state.v>0&&(state.x>=p.positionTarget||!stop.feasible)) supervisorLatched=true;
      if (supervisorLatched&&!holdAtTarget) { request=-p.brakeMax; supervisor=true; }
    }
    const result=advance?step(state,request,disturbance,p):{state,forces:forceBalance(state.v,request,disturbance,p),stopEvent:false,stopTimeOffset:-1,distance:0};
    const zRate=supervisor?0:control.integralRate;
    return {time,x:state.x,v:state.v,z,disturbance,control,feasibility:stop,request,...result.forces,supervisor,supervisorLatched,antiWindup:control.antiWindup||supervisor,zRate,nextState:result.state,nextZ:advance?z+p.dt*zRate:z,stopEvent:result.stopEvent,stopTimeOffset:result.stopTimeOffset};
  }

  function simulate(input) {
    const p={...input};validate(p);
    const count=Math.round(p.duration/p.dt),frames=[];let state={x:p.initialPosition,v:p.initialSpeed},z=p.initialIntegral,supervisorLatched=false;
    for(let index=0;index<count;index++){
      const frame=evaluate(state,z,index*p.dt,p,true,supervisorLatched);frames.push(frame);state=frame.nextState;z=frame.nextZ;supervisorLatched=frame.supervisorLatched;
    }
    frames.push(evaluate(state,z,p.duration,p,false,supervisorLatched));
    return {p,frames,final:frames[frames.length-1]};
  }

  // The notes' constant-deceleration comparison is a kinematic reference,
  // not an Euler run or a prediction of the resistance-based plant.
  function simulateConstantStop(p) {
    validate(p);
    const a=-p.fixedForce/p.mass;
    if (!(a>0) || p.mode!=='open_loop' || p.rollingResistance || p.velocityResistance || p.disturbance || p.fixedForce < -p.brakeMax) throw new Error('Constant-stop reference requires ideal, unsaturated braking');
    const stopTime=p.initialSpeed/a;
    const frames=Array.from({length:Math.round(p.duration/p.dt)+1},(_,i)=>{
      const time=i*p.dt,t=Math.min(time,stopTime);
      const state={x:p.initialPosition+p.initialSpeed*t-.5*a*t*t,v:Math.max(0,p.initialSpeed-a*t)};
      return evaluate(state,0,time,time>=stopTime?{...p,fixedForce:0}:p,false);
    });
    return {p,frames,final:frames.at(-1)};
  }

  root.TrainModel={validate,controller,feasibility,forceBalance,step,simulate,simulateConstantStop};
})(globalThis);
