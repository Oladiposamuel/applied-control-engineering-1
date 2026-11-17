# Project 02 — Coupled Two-Tank Control

This project extends the single-tank study to two constant-area tanks connected
by a one-way link. The pump feeds Tank 1, while feedback measures and controls
Tank 2. The indirect actuator path introduces an additional dynamic state.

```text
pump u → [ Tank 1: h1 ] ── q12 ──→ [ Tank 2: h2 ] → qout
                                      └────────────→ demand d
```

| Parameter | Default | Units |
|---|---:|---|
| Tank areas `A1`, `A2` | 2.0, 1.5 | m² |
| Link coefficient `k12` | 0.012 | m²/s |
| Tank 2 outlet coefficient `k2` | 0.008 | m²/s |
| Initial levels `h1`, `h2` | 2.0, 0.5 | m |
| Tank 2 target | 3.0 | m |
| Pump bounds | 0–0.10 | m³/s |
| P/PI gains | 0.02, 0.0001 | m²/s, m/s² |

The normal-regime equations are
`A1 dh1/dt = u-q12`, `A2 dh2/dt = q12-k2*h2-d`, and
`q12=max(0,k12*(h1-h2))`. The controller reads `h2`; its pump action first
changes `h1`, then the height difference changes `q12`, and only then does Tank
2 respond.

At the 3 m target, the nominal bias is `k2*3 = 0.024 m³/s` and Tank 1 must
settle at 5 m. With an added demand of `0.006 m³/s`, PI must retain
`z=d/Ki=60 m·s`, the pump must supply `0.030 m³/s`, and Tank 1 must rise to
5.5 m. P alone instead settles at `h2=2.785714 m` because it needs a permanent
error to request extra flow.

The constrained step implementation executes volumes rather than independently
clamping heights. Link transfer is limited by Tank 1 inventory. If Tank 2 lacks
water, its normal outlet and extra demand are scaled proportionally and unmet
demand is recorded. This preserves
`Δ(A1*h1+A2*h2)=dt*(pump-qout_actual-demand_served)` every step.

Run from the workspace root:

```bash
make test
make run
```

Results are written to `results/`: open-loop, tracking, disturbance rejection,
and conditional anti-windup comparisons. The gains are illustrative, not
optimized. The model neglects nonlinear orifices, pipe elevation losses,
sensor noise, pump lag, overflow, and real hydraulic validation.
