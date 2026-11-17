# Phase 3 Engineering Note — PI Water-Level Control

## The P-only problem being answered

The biased P controller holds 3 m in the nominal model because its bias exactly
balances the known outlet and leak. If an unmodelled permanent leak appears,
the controller can request more flow only while `e = r - h` is nonzero. It
therefore settles below the reference. With `Kp=0.02 m²/s` and an extra
`0.007 m³/s` leak, its analytical equilibrium is `2.7666666667 m`, leaving a
`0.2333333333 m` error.

PI control answers that problem by storing past error in `z`. Once `z` has
reached `7 m·s`, the integral contribution is `Ki*z = 0.007 m³/s`. The pump can
then deliver the new required `0.042 m³/s` while the current error is zero.

## Equations and numerical method

The plant and controller are:

```text
A dh/dt = u - k h - q_leak(t)
e        = r - h
dz/dt    = e
u_raw    = u_bias + Kp e + Ki z
u        = clip(u_raw, u_min, u_max)
```

At each sample, `u[n]`, `dh/dt`, and `dz/dt` are calculated from the old state
`h[n], z[n]`. Only then are both next states written. This is an explicit,
simultaneous Forward Euler step, not a semi-implicit update.

Conditional anti-windup freezes `z` only when its current error would push an
already saturated command farther beyond the same limit. It permits integration
in the opposite direction so stored error can unwind. The `none` strategy is
retained for comparison. If `Ki=0`, `z` is still recorded but cannot affect the
pump command, making the implementation observationally equivalent to P control.

## Reproducible scenarios

All runs use `dt=0.1 s`, `A=2 m²`, `k=0.01 m²/s`, nominal leakage
`0.005 m³/s`, reference `3 m`, bias `0.035 m³/s`, `Kp=0.02 m²/s`, and
`Ki=0.001 m/s²` unless explicitly stated.

- Reference tracking: start at 1 m; compare biased P and conditional PI for
  1200 s.
- Disturbance rejection: start at `h=3 m, z=0`; apply the extra
  `0.007 m³/s` leak at exactly 600 s and run to 2400 s.
- Windup: start at 0 m with `u_max=0.05 m³/s`; compare `none` and
  `conditional` for 1800 s under identical conditions.

Actual disturbance-run endpoint: `h=3.000000 m`, `z=6.999989 m·s`, and
`u=0.042000 m³/s`. Numerical differences from the analytical values are due to
finite simulation horizon and integration precision.

## Reproduce

From the workspace root after `make setup`:

```bash
make test
PYTHONPATH=projects/01-water-tank/src MPLBACKEND=Agg \
  .venv/bin/python -m water_tank.pi_control
```

The Python module separates controller arithmetic, anti-windup logic,
simulation, and plotting so every state update can be tested without rendering.
