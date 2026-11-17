# Validation references

- Initial `q12=0.018`, `qout=0.004`, `h1_dot=0.011`, `h2_dot=0.0093333333`.
- One 0.1 s step gives `h1=2.0011`, `h2=0.5009333333`.
- Open-loop equilibrium at `u=0.04`: `(h1,h2)=(8.3333333,5)`.
- Nominal target equilibrium: `(h1,h2,u)=(5,3,0.024)`.
- Disturbed target equilibrium: `(5.5,3,0.030)` and PI `z=60`.
- Disturbed P equilibrium: `h2=2.7857143`, `h1=5.1428571`.

Automated tests verify these values, controller direction, pump limits, dry-tank
demand allocation, disturbance timing, and per-step conservation residuals.
