# Applied Control Engineering Projects

This workspace contains practical projects based on Mark Misin's *Applied
Control Engineering 1* course. Each project is isolated under `projects/` and
uses the same predictable layout:

```text
projects/<number>-<name>/
├── README.md        # theory, assumptions, and project-specific commands
├── requirements.txt # runtime and test dependencies
├── results/         # generated plots and recorded output
├── src/<package>/   # importable Python source
└── tests/           # automated verification
```

## Projects

| Project | Topic | Status |
| --- | --- | --- |
| [01 — Water tank](projects/01-water-tank/README.md) | Open-loop, P, and PI water-level control | Phases 1–3 complete |
| [02 — Coupled two-tank](projects/02-coupled-two-tank/README.md) | Mass-conserving two-tank dynamics, P and PI control | Complete |

## Interactive dashboard

The browser-based Control Lab provides live, mathematically synchronized
simulations for proportional control, PI control, and the coupled two-tank
system:

- [Open Control Lab](https://control-lab-alpha.vercel.app/)
- [Read the proportional-control engineering report](https://control-lab-alpha.vercel.app/p-control-report)

The deployable static application is maintained in [`apps/control-lab`](apps/control-lab/README.md).

## Quick start

Python 3.10 or newer is required. From this workspace root:

```bash
make setup
make test
make run
```

Activate the environment for interactive work with:

```bash
source .venv/bin/activate
```

To add another course exercise, copy `projects/PROJECT_TEMPLATE`, rename the
package, and document its model and assumptions in its README.

