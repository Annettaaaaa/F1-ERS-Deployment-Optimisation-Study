# F1 ERS Deployment Optimisation Study

A minimum-lap-time simulator that studies optimal Energy Recovery
System (ERS / MGU-K) deployment strategy, built with
[CasADi](https://web.casadi.org/) + IPOPT, following the same
distance-domain direct-collocation approach as TUMFTM's
[`global_racetrajectory_optimization`](https://github.com/TUMFTM/global_racetrajectory_optimization)
`opt_mintime` module (a good reference implementation to read once a
piece here isn't behaving).

## Why distance domain

Lap length is fixed but lap time isn't, so every state is integrated
with respect to distance `s` along the track rather than time `t`
(`dt/ds = 1/v` instead of the usual `dv/dt = ...`). See
`solvers/smoke_test.py` for a minimal worked example of the trick.

## Layout

- `track_data/` -- raw centerline/width data and the processed
  `s, kappa(s), width` arrays that are the actual solver input.
- `physics_models/vehicle_model.py` -- the distance-domain vehicle
  dynamics (point mass -> bicycle model as it matures).
- `physics_models/ers_model.py` -- battery SoC dynamics and
  deploy/harvest limits, layered onto the vehicle model.
- `solvers/smoke_test.py` -- working CasADi + IPOPT minimum-time
  example (no track, straight line) that validates the toolchain.
- `solvers/mintime_solver.py` -- the real NLP builder, once the models
  above exist.
- `main.py` -- entry point.

## Roadmap

1. **Toolchain smoke test** (done) -- `solvers/smoke_test.py` solves a
   trivial distance-domain minimum-time problem and checks it against
   the analytic answer.
2. **Track data pipeline** -- pick a source track (TUMFTM's
   [`racetrack-database`](https://github.com/TUMFTM/racetrack-database)
   is the easiest starting point), resample to equal arc-length steps,
   compute signed curvature `kappa(s)`. See `track_data/README.md`.
3. **Point-mass vehicle model, no ERS** -- extend the smoke test's
   dynamics with lateral position `n(s)`, heading deviation, a friction
   circle, and track-width constraints driven by `kappa(s)`. Solve a
   full closed lap (periodic boundary conditions instead of a fixed
   start). This gives a baseline lap time.
4. **Add ERS** -- battery SoC state, deploy/harvest controls, lap
   energy budget. Compare lap time and deployment traces (where on
   track power gets used) across different strategies/energy budgets --
   that comparison is the actual study.
5. *(optional)* Upgrade point mass to a full dynamic bicycle model with
   load transfer and a nonlinear tyre model, if the point-mass results
   need more fidelity.

## Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python solvers/smoke_test.py   # should print a time matching the analytic check
```
