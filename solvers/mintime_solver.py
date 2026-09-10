"""
Minimum-time NLP builder. NOT YET IMPLEMENTED.

Once `physics_models/vehicle_model.py` defines dx/ds = f(x, u, kappa(s)),
this module discretizes it over the track's distance nodes and builds
the NLP with `casadi.Opti`, exactly like `smoke_test.py` does for a
straight line:

  1. one state vector per distance node (v, n, chi, t, [soc once ERS
     is added]) and one control vector per step
  2. RK4 (or collocation, later) to turn dx/ds into equality
     constraints linking consecutive nodes -- see the `for k in
     range(N)` loop in smoke_test.py
  3. path constraints per node: friction circle, track width
     (-w_right <= n <= w_left), speed/power limits
  4. periodicity constraints if solving a closed lap (state at node 0
     == state at node N, not a fixed start value like smoke_test.py
     uses)
  5. objective: minimize t[-1] (or t[-1] - t[0] for a closed lap)
  6. opti.solver("ipopt", ...) and solve

Keep `smoke_test.py` around and runnable -- it's the fastest way to
tell "my model is wrong" apart from "my solver setup is wrong" while
debugging this.
"""
