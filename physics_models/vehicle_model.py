"""
Distance-domain vehicle model. NOT YET IMPLEMENTED -- this is the next
piece to build after `solvers/smoke_test.py` proves out the toolchain.

Goal: given track curvature kappa(s) (from `track_data/`), define the
state/control vector and the dynamics dx/ds = f(x, u, kappa(s)) that
`solvers/mintime_solver.py` will integrate, the same way
`smoke_test.py` integrates dv/ds and dt/ds for a straight line.

Start with the simplest model that still needs a track (point mass with
a friction circle), get a full lap solving end to end, and only then
upgrade to a bicycle model with load transfer / Pacejka tyres -- don't
build the complex model first, there is nothing to validate it against.

Suggested first version (point mass, distance domain):

    states:   v      (speed)
              n      (lateral deviation from centerline, for track limits)
              chi    (heading deviation from track tangent)
              t      (elapsed time, same trick as the smoke test)
    controls: F_long (net longitudinal force, engine/brake)
              F_lat  (lateral force, i.e. how hard you're cornering)

    constraint tying the controls together (friction circle):
        (F_long / F_long_max)**2 + (F_lat / F_lat_max)**2 <= 1

    ds/dt = v * cos(chi) / (1 - n * kappa(s))   -> invert for dt/ds
    (this factor is why curvature enters the dynamics: a given v means
    a different dt/ds depending how far off the racing line you are)

Reference: TUMFTM global_racetrajectory_optimization, `opt_mintime`
module -- read `src/vehicle_dynamics_KM.py` (kinematic/point-mass) then
`src/vehicle_dynamics_dyn5.py` (full dynamic bicycle) there for the
exact equations once you're past the point-mass version.
"""
