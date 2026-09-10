"""
ERS (Energy Recovery System) model. NOT YET IMPLEMENTED -- build this
only after `vehicle_model.py` solves a plain minimum-time lap with no
ERS, so you have a baseline lap time to compare deployment strategies
against.

This is the part that makes the project a "deployment optimisation
study" rather than a lap-time-sim clone: everything above exists to
give this module something realistic to plug into.

Sketch:

    state:    soc(s)          battery state of charge (or energy, MJ)
    control:  P_deploy(s)     MGU-K power to the wheels (>=0)
              P_harvest(s)    MGU-K power recovered under braking (>=0)

    dsoc/ds = (-P_deploy + eta_harvest * P_harvest) / v   [dE/ds = dE/dt * dt/ds = P / v]

    constraints:
      0 <= soc <= soc_max
      0 <= P_deploy <= P_deploy_max          (MGU-K deploy power limit)
      0 <= P_harvest <= P_harvest_max(v)      (harvest limited by braking force available)
      integral of P_deploy dt over the lap <= E_deploy_max_per_lap   (regulatory energy limit)

    P_deploy adds directly to F_long in the vehicle model (extra
    tractive force), so this module doesn't stand alone -- it extends
    the control vector and the longitudinal force balance in
    `vehicle_model.py`, it isn't a separate dynamics block.

The actual "study" is then: solve the mintime OCP for several values of
E_deploy_max_per_lap / different harvest strategies, and compare
lap time deltas and *where* on the track deployment gets used (corner
exit vs straight) -- that comparison is the deliverable, not just one
solve.
"""
