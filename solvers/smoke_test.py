"""
Toolchain smoke test: minimum-time optimal control with CasADi + IPOPT,
formulated in the DISTANCE domain (independent variable is s, not t).

This is the same trick TUMFTM's mintime solver uses for the full lap:
integrating in time is awkward because lap length is fixed but lap time
isn't, so instead every state's derivative is taken with respect to
distance s. For a point mass:

    ds/dt = v          ->      dt/ds = 1/v
    dv/dt = F/m         ->      dv/ds = F/(m*v)

so time itself becomes a state you integrate up alongside velocity, and
the objective is simply "minimize the accumulated time at the end of
the track" -- which is exactly the lap-time objective you'll reuse
later, just with a point mass here instead of a track + tyre model.

Run this file directly. If it prints a final time close to the
analytic bang-bang answer (see bottom) and IPOPT reports
"Optimal Solution Found", your CasADi + IPOPT install is good and you
have a working template for the real distance-domain OCP.
"""

import casadi as ca
import numpy as np

# ---- problem data -----------------------------------------------------
# Accelerate a point mass over a fixed distance as fast as possible,
# subject to a bounded net longitudinal force (stand-in for combined
# engine/brake force before any tyre/friction-circle model exists).
DISTANCE = 200.0       # m, fixed track length (this is what's fixed instead of time)
MASS = 800.0            # kg
F_MAX = 6000.0          # N, symmetric accel/brake limit
V_MIN = 1.0             # m/s, avoid division by zero in dv/ds = F/(m*v)
V0 = 5.0                # m/s, start speed
N = 100                  # number of distance steps


def build_and_solve():
    ds = DISTANCE / N

    opti = ca.Opti()

    # states, indexed by distance node k = 0..N
    v = opti.variable(N + 1)   # velocity at each node
    t = opti.variable(N + 1)   # elapsed time at each node (this is what we minimize)
    F = opti.variable(N)        # control: net force, held constant over each step

    # dynamics: RK4 integration of [dv/ds, dt/ds] over each distance step
    def deriv(v_k, F_k):
        dv_ds = F_k / (MASS * v_k)
        dt_ds = 1.0 / v_k
        return ca.vertcat(dv_ds, dt_ds)

    for k in range(N):
        x_k = ca.vertcat(v[k], t[k])
        k1 = deriv(x_k[0], F[k])
        k2 = deriv(x_k[0] + ds / 2 * k1[0], F[k])
        k3 = deriv(x_k[0] + ds / 2 * k2[0], F[k])
        k4 = deriv(x_k[0] + ds * k3[0], F[k])
        x_next = x_k + ds / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        opti.subject_to(v[k + 1] == x_next[0])
        opti.subject_to(t[k + 1] == x_next[1])

    # bounds / boundary conditions
    opti.subject_to(opti.bounded(V_MIN, v, ca.inf))
    opti.subject_to(opti.bounded(-F_MAX, F, F_MAX))
    opti.subject_to(v[0] == V0)
    opti.subject_to(t[0] == 0.0)

    # objective: minimize total elapsed time over the fixed distance
    opti.minimize(t[-1])

    # initial guess
    opti.set_initial(v, V0)
    opti.set_initial(t, np.linspace(0, DISTANCE / V0, N + 1))
    opti.set_initial(F, F_MAX)

    opti.solver("ipopt", {"print_time": False}, {"print_level": 0, "sb": "yes"})
    sol = opti.solve()

    return sol.value(v), sol.value(t), sol.value(F)


if __name__ == "__main__":
    v_opt, t_opt, F_opt = build_and_solve()

    print(f"final time: {t_opt[-1]:.4f} s")
    print(f"final speed: {v_opt[-1]:.4f} m/s")
    print(f"control saturates at F_MAX for {np.mean(np.isclose(F_opt, F_MAX, atol=1.0)) * 100:.0f}% of steps")

    # Sanity check against the analytic solution: with a hard force
    # bound and no speed limit, minimum time is always max-accel the
    # whole way (bang-bang with no room for a braking phase), i.e.
    # constant acceleration a = F_MAX / MASS for the whole distance.
    a = F_MAX / MASS
    t_analytic = (-V0 + np.sqrt(V0**2 + 2 * a * DISTANCE)) / a
    print(f"analytic constant-accel time: {t_analytic:.4f} s")
