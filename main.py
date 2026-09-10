"""
Entry point -- wires track data -> vehicle model -> mintime solver ->
results, once those pieces exist. Currently just runs the toolchain
smoke test so `python main.py` always does something meaningful.
"""

from solvers.smoke_test import build_and_solve

if __name__ == "__main__":
    v, t, F = build_and_solve()
    print(f"[smoke test] final time: {t[-1]:.4f} s, final speed: {v[-1]:.4f} m/s")
