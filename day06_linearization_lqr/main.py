import os
import numpy as np
import matplotlib.pyplot as plt

from model import pendulum_nonlinear, pendulum_linearized
from simulate import simulate_nonlinear, simulate_linear
from utils import lqr, rms, peak_abs, settling_time


def save_notes(path, lines):
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def main():
   
    # Params

    params = {"g": 9.81, "l": 1.0, "m": 1.0}

    # Linearized model about theta ~ 0
    A, B = pendulum_linearized(params)

    # LQR weights (tune if you want)
    Q = np.diag([20.0, 2.0])   # penalize theta more than omega
    R = np.array([[0.5]])     # penalize control

    K = lqr(A, B, Q, R)
    K = np.array(K).reshape(1, -1)

    print("LQR Gain K:", K)


    # Simulation config

    T = 10.0
    dt = 0.001
    method = "rk4"      # "rk4" recommended for nonlinear
    u_limit = None      # set e.g. 5.0 or ( -5.0, 5.0 ) to see saturation effects

    # Initial conditions
    x0_small = [0.2, 0.0]   # ~11 deg
    x0_large = [1.0, 0.0]   # ~57 deg (nonlinear region)


    # Simulate: small

    t_nl_s, x_nl_s, u_nl_s = simulate_nonlinear(
        pendulum_nonlinear, x0_small, K, params, T=T, dt=dt, method=method, u_limit=u_limit
    )
    t_li_s, x_li_s, u_li_s = simulate_linear(
        A, B, x0_small, K, T=T, dt=dt, u_limit=u_limit
    )


    # Simulate: large

    t_nl_l, x_nl_l, u_nl_l = simulate_nonlinear(
        pendulum_nonlinear, x0_large, K, params, T=T, dt=dt, method=method, u_limit=u_limit
    )
    t_li_l, x_li_l, u_li_l = simulate_linear(
        A, B, x0_large, K, T=T, dt=dt, u_limit=u_limit
    )

    os.makedirs("results", exist_ok=True)


    # Plot 1: Small angle θ, ω (linear vs nonlinear)

    plt.figure()
    plt.plot(t_nl_s, x_nl_s[:, 0], label="Nonlinear θ (small)")
    plt.plot(t_li_s, x_li_s[:, 0], "--", label="Linear θ (small)")
    plt.plot(t_nl_s, x_nl_s[:, 1], label="Nonlinear ω (small)")
    plt.plot(t_li_s, x_li_s[:, 1], "--", label="Linear ω (small)")
    plt.legend()
    plt.title("Small Angle: Linearization Matches Nonlinear")
    plt.xlabel("Time (s)")
    plt.ylabel("State")
    plt.savefig("results/small_angle_states.png")
    plt.close()


    # Plot 2: Large angle θ, ω (linear vs nonlinear)

    plt.figure()
    plt.plot(t_nl_l, x_nl_l[:, 0], label="Nonlinear θ (large)")
    plt.plot(t_li_l, x_li_l[:, 0], "--", label="Linear θ (large)")
    plt.plot(t_nl_l, x_nl_l[:, 1], label="Nonlinear ω (large)")
    plt.plot(t_li_l, x_li_l[:, 1], "--", label="Linear ω (large)")
    plt.legend()
    plt.title("Large Angle: Linear Model Deviates (Nonlinearity Dominates)")
    plt.xlabel("Time (s)")
    plt.ylabel("State")
    plt.savefig("results/large_angle_states.png")
    plt.close()


    # Plot 3: Control effort comparison

    plt.figure()
    plt.plot(t_nl_s, u_nl_s, label="u nonlinear (small)")
    plt.plot(t_li_s, u_li_s, "--", label="u linear (small)")
    plt.plot(t_nl_l, u_nl_l, label="u nonlinear (large)")
    plt.plot(t_li_l, u_li_l, "--", label="u linear (large)")
    plt.legend()
    plt.title("Control Effort: Linear vs Nonlinear, Small vs Large IC")
    plt.xlabel("Time (s)")
    plt.ylabel("u")
    plt.savefig("results/control_effort.png")
    plt.close()


    # Notes (metrics)

    lines = []
    lines.append("Day06: Nonlinear Pendulum + Linearization + LQR")
    lines.append("")
    lines.append(f"Params: g={params['g']}  l={params['l']}  m={params['m']}")
    lines.append(f"Sim: T={T}, dt={dt}, nonlinear_integrator={method}, u_limit={u_limit}")
    lines.append("")
    lines.append(f"LQR: Q=diag({Q[0,0]}, {Q[1,1]})  R={R[0,0]}")
    lines.append(f"K = {K}")
    lines.append("")

    def summarize_case(name, t, x, u):
        st = settling_time(t, x[:, 0], band=0.02)
        lines.append(f"[{name}]")
        lines.append(f"  peak |theta| = {peak_abs(x[:, 0]):.4f} rad")
        lines.append(f"  peak |omega| = {peak_abs(x[:, 1]):.4f} rad/s")
        lines.append(f"  RMS(u) = {rms(u):.4f}")
        lines.append(f"  peak |u| = {peak_abs(u):.4f}")
        lines.append(f"  settling_time(theta) ~ {st if st is not None else 'None'} s")
        lines.append("")

    summarize_case("Nonlinear small", t_nl_s, x_nl_s, u_nl_s)
    summarize_case("Linear small",    t_li_s, x_li_s, u_li_s)
    summarize_case("Nonlinear large", t_nl_l, x_nl_l, u_nl_l)
    summarize_case("Linear large",    t_li_l, x_li_l, u_li_l)

    save_notes("results/notes.txt", lines)

    print("Saved plots + notes to ./results/")
    print("  - small_angle_states.png")
    print("  - large_angle_states.png")
    print("  - control_effort.png")
    print("  - notes.txt")


if __name__ == "__main__":
    main()