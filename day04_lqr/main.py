import os
import numpy as np
import matplotlib.pyplot as plt

from model import SecondOrderPlant, lqr_gain, pd_gain_for_second_order
from simulate import SimConfig, simulate_state_feedback, metrics


def ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def plot_state_response(t, x_pd, x_lqr, outpath: str) -> None:
    plt.figure()
    plt.plot(t, x_pd[:, 0], label="pos (PD)")
    plt.plot(t, x_lqr[:, 0], label="pos (LQR)")
    plt.plot(t, x_pd[:, 1], label="vel (PD)", linestyle="--")
    plt.plot(t, x_lqr[:, 1], label="vel (LQR)", linestyle="--")
    plt.xlabel("t [s]")
    plt.ylabel("state")
    plt.title("State Response: PD vs LQR")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(outpath, dpi=200)
    plt.close()


def plot_control_effort(t, u_pd, u_lqr, outpath: str) -> None:
    plt.figure()
    plt.plot(t, u_pd, label="u (PD)")
    plt.plot(t, u_lqr, label="u (LQR)")
    plt.xlabel("t [s]")
    plt.ylabel("u")
    plt.title("Control Effort: PD vs LQR")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(outpath, dpi=200)
    plt.close()


def main():
    results_dir = "results"
    ensure_dir(results_dir)

    # Plant
    plant = SecondOrderPlant(w=1.0, zeta=0.2)
    A, B = plant.matrices()

    # Initial condition: displaced, zero velocity
    x0 = np.array([1.0, 0.0], dtype=float)

    # Simulation config
    cfg = SimConfig(t0=0.0, tf=10.0, dt=0.002, u_clip=None)

    # PD baseline
    K_pd = pd_gain_for_second_order(kp=10.0, kd=3.0)
    sim_pd = simulate_state_feedback(A, B, K_pd, x0, cfg)

    # LQR default
    Q = np.diag([10.0, 1.0])
    R = np.array([[1.0]])
    K_lqr = lqr_gain(A, B, Q, R)
    sim_lqr = simulate_state_feedback(A, B, K_lqr, x0, cfg)

    # Metrics
    m_pd = metrics(sim_pd["t"], sim_pd["x"], sim_pd["u"])
    m_lqr = metrics(sim_lqr["t"], sim_lqr["x"], sim_lqr["u"])

    print("Day04: LQR")
    print("A=\n", A)
    print("B=\n", B)
    print("\nPD  K =", K_pd)
    print("LQR Q=\n", Q)
    print("LQR R=\n", R)
    print("LQR K =", K_lqr)

    print("\n--- Metrics ---")
    print("PD  :", m_pd)
    print("LQR :", m_lqr)

    # Save plots
    plot_state_response(
        sim_pd["t"], sim_pd["x"], sim_lqr["x"],
        os.path.join(results_dir, "state_response.png")
    )
    plot_control_effort(
        sim_pd["t"], sim_pd["u"], sim_lqr["u"],
        os.path.join(results_dir, "control_effort.png")
    )

    # Small Q/R sweep
    sweep = [
        (np.diag([10.0, 1.0]), np.array([[0.1]])),
        (np.diag([10.0, 1.0]), np.array([[1.0]])),
        (np.diag([10.0, 1.0]), np.array([[10.0]])),
        (np.diag([100.0, 1.0]), np.array([[1.0]])),
        (np.diag([10.0, 10.0]), np.array([[1.0]])),
    ]

    print("\nLQR Sweep (Q/R -> K, u_rms, settling)")
    for Qi, Ri in sweep:
        Ki = lqr_gain(A, B, Qi, Ri)
        si = simulate_state_feedback(A, B, Ki, x0, cfg)
        mi = metrics(si["t"], si["x"], si["u"])
        print(
            f"Qdiag={np.diag(Qi)} R={Ri[0,0]:.3g}  "
            f"K={Ki}  u_rms={mi['u_rms']:.3f}  ts={mi['settling_time_2pct']}"
        )

    print("\nSaved plots -> results/state_response.png and results/control_effort.png")


if __name__ == "__main__":
    main()