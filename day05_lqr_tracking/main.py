# day05_lqr_tracking/main.py
import os
import numpy as np
import matplotlib.pyplot as plt

from model import mass_spring_damper, augment_with_integral
from lqr import lqr_continuous
from simulate import simulate_tracking, simulate_plain_lqr

def step_reference(t: float, step_time: float = 1.0, amplitude: float = 1.0):
    return amplitude if t >= step_time else 0.0

def ensure_results_dir():
    os.makedirs("results", exist_ok=True)

def main():
    ensure_results_dir()

    wn = 1.0
    zeta = 0.2
    A, B, C, D = mass_spring_damper(wn=wn, zeta=zeta)

    # Simulation setup
    T = 12.0
    dt = 0.001
    x0 = np.array([0.0, 0.0])
    u_limit = 10.0  # optional saturation

    r_func = lambda t: step_reference(t, step_time=1.0, amplitude=1.0)

    # Plain LQR (regulator)
    Q = np.diag([10.0, 1.0])
    R = np.array([[1.0]])
    K_plain, _ = lqr_continuous(A, B, Q, R)

    sim_plain = simulate_plain_lqr(A, B, C, K_plain, r_func, x0=x0, T=T, dt=dt, u_limit=u_limit)

    # LQR Tracking via integral augmentation
    Aa, Ba, Er = augment_with_integral(A, B, C)

    Qa = np.diag([10.0, 1.0, 50.0])
    Ra = np.array([[1.0]])

    K_aug, _ = lqr_continuous(Aa, Ba, Qa, Ra)
    Kx = K_aug[:, :2]
    Ki = K_aug[:, 2:].reshape(1, 1)

    sim_track = simulate_tracking(A, B, C, Kx, Ki, r_func, x0=x0, xI0=0.0, T=T, dt=dt, u_limit=u_limit)

    t = sim_track["t"]

    # 1) tracking response
    plt.figure()
    plt.plot(t, sim_track["r"][:, 0], label="reference r(t)")
    plt.plot(t, sim_track["y"][:, 0], label="y(t) with LQR+Integral")
    plt.plot(t, sim_plain["y"][:, 0], label="y(t) with plain LQR")
    plt.xlabel("Time (s)")
    plt.ylabel("Output (position)")
    plt.title("Day05: LQR Tracking (Integral Augmentation)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("results/tracking_response.png", dpi=200)

    # 2) control effort
    plt.figure()
    plt.plot(t, sim_track["u"][:, 0], label="u(t) LQR+Integral")
    plt.plot(t, sim_plain["u"][:, 0], label="u(t) plain LQR")
    plt.xlabel("Time (s)")
    plt.ylabel("Control input u")
    plt.title("Control Effort Comparison")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("results/control_effort.png", dpi=200)

    # 3) steady-state error
    plt.figure()
    plt.plot(t, sim_track["e"][:, 0], label="e(t)=r-y (LQR+Integral)")
    plt.plot(t, sim_plain["e"][:, 0], label="e(t)=r-y (plain LQR)")
    plt.xlabel("Time (s)")
    plt.ylabel("Error")
    plt.title("Steady-State Error: Plain LQR vs LQR+Integral")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("results/steady_state_error.png", dpi=200)

    # Notes
    with open("results/notes.txt", "w") as f:
        f.write("Day05: LQR Tracking\n")
        f.write(f"wn={wn}, zeta={zeta}\n")
        f.write("Plain LQR:\n")
        f.write(f"K_plain = {K_plain}\n\n")
        f.write("Augmented LQR:\n")
        f.write(f"Kx = {Kx}\n")
        f.write(f"Ki = {Ki}\n")
        f.write("\nObservations:\n")
        f.write("- Plain LQR is a regulator; tracking step reference leaves steady-state error.\n")
        f.write("- Integral augmentation drives steady-state error to ~0 for constant reference.\n")
        f.write("- Control effort increases slightly with integral action.\n")

    print("Saved plots to results/")

if __name__ == "__main__":
    main()