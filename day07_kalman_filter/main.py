# day07_kalman_filter/main.py
from __future__ import annotations

import argparse
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

from model import constant_velocity_1d, KalmanFilter
from simulate import simulate_linear_system


def run(args: argparse.Namespace) -> None:
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # System
    sys = constant_velocity_1d(
        dt=args.dt,
        q_pos=args.q_pos,
        q_vel=args.q_vel,
        r_meas=args.r_meas
    )

    # Simulate truth + noisy measurements
    x0_true = np.array([[args.x0_pos],
                        [args.x0_vel]], dtype=float)
    data = simulate_linear_system(
        system=sys,
        x0=x0_true,
        T=args.T,
        dt=args.dt,
        u_policy=args.u_policy,
        seed=args.seed
    )

    # Filter init (deliberately imperfect initial state)
    x0_hat = np.array([[args.xhat0_pos],
                       [args.xhat0_vel]], dtype=float)
    P0 = np.diag([args.P0_pos, args.P0_vel]).astype(float)

    kf = KalmanFilter(system=sys, x0=x0_hat, P0=P0)

    T = args.T
    n = sys.A.shape[0]
    x_hat_hist = np.zeros((T, n), dtype=float)
    P_trace = np.zeros(T, dtype=float)
    K_hist = np.zeros((T, n), dtype=float)  # since m=1, K is (n,1)

    for k in range(T):
        uk = data.u[k].reshape(-1, 1)
        kf.predict(uk)

        yk = data.y_meas[k].reshape(-1, 1)
        x_hat, P = kf.update(yk)

        x_hat_hist[k] = x_hat.ravel()
        P_trace[k] = float(np.trace(P))
        if kf.K_last is not None:
            K_hist[k] = kf.K_last.ravel()

    # Plots
    t = data.t
    x_true = data.x_true
    y = data.y_meas

    # 1) State vs estimate (position & velocity)
    plt.figure()
    plt.plot(t, x_true[:, 0], label="true position")
    plt.plot(t, y[:, 0], label="measured position")
    plt.plot(t, x_hat_hist[:, 0], label="estimated position")
    plt.xlabel("time (s)")
    plt.ylabel("position")
    plt.title("Kalman Filter: Position (true vs measured vs estimated)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_dir / "state_position.png", dpi=200)
    plt.close()

    plt.figure()
    plt.plot(t, x_true[:, 1], label="true velocity")
    plt.plot(t, x_hat_hist[:, 1], label="estimated velocity")
    plt.xlabel("time (s)")
    plt.ylabel("velocity")
    plt.title("Kalman Filter: Velocity (true vs estimated)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_dir / "state_velocity.png", dpi=200)
    plt.close()

    # 2) Covariance evolution
    plt.figure()
    plt.plot(t, P_trace, label="trace(P)")
    plt.xlabel("time (s)")
    plt.ylabel("trace(P)")
    plt.title("Covariance Evolution (trace of P)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_dir / "covariance_trace.png", dpi=200)
    plt.close()

    # 3) Kalman gain (optional but useful)
    plt.figure()
    plt.plot(t, K_hist[:, 0], label="K[0] (pos gain)")
    plt.plot(t, K_hist[:, 1], label="K[1] (vel gain)")
    plt.xlabel("time (s)")
    plt.ylabel("Kalman gain")
    plt.title("Kalman Gain Evolution")
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_dir / "kalman_gain.png", dpi=200)
    plt.close()

    print(f"[OK] Saved plots to: {out_dir.resolve()}")
    print(" - state_position.png")
    print(" - state_velocity.png")
    print(" - covariance_trace.png")
    print(" - kalman_gain.png")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Day 07: Kalman Filter (Linear Gaussian)")
    p.add_argument("--T", type=int, default=300)
    p.add_argument("--dt", type=float, default=0.1)
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--u_policy", type=str, default="sine", choices=["zero", "sine", "steps"])
    p.add_argument("--out_dir", type=str, default="results")

    # True initial state
    p.add_argument("--x0_pos", type=float, default=0.0)
    p.add_argument("--x0_vel", type=float, default=1.0)

    # Filter initial guess (intentionally can be wrong)
    p.add_argument("--xhat0_pos", type=float, default=2.0)
    p.add_argument("--xhat0_vel", type=float, default=-0.5)

    # Noise parameters
    p.add_argument("--q_pos", type=float, default=1e-3)
    p.add_argument("--q_vel", type=float, default=1e-3)
    p.add_argument("--r_meas", type=float, default=5e-2)

    # Initial covariance
    p.add_argument("--P0_pos", type=float, default=10.0)
    p.add_argument("--P0_vel", type=float, default=10.0)
    return p


if __name__ == "__main__":
    parser = build_parser()
    run(parser.parse_args())