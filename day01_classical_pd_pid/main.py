import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from model import first_order, second_order_standard, msd_plant
from simulate import simulate_ode, step_metrics, safe_t_eval

RESULTS_DIR = os.path.join(os.path.dirname(__file__), "results")

def ensure_dirs():
    os.makedirs(RESULTS_DIR, exist_ok=True)

def run_first_order_sweep():
    taus = [0.2, 0.5, 1.0, 2.0]
    t_final, dt = 8.0, 0.002

    plt.figure()
    for tau in taus:
        t, x = simulate_ode(first_order, t_final=t_final, dt=dt, x0=[0.0], tau=tau, u=1.0)
        y = x[:, 0]
        plt.plot(t, y, label=f"tau={tau}")
    plt.title("First-Order Step Responses (varying τ)")
    plt.xlabel("t (s)")
    plt.ylabel("y")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(RESULTS_DIR, "first_order_step.png"), dpi=200)
    plt.close()

def theoretical_overshoot_pct(zeta):
    """
    For standard 2nd-order underdamped step response:
        Mp = exp(-pi*zeta/sqrt(1-zeta^2))
    returns percent overshoot.
    """
    zeta = float(zeta)
    if zeta <= 0.0:
        return np.inf
    if zeta >= 1.0:
        return 0.0
    return float(np.exp(-np.pi * zeta / np.sqrt(1.0 - zeta**2)) * 100.0)

def run_second_order_damping_sweep():
    wn = 2.0
    zetas = [0.2, 0.7, 1.0, 2.0]

    # Slightly longer horizon helps overdamped cases settle (ζ=2 can be slow)
    t_final, dt = 15.0, 0.001

    rows = []
    plt.figure()
    for zeta in zetas:
        t, x = simulate_ode(
            second_order_standard,
            t_final=t_final,
            dt=dt,
            x0=[0.0, 0.0],
            wn=wn,
            zeta=zeta,
            u=1.0
        )
        y = x[:, 0]
        m = step_metrics(t, y, y_final=1.0)

        rows.append({
            "wn": wn,
            "zeta": float(zeta),
            "overshoot_pct_measured": m["overshoot_pct"],
            "overshoot_pct_theory": theoretical_overshoot_pct(zeta),
            "settling_time_s": m["settling_time"],
            "peak": m["peak"],
        })

        plt.plot(t, y, label=f"zeta={zeta}")

    plt.title("Second-Order Step Response vs Damping Ratio (ωn fixed)")
    plt.xlabel("t (s)")
    plt.ylabel("y")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(RESULTS_DIR, "second_order_damping.png"), dpi=200)
    plt.close()

    df = pd.DataFrame(rows).sort_values("zeta")
    df.to_csv(os.path.join(RESULTS_DIR, "overshoot_table.csv"), index=False)
    return df

def poles_for_second_order(wn, zeta):
    """
    Poles of: s^2 + 2*zeta*wn*s + wn^2 = 0
    """
    wn = float(wn)
    zeta = float(zeta)
    disc = (2.0 * zeta * wn) ** 2 - 4.0 * (wn ** 2)

    if disc >= 0:
        r1 = (-2.0 * zeta * wn + np.sqrt(disc)) / 2.0
        r2 = (-2.0 * zeta * wn - np.sqrt(disc)) / 2.0
        return np.array([r1, r2], dtype=complex)

    re = -zeta * wn
    im = wn * np.sqrt(1.0 - zeta**2)
    return np.array([re + 1j * im, re - 1j * im], dtype=complex)

def run_poles_vs_response_plot(df):
    wn = float(df["wn"].iloc[0])
    zetas = df["zeta"].to_numpy()
    # Use measured overshoot if available; fall back if user deletes theory column
    if "overshoot_pct_measured" in df.columns:
        overs = df["overshoot_pct_measured"].to_numpy()
    else:
        overs = df["overshoot_pct"].to_numpy()

    plt.figure()
    for zeta, ov in zip(zetas, overs):
        poles = poles_for_second_order(wn, float(zeta))
        plt.scatter(poles.real, poles.imag, s=60)
        plt.annotate(
            f"ζ={zeta}, OS={ov:.1f}%",
            (poles[0].real, poles[0].imag),
            textcoords="offset points",
            xytext=(6, 6)
        )

    plt.axvline(0, linestyle="--")
    plt.title("Second-Order Poles (ωn fixed) annotated with Overshoot")
    plt.xlabel("Re(s)")
    plt.ylabel("Im(s)")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(RESULTS_DIR, "poles_vs_response.png"), dpi=200)
    plt.close()

def run_pd_pid_on_msd():
    """
    Plant: m*y'' + b*y' + k*y = u
    PD:  u = Kp*(r - y) - Kd*y'
    PID: u = Kp*(r - y) + Ki*xi - Kd*y' , xi_dot = (r - y)
    """
    m, b, k = 1.0, 0.8, 4.0
    r = 1.0
    t_final, dt = 8.0, 0.001
    t_eval = safe_t_eval(t_final, dt)

    # PD gains
    pd_gains = dict(Kp=25.0, Kd=6.0)
    # PID gains
    pid_gains = dict(Kp=20.0, Ki=18.0, Kd=6.0)

    # PD closed-loop RHS
    def rhs_pd(t, x):
        y, ydot = x
        e = r - y
        u = pd_gains["Kp"] * e - pd_gains["Kd"] * ydot
        return msd_plant(t, x, m=m, b=b, k=k, u=u)

    # PID closed-loop RHS
    def rhs_pid(t, x):
        y, ydot, xi = x
        e = r - y
        u = pid_gains["Kp"] * e + pid_gains["Ki"] * xi - pid_gains["Kd"] * ydot

        # plant part
        yddot = (u - b * ydot - k * y) / m
        return [ydot, yddot, e]

    sol_pd = solve_ivp(rhs_pd, (0.0, t_final), [0.0, 0.0], t_eval=t_eval, rtol=1e-8, atol=1e-10)
    t_pd = sol_pd.t
    y_pd = sol_pd.y[0, :]

    sol_pid = solve_ivp(rhs_pid, (0.0, t_final), [0.0, 0.0, 0.0], t_eval=t_eval, rtol=1e-8, atol=1e-10)
    t_pid = sol_pid.t
    y_pid = sol_pid.y[0, :]

    # PD plot
    plt.figure()
    plt.plot(t_pd, y_pd, label="PD output y(t)")
    plt.axhline(r, linestyle="--", label="reference")
    plt.title(f"PD on MSD (Kp={pd_gains['Kp']}, Kd={pd_gains['Kd']})")
    plt.xlabel("t (s)")
    plt.ylabel("y")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(RESULTS_DIR, "pd_response.png"), dpi=200)
    plt.close()

    # PID plot
    plt.figure()
    plt.plot(t_pid, y_pid, label="PID output y(t)")
    plt.axhline(r, linestyle="--", label="reference")
    plt.title(f"PID on MSD (Kp={pid_gains['Kp']}, Ki={pid_gains['Ki']}, Kd={pid_gains['Kd']})")
    plt.xlabel("t (s)")
    plt.ylabel("y")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(RESULTS_DIR, "pid_response.png"), dpi=200)
    plt.close()

    # Notes
    with open(os.path.join(RESULTS_DIR, "notes.txt"), "w") as f:
        f.write("Day 1 notes:\n")
        f.write("- first_order_step.png: varying τ shifts single pole -> speed changes.\n")
        f.write("- second_order_damping.png: varying ζ shows overshoot/oscillation tradeoff.\n")
        f.write("- poles_vs_response.png: complex poles ↔ overshoot; real poles ↔ no overshoot.\n")
        f.write("- pd_response.png: derivative adds damping; reduces overshoot.\n")
        f.write("- pid_response.png: integral reduces steady-state error but may add overshoot.\n")

def main():
    ensure_dirs()
    run_first_order_sweep()
    df = run_second_order_damping_sweep()
    run_poles_vs_response_plot(df)
    run_pd_pid_on_msd()
    print("Saved artifacts to:", RESULTS_DIR)

if __name__ == "__main__":
    main()