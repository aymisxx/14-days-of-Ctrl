import numpy as np
from scipy.integrate import solve_ivp

def step(t, t0=0.0, amp=1.0):
    return amp if t >= t0 else 0.0

def safe_t_eval(t_final, dt):
    """
    Returns a strictly increasing t_eval that always ends exactly at t_final,
    and stays within [0, t_final] to keep solve_ivp happy.
    """
    t_final = float(t_final)
    dt = float(dt)
    t_eval = np.arange(0.0, t_final, dt)
    if t_eval.size == 0 or t_eval[-1] < t_final:
        t_eval = np.append(t_eval, t_final)
    return t_eval

def simulate_ode(rhs, t_final=10.0, dt=0.001, x0=None, **rhs_kwargs):
    """
    Generic ODE simulation wrapper using solve_ivp.
    Returns:
        t: (N,)
        X: (N, state_dim)
    """
    if x0 is None:
        raise ValueError("x0 must be provided.")
    t_eval = safe_t_eval(t_final, dt)

    def f(t, x):
        return rhs(t, x, **rhs_kwargs)

    sol = solve_ivp(
        f,
        (0.0, float(t_final)),
        np.array(x0, dtype=float),
        t_eval=t_eval,
        rtol=1e-8,
        atol=1e-10
    )
    return sol.t, sol.y.T

def step_metrics(t, y, y_final=None, settling_band=0.02, ref=1.0):
    """
    Basic step metrics for y(t).
    Returns:
        overshoot_pct, settling_time, steady_state_error, etc.
    """
    y = np.asarray(y).flatten()

    if y_final is None:
        # tail-average for robustness
        tail = y[-max(10, len(y)//50):]
        y_final = float(np.mean(tail))

    peak = float(np.max(y))

    overshoot_pct = 0.0
    if abs(y_final) > 1e-12:
        overshoot_pct = max(0.0, (peak - y_final) / abs(y_final) * 100.0)

    # settling time: earliest time after which response stays within band
    band_hi = y_final * (1.0 + settling_band)
    band_lo = y_final * (1.0 - settling_band)
    lo, hi = (min(band_lo, band_hi), max(band_lo, band_hi))

    inside = (y >= lo) & (y <= hi)
    settling_time = np.nan
    for i in range(len(t)):
        if inside[i] and np.all(inside[i:]):
            settling_time = float(t[i])
            break

    steady_state_error = float(abs(y_final - ref))

    return {
        "y_final": float(y_final),
        "peak": peak,
        "overshoot_pct": float(overshoot_pct),
        "settling_time": settling_time,
        "steady_state_error": steady_state_error,
    }