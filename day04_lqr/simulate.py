import numpy as np
from dataclasses import dataclass
from typing import Dict, Optional
from scipy.integrate import solve_ivp


@dataclass
class SimConfig:
    t0: float = 0.0
    tf: float = 10.0
    dt: float = 0.002
    u_clip: Optional[float] = None  # e.g. 20.0 to simulate saturation


def simulate_state_feedback(
    A: np.ndarray,
    B: np.ndarray,
    K: np.ndarray,
    x0: np.ndarray,
    cfg: SimConfig,
) -> Dict[str, np.ndarray]:
    """
    Simulate closed-loop:
        ẋ = A x + B u
        u = -K x (optionally clipped)

    Returns:
        dict with keys: t (N,), x (N,n), u (N,)
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    K = np.asarray(K, dtype=float)
    x0 = np.asarray(x0, dtype=float).reshape(-1)

    n = A.shape[0]
    if A.shape != (n, n):
        raise ValueError(f"A must be square (got {A.shape}).")
    if B.shape[0] != n:
        raise ValueError(f"B rows must match A (got B {B.shape}, A {A.shape}).")
    if K.shape[1] != n:
        raise ValueError(f"K must be (m,n) (got {K.shape}).")
    if x0.shape != (n,):
        raise ValueError(f"x0 must be shape (n,) (got {x0.shape}).")

    def u_of_x(x: np.ndarray) -> float:
        x = np.asarray(x, dtype=float).reshape(-1)
        u = float(-(K @ x.reshape(-1, 1))[0, 0])
        if cfg.u_clip is not None:
            u = float(np.clip(u, -cfg.u_clip, cfg.u_clip))
        return u

    def dyn(t, x):
        x = np.asarray(x, dtype=float).reshape(-1)
        u = u_of_x(x)
        dx = (A @ x.reshape(-1, 1) + B * u).reshape(-1)
        return dx

    t_eval = np.arange(cfg.t0, cfg.tf + cfg.dt, cfg.dt, dtype=float)

    sol = solve_ivp(
        fun=dyn,
        t_span=(cfg.t0, cfg.tf),
        y0=x0,
        t_eval=t_eval,
        rtol=1e-8,
        atol=1e-10,
        method="RK45",
    )

    if not sol.success:
        raise RuntimeError(f"Integration failed: {sol.message}")

    X = sol.y.T  # (N, n)
    U = np.array([u_of_x(X[i]) for i in range(X.shape[0])], dtype=float)

    return {"t": sol.t, "x": X, "u": U}


def metrics(t: np.ndarray, x: np.ndarray, u: np.ndarray) -> Dict[str, float]:
    """
    Simple metrics:
      - peak_abs_pos: max |position|
      - settling_time_2pct: first time after which |pos| stays within 2% of |pos(0)|
      - u_rms: RMS of control input
    """
    t = np.asarray(t, dtype=float)
    x = np.asarray(x, dtype=float)
    u = np.asarray(u, dtype=float)

    pos = x[:, 0]
    x0 = float(pos[0])

    peak_abs_pos = float(np.max(np.abs(pos)))
    u_rms = float(np.sqrt(np.mean(u**2)))

    tol = 0.02 * max(1e-9, abs(x0))
    settled_idx = None
    for i in range(len(t)):
        if np.all(np.abs(pos[i:]) <= tol):
            settled_idx = i
            break
    ts = float(t[settled_idx]) if settled_idx is not None else float("nan")

    return {"peak_abs_pos": peak_abs_pos, "settling_time_2pct": ts, "u_rms": u_rms}