# day07_kalman_filter/simulate.py
from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class SimulationData:
    t: np.ndarray            # (T,)
    x_true: np.ndarray       # (T, n)
    y_meas: np.ndarray       # (T, m)
    u: np.ndarray            # (T, nu)


def _randn_cov(rng: np.random.Generator, cov: np.ndarray, size: int) -> np.ndarray:
    """
    Generate samples ~ N(0, cov), returns shape (size, dim)
    Uses Cholesky when possible; falls back to eig.
    """
    cov = np.asarray(cov, dtype=float)
    dim = cov.shape[0]
    try:
        L = np.linalg.cholesky(0.5 * (cov + cov.T))
        z = rng.standard_normal((size, dim))
        return z @ L.T
    except np.linalg.LinAlgError:
        # PSD but not PD
        w, V = np.linalg.eigh(0.5 * (cov + cov.T))
        w = np.clip(w, 0.0, None)
        L = V @ np.diag(np.sqrt(w))
        z = rng.standard_normal((size, dim))
        return z @ L.T


def simulate_linear_system(system,
                           x0: np.ndarray,
                           T: int = 300,
                           dt: float = 0.1,
                           u_policy: str = "sine",
                           seed: int = 7) -> SimulationData:
    """
    Simulate:
      x_{k+1} = A x_k + B u_k + w_k
      y_k     = C x_k + v_k
    """
    A, B, C, Q, R = system.A, system.B, system.C, system.Q, system.R

    x0 = np.asarray(x0, dtype=float).reshape(-1, 1)
    n = A.shape[0]
    m = C.shape[0]
    nu = B.shape[1]
    if x0.shape != (n, 1):
        raise ValueError(f"x0 must be (n,1), got {x0.shape}, n={n}")

    rng = np.random.default_rng(seed)

    t = np.arange(T, dtype=float) * dt
    x_true = np.zeros((T, n), dtype=float)
    y_meas = np.zeros((T, m), dtype=float)
    u = np.zeros((T, nu), dtype=float)

    # Build control signal
    if u_policy == "zero":
        u[:, :] = 0.0
    elif u_policy == "sine":
        # smooth-ish acceleration
        for k in range(T):
            u[k, 0] = 0.6 * np.sin(0.35 * t[k])
    elif u_policy == "steps":
        for k in range(T):
            u[k, 0] = 0.7 if (k // 60) % 2 == 0 else -0.3
    else:
        raise ValueError("u_policy must be one of: zero, sine, steps")

    w = _randn_cov(rng, Q, size=T)  # (T, n)
    v = _randn_cov(rng, R, size=T)  # (T, m)

    x = x0.copy()
    for k in range(T):
        x_true[k] = x.ravel()
        y_meas[k] = (C @ x).ravel() + v[k]

        uk = u[k].reshape(nu, 1)
        # state propagation with process noise
        x = A @ x + B @ uk + w[k].reshape(n, 1)

    return SimulationData(t=t, x_true=x_true, y_meas=y_meas, u=u)