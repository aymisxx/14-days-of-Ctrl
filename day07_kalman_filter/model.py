# day07_kalman_filter/model.py
from __future__ import annotations

from dataclasses import dataclass
import numpy as np


def _as_2d(a: np.ndarray) -> np.ndarray:
    a = np.asarray(a, dtype=float)
    if a.ndim == 0:
        return a.reshape(1, 1)
    if a.ndim == 1:
        return a.reshape(-1, 1)
    if a.ndim == 2:
        return a
    raise ValueError(f"Array must be <=2D, got shape {a.shape}")


def _symmetrize(P: np.ndarray) -> np.ndarray:
    return 0.5 * (P + P.T)


def _is_psd(M: np.ndarray, tol: float = 1e-10) -> bool:
    M = _symmetrize(M)
    # PSD check via eigenvalues (small negative due to numerical noise allowed)
    eigvals = np.linalg.eigvalsh(M)
    return bool(np.min(eigvals) >= -tol)


@dataclass(frozen=True)
class LinearGaussianSystem:
    """
    Discrete-time linear system with Gaussian process/measurement noise:
      x_{k+1} = A x_k + B u_k + w_k,   w_k ~ N(0, Q)
      y_k     = C x_k + v_k,           v_k ~ N(0, R)
    """
    A: np.ndarray
    B: np.ndarray
    C: np.ndarray
    Q: np.ndarray
    R: np.ndarray

    def __post_init__(self):
        A = _as_2d(self.A)
        B = _as_2d(self.B)
        C = _as_2d(self.C)
        Q = _as_2d(self.Q)
        R = _as_2d(self.R)

        n = A.shape[0]
        if A.shape != (n, n):
            raise ValueError(f"A must be square, got {A.shape}")

        if B.shape[0] != n:
            raise ValueError(f"B rows must match A, got B {B.shape}, A {A.shape}")

        m = C.shape[0]
        if C.shape[1] != n:
            raise ValueError(f"C cols must match A, got C {C.shape}, A {A.shape}")

        if Q.shape != (n, n):
            raise ValueError(f"Q must be (n,n), got {Q.shape}, n={n}")
        if R.shape != (m, m):
            raise ValueError(f"R must be (m,m), got {R.shape}, m={m}")

        if not _is_psd(Q):
            raise ValueError("Q must be PSD (process noise covariance).")
        if not _is_psd(R):
            raise ValueError("R must be PSD (measurement noise covariance).")


def constant_velocity_1d(dt: float,
                         q_pos: float = 1e-3,
                         q_vel: float = 1e-3,
                         r_meas: float = 1e-2) -> LinearGaussianSystem:
    """
    1D constant-velocity model with optional acceleration control input u (scalar).
      x = [position, velocity]^T
      u = acceleration (assumed constant over dt)
    """
    if dt <= 0:
        raise ValueError("dt must be > 0")

    A = np.array([[1.0, dt],
                  [0.0, 1.0]], dtype=float)

    # acceleration input: p += 0.5*a*dt^2, v += a*dt
    B = np.array([[0.5 * dt * dt],
                  [dt]], dtype=float)

    # measure position only
    C = np.array([[1.0, 0.0]], dtype=float)

    Q = np.diag([float(q_pos), float(q_vel)])
    R = np.array([[float(r_meas)]], dtype=float)

    return LinearGaussianSystem(A=A, B=B, C=C, Q=Q, R=R)


class KalmanFilter:
    """
    Standard discrete-time Kalman Filter with Joseph-form covariance update
    for numerical stability.

    State estimate: x_hat (n,1)
    Covariance:     P     (n,n)
    """
    def __init__(self, system: LinearGaussianSystem, x0: np.ndarray, P0: np.ndarray):
        self.sys = system
        self.x = _as_2d(x0)
        self.P = _as_2d(P0)

        n = self.sys.A.shape[0]
        if self.x.shape != (n, 1):
            raise ValueError(f"x0 must be (n,1), got {self.x.shape}, n={n}")
        if self.P.shape != (n, n):
            raise ValueError(f"P0 must be (n,n), got {self.P.shape}, n={n}")
        if not _is_psd(self.P):
            raise ValueError("P0 must be PSD.")

        self.K_last = None
        self.innov_last = None

    def predict(self, u: np.ndarray | float = 0.0) -> tuple[np.ndarray, np.ndarray]:
        A, B, Q = self.sys.A, self.sys.B, self.sys.Q
        u = _as_2d(u)
        if u.shape != (B.shape[1], 1):
            raise ValueError(f"u must be ({B.shape[1]},1), got {u.shape}")

        self.x = A @ self.x + B @ u
        self.P = _symmetrize(A @ self.P @ A.T + Q)

        # Basic sanity
        if not np.all(np.isfinite(self.x)):
            raise FloatingPointError("Non-finite state after predict.")
        if not np.all(np.isfinite(self.P)):
            raise FloatingPointError("Non-finite covariance after predict.")
        return self.x.copy(), self.P.copy()

    def update(self, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        C, R = self.sys.C, self.sys.R
        y = _as_2d(y)

        m = C.shape[0]
        if y.shape != (m, 1):
            raise ValueError(f"y must be (m,1), got {y.shape}, m={m}")

        # Innovation
        y_hat = C @ self.x
        innov = y - y_hat

        # Innovation covariance
        S = C @ self.P @ C.T + R
        S = _symmetrize(S)

        # Robust inverse (S should be SPD if R>0; use solve instead of inv)
        try:
            # K = P C^T S^{-1}
            K = np.linalg.solve(S.T, (self.P @ C.T).T).T
        except np.linalg.LinAlgError as e:
            raise np.linalg.LinAlgError(f"Failed to solve for Kalman gain; S may be singular. {e}")

        I = np.eye(self.P.shape[0])

        # State update
        self.x = self.x + K @ innov

        # Joseph-form covariance update (numerically stable)
        KC = K @ C
        self.P = _symmetrize((I - KC) @ self.P @ (I - KC).T + K @ R @ K.T)

        self.K_last = K.copy()
        self.innov_last = innov.copy()

        # Sanity
        if not np.all(np.isfinite(self.x)):
            raise FloatingPointError("Non-finite state after update.")
        if not np.all(np.isfinite(self.P)):
            raise FloatingPointError("Non-finite covariance after update.")
        if not _is_psd(self.P, tol=1e-8):
            # Joseph form *should* preserve PSD; if not, something is off.
            raise FloatingPointError("Covariance lost PSD property (unexpected).")

        return self.x.copy(), self.P.copy()