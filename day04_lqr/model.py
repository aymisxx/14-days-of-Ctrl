import numpy as np
from dataclasses import dataclass
from scipy.linalg import solve_continuous_are


@dataclass
class SecondOrderPlant:
    """
    Second-order system:
        ẍ + 2ζω ẋ + ω^2 x = u

    State x = [position, velocity]^T
        ẋ = A x + B u
    """
    w: float = 1.0
    zeta: float = 0.2

    def matrices(self):
        w = float(self.w)
        z = float(self.zeta)
        A = np.array([[0.0, 1.0],
                      [-(w**2), -2.0 * z * w]], dtype=float)
        B = np.array([[0.0],
                      [1.0]], dtype=float)
        return A, B


def lqr_gain(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray) -> np.ndarray:
    """
    Continuous-time LQR:
        minimize ∫ (x^T Q x + u^T R u) dt

    Solve CARE:
        A^T P + P A - P B R^-1 B^T P + Q = 0

    Gain:
        K = R^-1 B^T P
    """
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    Q = np.asarray(Q, dtype=float)
    R = np.asarray(R, dtype=float)

    n = A.shape[0]
    if A.shape != (n, n):
        raise ValueError(f"A must be square (got {A.shape}).")
    if B.shape[0] != n:
        raise ValueError(f"B rows must match A (got B {B.shape}, A {A.shape}).")
    if Q.shape != (n, n):
        raise ValueError(f"Q must be (n,n) (got {Q.shape}).")
    if R.shape[0] != R.shape[1] or R.shape[0] != B.shape[1]:
        raise ValueError(f"R must be (m,m) where m=B cols (got R {R.shape}, B {B.shape}).")

    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ (B.T @ P)
    return K


def pd_gain_for_second_order(kp: float = 10.0, kd: float = 3.0) -> np.ndarray:
    """
    PD baseline for ẍ + ... = u with state x=[x, ẋ].
    Control law: u = -kp*x - kd*ẋ  =>  K = [kp, kd]
    """
    return np.array([[float(kp), float(kd)]], dtype=float)