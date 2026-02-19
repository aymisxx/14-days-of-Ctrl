# day05_lqr_tracking/lqr.py
import numpy as np
from scipy.linalg import solve_continuous_are

def lqr_continuous(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray):
    """
    Continuous-time LQR:
        Solve CARE: A'P + P A - P B R^-1 B' P + Q = 0
        K = R^-1 B' P
        u = -K x
    """
    assert A.shape[0] == A.shape[1], "A must be square"
    assert B.shape[0] == A.shape[0], "B rows must match A"
    assert Q.shape == A.shape, "Q must match A shape"
    assert R.shape[0] == R.shape[1] == B.shape[1], "R must be (m x m)"

    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)  # R^-1 B' P
    return K, P