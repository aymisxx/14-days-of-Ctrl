import numpy as np

def build_state_space(zeta: float = 0.5, wn: float = 2.0):
    """
    Second-order system:
        y¨ + 2ζω_n y˙ + ω_n^2 y = u
    State choice:
        x1 = y, x2 = y˙
    """
    A = np.array([
        [0.0, 1.0],
        [-wn**2, -2.0*zeta*wn]
    ])

    B = np.array([
        [0.0],
        [1.0]
    ])

    C = np.array([[1.0, 0.0]])
    D = np.array([[0.0]])

    return A, B, C, D


def controllability_matrix(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """
    Builds controllability matrix C = [B, AB, A^2B, ..., A^{n-1}B]
    """
    n = A.shape[0]
    ctrb = B
    for i in range(1, n):
        ctrb = np.hstack((ctrb, np.linalg.matrix_power(A, i) @ B))
    return ctrb