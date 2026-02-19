# day05_lqr_tracking/model.py
import numpy as np

def mass_spring_damper(wn: float = 1.0, zeta: float = 0.2):
    """
    Second-order system:
        x_ddot + 2*zeta*wn*x_dot + wn^2*x = u
    State:
        x1 = x (position), x2 = x_dot (velocity)
        y  = x1
    """
    A = np.array([
        [0.0, 1.0],
        [-wn**2, -2.0*zeta*wn]
    ], dtype=float)

    B = np.array([
        [0.0],
        [1.0]
    ], dtype=float)

    C = np.array([[1.0, 0.0]], dtype=float)
    D = np.array([[0.0]], dtype=float)
    return A, B, C, D


def augment_with_integral(A: np.ndarray, B: np.ndarray, C: np.ndarray):
    """
    Integral augmentation for tracking constant reference r:
        xI_dot = r - y = r - Cx

    Augmented state xa = [x; xI]
    xa_dot = Aa * xa + Ba * u + Er * r
    """
    n = A.shape[0]

    Aa = np.block([
        [A, np.zeros((n, 1))],
        [-C, np.zeros((1, 1))]
    ])

    Ba = np.vstack([B, np.zeros((1, B.shape[1]))])

    # Reference enters only the integrator channel: xI_dot += r
    Er = np.vstack([np.zeros((n, 1)), np.ones((1, 1))])

    return Aa, Ba, Er