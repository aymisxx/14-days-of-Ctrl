import numpy as np
from scipy.linalg import solve_continuous_are


def lqr(A, B, Q, R):
    """
    Continuous-time LQR.
    Solves CARE: A'P + PA - PBR^-1B'P + Q = 0
    Returns K = R^-1 B' P
    """
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ (B.T @ P)
    return K


def clamp(u, u_limit=None):
    """
    Saturation (optional). If u_limit is None -> no clamp.
    u_limit can be scalar or (umin, umax)
    """
    if u_limit is None:
        return u

    if np.isscalar(u_limit):
        umin, umax = -float(u_limit), float(u_limit)
    else:
        umin, umax = float(u_limit[0]), float(u_limit[1])

    return float(np.clip(u, umin, umax))


def rms(x):
    x = np.asarray(x, dtype=float)
    return float(np.sqrt(np.mean(x**2)))


def peak_abs(x):
    x = np.asarray(x, dtype=float)
    return float(np.max(np.abs(x)))


def settling_time(t, y, band=0.02):
    """
    Rough settling time estimate: time when |y| stays within band * |y0|
    for the rest of the simulation.
    If y0 is tiny, uses absolute band instead.
    """
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)

    if len(t) == 0:
        return None

    y0 = abs(y[0])
    thresh = band * y0 if y0 > 1e-6 else band

    inside = np.abs(y) <= thresh
    # find earliest index i such that all j>=i are inside band
    for i in range(len(t)):
        if np.all(inside[i:]):
            return float(t[i])
    return None