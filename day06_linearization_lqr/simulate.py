import numpy as np
from utils import clamp


def rk4_step(f, t, x, u, dt, params):
    k1 = f(t, x, u, params)
    k2 = f(t + 0.5 * dt, x + 0.5 * dt * k1, u, params)
    k3 = f(t + 0.5 * dt, x + 0.5 * dt * k2, u, params)
    k4 = f(t + dt, x + dt * k3, u, params)
    return x + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


def _u_from_Kx(K, x):
    """
    Robust scalar control extraction.
    Works for K shapes: (1,2), (2,), etc.
    """
    Kvec = np.asarray(K, dtype=float).reshape(-1)   # (2,)
    xvec = np.asarray(x, dtype=float).reshape(-1)   # (2,)
    return float(-(Kvec @ xvec))                    # scalar


def simulate_nonlinear(f, x0, K, params, T=10.0, dt=0.001, method="rk4", u_limit=None):
    """
    Simulate nonlinear system with state feedback u = -Kx.
    method: "euler" or "rk4"
    u_limit: None or scalar or (umin, umax)
    """
    steps = int(T / dt)
    x = np.array(x0, dtype=float)

    time = np.zeros(steps)
    states = np.zeros((steps, len(x0)))
    controls = np.zeros(steps)

    for i in range(steps):
        t = i * dt

        u = _u_from_Kx(K, x)
        u = clamp(u, u_limit)

        if method.lower() == "euler":
            x_dot = f(t, x, u, params)
            x = x + dt * x_dot
        else:
            x = rk4_step(f, t, x, u, dt, params)

        time[i] = t
        states[i] = x
        controls[i] = u

    return time, states, controls


def simulate_linear(A, B, x0, K, T=10.0, dt=0.001, u_limit=None):
    """
    Simulate linear system xdot = Ax + Bu with u = -Kx.
    Euler integration is fine at small dt for this sprint.
    """
    steps = int(T / dt)
    x = np.array(x0, dtype=float)

    time = np.zeros(steps)
    states = np.zeros((steps, len(x0)))
    controls = np.zeros(steps)

    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float).reshape(-1)  # (2,)

    for i in range(steps):
        t = i * dt

        u = _u_from_Kx(K, x)
        u = clamp(u, u_limit)

        x_dot = A @ x + B * u
        x = x + dt * x_dot

        time[i] = t
        states[i] = x
        controls[i] = u

    return time, states, controls