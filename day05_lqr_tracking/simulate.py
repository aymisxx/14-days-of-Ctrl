# day05_lqr_tracking/simulate.py
import numpy as np

def rk4_step(f, t, x, dt):
    k1 = f(t, x)
    k2 = f(t + dt/2.0, x + dt*k1/2.0)
    k3 = f(t + dt/2.0, x + dt*k2/2.0)
    k4 = f(t + dt, x + dt*k3)
    return x + (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)

def _scalar_y(C: np.ndarray, x: np.ndarray) -> float:
    """
    Robust scalar output y = Cx for C shape (1,n) and x shape (n,) or (n,1).
    Avoids numpy float conversion issues (e.g., float(array([..]))).
    """
    x = x.reshape(-1, 1)          # (n,1)
    return (C @ x)[0, 0]          # scalar


def simulate_tracking(A, B, C, Kx, Ki, r_func, x0=None, xI0=0.0, T=10.0, dt=0.001, u_limit=None):
    """
    Simulate augmented LQR controller:
        u = -Kx x - Ki xI
        xI_dot = r - Cx

    Returns time series dict.
    """
    n = A.shape[0]
    m = B.shape[1]

    if x0 is None:
        x0 = np.zeros((n,))
    x = np.array(x0, dtype=float).reshape(n,)
    xI = float(xI0)

    Kx = np.array(Kx, dtype=float).reshape(m, n)
    Ki = np.array(Ki, dtype=float).reshape(m, 1)

    N = int(np.floor(T / dt)) + 1
    t_arr = np.linspace(0.0, T, N)

    xs = np.zeros((N, n))
    y = np.zeros((N, 1))
    u = np.zeros((N, m))
    r = np.zeros((N, 1))
    e = np.zeros((N, 1))
    xIs = np.zeros((N, 1))

    def dyn(t, state):
        # state = [x; xI]
        x_local = state[:n]
        xI_local = state[n:]  # shape (1,)

        r_t = float(r_func(t))
        y_t = _scalar_y(C, x_local)
        e_t = r_t - y_t

        u_t = - (Kx @ x_local.reshape(n, 1) + Ki * xI_local.reshape(1, 1))
        u_t = u_t.reshape(m,)

        if u_limit is not None:
            u_t = np.clip(u_t, -abs(u_limit), abs(u_limit))

        x_dot = (A @ x_local.reshape(n, 1) + B @ u_t.reshape(m, 1)).reshape(n,)
        xI_dot = np.array([e_t], dtype=float)  # integrator tracks error

        return np.concatenate([x_dot, xI_dot])

    state = np.concatenate([x, np.array([xI])])

    for i, t in enumerate(t_arr):
        x = state[:n]
        xI = state[n]

        r_t = float(r_func(t))
        y_t = _scalar_y(C, x)
        e_t = r_t - y_t

        u_t = - (Kx @ x.reshape(n, 1) + Ki * np.array([[xI]]))
        u_t = u_t.reshape(m,)
        if u_limit is not None:
            u_t = np.clip(u_t, -abs(u_limit), abs(u_limit))

        xs[i] = x
        xIs[i] = xI
        y[i] = y_t
        r[i] = r_t
        e[i] = e_t
        u[i] = u_t

        if i < N - 1:
            state = rk4_step(dyn, t, state, dt)

    return {
        "t": t_arr,
        "x": xs,
        "xI": xIs,
        "y": y,
        "r": r,
        "e": e,
        "u": u,
    }


def simulate_plain_lqr(A, B, C, K, r_func, x0=None, T=10.0, dt=0.001, u_limit=None):
    """
    Plain LQR regulator used for tracking anyway:
        u = -K x
    (This typically yields nonzero steady-state error for step references.)
    """
    n = A.shape[0]
    m = B.shape[1]

    if x0 is None:
        x0 = np.zeros((n,))
    x = np.array(x0, dtype=float).reshape(n,)

    K = np.array(K, dtype=float).reshape(m, n)

    N = int(np.floor(T / dt)) + 1
    t_arr = np.linspace(0.0, T, N)

    xs = np.zeros((N, n))
    y = np.zeros((N, 1))
    u = np.zeros((N, m))
    r = np.zeros((N, 1))
    e = np.zeros((N, 1))

    def dyn(t, x_local):
        # r_t not used in control; only regulator action
        u_t = -(K @ x_local.reshape(n, 1)).reshape(m,)
        if u_limit is not None:
            u_t = np.clip(u_t, -abs(u_limit), abs(u_limit))
        x_dot = (A @ x_local.reshape(n, 1) + B @ u_t.reshape(m, 1)).reshape(n,)
        return x_dot

    for i, t in enumerate(t_arr):
        r_t = float(r_func(t))
        y_t = _scalar_y(C, x)
        e_t = r_t - y_t

        u_t = -(K @ x.reshape(n, 1)).reshape(m,)
        if u_limit is not None:
            u_t = np.clip(u_t, -abs(u_limit), abs(u_limit))

        xs[i] = x
        y[i] = y_t
        r[i] = r_t
        e[i] = e_t
        u[i] = u_t

        if i < N - 1:
            x = rk4_step(lambda tt, xx: dyn(tt, xx), t, x, dt)

    return {
        "t": t_arr,
        "x": xs,
        "y": y,
        "r": r,
        "e": e,
        "u": u,
    }