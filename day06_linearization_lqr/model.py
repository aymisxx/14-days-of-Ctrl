import numpy as np


def pendulum_nonlinear(t, x, u, params):
    """
    Nonlinear pendulum dynamics:
        theta_dot = omega
        omega_dot = -(g/l) sin(theta) + u/(m l^2)
    """
    theta, omega = x
    g = params["g"]
    l = params["l"]
    m = params["m"]

    theta_dot = omega
    omega_dot = -(g / l) * np.sin(theta) + u / (m * l**2)

    return np.array([theta_dot, omega_dot], dtype=float)


def pendulum_linearized(params):
    """
    Linearization about theta ~ 0 (downward equilibrium):
        sin(theta) ~ theta
    """
    g = params["g"]
    l = params["l"]
    m = params["m"]

    A = np.array([
        [0.0, 1.0],
        [-(g / l), 0.0]
    ], dtype=float)

    B = np.array([
        [0.0],
        [1.0 / (m * l**2)]
    ], dtype=float)

    return A, B