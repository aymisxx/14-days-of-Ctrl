import numpy as np
from scipy.integrate import solve_ivp
from scipy import signal

def simulate_state_space(A, B, C, D, t, x0=None, u_step=1.0):
    """
    Simulates xdot = Ax + Bu for a constant step input u = u_step.
    """
    n = A.shape[0]
    if x0 is None:
        x0 = np.zeros(n)

    B_vec = B.reshape(n,)  # force shape (n,)

    def dynamics(_t, x):
        u = u_step
        return A @ x + B_vec * u

    sol = solve_ivp(
        dynamics,
        (t[0], t[-1]),
        x0,
        t_eval=t,
        rtol=1e-8,
        atol=1e-10
    )

    # Output: y = Cx + Du
    y = (C @ sol.y) + (D * u_step)  # y shape (1, N)

    return sol.t, sol.y, y


def simulate_transfer_function(zeta=0.5, wn=2.0, t=None):
    """
    Transfer function of the same second-order system:
        G(s) = 1 / (s^2 + 2ζω_n s + ω_n^2)
    """
    num = [1.0]
    den = [1.0, 2.0*zeta*wn, wn**2]
    system = signal.TransferFunction(num, den)

    t_out, y_out = signal.step(system, T=t)
    return t_out, y_out