import numpy as np

def first_order(t, y, tau=1.0, u=1.0):
    """
    First-order system: tau*dy/dt + y = u
    State: y = [y]
    """
    return [(u - y[0]) / tau]

def second_order_standard(t, x, wn=1.0, zeta=0.7, u=1.0):
    """
    Standard 2nd-order system:
        y'' + 2*zeta*wn*y' + wn^2*y = wn^2*u
    State: x = [y, ydot]
    """
    y, ydot = x
    yddot = wn**2 * (u - y) - 2.0 * zeta * wn * ydot
    return [ydot, yddot]

def msd_plant(t, x, m=1.0, b=0.5, k=1.0, u=0.0):
    """
    Mass-spring-damper plant:
        m*y'' + b*y' + k*y = u
    State: x = [y, ydot]
    """
    y, ydot = x
    yddot = (u - b * ydot - k * y) / m
    return [ydot, yddot]