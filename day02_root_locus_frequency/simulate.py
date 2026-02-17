import numpy as np
import matplotlib.pyplot as plt
import control


def plot_root_locus(G, out_path="results/root_locus.png"):
    """
    Root locus plot for open-loop G(s).
    """
    fig = plt.figure()
    control.root_locus(G, grid=True)
    plt.title("Root Locus: G(s)")
    plt.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)


def plot_bode_with_margins(G, out_path="results/bode_plot.png"):
    """
    Bode plot for open-loop G(s) + compute gain/phase margins.

    Returns:
      gm: gain margin (absolute, not dB). Can be inf if no -180° crossing.
      pm: phase margin (deg)
      wg: frequency for gain margin (rad/s). Can be nan if gm is inf.
      wp: frequency for phase margin (rad/s)
    """
    gm, pm, wg, wp = control.margin(G)

    fig = plt.figure()
    control.bode_plot(G, dB=True, deg=True)
    fig.suptitle("Bode Plot: G(s)", y=0.98)
    plt.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)

    return gm, pm, wg, wp


def gain_sweep_poles(G, K_min=0.0, K_max=50.0, N=200):
    """
    Gain sweep: closed-loop poles of T(s) = feedback(K*G, 1)
    for K in [K_min, K_max].

    Returns:
      gains: (N,) array
      poles_list: list of arrays of poles for each K
    """
    gains = np.linspace(K_min, K_max, N)
    poles_list = []

    for K in gains:
        T = control.feedback(K * G, 1)
        poles = control.poles(T)  # ✅ correct API
        poles_list.append(poles)

    return gains, poles_list


def plot_gain_sweep_poles(gains, poles_list, out_path="results/gain_sweep_poles.png"):
    """
    Plot how closed-loop poles move as K increases.
    """
    fig = plt.figure()

    # Plot real and imaginary parts for each pole over the sweep
    # (Scatter in s-plane)
    for i, K in enumerate(gains):
        poles = poles_list[i]
        plt.scatter(np.real(poles), np.imag(poles), s=8)

    plt.axhline(0, linewidth=1)
    plt.axvline(0, linewidth=1)
    plt.title("Closed-Loop Pole Movement (Gain Sweep)")
    plt.xlabel("Real(s)")
    plt.ylabel("Imag(s)")
    plt.grid(True)
    plt.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)