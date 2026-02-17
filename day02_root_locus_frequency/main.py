import os
import numpy as np
from model import get_transfer_function
from simulate import (
    plot_root_locus,
    plot_bode_with_margins,
    gain_sweep_poles,
    plot_gain_sweep_poles,
)


def fmt_val(x):
    """Pretty print for nan/inf floats."""
    if x is None:
        return "None"
    if isinstance(x, (float, np.floating)):
        if np.isnan(x):
            return "nan"
        if np.isinf(x):
            return "inf"
    return str(x)


def main():
    os.makedirs("results", exist_ok=True)

    G = get_transfer_function()

    # 1) Root Locus
    plot_root_locus(G, out_path="results/root_locus.png")

    # 2) Bode + margins
    gm, pm, wg, wp = plot_bode_with_margins(G, out_path="results/bode_plot.png")

    print("\nFrequency Margins (Open-loop G(s))")
    print(f"Gain margin (gm): {fmt_val(gm)}")
    print(f"Phase margin (pm): {fmt_val(pm)} deg")
    print(f"Gain crossover freq (wg): {fmt_val(wg)} rad/s")
    print(f"Phase crossover freq (wp): {fmt_val(wp)} rad/s")

    # 3) Gain sweep poles + plot
    gains, poles_list = gain_sweep_poles(G, K_min=0.0, K_max=50.0, N=200)
    plot_gain_sweep_poles(gains, poles_list, out_path="results/gain_sweep_poles.png")

    print("\nSaved artifacts:")
    print(" - results/root_locus.png")
    print(" - results/bode_plot.png")
    print(" - results/gain_sweep_poles.png\n")


if __name__ == "__main__":
    main()