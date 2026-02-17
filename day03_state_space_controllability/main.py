import numpy as np
import os
import matplotlib.pyplot as plt

from model import build_state_space, controllability_matrix
from simulate import simulate_state_space, simulate_transfer_function


def main():
    os.makedirs("results", exist_ok=True)

    zeta = 0.5
    wn = 2.0

    A, B, C, D = build_state_space(zeta, wn)

    ctrb = controllability_matrix(A, B)
    rank = np.linalg.matrix_rank(ctrb)

    print("A matrix:\n", A)
    print("B matrix:\n", B)
    print("Controllability Matrix:\n", ctrb)
    print("Rank:", rank)

    with open("results/controllability_check.txt", "w") as f:
        f.write(f"A:\n{A}\n\n")
        f.write(f"B:\n{B}\n\n")
        f.write(f"Controllability Matrix:\n{ctrb}\n\n")
        f.write(f"Rank: {rank}\n")

    t = np.linspace(0, 10, 1000)

    t_ss, x_ss, y_ss = simulate_state_space(A, B, C, D, t, x0=[0, 0], u_step=1.0)
    t_tf, y_tf = simulate_transfer_function(zeta, wn, t)

    plt.figure()
    plt.plot(t_ss, y_ss.flatten(), label="State-Space")
    plt.plot(t_tf, y_tf, "--", label="Transfer Function")
    plt.xlabel("Time (s)")
    plt.ylabel("Output y(t)")
    plt.title("State-Space vs Transfer Function Step Response")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig("results/state_response.png")
    plt.close()


if __name__ == "__main__":
    main()