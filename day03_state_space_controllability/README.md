# Day 03: **State-Space Formulation & Controllability**

# Objective

The goal of Day 03 is to transition from transfer-function-based
modeling to full **state-space representation** and verify structural
system properties using:

-   State-space conversion.
-   Controllability matrix computation.
-   Rank condition verification.
-   Numerical simulation of state evolution.
-   Validation against transfer function response.

This day marks the conceptual shift from classical scalar dynamics to
modern state-based control.

------------------------------------------------------------------------

# Problem Formulation

We consider the standard second-order LTI system.

For simulation:

-   $ζ = 0.5$.
-   $ωₙ = 2.0$.

------------------------------------------------------------------------

# Transfer Function Representation

Taking Laplace Transform (zero initial conditions):

$$s²Y(s) + 2ζωₙ sY(s) + ωₙ² Y(s) = U(s)$$

$$G(s) = Y(s)/U(s) = 1 / (s² + 2ζωₙ s + ωₙ²)$$

Substituting parameters:

$$G(s) = 1 / (s² + 2s + 4)$$

DC gain:

$$G(0) = 1/4 = 0.25$$

This matches the steady-state value observed in simulation.

------------------------------------------------------------------------

# State-Space Conversion (Absolute Detail)

Define state variables:

$$x₁ = y$$

$$x₂ = ẏ$$

Then:

$$ẋ₁ = x₂$$

$$ẋ₂ = -2ζωₙ x₂ - ωₙ² x₁ + u$$

------------------------------------------------------------------------

## State-Space Form

$$ẋ = Ax + Bu$$

$$y = Cx + Du$$

Where:

```
A = [ [0, 1],
    [-4, -2]]

B = [[0],
    [1]]

C = [[1, 0]]

D = [[0]]
```

------------------------------------------------------------------------

# Controllability Theory

A system is **controllable** if it can be driven from any initial state
to any final state in finite time using input $u(t)$.

Controllability matrix:

𝒞 = \[B AB A²B ... Aⁿ⁻¹B\]

System is controllable if:

$$rank(𝒞) = n$$

------------------------------------------------------------------------

## Controllability Calculation (n = 2)

𝒞 = \[B AB\]

AB = A·B = \[\[1\], \[-2\]\]

Thus:

```
𝒞 = [[0, 1],
    [1, -2]]
```

$$det(𝒞) = -1 ≠ 0$$

$$⇒ rank = 2$$

**The system is fully controllable.**

------------------------------------------------------------------------

# Simulation Methodology

We simulate:

$$ẋ = Ax + Bu$$

Using:

-   `scipy.integrate.solve_ivp`.
-   Unit step input $u(t)=1$.
-   Zero initial state.
-   Time horizon: 0-10 seconds.

Transfer function response is computed using:

scipy.signal.TransferFunction

Both responses are overlaid for validation.

------------------------------------------------------------------------

# Results

-   Steady-state value ≈ 0.25.
-   Underdamped behavior (ζ = 0.5).
-   Peak ≈ 0.29.
-   Overshoot ≈ 16%.
-   State-space and transfer function responses perfectly overlap.

This confirms:

-   Model correctness.
-   Structural equivalence.
-   No simulation inconsistency.
-   Correct parameter substitution.

------------------------------------------------------------------------

# Eigenvalue Verification

Eigenvalues of A:

$$λ = -1 ± j√3$$

These match roots of:

$$s² + 2s + 4 = 0$$

Confirms equivalence between transfer function poles and state matrix
eigenvalues.

------------------------------------------------------------------------

# Folder Structure

```
    day03_state_space_controllability/
    │
    ├── main.py
    ├── model.py
    ├── simulate.py
    ├── README.md
    └── results/
        ├── state_response.png
        └── controllability_check.txt
```

------------------------------------------------------------------------

# How to Execute

From repository root:

```
    cd day03_state_space_controllability
    python main.py
```

Outputs:

-   Console matrices and rank
-   results/state_response.png
-   results/controllability_check.txt

------------------------------------------------------------------------

# Results Discussion

1.  State-space and transfer-function responses match exactly.
2.  DC gain equals $1/ωₙ²$.
3.  System is fully controllable → pole placement & LQR possible.
4.  Underdamped response confirms correct damping implementation.

------------------------------------------------------------------------

# Engineering Interpretation

-   Transfer functions hide internal structure.
-   State-space reveals internal dynamics.
-   Controllability determines feasibility of optimal control.
-   Modern control methods require state-space representation.

Without controllability:

-   LQR fails.
-   Pole placement fails.
-   MPC becomes meaningless.

This day verifies structural controllability before optimal control
design.

------------------------------------------------------------------------

# Key Takeaways

-   State-space exposes internal system structure.
-   Controllability is a rank condition.
-   Poles = eigenvalues of A.
-   Structural validation must precede controller design.

------------------------------------------------------------------------

# What Was Tricky

-   Constructing controllability matrix correctly.
-   Avoiding simulation shape mismatches.
-   Verifying DC gain.
-   Understanding structural vs numerical controllability.

------------------------------------------------------------------------