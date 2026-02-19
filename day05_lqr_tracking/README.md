# Day 05: LQR Tracking (Integral Augmentation)

## Objective

Extend standard continuous-time LQR from **regulation** (drive state →
0) to **reference tracking** (drive output → r).

Standard LQR minimizes:

$$J = ∫₀^∞ (xᵀQx + uᵀRu) dt$$

But it does **not inherently guarantee zero steady-state error** for
nonzero references.

This implementation introduces **integral augmentation** to enforce
asymptotic tracking of constant references.

------------------------------------------------------------------------

## System Model

Second-order mass-spring-damper:

$$ẍ + 2ζωₙ ẋ + ωₙ² x = u$$

State-space representation:

```
x₁ = x
x₂ = ẋ

ẋ = [ 0 1 -ωₙ² -2ζωₙ] x + [0 1] u
```

Output:

```
y = [1 0] x
```

Parameters used:

-   $ωₙ = 1.0$
-   $ζ = 0.2$

------------------------------------------------------------------------

## Why Plain LQR Fails for Tracking

Plain LQR control law:

$$u = -Kx$$

It regulates the state to zero.
If a step reference r = 1 is applied:

-   The controller does not "see" the reference.
-   It stabilizes at x = 0.
-   Steady-state error = 1.

This is expected behavior.

------------------------------------------------------------------------

## Integral Augmentation

Define integral state:

$$x_I = ∫ (r - y) dt$$

$$ẋ_I = r - Cx$$

Augmented state:

```
x_a = [ x x_I ]
```

Augmented dynamics:

```
ẋ_a = [ A 0 -C 0 ] x_a + [ B 0 ] u + [ 0 1 ] r
```

LQR is applied to the augmented system:

$$u = -K_x x - K_I x_I$$

This forces:

$$_{lim (t→∞)} (r - y) = 0$$

for constant references.

------------------------------------------------------------------------

## Controller Gains (Simulation Output)

Plain LQR:

```
K = [[2.3166 2.0069]]
```

Integral LQR:

```
Kx = [[7.1644 3.5356]]
Ki = [[-7.0711]]
```

Note: Negative $K_I$ is expected due to sign convention in the control
law.

------------------------------------------------------------------------

## Simulation Results

Artifacts saved in `results/`:

-   tracking_response.png
-   control_effort.png
-   steady_state_error.png

### Observations

-   Plain LQR → output remains at zero.
-   LQR + Integral → tracks reference to 1.
-   Steady-state error → driven to zero.
-   Slight increase in control effort due to integral action.
-   Small overshoot (\~2--3%), stable convergence.

------------------------------------------------------------------------

## Design Tradeoff

```
  Controller       Steady-State Error   Control Effort   Overshoot
  ---------------- -------------------- ---------------- -----------
  Plain LQR        Nonzero              Minimal          None
  LQR + Integral   Zero                 Higher           Small
```

Integral augmentation improves tracking at the cost of increased
actuation demand.

------------------------------------------------------------------------

## Numerical Method

-   Continuous-time Riccati equation solved using `solve_continuous_are`.
-   4th-order Runge-Kutta integration (RK4).
-   Optional input saturation included for realism.

------------------------------------------------------------------------

## Key Takeaways

-   Standard LQR is a regulator, not a tracker.
-   Integral state augmentation enforces zero steady-state error.
-   Optimal control framework naturally incorporates integral action.
-   Tracking performance depends strongly on Q/R weighting of integrator
    state.

------------------------------------------------------------------------

## What Broke / What Was Tricky

-   Careful handling of NumPy shape issues for scalar outputs.
-   Tuning integral weight: too small → aggressive oscillations; too
    large → slow tracking.
-   Ensuring sign convention consistency for $K_I$.

------------------------------------------------------------------------

## Engineering Insight

Integral augmentation transforms LQR from a stabilization tool into a
practical tracking controller.

This concept generalizes directly to:

-   UAV altitude hold.
-   Spacecraft attitude tracking.
-   Bias rejection in guidance systems.
-   MPC tracking formulations.
-   Hybrid optimal + adaptive control frameworks.

------------------------------------------------------------------------