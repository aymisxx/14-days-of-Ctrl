# Day 06: **Linearization** (LQR)

### Nonlinear Pendulum → Linearization → LQR

------------------------------------------------------------------------

## Objective

Demonstrate:

1. Nonlinear system modeling.
2. Local linearization about equilibrium.
3. LQR controller design on the linear model.
4. Performance comparison on both linear and nonlinear dynamics.

This experiment highlights the **validity region of linearization** and
the robustness of LQR under moderate model mismatch.

------------------------------------------------------------------------

## System Model

### Nonlinear Pendulum

$$\dot{\theta} = \omega$$

$$\dot{\omega} = -\frac{g}{l}\sin(\theta) + \frac{1}{ml^2}u$$

**Parameters used:**

-   $g$ = 9.81.
-   $l$ = 1.0.
-   $m$ = 1.0.

------------------------------------------------------------------------

### Linearization (about θ ≈ 0)

Using small-angle approximation:

$$sin(\theta) \approx (\theta)$$

State-space form:

$$\dot{x} = Ax + Bu$$

$$A = \begin{bmatrix} 0 & 1 \\ -\frac{g}{l} & 0 \end{bmatrix} \quad$$

$$B = \begin{bmatrix} 0 \\ \frac{1}{ml^2} \end{bmatrix}$$

This model is valid locally near the downward equilibrium.

------------------------------------------------------------------------

## Controller Design

Continuous-time LQR solves the CARE:

$$A^T P + P A - P B R^{-1} B^T P + Q = 0$$

Control law:

$$u = -Kx$$

**Weights used:**

$$Q = \text{diag}(20, 2), \quad R = 0.5$$

Resulting gain:

```
    K = [[1.8620  2.7792]]
```

Interpretation:

-   Strong penalty on angle error.
-   Moderate penalty on angular velocity.
-   Balanced control aggressiveness.

------------------------------------------------------------------------

## Simulation Setup

-   Time horizon: 10 seconds.
-   dt = 0.001.
-   Nonlinear integration: RK4.
-   No actuator saturation.

Two initial conditions tested:

-   Small angle: θ₀ = 0.2 rad (\~11°).
-   Large angle: θ₀ = 1.0 rad (\~57°).

------------------------------------------------------------------------

## Results

### 1. Small Angle Case

-   Linear and nonlinear responses almost perfectly overlap.
-   Control effort nearly identical.
-   Settling time ≈ 2.46 s.

**Conclusion:** Linearization is valid near equilibrium.

------------------------------------------------------------------------

### 2. Large Angle Case

-   Linear and nonlinear models diverge slightly.
-   Linear model predicts slightly higher peak angular velocity.
-   Linear controller demands slightly higher control effort.
-   Both stabilize in ≈ 2.49 s.

**Physical Insight:**

For larger angles:

$$\sin(\theta) < \theta$$

The linear model overestimates restoring torque, producing slight
control overreaction compared to the true nonlinear dynamics.

Despite this mismatch, LQR still stabilizes the nonlinear system.

------------------------------------------------------------------------

## Engineering Insight

This experiment demonstrates:

-   Linearization is a **local approximation**.
-   LQR is **locally optimal**.
-   Moderate nonlinearity does not immediately destabilize linear
    controllers.
-   Controller robustness depends on region of attraction.

This is the bridge between classical state-space control and nonlinear
control theory.

------------------------------------------------------------------------

## Files

```
    main.py        → Experiment orchestration
    model.py       → Nonlinear + linearized dynamics
    simulate.py    → RK4 + linear simulation
    utils.py       → LQR + metrics
    results/       → Plots + notes
```

------------------------------------------------------------------------

## Key Takeaways

-   Linearization accurately predicts behavior near equilibrium.
-   LQR designed on linear model can stabilize moderately nonlinear
    dynamics.
-   Model mismatch increases control effort but does not necessarily
    break stability.

------------------------------------------------------------------------

## What Broke / What Was Tricky

-   Scalar extraction from NumPy matrix multiplication (`K @ x`)
    required careful reshaping.
-   Large-angle behavior required nonlinear integration (RK4) to avoid
    numerical artifacts.

------------------------------------------------------------------------