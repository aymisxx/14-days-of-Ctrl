# Day 04: Linear Quadratic Regulator (LQR)

## Objective

Implement a full continuous-time Linear Quadratic Regulator (LQR) for a
second-order dynamical system, compare it against a manually tuned PD
controller, and analyze tradeoffs between control performance and energy
usage.

This marks the transition from classical heuristic control to optimal
control.

------------------------------------------------------------------------

# 1. System Model

We consider a second-order system:

$$ẍ + 2ζω ẋ + ω² x = u$$

Where:
- $x$ = position
- $ẋ$ = velocity
- $u$ = control input
- $ω$ = 1.0
- $ζ$ = 0.2

------------------------------------------------------------------------

## State-Space Form

Define the state:

$$x = [x₁, x₂]ᵀ = [x, ẋ]ᵀ$$

Then:

$$ẋ = A x + B u$$

With:

```
    A = [[0, 1], [-1, -0.4]]

    B = [[0], [1]]
```

------------------------------------------------------------------------

# 2. Control Design

## 2.1 PD Controller (Baseline)

Control law:

$$u = -K_{PD} x$$

With:

$$K_{PD} = [10  3]$$

Manually tuned for fast settling.

------------------------------------------------------------------------

## 2.2 Linear Quadratic Regulator (LQR)

The LQR minimizes the infinite-horizon quadratic cost:

$$J = ∫ (xᵀ Q x + uᵀ R u) dt$$

Where:

- Q penalizes state deviation.
- R penalizes control effort.

The optimal gain:

$$K = R⁻¹ Bᵀ P$$

Where $P$ solves the Continuous Algebraic Riccati Equation (CARE):

$$AᵀP + PA - PBR⁻¹BᵀP + Q = 0$$

For default weights:

$$Q = diag(10, 1)$$
$$R = 1$$

Computed gain:

$$K_{LQR} = [2.3166  2.0069]$$

------------------------------------------------------------------------

# 3. Folder Structure

```
    day04_lqr/
    │
    ├── main.py
    ├── model.py
    ├── simulate.py
    ├── README.md
    └── results/
        ├── state_response.png
        └── control_effort.png
```

------------------------------------------------------------------------

# 4. How to Execute

From inside `day04_lqr/`:

```
    python main.py
```

This will: - Solve CARE - Simulate PD and LQR - Print metrics - Save
plots to `results/`

Dependencies:

- numpy
- scipy
- matplotlib

------------------------------------------------------------------------

# 5. Results

## Performance Metrics

```
  Controller   Settling Time (2%)   RMS Control
  ------------ -------------------- -------------
  PD           2.39 s               1.217
  LQR          3.30 s               0.345
```

------------------------------------------------------------------------

## Control Effort

-   PD produces a large initial spike (\~ -10).
-   LQR produces smooth, low-amplitude input.
-   LQR reduces RMS control effort by \~3.5×.

------------------------------------------------------------------------

## State Response

-   PD settles faster.
-   LQR shows smoother velocity behavior.
-   LQR avoids aggressive overshoot.
-   Lower R makes LQR aggressive.
-   Higher R reduces control magnitude but slows settling.

------------------------------------------------------------------------

# 6. Q/R Sensitivity Study

Observations:

-   Lower R → aggressive behavior, fast settling.
-   Higher R → energy-efficient, slower convergence.
-   Higher position weight in Q → stronger regulation.
-   Higher velocity weight → increased damping.

LQR continuously interpolates between energy-optimal and
performance-aggressive regimes depending on weighting.

------------------------------------------------------------------------

# 7. Engineering Takeaways

1.  LQR formalizes the performance vs effort tradeoff via Q and R.
2.  Optimal control produces smoother actuator demand than heuristic PD
    tuning.
3.  Weight selection strongly influences aggressiveness and energy
    consumption.
4.  LQR generalizes directly to tracking, MPC, estimation, and hybrid
    control frameworks.

------------------------------------------------------------------------

# 8. What Was Tricky

-   LQR does not directly optimize settling time.
-   Proper interpretation of Q/R weight effects.
-   Ensuring correct matrix dimensions for CARE.

------------------------------------------------------------------------