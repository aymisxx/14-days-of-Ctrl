# Day 07: **Kalman Filter**

### **State Estimation Under Uncertainty**

> When measurements lie, estimation leads.

------------------------------------------------------------------------

## Objective

Implement a discrete-time **Kalman Filter** from scratch for a linear
Gaussian system with noisy measurements.

Demonstrate:

-   True vs measured vs estimated state.
-   Covariance evolution (uncertainty collapse).
-   Kalman gain convergence.
-   Cross-state estimation (velocity inferred from position).

No external filtering libraries.
Full matrix implementation.
Joseph-form covariance update for numerical stability.

------------------------------------------------------------------------

## System Model

Discrete-time linear Gaussian dynamics:

$$x_{k+1} = A x_k + B u_k + w_k, \quad w_k \sim \mathcal{N}(0, Q)$$

$$y_k = C x_k + v_k, \quad v_k \sim \mathcal{N}(0, R)$$

### State Definition

$$x = \begin{bmatrix} p \\ v \end{bmatrix}$$

Where:

```
-   ( p ) = position
-   ( v ) = velocity
```
### Constant-Velocity Model

$$A = \begin{bmatrix} 1 & dt \\ 0 & 1 \end{bmatrix}$$

$$B =\begin{bmatrix} 0.5 dt^2 \\ dt\end{bmatrix}$$

$$C = \begin{bmatrix} 1 & 0 \end{bmatrix}$$

Only **position is measured**.
Velocity is inferred through estimation.

------------------------------------------------------------------------

## Kalman Filter Equations

### Prediction

$$\hat{x}_{k|k-1} = A\hat{x}_{k-1} + Bu_k$$

$$P_{k|k-1} = A P_{k-1} A^T + Q$$

------------------------------------------------------------------------

### Update

Innovation:

$$\tilde{y}*k = y_k - C\hat{x}_{k|k-1}$$

Kalman Gain:

$$K_k = P C^T (CPC^T + R)^{-1}$$

State Correction:

$$\hat{x}k = \hat{x}_{k|k-1} + K_k \tilde{y}_k$$

------------------------------------------------------------------------

### Covariance Update (Joseph Form)

$$P_k = (I - KC)P(I - KC)^T + KRK^T$$

Joseph form preserves:

-   Symmetry.
-   Positive semi-definiteness.
-   Numerical stability.

------------------------------------------------------------------------

## Results

### Covariance Evolution

Uncertainty collapses rapidly from high initial variance and converges
to steady-state.

Demonstrates:

-   Observability.
-   Proper Q/R tuning.
-   Stable estimator dynamics.

------------------------------------------------------------------------

### Kalman Gain Evolution

-   Large initial gains (aggressive correction).
-   Smooth convergence to steady-state values.
-   Cross-state coupling visible (velocity gain non-zero despite no
    direct velocity measurement).

------------------------------------------------------------------------

### Position Estimation

Measured position is noisy.
Estimated position is smooth and tracks ground truth.

------------------------------------------------------------------------

### Velocity Estimation

Velocity is never directly measured.

Yet the filter reconstructs it accurately using:

-   State transition structure.
-   Innovation propagation.
-   Cross-covariance dynamics.

------------------------------------------------------------------------

## How to Run

From inside:

```
day07_kalman_filter/
```

Run:

``` bash
python main.py
```

Stress test examples:

``` bash
python main.py --r_meas 0.3
python main.py --q_pos 1e-6 --q_vel 1e-6
python main.py --u_policy steps
```

------------------------------------------------------------------------

## Engineering Notes

-   Gain computed via linear solve (no explicit matrix inverse).
-   Covariance symmetrized after propagation.
-   Joseph-form update used for numerical robustness.
-   PSD checks included for safety.

------------------------------------------------------------------------

## Outputs

Saved in `results/`:

-   `state_position.png`
-   `state_velocity.png`
-   `covariance_trace.png`
-   `kalman_gain.png`

------------------------------------------------------------------------

## Interpretation

-   Large initial covariance → high uncertainty.
-   Rapid trace(P) drop → belief convergence.
-   Steady-state gain → time-invariant optimal estimator.
-   Velocity recovery → system observability confirmed.

------------------------------------------------------------------------

## Why This Matters

The Kalman Filter is the backbone of:

-   Autonomous navigation.
-   UAV state estimation.
-   Missile guidance.
-   Sensor fusion.
-   SLAM systems.
-   LQG control architectures

$$\mathbf{LQG = LQR + Kalman\ Filter}$$

Day 07 completes the estimator core required for optimal control under
uncertainty.

------------------------------------------------------------------------

## Key Takeaways

-   Optimal estimation balances model trust (Q) vs measurement trust
    (R).
-   Covariance evolution reflects information gain over time.
-   Steady-state gains emerge naturally in observable linear systems.
-   Joseph-form update improves numerical reliability.

------------------------------------------------------------------------

## What Was Tricky

-   Ensuring covariance remains PSD under finite precision.
-   Proper gain computation without unstable matrix inversion.
-   Maintaining symmetry of P over long simulations.

------------------------------------------------------------------------