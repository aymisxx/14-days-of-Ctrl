# Day 01: **Classical Control Foundations**

## First-Order & Second-Order Systems + PD/PID Control

------------------------------------------------------------------------

# 1. Objective

This module revises the foundations of classical control through:

-   First-order system dynamics.
-   Second-order system damping behavior.
-   Pole--response relationship.
-   Manual PD control implementation.
-   Manual PID control implementation.
-   Gain intuition through simulation.

The goal is to rebuild geometric and mathematical intuition about
stability before moving into optimal and modern control methods.

------------------------------------------------------------------------

# 2. Mathematical Foundations

## 2.1 First-Order System

### Differential Equation

$$τ ẏ + y = u$$

### Transfer Function

$$G(s) = 1 / (τs + 1)$

### Pole Location

$$s = -1/τ$$

### Step Response

For unit step input:

$$y(t) = 1 - exp(-t/τ)$$

### Key Properties

-   Single real pole.
-   No oscillation.
-   Monotonic response.
-   Speed controlled entirely by pole location.

------------------------------------------------------------------------

## 2.2 Second-Order System

### Differential Equation

$$ÿ + 2ζωₙ ẏ + ωₙ² y = ωₙ² u$$

### Transfer Function

$$G(s) = ωₙ² / (s² + 2ζωₙs + ωₙ²)$$

### Parameters

-   $ωₙ$ → natural frequency (speed scaling).
-   $ζ$ → damping ratio (shape control).

### Pole Locations

$$s = -ζωₙ ± jωₙ√(1 - ζ²)$$

### Underdamped (ζ \< 1)

Percent overshoot:

$$M_p = exp(-πζ / √(1 - ζ²)) × 100%$$

Overshoot depends only on $ζ$.

### Critically Damped (ζ = 1)

Fastest non-oscillatory response.

### Overdamped (ζ \> 1)

Two real poles. Dominant pole governs behavior.

------------------------------------------------------------------------

# 3. PD Control

$$u = K_p(r - y) - Kdẏ$$

Closed-loop effect on MSD:

$$mÿ + (b + K_d)ẏ + (k + K_p)y = K_p r$$

Steady-state value:

$$y_{ss} = (K_p / (k + K_p)) r$$

PD reduces oscillation but does not eliminate steady-state error.

------------------------------------------------------------------------

# 4. PID Control

$$u = K_p e + K_i ∫e dt + K_d ė$$

Integral introduces pole at origin and eliminates steady-state error for
step input, but reduces stability margin.

------------------------------------------------------------------------

# 5. Folder Structure

```
day01_classical_pd_pid/
│
├── main.py
├── model.py
├── simulate.py
├── README.md
└── results/
    ├── first_order_step.png
    ├── second_order_damping.png
    ├── poles_vs_response.png
    ├── pd_response.png
    ├── pid_response.png
    ├── overshoot_table.csv
    └── notes.txt
```

------------------------------------------------------------------------

# 6. How to Execute

```
pip install numpy scipy matplotlib pandas
python main.py
```

Artifacts will be saved to `results/`.

------------------------------------------------------------------------

# 7. Results Summary

-   First-order: real pole controls speed.\
-   Second-order: ζ controls overshoot.\
-   PD: improves damping but steady-state error remains.\
-   PID: eliminates steady-state error but increases overshoot risk.

------------------------------------------------------------------------

# 8. Engineering Takeaways

1.  Classical control is pole shaping.
2.  Real part governs decay speed.
3.  Imaginary part governs oscillation.
4.  Derivative adds damping.
5.  Integral eliminates steady-state error at cost of robustness.

------------------------------------------------------------------------

## Key Takeaways

-   Pole location determines transient response.
-   Damping ratio governs overshoot.
-   PD improves damping.
-   PID improves accuracy.

## What Was Tricky

-   Understanding steady-state error in PD.
-   Interpreting pole geometry visually.

---