# Day 02: **Root Locus & Frequency Response**

## Objective

Understand closed-loop stability and robustness using:

-   Root Locus analysis.
-   Gain sweep → pole movement.
-   Bode frequency response.
-   Gain Margin & Phase Margin interpretation.

This day bridges time-domain intuition (Day 01) with frequency-domain
robustness analysis.

------------------------------------------------------------------------

# Problem Formulation

We analyze the continuous-time plant:

$$G(s) = \frac{1}{s(s+2)}$$

This is a second-order open-loop system with:

-   One pole at the origin (integrator)
-   One real pole at ( $s = -2$ )

We apply unity feedback with proportional gain ( $K$ ):

$$T(s) = \frac{K G(s)}{1 + K G(s)}$$

We investigate:

-   How poles move as ($K$) varies.
-   Frequency-domain robustness.
-   Stability guarantees.

------------------------------------------------------------------------

# Mathematical Model

## 2.1 Open-Loop Transfer Function

$$G(s) = \frac{1}{s(s+2)}$$

Denominator expansion:

$$ s(s+2) = s^2 + 2s $$

So:

$$G(s) = \frac{1}{s^2 + 2s}$$

------------------------------------------------------------------------

## 2.2 Closed-Loop Characteristic Equation

For unity feedback:

$$1 + K G(s) = 0$$

Substitute ($G(s)$):

$$1 + \frac{K}{s(s+2)}$$

Multiply both sides by ($s(s+2)$):

$s(s+2) + K = 0$

$s^2 + 2s + K = 0$

This is the **closed-loop characteristic equation**.

------------------------------------------------------------------------

## 2.3 Closed-Loop Poles

Solve quadratic:

$$s = \frac{-2 \pm \sqrt{4 - 4K}}{2}$$

$$s = -1 \pm\sqrt{1 - K}$$

### Case 1: ( K < 1 )

Two real poles → overdamped.

### Case 2: ( K = 1 )

Repeated real pole → critically damped.

### Case 3: ( K > 1 )

$$s = -1 \pm \sqrt{K-1}$$

Complex conjugate poles.

**Important insight:**

$$\text{Re}(s) = -1$$

$$(\forall K > 0)$$

$$\boxed{\text{Closed-loop system is stable for all } K > 0}$$

------------------------------------------------------------------------

# Root Locus Theory

Root locus shows how closed-loop poles move as gain ($K$) varies from $0$ to $∞$.

Rules:

1.  Number of branches = number of open-loop poles.
2.  Branches start at open-loop poles.
3.  Branches end at open-loop zeros (or infinity).
4.  Real-axis segments exist where odd number of poles+zeros lie to the
    right.
5.  Asymptotes govern behavior at infinity.

For this system:

-   Open-loop poles: $0$ and $-2$.
-   No zeros.
-   Breakaway at -1.

------------------------------------------------------------------------

# Frequency Response Analysis

$$G(j\omega) = \frac{1}{j\omega(j\omega + 2)}$$

Magnitude:

$$\|G(j\omega\| = \frac{1}{\omega \sqrt{\omega^2 + 4}}$$

Phase:

$$\angle G(j\omega) = -90^\circ-\tan^{-1}\left(\frac{\omega}{2}\right)$$

Low frequency:

-   Magnitude → $∞$.
-   Phase → $-90°$.
-   Slope: $-20 dB/dec$.

High frequency:

-   Magnitude slope → $-40$ dB/dec.
-   Phase → $-180°$.

------------------------------------------------------------------------

# Gain Margin & Phase Margin

From simulation:

-   Gain Margin = $∞$.
-   Phase Margin ≈ $76.35°$.
-   Gain crossover ≈ $0.486$ rad/s.

### Why Gain Margin = $∞$?

The phase approaches $−180°$ but does not produce a finite crossover.

$$\boxed{\text{System cannot be destabilized by gain increase alone}}$$

Phase margin of $~76°$ indicates strong damping and robustness.

------------------------------------------------------------------------

# Folder Structure

```
    day02_root_locus_frequency/
    │
    ├── main.py
    ├── model.py
    ├── simulate.py
    ├── README.md
    └── results/
        ├── root_locus.png
        ├── bode_plot.png
        └── gain_sweep_poles.png
```

------------------------------------------------------------------------

# How to Execute

```
    pip install control matplotlib numpy
    python main.py
```
Artifacts saved in:

```
    results/
```

------------------------------------------------------------------------

# Results Summary

-   Poles move vertically with real part fixed at $-1$.
-   System stable ∀ K (\> 0).
-   Infinite gain margin.
-   High phase margin ($~76°$).

------------------------------------------------------------------------

# Engineering Takeaway

1.  Root locus gives structural stability insight.
2.  Bode margins quantify robustness.
3.  Infinite gain margin ≠ infinite robustness.
4.  Integrator systems can remain stable under proportional control.

------------------------------------------------------------------------

# Key Takeaways

-   Closed-loop poles: ($s = -1 \pm\sqrt{1-K}$).
-   Stable for all positive $K$.
-   Root locus and Bode provide complementary stability views.

------------------------------------------------------------------------

# What Was Tricky

-   Interpreting infinite gain margin correctly.
-   Reconciling algebraic and frequency-domain viewpoints.

------------------------------------------------------------------------