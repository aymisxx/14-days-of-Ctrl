# **14-days-of-Ctrl** (Revision Challenge)

A 14-day engineering sprint through classical, optimal, robust, and reinforcement learning control systems.

This repository documents a structured daily revision build cycle focused on theory-grounded implementation, simulation, and comparison of modern control strategies.

------------------------------------------------------------------------

## Objective

Reinforce control systems intuition by implementing:

-   Classical feedback control.
-   State-space methods.
-   Optimal control (LQR).
-   Estimation (Kalman filtering).
-   Constrained control (MPC).
-   Robust control (Sliding Mode).
-   Multi-agent consensus.
-   Reinforcement learning.
-   Hybrid classical--learning controllers.

Each day is implemented from scratch with clean simulations and
structured notes.

------------------------------------------------------------------------

## Schedule (14-Day Roadmap)

  (01)    First/Second Order Systems + PD/PID.\
  (02)    Root Locus & Frequency Response.\
  (03)    State-Space Modeling + Controllability.\
  (04)    LQR (Full Implementation).\
  (05)    LQR Tracking.\
  (06)    Nonlinear System Linearization + LQR.\
  (07)    Kalman Filter.\
  (08)    Model Predictive Control (Constrained).\
  (09)    Sliding Mode Control.\
  (10)    Multi-Agent Consensus Dynamics.\
  (11)    Disturbance Observer / Adaptive Control.\
  (12)    Policy Gradient (REINFORCE).\
  (13)    LQR vs RL Comparative Study.\
  (14)    Hybrid Control (LQR + RL Adaptation).

------------------------------------------------------------------------

## Standard Folder Structure

Each day follows:

```
dayXX_topic/
├── main.py
├── model.py
├── simulate.py
├── README.md
└──results/
```

Each daily README includes:

- Problem formulation.
- Mathematical model.
- Controller design.
- Simulation results.
- Key takeaways.
- What was tricky / limitations.

------------------------------------------------------------------------

## Installation

Create a virtual environment:

```
python -m venv venv
source .venv/bin/activate
pip install -r requirements.txt
```

If using NVIDIA GPU (CUDA-enabled PyTorch):

```
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu124
```

------------------------------------------------------------------------

## Design Philosophy

This repository emphasizes:

-   Minimal external abstractions
-   Mathematical transparency
-   Clean reproducible simulations
-   Explicit comparison between methods
-   Engineering tradeoff analysis (stability, robustness, control effort)

The goal is not just working code, but structural understanding.

------------------------------------------------------------------------

## License

MIT License. See LICENSE for details.

------------------------------------------------------------------------

## Author

#### Ayushman Mishra

> https://github.com/aymisxx

------------------------------------------------------------------------