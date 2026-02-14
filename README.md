# B747 C* Longitudinal Flight Control Simulation

## Overview

This project implements a linear longitudinal flight control simulation of the Boeing 747-400 using a classical **C\*** control architecture.

The objective is to demonstrate closed-loop load-factor tracking using:

- Linearized longitudinal aircraft dynamics  
- Second-order elevator actuator model  
- C\* blending (load factor + pitch rate)  
- PI control (integrator on Δn_z error)  
- Pitch-rate washout damping  
- Back-calculation anti-windup  

This project focuses on control-architecture design and dynamic shaping rather than full nonlinear aircraft simulation.

---

## Aircraft Model

The longitudinal state vector is:

$$
x = [u,\; w,\; q,\; \theta]^T
$$

Where:

- u      — forward velocity perturbation  
- w      — vertical velocity perturbation  
- q      — pitch rate  
- theta  — pitch angle  

The linearized state-space form is:


$$
\dot{x} = A x + B \delta_e
$$

Angle-of-attack derivatives are converted using the small-angle approximation:

$$
\alpha \approx \frac{w}{U_0}
$$


The Z_alpha_dot correction is included in the heave equation to preserve realistic short-period dynamics.

---

## Elevator Actuator Model

Elevator dynamics are modeled as a second-order system:


$$
\ddot{\delta}_e + 2 \zeta \omega_0 \dot{\delta}_e + \omega_0^2 \delta_e
= \omega_0^2 \delta_{e,\mathrm{cmd}}
$$


This captures actuator bandwidth and phase lag effects.

---

## C\* Control Law

The C* handling-quality variable is defined as:

$$
C^* = \Delta n_z + \frac{V_{co}}{g} q
$$

Where:

- Delta_nz — incremental load factor  
- q        — pitch rate  
- Vco      — blending constant  
- g        — gravitational acceleration  

This blending reflects pilot sensitivity to both acceleration and pitch rate.

---

## Control Architecture

The elevator command (before saturation) is computed as:

$$
\delta_{e,cmd}
=
K_c\,(C^{*}_{cmd}-C^{*})
+K_i\,\xi
-k_q\,(q-q_f)
-k_{nz}\,\Delta n_z
$$


### Integral Term (Load-Factor Tracking)

The integrator state evolves as:

$$
\dot{\xi} = \Delta n_{z,\mathrm{cmd}} - \Delta n_z
$$

This ensures zero steady-state load-factor error.

---

## Pitch-Rate Washout Filter

To prevent steady trim bias from pitch-rate feedback, a washout filter is introduced:

$$
\dot{q}_f = \frac{q - q_f}{T_w}
$$

$$
q_{\mathrm{wash}} = q - q_f
$$

This preserves damping while removing low-frequency pitch-rate bias.

---

## Anti-Windup Mechanism

Back-calculation anti-windup is used:


$$
\dot{\xi}
=
\left( \Delta n_{z,\mathrm{cmd}} - \Delta n_z \right)
+
\frac{\delta_{e,\mathrm{cmd}} - \delta_{e,\mathrm{raw}}}{K_{aw}}
$$

Where:

$$
e_{nz} = \Delta n_{z,\mathrm{cmd}} - \Delta n_z
$$

This prevents integrator runaway during actuator saturation.

---

## Simulation Scenario

A +0.2 g step command is applied at **t = 1 s**.

Results are shown over 30 seconds to highlight:

- Transient response  
- Damping characteristics  
- Settling behavior  

Longer simulations may show slow bias drift due to the linear perturbation model not re-trimming the aircraft.

---

## Results

Closed-loop behavior demonstrates:

- Fast load-factor rise  
- Small overshoot (~2–3%)  
- Smooth settling  
- Stable pitch-rate dynamics  
- Realistic actuator response  
- Near-zero steady-state load-factor error  

The C\* response exhibits an initial undershoot due to pitch-rate contribution in the blended metric. This is expected behavior.

---

## Design Choices

- Integrator placed on $\Delta n_z$ (not C\*) to prioritize acceleration tracking  
- Washout added to prevent pitch-rate trim conflict  
- Anti-windup required due to actuator limits  
- Gains tuned for moderate damping rather than aggressive response  

---

## Limitations

- Linearized perturbation model  
- No nonlinear re-trimming  
- No full flight-envelope variation  
- No sensor or actuator nonlinearities  
- Not a certified flight-control implementation  

This project is intended as a **control-system design demonstration**.

---

## How to Run

Install dependencies:

```bash
pip install numpy scipy matplotlib
