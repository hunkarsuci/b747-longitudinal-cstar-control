# ✈️ B747 C* Longitudinal Flight Control Simulation

## Overview

This project implements a **linear longitudinal flight control simulation** of the Boeing 747-400 using a classical **C\*** control law architecture.

The objective is to demonstrate closed-loop load-factor tracking using:

- Linearized longitudinal aircraft dynamics  
- Second-order elevator actuator model  
- C\* blending (load factor + pitch rate)  
- PI control (integrator on Δn_z error)  
- Pitch-rate washout damping  
- Back-calculation anti-windup  

This project focuses on **control architecture design and dynamic shaping**, not full nonlinear aircraft simulation.

---

## Aircraft Model

The longitudinal state vector is:

\[
x = [u,\; w,\; q,\; \theta]^T
\]

Where:

- \( u \) — forward velocity perturbation  
- \( w \) — vertical velocity perturbation  
- \( q \) — pitch rate  
- \( \theta \) — pitch angle  

The system is linearized around steady cruise trim conditions using published B747-400 dimensional derivatives.

### State-Space Representation

\[
\dot{x} = A x + B \delta_e
\]

Alpha-derivatives are converted using:

\[
\alpha \approx \frac{w}{U_0}
\]

The \( Z_{\dot{\alpha}} \) correction is included in the heave equation to preserve realistic short-period dynamics.

---

## Elevator Actuator Model

Elevator dynamics are modeled as a second-order system:

\[
\ddot{\delta}_e + 2\zeta\omega_0 \dot{\delta}_e + \omega_0^2 \delta_e
=
\omega_0^2 \delta_{e,cmd}
\]

This captures actuator bandwidth and phase lag effects.

---

## C\* Control Law

The C\* handling-quality variable is defined as:

\[
C^* = \Delta n_z + \frac{V_{co}}{g} q
\]

Where:

- \( \Delta n_z \) — incremental load factor  
- \( q \) — pitch rate  
- \( V_{co} \) — blending constant  
- \( g \) — gravitational acceleration  

This blending reflects pilot handling sensitivity to both acceleration and pitch rate.

---

## Control Architecture

The elevator command is computed as:

\[
\delta_{e,cmd} =
K_c (C^*_{cmd} - C^*)
+
K_i \xi
-
k_q (q - q_f)
-
k_{nz} \Delta n_z
\]

### Control Components

**Proportional term (C\*)**  
Shapes transient response and handling feel.

**Integral term (Δn_z error)**  

\[
\dot{\xi} = \Delta n_{z,cmd} - \Delta n_z
\]

Ensures steady-state load-factor tracking.

**Pitch-rate washout filter**

\[
\dot{q}_f = \frac{q - q_f}{T_w}
\]

\[
q_{wash} = q - q_f
\]

Provides damping without introducing steady trim bias.

**Back-calculation anti-windup**

\[
\dot{\xi} =
e_{nz} + \frac{\delta_{e,cmd} - \delta_{e,raw}}{K_{aw}}
\]

Prevents integrator runaway during actuator saturation.

---

## Simulation Scenario

A +0.2 g step command is applied at **t = 1 s**.

Results are shown over the first 30 seconds to emphasize transient and settling behavior.  
Longer simulations exhibit slow bias drift typical of linear perturbation models without nonlinear re-trimming.

---

## Results

Closed-loop response demonstrates:

- Fast load-factor rise  
- Small overshoot (~2–3%)  
- Smooth settling  
- Stable pitch-rate dynamics  
- Realistic actuator response  
- Near-zero steady-state load-factor error  

The C\* response exhibits an initial undershoot due to pitch-rate transient contribution. This behavior is expected given the blended control structure and linearized aircraft model.

---

## Design Considerations

Key architectural decisions:

- Integrator placed on Δn_z rather than C\* to prioritize acceleration tracking  
- Washout added to prevent pitch-rate feedback from fighting steady trim  
- Anti-windup required due to actuator limits  
- Gains tuned for moderate damping rather than aggressive response  

---

## Limitations

- Linearized perturbation model  
- No nonlinear re-trimming  
- No full flight-envelope variation  
- No sensor or actuator nonlinearities  
- Not a certified flight-control implementation  

This project is intended as a **control design demonstration**.

---

## How to Run

Install dependencies:

```bash
pip install -r requirements.txt
