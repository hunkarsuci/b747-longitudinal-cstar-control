import numpy as np
from dataclasses import dataclass
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


# ============================================================
# 1) PARAMETERS (B747-400 paper) + actuator + C* gains
# ============================================================

@dataclass
class B747Params:
    # Trim / constants (paper uses English units)
    U0: float = 673.0          # ft/s
    g: float = 32.174          # ft/s^2

    # Dimensional longitudinal derivatives (from Table 3.3)
    Xu: float = -0.0059308
    Xalpha: float = 15.9658

    Zu: float = -0.110314
    Zalpha: float = -355.239
    Zalphadot: float = -11.3338
    Zq: float = -10.6862

    Mu: float = 0.0
    Malpha: float = -1.30281
    Mq: float = -0.541693

    Xde: float = 0.0
    Zde: float = -25.5453
    Mde: float = -1.69366

    # Elevator actuator: 2nd order
    omega0: float = 60.0       # rad/s
    zeta: float = 0.7

    # ---------- C* blending ----------
    # C* = Δn_z + (Vco/g) q
    Vco: float = 200.0         # ft/s

    # ---------- Controller gains ----------
    # Strategy:
    #   P term on C* error (transient/feel)
    #   I term on Δn_z error (steady-state nz tracking)
    #
    # δe_cmd_raw = Kc*(C*_cmd - C*) + Ki*xi - kq*q - knz*Δn_z
    #
    Kc: float = 0.06
    Ki: float = 1.1

    # Add damping + direct load-factor feedback
    kq: float = 5.0            # pitch-rate damping feedback
    knz: float = 0.5          # direct Δn_z feedback

    # Anti-windup back-calculation gain (larger -> faster unwind)
    Kaw: float = 0.3

    # Pilot step in Δn_z (g's)
    dnz_step: float = 0.20     # +0.20 g demand

    # Limits
    de_lim_deg: float = 25.0   # elevator command saturation (deg)
    xi_lim: float = 60.0       # allow more bias before clamping
    
    Tw: float = 7.0   # [s] q-washout time constant (3–8s typical for this kind of demo)

# ============================================================
# 2) BUILD LONGITUDINAL STATE SPACE (x = [u, w, q, theta])
# ============================================================

def build_longitudinal_state_space(p: B747Params):
    """
    States: x = [u, w, q, theta]^T

    Convert alpha-derivatives using small-angle relation:
        alpha ≈ w/U0  ->  Xw = Xalpha/U0 , Zw = Zalpha/U0 , Mw = Malpha/U0

    Include Zw_dot correction:
        Zw_dot = Zalphadot/U0
    and scale the w_dot equation by 1/(1 - Zw_dot).
    """
    U0 = p.U0

    Xw = p.Xalpha / U0
    Zw = p.Zalpha / U0
    Mw = p.Malpha / U0

    Zw_dot = p.Zalphadot / U0
    heave_scale = 1.0 / (1.0 - Zw_dot)

    A = np.zeros((4, 4))
    B = np.zeros((4, 1))

    # u_dot = Xu*u + Xw*w - g*theta + Xde*de
    A[0, 0] = p.Xu
    A[0, 1] = Xw
    A[0, 3] = -p.g
    B[0, 0] = p.Xde

    # w_dot = (1/(1-Zw_dot))*( Zu*u + Zw*w + (Zq+U0)*q + g*theta + Zde*de )
    A[1, 0] = heave_scale * p.Zu
    A[1, 1] = heave_scale * Zw
    A[1, 2] = heave_scale * (p.Zq + U0)
    A[1, 3] = heave_scale * p.g
    B[1, 0] = heave_scale * p.Zde

    # q_dot = Mu*u + Mw*w + Mq*q + Mde*de
    A[2, 0] = p.Mu
    A[2, 1] = Mw
    A[2, 2] = p.Mq
    B[2, 0] = p.Mde

    # theta_dot = q
    A[3, 2] = 1.0

    aux = {"Zw": Zw, "Zw_dot": Zw_dot, "heave_scale": heave_scale}
    return A, B, aux


# ============================================================
# 3) ACTUATOR (2nd order): delta_e follows delta_e_cmd
# ============================================================

def actuator_rhs(xa, delta_e_cmd, omega0, zeta):
    """
    xa = [delta_e, delta_e_dot]
    delta_e_ddot = -ω0^2*delta_e - 2ζω0*delta_e_dot + ω0^2*delta_e_cmd
    """
    delta_e, delta_e_dot = xa
    delta_e_ddot = -(omega0**2) * delta_e - 2.0*zeta*omega0*delta_e_dot + (omega0**2) * delta_e_cmd
    return np.array([delta_e_dot, delta_e_ddot])


# ============================================================
# 4) PILOT COMMAND: step in Δn_z command
# ============================================================

def dnz_command(t, p: B747Params):
    return 0.0 if t < 1.0 else p.dnz_step


# ============================================================
# 5) MEASUREMENTS: compute Δn_z and C* from current states
# ============================================================

def compute_measurements(x, de, p: B747Params, A, B):
    """
    Use dnz computed from the model-consistent w_dot:
        x_dot = A x + B de
        w_dot = x_dot[1]
        Δn_z ≈ -w_dot/g
    """
    x_dot = A @ x + B.flatten() * de
    w_dot = x_dot[1]

    dnz = -w_dot / p.g
    nz = 1.0 + dnz

    q = x[2]
    Cstar = dnz + (p.Vco / p.g) * q
    return nz, dnz, Cstar


# ============================================================
# 6) C* CONTROLLER (P on C*, I on Δn_z) + BACK-CALC ANTI-WINDUP
# ============================================================

def cstar_controller(t, x, de, xi, qf, p: B747Params, A, B):
    """
    What this fixes:

    1) "Why no q feedback?"
       -> Add -kq*q to provide pitch-rate damping and reduce long-period drift.

    2) "Why is the integrator saturating?"
       -> Use back-calculation anti-windup:
             xi_dot = e_nz + (de_cmd - de_cmd_raw)/Kaw
          instead of freezing. This actively "unwinds" when saturated.

    3) "Why doesn't nz track?"
       -> Integrate load-factor error directly:
             e_nz = dnz_cmd - dnz
          That produces the steady elevator bias needed for nz tracking.
       -> Keep proportional action on C* error for transient/handling feel.
    """
    nz, dnz, Cstar = compute_measurements(x, de, p, A, B)

    dnz_cmd = dnz_command(t, p)
    Cstar_cmd = dnz_cmd  # command is still "g-demand"; q term is for feel via feedback

    # Errors
    e_c = Cstar_cmd - Cstar       # for P term (handling qualities / transient)
    e_nz = dnz_cmd - dnz          # for I term (steady-state nz tracking)

    q = x[2]
    q_wash = q - qf
    de_lim = np.deg2rad(p.de_lim_deg)

    # Unsaturated command
    de_cmd_raw = p.Kc * e_c + p.Ki * xi - p.kq * q_wash - p.knz * dnz

    # Saturate
    de_cmd = np.clip(de_cmd_raw, -de_lim, +de_lim)

    # Back-calculation anti-windup (drives xi to a value consistent with saturation)
    # When de_cmd == de_cmd_raw, the AW term is 0.
    xi_dot = e_nz + (de_cmd - de_cmd_raw) / max(p.Kaw, 1e-9)

    # Soft clamp xi so it can't blow up if gains are extreme
    if (xi >= p.xi_lim and xi_dot > 0.0) or (xi <= -p.xi_lim and xi_dot < 0.0):
        xi_dot = 0.0

    return de_cmd, xi_dot, dnz_cmd, Cstar_cmd, Cstar, nz, dnz


# ============================================================
# 7) FULL ODE: plant + actuator + integrator state
# ============================================================

def ode(t, X, A, B, p: B747Params):
    """
    Combined state:
        X = [u, w, q, theta, delta_e, delta_e_dot, xi, qf]
    where xi integrates Δn_z error (with anti-windup back-calc).
       qf = washout filter state for pitch rate
    """
    x = X[0:4]
    xa = X[4:6]
    xi = X[6]
    qf = X[7]
    
    q = x[2]
    qf_dot = (q - qf) / p.Tw

    de = xa[0]  # actual elevator

    # Controller computes elevator command and integrator derivative
    de_cmd, xi_dot, *_ = cstar_controller(t, x, de, xi, qf, p, A, B)

    # Plant dynamics
    x_dot = A @ x + B.flatten() * de

    # Actuator dynamics
    xa_dot = actuator_rhs(xa, de_cmd, p.omega0, p.zeta)

    return np.concatenate([x_dot, xa_dot, np.array([xi_dot, qf_dot])])


# ============================================================
# 8) SIM + PLOTS
# ============================================================

def main():
    p = B747Params()
    A, B, _ = build_longitudinal_state_space(p)

    X0 = np.zeros(8)

    t_span = (0.0, 30.0)
    t_eval = np.linspace(t_span[0], t_span[1], 3001)

    sol = solve_ivp(
        fun=lambda t, X: ode(t, X, A, B, p),
        t_span=t_span,
        y0=X0,
        t_eval=t_eval,
        rtol=1e-8,
        atol=1e-10
    )

    # Extract states
    u = sol.y[0, :]
    w = sol.y[1, :]
    q = sol.y[2, :]
    theta = sol.y[3, :]
    de = sol.y[4, :]
    xi = sol.y[6, :]
    qf = sol.y[7, :]


    # Recompute outputs for plotting
    nz = np.zeros_like(sol.t)
    dnz = np.zeros_like(sol.t)
    dnz_cmd = np.zeros_like(sol.t)
    de_cmd_hist = np.zeros_like(sol.t)
    Cstar_hist = np.zeros_like(sol.t)

    for i, t in enumerate(sol.t):
        x_i = np.array([u[i], w[i], q[i], theta[i]])
        de_i = de[i]
        xi_i = xi[i]

        qf_i = qf[i]
        de_cmd_i, _, dnz_cmd_i, _, Cstar_i, nz_i, dnz_i = cstar_controller(t, x_i, de_i, xi_i, qf_i, p, A, B)

        nz[i] = nz_i
        dnz[i] = dnz_i
        dnz_cmd[i] = dnz_cmd_i
        de_cmd_hist[i] = de_cmd_i
        Cstar_hist[i] = Cstar_i

    # ---- Plot 1: Pitch rate ----
    plt.figure()
    plt.plot(sol.t, q)
    plt.xlabel("t [s]")
    plt.ylabel("q [rad/s]")
    plt.title("B747-400 Closed-Loop Pitch Rate (C* control with q feedback)")
    plt.grid(True)

    # ---- Plot 2: Load factor tracking ----
    plt.figure()
    plt.plot(sol.t, 1.0 + dnz_cmd, label="n_z command")
    plt.plot(sol.t, nz, label="n_z achieved")
    plt.xlabel("t [s]")
    plt.ylabel("n_z [g]")
    plt.title("B747-400 Closed-Loop Load Factor (I on Δn_z error)")
    plt.grid(True)
    plt.legend()

    # ---- Plot 3: Elevator commanded vs actual ----
    plt.figure()
    plt.plot(sol.t, np.rad2deg(de_cmd_hist), label="δe_cmd [deg]")
    plt.plot(sol.t, np.rad2deg(de), label="δe actual [deg]")
    plt.xlabel("t [s]")
    plt.ylabel("elevator [deg]")
    plt.title("Elevator Command vs Actuator Output")
    plt.grid(True)
    plt.legend()

    # ---- Plot 4: integrator state ----
    plt.figure()
    plt.plot(sol.t, xi)
    plt.xlabel("t [s]")
    plt.ylabel("xi [Δn_z error·s]")
    plt.title("Integrator State (with back-calculation anti-windup)")
    plt.grid(True)

    # ---- Plot 5: C* ----
    plt.figure()
    plt.plot(sol.t, dnz_cmd, label="C*_cmd (Δn_z_cmd)")
    plt.plot(sol.t, Cstar_hist, label="C* achieved")
    plt.xlabel("t [s]")
    plt.ylabel("C* [g + (Vco/g)rad/s]")
    plt.title("C* Response")
    plt.grid(True)
    plt.legend()

    plt.show()


if __name__ == "__main__":
    main()
