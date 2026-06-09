from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.integrate import solve_ivp


@dataclass(frozen=True)
class B747Params:
    """B747-400 longitudinal model, actuator, and C* controller parameters."""

    U0: float = 673.0
    g: float = 32.174
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
    omega0: float = 60.0
    zeta: float = 0.7
    Vco: float = 200.0
    Kc: float = 0.02
    Ki: float = 4.5
    kq: float = 5.0
    knz: float = 0.8
    Kaw: float = 0.3
    Tw: float = 7.0
    dnz_step: float = 0.20
    step_time: float = 1.0
    de_lim_deg: float = 25.0
    xi_lim: float = 60.0


@dataclass(frozen=True)
class SimulationResult:
    t: np.ndarray
    states: np.ndarray
    nz: np.ndarray
    dnz: np.ndarray
    dnz_cmd: np.ndarray
    cstar: np.ndarray
    cstar_cmd: np.ndarray
    de_cmd: np.ndarray
    params: B747Params


def build_longitudinal_state_space(p: B747Params) -> tuple[np.ndarray, np.ndarray, dict[str, float]]:
    """Build the linear longitudinal model with x = [u, w, q, theta]."""
    xw = p.Xalpha / p.U0
    zw = p.Zalpha / p.U0
    mw = p.Malpha / p.U0
    zw_dot = p.Zalphadot / p.U0
    heave_scale = 1.0 / (1.0 - zw_dot)

    A = np.zeros((4, 4))
    B = np.zeros((4, 1))

    A[0, 0] = p.Xu
    A[0, 1] = xw
    A[0, 3] = -p.g
    B[0, 0] = p.Xde

    A[1, 0] = heave_scale * p.Zu
    A[1, 1] = heave_scale * zw
    A[1, 2] = heave_scale * (p.Zq + p.U0)
    A[1, 3] = heave_scale * p.g
    B[1, 0] = heave_scale * p.Zde

    A[2, 0] = p.Mu
    A[2, 1] = mw
    A[2, 2] = p.Mq
    B[2, 0] = p.Mde
    A[3, 2] = 1.0

    return A, B, {"Zw": zw, "Zw_dot": zw_dot, "heave_scale": heave_scale}


def actuator_rhs(xa: np.ndarray, delta_e_cmd: float, omega0: float, zeta: float) -> np.ndarray:
    """Second-order elevator actuator: xa = [delta_e, delta_e_dot]."""
    delta_e, delta_e_dot = xa
    delta_e_ddot = -(omega0**2) * delta_e - 2.0 * zeta * omega0 * delta_e_dot
    delta_e_ddot += (omega0**2) * delta_e_cmd
    return np.array([delta_e_dot, delta_e_ddot])


def dnz_command(t: float, p: B747Params) -> float:
    return 0.0 if t < p.step_time else p.dnz_step


def compute_measurements(
    x: np.ndarray, de: float, p: B747Params, A: np.ndarray, B: np.ndarray
) -> tuple[float, float, float]:
    """Compute nz, incremental nz, and C* from the model-consistent w_dot."""
    x_dot = A @ x + B.ravel() * de
    dnz = -x_dot[1] / p.g
    nz = 1.0 + dnz
    cstar = dnz + (p.Vco / p.g) * x[2]
    return nz, dnz, cstar


def cstar_controller(
    t: float,
    x: np.ndarray,
    de: float,
    xi: float,
    qf: float,
    p: B747Params,
    A: np.ndarray,
    B: np.ndarray,
) -> tuple[float, float, float, float, float, float, float]:
    """C* proportional feedback plus load-factor integral action and anti-windup."""
    nz, dnz, cstar = compute_measurements(x, de, p, A, B)
    dnz_cmd = dnz_command(t, p)
    cstar_cmd = dnz_cmd

    e_c = cstar_cmd - cstar
    e_nz = dnz_cmd - dnz
    q_wash = x[2] - qf
    de_lim = np.deg2rad(p.de_lim_deg)

    de_cmd_raw = p.Kc * e_c + p.Ki * xi - p.kq * q_wash - p.knz * dnz
    de_cmd = float(np.clip(de_cmd_raw, -de_lim, de_lim))

    xi_dot = e_nz + p.Kaw * (de_cmd - de_cmd_raw)
    if (xi >= p.xi_lim and xi_dot > 0.0) or (xi <= -p.xi_lim and xi_dot < 0.0):
        xi_dot = 0.0

    return de_cmd, xi_dot, dnz_cmd, cstar_cmd, cstar, nz, dnz


def ode(t: float, X: np.ndarray, A: np.ndarray, B: np.ndarray, p: B747Params) -> np.ndarray:
    x = X[0:4]
    xa = X[4:6]
    xi = X[6]
    qf = X[7]

    qf_dot = (x[2] - qf) / p.Tw
    de_cmd, xi_dot, *_ = cstar_controller(t, x, xa[0], xi, qf, p, A, B)
    x_dot = A @ x + B.ravel() * xa[0]
    xa_dot = actuator_rhs(xa, de_cmd, p.omega0, p.zeta)
    return np.concatenate([x_dot, xa_dot, np.array([xi_dot, qf_dot])])


def run_simulation(
    p: B747Params | None = None,
    t_final: float = 30.0,
    samples: int = 3001,
    rtol: float = 1e-8,
    atol: float = 1e-10,
) -> SimulationResult:
    p = p or B747Params()
    A, B, _ = build_longitudinal_state_space(p)
    t_eval = np.linspace(0.0, t_final, samples)
    sol = solve_ivp(
        fun=lambda t, X: ode(t, X, A, B, p),
        t_span=(0.0, t_final),
        y0=np.zeros(8),
        t_eval=t_eval,
        rtol=rtol,
        atol=atol,
    )
    if not sol.success:
        raise RuntimeError(f"Simulation failed: {sol.message}")

    nz = np.zeros_like(sol.t)
    dnz = np.zeros_like(sol.t)
    dnz_cmd = np.zeros_like(sol.t)
    cstar = np.zeros_like(sol.t)
    cstar_cmd = np.zeros_like(sol.t)
    de_cmd = np.zeros_like(sol.t)

    for i, t in enumerate(sol.t):
        de_cmd_i, _, dnz_cmd_i, cstar_cmd_i, cstar_i, nz_i, dnz_i = cstar_controller(
            t, sol.y[0:4, i], sol.y[4, i], sol.y[6, i], sol.y[7, i], p, A, B
        )
        de_cmd[i] = de_cmd_i
        dnz_cmd[i] = dnz_cmd_i
        cstar_cmd[i] = cstar_cmd_i
        cstar[i] = cstar_i
        nz[i] = nz_i
        dnz[i] = dnz_i

    return SimulationResult(sol.t, sol.y, nz, dnz, dnz_cmd, cstar, cstar_cmd, de_cmd, p)


def response_metrics(result: SimulationResult) -> dict[str, float]:
    """Return core closed-loop quality metrics for the load-factor response."""
    p = result.params
    mask = result.t >= p.step_time
    target = p.dnz_step
    final_error = float(result.dnz[-1] - target)
    peak = float(np.max(result.dnz[mask]))
    overshoot_pct = max(0.0, (peak - target) / target * 100.0) if target else 0.0

    band = 0.02 * abs(target)
    settling_time = float("nan")
    indices = np.flatnonzero(mask)
    for idx in indices:
        if np.all(np.abs(result.dnz[idx:] - target) <= band):
            settling_time = float(result.t[idx] - p.step_time)
            break

    return {
        "final_dnz": float(result.dnz[-1]),
        "final_error": final_error,
        "peak_dnz": peak,
        "overshoot_pct": overshoot_pct,
        "settling_time_s": settling_time,
        "max_elevator_deg": float(np.max(np.abs(np.rad2deg(result.states[4])))),
        "max_commanded_elevator_deg": float(np.max(np.abs(np.rad2deg(result.de_cmd)))),
    }


def plot_responses(result: SimulationResult, save_dir: str | Path | None = None) -> list[plt.Figure]:
    t = result.t
    q = result.states[2]
    de = result.states[4]
    xi = result.states[6]
    figures: list[plt.Figure] = []

    def finish(fig: plt.Figure, filename: str) -> None:
        figures.append(fig)
        if save_dir is not None:
            path = Path(save_dir)
            path.mkdir(parents=True, exist_ok=True)
            fig.savefig(path / filename, dpi=160, bbox_inches="tight")

    fig, ax = plt.subplots()
    ax.plot(t, q)
    ax.set(xlabel="t [s]", ylabel="q [rad/s]", title="B747-400 Closed-Loop Pitch Rate")
    ax.grid(True)
    finish(fig, "q_response.png")

    fig, ax = plt.subplots()
    ax.plot(t, 1.0 + result.dnz_cmd, label="n_z command")
    ax.plot(t, result.nz, label="n_z achieved")
    ax.set(xlabel="t [s]", ylabel="n_z [g]", title="B747-400 Closed-Loop Load Factor")
    ax.grid(True)
    ax.legend()
    finish(fig, "nz_response.png")

    fig, ax = plt.subplots()
    ax.plot(t, np.rad2deg(result.de_cmd), label="delta_e command [deg]")
    ax.plot(t, np.rad2deg(de), label="delta_e actual [deg]")
    ax.set(xlabel="t [s]", ylabel="elevator [deg]", title="Elevator Command vs Actuator Output")
    ax.grid(True)
    ax.legend()
    finish(fig, "elevator_response.png")

    fig, ax = plt.subplots()
    ax.plot(t, xi)
    ax.set(xlabel="t [s]", ylabel="xi [dnz error*s]", title="Integrator State")
    ax.grid(True)
    finish(fig, "xi_response.png")

    fig, ax = plt.subplots()
    ax.plot(t, result.cstar_cmd, label="C*_cmd")
    ax.plot(t, result.cstar, label="C* achieved")
    ax.set(xlabel="t [s]", ylabel="C* [g]", title="C* Response")
    ax.grid(True)
    ax.legend()
    finish(fig, "cstar_response.png")

    return figures


def create_3d_animation(
    result: SimulationResult,
    interval_ms: int = 30,
    frame_step: int = 8,
    save_path: str | Path | None = None,
) -> animation.FuncAnimation:
    """Animate a simple 3D aircraft body using simulated pitch and vertical motion."""
    t = result.t[::frame_step]
    theta = result.states[3, ::frame_step]
    w = result.states[1, ::frame_step]
    dt = float(np.mean(np.diff(result.t)))
    forward_nm = result.params.U0 * t / 6076.12
    altitude_ft = -np.cumsum(w) * dt * frame_step
    altitude_ft -= altitude_ft[0]

    body = np.array(
        [
            [90.0, 0.0, 0.0],
            [-70.0, 0.0, 10.0],
            [-85.0, 0.0, -10.0],
            [15.0, -95.0, 0.0],
            [-20.0, 95.0, 0.0],
            [-65.0, 0.0, 38.0],
        ]
    )
    faces = [[0, 3, 1], [0, 1, 4], [0, 2, 3], [0, 4, 2], [1, 5, 4], [1, 3, 5]]

    fig = plt.figure(figsize=(9, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title("B747-400 C* Longitudinal Response - 3D Animation")
    ax.set_xlabel("range [nmi]")
    ax.set_ylabel("lateral [ft]")
    ax.set_zlabel("altitude perturbation [ft]")
    ax.set_xlim(float(forward_nm.min()), float(forward_nm.max()))
    ax.set_ylim(-180.0, 180.0)
    z_pad = max(80.0, float(np.ptp(altitude_ft)) * 0.8)
    ax.set_zlim(float(altitude_ft.min() - z_pad), float(altitude_ft.max() + z_pad))
    ax.view_init(elev=18, azim=-65)

    (path_line,) = ax.plot([], [], [], color="#2563eb", linewidth=2)
    aircraft = Poly3DCollection([], facecolor="#d9dde6", edgecolor="#293241", linewidth=0.7, alpha=0.95)
    ax.add_collection3d(aircraft)
    time_text = ax.text2D(0.03, 0.94, "", transform=ax.transAxes)

    def transform_aircraft(i: int) -> list[np.ndarray]:
        pitch = theta[i]
        rotation = np.array(
            [
                [np.cos(pitch), 0.0, np.sin(pitch)],
                [0.0, 1.0, 0.0],
                [-np.sin(pitch), 0.0, np.cos(pitch)],
            ]
        )
        scaled = body @ rotation.T
        scaled[:, 0] = scaled[:, 0] / 6076.12 + forward_nm[i]
        scaled[:, 2] = scaled[:, 2] + altitude_ft[i]
        return [scaled[face] for face in faces]

    def update(i: int):
        path_line.set_data(forward_nm[: i + 1], np.zeros(i + 1))
        path_line.set_3d_properties(altitude_ft[: i + 1])
        aircraft.set_verts(transform_aircraft(i))
        time_text.set_text(f"t = {t[i]:.1f} s")
        return path_line, aircraft, time_text

    anim = animation.FuncAnimation(fig, update, frames=len(t), interval=interval_ms, blit=False)
    if save_path is not None:
        anim.save(save_path)
    return anim


def main(argv: list[str] | None = None) -> None:
    import argparse

    parser = argparse.ArgumentParser(description="B747-400 C* longitudinal control simulation")
    parser.add_argument("--save-figures", action="store_true", help="write response plots to figures/")
    parser.add_argument("--animate-3d", action="store_true", help="show a 3D aircraft response animation")
    parser.add_argument("--save-animation", type=Path, help="save the 3D animation, e.g. figures/b747_3d.gif")
    args = parser.parse_args(argv)

    result = run_simulation()
    for name, value in response_metrics(result).items():
        print(f"{name}: {value:.6g}")

    plot_responses(result, save_dir="figures" if args.save_figures else None)
    if args.animate_3d or args.save_animation:
        create_3d_animation(result, save_path=args.save_animation)
    plt.show()


if __name__ == "__main__":
    main()
