import numpy as np

from cstar_b747 import (
    B747Params,
    actuator_rhs,
    build_longitudinal_state_space,
    cstar_controller,
    response_metrics,
    run_simulation,
)


def test_state_space_dimensions_and_heave_correction():
    p = B747Params()
    A, B, aux = build_longitudinal_state_space(p)

    assert A.shape == (4, 4)
    assert B.shape == (4, 1)
    assert aux["Zw_dot"] < 0.0
    assert aux["heave_scale"] == 1.0 / (1.0 - aux["Zw_dot"])
    assert A[3, 2] == 1.0


def test_actuator_accelerates_toward_command_from_rest():
    rhs = actuator_rhs(np.array([0.0, 0.0]), delta_e_cmd=0.1, omega0=60.0, zeta=0.7)

    assert rhs[0] == 0.0
    assert rhs[1] > 0.0


def test_controller_respects_elevator_limit():
    p = B747Params(Ki=100.0, xi_lim=60.0, de_lim_deg=25.0)
    A, B, _ = build_longitudinal_state_space(p)

    de_cmd, *_ = cstar_controller(2.0, np.zeros(4), 0.0, 10.0, 0.0, p, A, B)

    assert np.rad2deg(abs(de_cmd)) <= p.de_lim_deg


def test_closed_loop_tracks_load_factor_without_unbounded_states():
    result = run_simulation(t_final=12.0, samples=601)
    metrics = response_metrics(result)

    assert np.all(np.isfinite(result.states))
    assert abs(metrics["final_error"]) < 0.035
    assert metrics["overshoot_pct"] < 35.0
    assert metrics["max_commanded_elevator_deg"] <= result.params.de_lim_deg + 1e-9
