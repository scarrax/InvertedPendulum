import math

from pendulum_game_controlled import LQRController


def test_zero_tau_at_equilibrium():
    controller = LQRController()
    tau = controller.compute(phi_fmu=math.pi, vphi=0.0, s=0.0, v=0.0)
    assert abs(tau) < 1e-9


def test_tau_clamped_to_max():
    controller = LQRController()
    tau = controller.compute(phi_fmu=0.0, vphi=50.0, s=100.0, v=50.0)
    assert -controller.MAX_TAU <= tau <= controller.MAX_TAU


def test_returns_plain_float():
    controller = LQRController()
    tau = controller.compute(phi_fmu=math.pi, vphi=0.1, s=0.0, v=0.0)
    assert isinstance(tau, float)
