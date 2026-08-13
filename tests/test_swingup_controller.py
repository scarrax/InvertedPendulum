import math

from pendulum_game_controlled import LQRController, SwingUpController, pendulum_energy


def test_energy_zero_at_hanging():
    assert pendulum_energy(m=0.5, l=0.5, g=9.81, phi=0.0, vphi=0.0) == 0.0


def test_energy_at_top_equals_two_mgl():
    m, l, g = 0.5, 0.5, 9.81
    energy = pendulum_energy(m, l, g, phi=math.pi, vphi=0.0)
    assert math.isclose(energy, 2 * m * g * l)


def test_starts_in_swingup_mode():
    controller = SwingUpController()
    assert controller.mode == "swingup"


def test_tau_pumps_energy_up_when_below_target():
    # phi=0 (hanging, far from pi), vphi=1.0: cos(phi)*vphi > 0, E << E_top,
    # so a_cmd is strongly negative -> tau saturates at -MAX_TAU.
    controller = SwingUpController()
    tau = controller.compute(phi_fmu=0.0, vphi=1.0, s=0.0, v=0.0)
    assert tau == -controller.MAX_TAU
    assert controller.mode == "swingup"


def test_tau_removes_energy_when_above_target():
    # phi=0, vphi=10.0: cos(phi)*vphi > 0, E > E_top (large vphi),
    # so a_cmd is strongly positive -> tau saturates at +MAX_TAU.
    controller = SwingUpController()
    tau = controller.compute(phi_fmu=0.0, vphi=10.0, s=0.0, v=0.0)
    assert tau == controller.MAX_TAU
    assert controller.mode == "swingup"


def test_switches_to_lqr_within_capture_region():
    controller = SwingUpController()
    phi = math.pi + math.radians(5)  # theta=5 deg < CAPTURE_THETA=10 deg
    vphi = 0.5  # < CAPTURE_VPHI=2.0

    tau = controller.compute(phi_fmu=phi, vphi=vphi, s=0.0, v=0.0)

    assert controller.mode == "lqr"
    assert tau == LQRController().compute(phi_fmu=phi, vphi=vphi, s=0.0, v=0.0)


def test_stays_in_lqr_within_hysteresis_band():
    controller = SwingUpController()
    controller.mode = "lqr"
    phi = math.pi + math.radians(15)  # between CAPTURE_THETA=10 and RELEASE_THETA=25

    controller.compute(phi_fmu=phi, vphi=0.5, s=0.0, v=0.0)

    assert controller.mode == "lqr"


def test_switches_back_to_swingup_outside_release_region():
    controller = SwingUpController()
    controller.mode = "lqr"
    phi = math.pi + math.radians(30)  # theta=30 deg > RELEASE_THETA=25 deg

    tau = controller.compute(phi_fmu=phi, vphi=0.5, s=0.0, v=0.0)

    assert controller.mode == "swingup"
    assert math.isclose(tau, 2.848797352523283)
