import math

from benchmark_controllers import (
    KickInjector,
    envelope_sweep,
    reaction_time,
    robustness,
    swingup_capture,
)


def test_kick_injector_triggers_once_after_hold_duration_then_stays_quiet():
    injector = KickInjector(tolerance_rad=math.radians(5), hold_duration=1.0, kick_tau=8.0, kick_steps=2)

    assert injector.offset_for(0.0, 0.0) == 0.0
    assert injector.offset_for(0.5, 0.0) == 0.0
    assert injector.offset_for(1.0, 0.0) == 8.0
    assert injector.kick_time == 1.0
    assert injector.offset_for(1.5, math.radians(90)) == 8.0
    assert injector.offset_for(2.0, math.radians(90)) == 0.0
    assert injector.offset_for(2.5, 0.0) == 0.0
    assert injector.offset_for(3.0, 0.0) == 0.0


def test_envelope_sweep_stops_after_two_consecutive_failures():
    def fake_simulate(fmu_path, controller, theta0_deg, duration):
        if theta0_deg <= 6:
            return [0.0, 1.0], [0.0, 0.0]
        return [0.0, 1.0], [math.radians(90), math.radians(90)]

    result = envelope_sweep(
        "unused.fmu", lambda: None,
        theta0_values_deg=[2, 4, 6, 8, 10, 12],
        tolerance_deg=5.0, hold_duration=1.0, duration=2.0,
        simulate_fn=fake_simulate,
    )

    assert result["envelope_deg"] == 6
    assert result["results_by_theta0"] == {2: True, 4: True, 6: True, 8: False, 10: False}
    assert 12 not in result["results_by_theta0"]


def test_reaction_time_records_settling_time_and_trajectory():
    def fake_simulate(fmu_path, controller, theta0_deg, duration):
        if theta0_deg == 2.0:
            return [0.0, 1.0], [0.0, 0.0]
        return [0.0, 1.0], [math.radians(90), math.radians(90)]

    result = reaction_time(
        "unused.fmu", lambda: None, theta0_values_deg=(2.0, 10.0),
        tolerance_deg=5.0, hold_duration=1.0, duration=1.0,
        simulate_fn=fake_simulate,
    )

    assert result["settling_times"][2.0] == 0.0
    assert result["settling_times"][10.0] is None
    assert result["trajectories"][2.0] == ([0.0, 1.0], [0.0, 0.0])


def test_robustness_measures_recovery_time_relative_to_kick():
    times = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5]
    thetas_deg = [0, 0, 0, 90, 90, 0, 0, 0]

    def fake_simulate(fmu_path, controller, theta0_deg, duration, on_frame=None):
        t_history, theta_history = [], []
        for t, theta_deg in zip(times, thetas_deg):
            theta = math.radians(theta_deg)
            if on_frame is not None:
                on_frame(t, theta, 0.0, 0.0, 0.0)
            t_history.append(t)
            theta_history.append(theta)
        return t_history, theta_history

    result = robustness(
        "unused.fmu", lambda: None, theta0_deg=2.0, tolerance_deg=5.0,
        hold_duration=1.0, kick_tau=8.0, kick_steps=1, duration=3.5,
        simulate_fn=fake_simulate,
    )

    assert result["kicked"] is True
    assert result["kick_time"] == 1.0
    assert result["recovery_time"] == 1.5


def test_robustness_reports_not_kicked_when_never_settled():
    def fake_simulate(fmu_path, controller, theta0_deg, duration, on_frame=None):
        t_history, theta_history = [], []
        for t in [0.0, 0.5, 1.0]:
            theta = math.radians(90)
            if on_frame is not None:
                on_frame(t, theta, 0.0, 0.0, 0.0)
            t_history.append(t)
            theta_history.append(theta)
        return t_history, theta_history

    result = robustness(
        "unused.fmu", lambda: None, simulate_fn=fake_simulate,
    )

    assert result["kicked"] is False
    assert result["recovery_time"] is None


def test_swingup_capture_uses_theta0_deg_none():
    seen = {}

    def fake_simulate(fmu_path, controller, theta0_deg, duration):
        seen["theta0_deg"] = theta0_deg
        return [0.0, 1.0], [0.0, 0.0]

    result = swingup_capture("unused.fmu", lambda: None, simulate_fn=fake_simulate)

    assert seen["theta0_deg"] is None
    assert result["capture_time"] == 0.0
