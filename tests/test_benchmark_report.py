from benchmark_controllers import generate_report


def _sample_results():
    return {
        "envelope": {
            "PD": {"results_by_theta0": {2: True, 4: False}, "envelope_deg": 2},
            "LQR": {"results_by_theta0": {2: True, 4: True}, "envelope_deg": 4},
        },
        "reaction_time": {
            "PD": {"settling_times": {2.0: 0.8, 10.0: None}, "trajectories": {}},
            "LQR": {"settling_times": {2.0: 0.3, 10.0: 1.1}, "trajectories": {}},
            "SwingUp": {"settling_times": {2.0: 0.3, 10.0: 1.0}, "trajectories": {}},
        },
        "robustness": {
            "PD": {"kicked": True, "recovery_time": 2.1, "kick_time": 1.0, "trajectory": ([], [])},
            "LQR": {"kicked": True, "recovery_time": None, "kick_time": 1.0, "trajectory": ([], [])},
            "SwingUp": {"kicked": False, "recovery_time": None, "kick_time": None, "trajectory": ([], [])},
        },
        "swingup_capture": {"capture_time": 3.9, "trajectory": ([], [])},
    }


def test_generate_report_includes_all_sections_and_values():
    report = generate_report(_sample_results())

    assert "PD" in report
    assert "LQR" in report
    assert "SwingUp" in report
    assert "2°" in report
    assert "0.80s" in report
    assert "kein Einschwingen" in report
    assert "2.10s" in report
    assert "keine Erholung" in report
    assert "Regler hat vor dem Puls nicht eingeschwungen" in report
    assert "3.90s" in report
    assert "N/A" in report
    assert "Kontext" in report
    assert "Methodik" in report
    assert "Diskussion" in report
