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
            "PD": {"kicked": True, "recovery_time": 2.1, "kick_time": 1.0,
                   "peak_post_kick_deviation_deg": 12.5, "trajectory": ([], [])},
            "LQR": {"kicked": True, "recovery_time": None, "kick_time": 1.0,
                    "peak_post_kick_deviation_deg": 40.0, "trajectory": ([], [])},
            "SwingUp": {"kicked": False, "recovery_time": None, "kick_time": None,
                        "peak_post_kick_deviation_deg": None, "trajectory": ([], [])},
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


def test_report_includes_peak_deviation_column():
    report = generate_report(_sample_results())

    assert "Max. Auslenkung nach Puls" in report
    assert "12.50°" in report
    assert "40.00°" in report
    assert "kein Puls ausgeloest" in report


def test_report_explains_the_zero_second_entries_and_pd_instability():
    report = generate_report(_sample_results())

    # The 0.00s caveat (reaction time + robustness share one mechanic).
    assert "0.00s" in report
    assert "Toleranzzone" in report
    # PD's finding must be named as linear instability, not "reacts locally".
    assert "Eigenwert" in report
    assert "positivem Realteil" in report
    # SwingUp's 2°/10° rows are its internal LQR submode, not its own result.
    assert "CAPTURE_THETA" in report
