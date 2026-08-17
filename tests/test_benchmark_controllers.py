import os

import pytest

FMU_PATH = os.path.abspath("InvertedPendulumMB.fmu")


@pytest.mark.skipif(
    not os.path.exists(FMU_PATH),
    reason="InvertedPendulumMB.fmu not present (gitignored build artifact, copy manually into worktrees that run the game)",
)
def test_pd_holds_a_small_deviation_but_not_a_large_one():
    from benchmark_controllers import CONTROLLER_FACTORIES, envelope_sweep

    # theta0=0.5 (not 2.0): a linear stability check of SimpleController's
    # closed loop (A - B*K_pd eigenvalues, K_pd=[0,0,K_PHI,K_VPHI]) shows a
    # positive real-part eigenvalue at both d_pend=0.15 and d_pend=0.01 —
    # SimpleController's near-upright loop is linearly unstable, not just
    # weakly damped. It still satisfies the 5-degree/1s hold criterion for
    # small enough theta0 (confirmed empirically: holds through 1.5deg,
    # fails at 2.0deg) because divergence takes longer than hold_duration
    # to reach the tolerance boundary from a small enough start — 0.5deg
    # gives comfortable margin under that ~1.5-2.0deg threshold.
    result = envelope_sweep(
        FMU_PATH, CONTROLLER_FACTORIES["PD"], theta0_values_deg=[0.5, 60], duration=5.0
    )

    assert result["results_by_theta0"][0.5] is True
    assert result["results_by_theta0"][60] is False


@pytest.mark.skipif(
    not os.path.exists(FMU_PATH),
    reason="InvertedPendulumMB.fmu not present (gitignored build artifact, copy manually into worktrees that run the game)",
)
def test_swingup_captures_from_the_real_initial_condition():
    from benchmark_controllers import CONTROLLER_FACTORIES, swingup_capture

    result = swingup_capture(FMU_PATH, CONTROLLER_FACTORIES["SwingUp"], duration=15.0)

    assert result["capture_time"] is not None
    assert result["capture_time"] < 15.0


@pytest.mark.skipif(
    not os.path.exists(FMU_PATH),
    reason="InvertedPendulumMB.fmu not present (gitignored build artifact, copy manually into worktrees that run the game)",
)
def test_lqr_recovers_from_the_force_pulse():
    from benchmark_controllers import TOLERANCE_DEG, CONTROLLER_FACTORIES, robustness

    result = robustness(FMU_PATH, CONTROLLER_FACTORIES["LQR"], duration=10.0)

    assert result["kicked"] is True
    assert result["recovery_time"] is not None

    # recovery_time alone is vacuous here: measured against the real FMU the
    # pulse peaks at only ~1.02° for LQR, i.e. it never leaves the 5°
    # tolerance band, so recovery_time is 0.00s by construction rather than
    # by being fast. Pin the peak instead, so a future change that made the
    # pulse a no-op (or removed the tolerance-band caveat's premise) fails
    # loudly. Lower bound 0.5° = comfortable margin under the measured
    # 1.02°; upper bound TOLERANCE_DEG documents *why* recovery is 0.00s.
    peak = result["peak_post_kick_deviation_deg"]
    assert peak is not None
    assert peak > 0.5, f"kick barely perturbed the pendulum (peak {peak}°)"
    assert peak < TOLERANCE_DEG
