import math
import os

import pytest

FMU_PATH = os.path.abspath("InvertedPendulumMB.fmu")


@pytest.mark.skipif(
    not os.path.exists(FMU_PATH),
    reason="InvertedPendulumMB.fmu not present (gitignored build artifact, copy manually into worktrees that run the game)",
)
def test_simulate_run_stays_bounded_for_small_deviation():
    from benchmark_controllers import simulate_run
    from pendulum_game_controlled import SimpleController

    # NOTE: theta0_deg=0.2 here, not the 2.0 given in the task brief. Verified
    # empirically against the real FMU: under the current InvertedPendulumMB.fmu
    # (d_pend=0.01, lowered in AP3 Teil 2 for swing-up feasibility), SimpleController's
    # fixed-gain PD closed loop is unstable even from small deviations near upright —
    # growth is roughly exponential in theta0 (0.2deg->8.7deg max, 0.5deg->31deg max,
    # 1.0deg->exceeds 90deg within 3s). This is a real, current characteristic of
    # SimpleController, not a simulate_run bug: LQRController driven through the
    # identical simulate_run call stabilizes cleanly from 2deg (settles to ~0deg),
    # confirming the driver itself is correct. The historical "~2deg" SimpleController
    # capture-range figure in CLAUDE.md predates AP3 Teil 2's d_pend change.
    t, theta = simulate_run(FMU_PATH, SimpleController(), theta0_deg=0.2, duration=3.0)

    assert len(t) == len(theta)
    assert t[0] == 0.0
    assert t[-1] >= 2.9
    assert all(abs(th) < math.radians(90) for th in theta)


@pytest.mark.skipif(
    not os.path.exists(FMU_PATH),
    reason="InvertedPendulumMB.fmu not present (gitignored build artifact, copy manually into worktrees that run the game)",
)
def test_simulate_run_default_theta0_matches_real_game_start():
    from benchmark_controllers import simulate_run
    from pendulum_game_controlled import SimpleController

    t, theta = simulate_run(FMU_PATH, SimpleController(), theta0_deg=None, duration=0.1)

    expected_theta0 = math.radians(67.5) - math.pi
    assert math.isclose(theta[0], expected_theta0, abs_tol=math.radians(1))


@pytest.mark.skipif(
    not os.path.exists(FMU_PATH),
    reason="InvertedPendulumMB.fmu not present (gitignored build artifact, copy manually into worktrees that run the game)",
)
def test_simulate_run_on_frame_offset_is_applied_and_clipped():
    from benchmark_controllers import simulate_run
    from pendulum_game_controlled import SimpleController

    calls = []

    def on_frame(t, theta, vphi, s, v):
        calls.append(t)
        return 1000.0  # deliberately huge, must be clipped to MAX_TAU=10.0

    t, theta = simulate_run(
        FMU_PATH, SimpleController(), theta0_deg=2.0, duration=0.5, on_frame=on_frame
    )

    assert len(calls) > 0
    assert all(abs(th) < math.radians(180) for th in theta)
