"""Headless benchmark comparing the three AP3 controllers against the real
InvertedPendulumMB.fmu on stability, reaction time, and robustness.

See docs/superpowers/specs/2026-08-17-ap3-controller-comparison-design.md
for the full methodology.
"""

import math

# Success criterion shared across all scenarios: a controller "holds" the
# upright position from the moment |theta| stays below TOLERANCE_DEG for at
# least HOLD_DURATION seconds. TOLERANCE_DEG matches the game's own
# tight_bonus_zone (pendulum_game_controlled.py's compute_score_increment).
TOLERANCE_DEG = 5.0
HOLD_DURATION = 1.0


def held_from(t, theta, tolerance_rad, hold_duration):
    """Return the earliest time in `t` from which |theta| stays below
    `tolerance_rad` for at least `hold_duration` seconds, or None if that
    never happens within the given data."""
    entry_time = None
    for ti, th in zip(t, theta):
        if abs(th) < tolerance_rad:
            if entry_time is None:
                entry_time = ti
            elif ti - entry_time >= hold_duration:
                return entry_time
        else:
            entry_time = None
    return None


def find_capture_envelope(results_by_theta0):
    """results_by_theta0: dict {theta0_deg: bool success}, assumed to come
    from a monotonically increasing sweep. Return the largest theta0_deg
    for which every smaller tested theta0_deg also succeeded (i.e. the
    value just before the first failure), or None if the smallest tested
    theta0_deg already failed or the input is empty."""
    best = None
    for theta0_deg in sorted(results_by_theta0):
        if results_by_theta0[theta0_deg]:
            best = theta0_deg
        else:
            break
    return best
