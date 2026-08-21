import math

from pendulum_game_controlled import K_STABILITY, compute_score_increment


def test_zero_beyond_max_angle():
    assert compute_score_increment(angle=math.pi / 2 + 0.01, stable_streak=0.0) == 0.0


def test_matches_manual_formula_in_tight_zone():
    angle = math.radians(3)
    stable_streak = 2.0

    max_angle = math.pi / 2
    bonus_zone = math.radians(15)
    tight_bonus_zone = math.radians(5)
    closeness = (max_angle - angle) / max_angle
    close2 = (bonus_zone - angle) / bonus_zone
    close3 = (tight_bonus_zone - angle) / tight_bonus_zone

    expected = (
        (0.1 + 0.2 * closeness)
        + 2 * (close2**2)
        + 3 * (close3**2)
        + K_STABILITY * stable_streak
    )

    assert math.isclose(compute_score_increment(angle, stable_streak), expected)


def test_stability_bonus_isolated():
    angle = math.radians(3)
    with_streak = compute_score_increment(angle, stable_streak=2.0)
    without_streak = compute_score_increment(angle, stable_streak=0.0)

    assert math.isclose(with_streak - without_streak, K_STABILITY * 2.0)


def test_custom_bonus_zone_widens_bonus_range():
    angle = math.radians(18)  # außerhalb der Standard-15°-Zone, innerhalb einer 20°-Zone
    default_increment = compute_score_increment(angle, stable_streak=0.0)
    widened_increment = compute_score_increment(
        angle, stable_streak=0.0, bonus_zone=math.radians(20), tight_bonus_zone=math.radians(5)
    )
    assert widened_increment > default_increment


def test_custom_tight_bonus_zone_narrows_bonus_range():
    angle = math.radians(4)  # innerhalb der Standard-5°-Zone, außerhalb einer 3°-Zone
    default_increment = compute_score_increment(angle, stable_streak=0.0)
    narrowed_increment = compute_score_increment(
        angle, stable_streak=0.0, bonus_zone=math.radians(15), tight_bonus_zone=math.radians(3)
    )
    assert narrowed_increment < default_increment


def test_custom_max_angle_zeroes_score_beyond_it():
    angle = math.radians(45)  # innerhalb der Standard-90°-Grenze, außerhalb einer 40°-Grenze
    assert compute_score_increment(angle, stable_streak=0.0) > 0.0
    assert compute_score_increment(angle, stable_streak=0.0, max_angle=math.radians(40)) == 0.0
