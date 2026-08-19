from pendulum_game_controlled import DIFFICULTY_ORDER, DIFFICULTY_LEVELS, next_difficulty


def test_difficulty_order_has_three_levels_in_expected_sequence():
    assert DIFFICULTY_ORDER == ("Leicht", "Standard", "Schwer")


def test_next_difficulty_cycles_forward():
    assert next_difficulty("Leicht") == "Standard"
    assert next_difficulty("Standard") == "Schwer"
    assert next_difficulty("Schwer") == "Leicht"


def test_standard_matches_ap3_original_constants():
    level = DIFFICULTY_LEVELS["Standard"]
    assert level["m_cart"] == 5.0
    assert level["m_pend"] == 0.5
    assert level["d_cart"] == 0.15
    assert level["d_pend"] == 0.01
    assert level["bonus_zone_deg"] == 15.0
    assert level["tight_bonus_zone_deg"] == 5.0


def test_all_levels_present_with_required_keys():
    required_keys = {
        "bonus_zone_deg",
        "tight_bonus_zone_deg",
        "m_cart",
        "m_pend",
        "d_cart",
        "d_pend",
    }
    for name in DIFFICULTY_ORDER:
        assert set(DIFFICULTY_LEVELS[name].keys()) == required_keys
