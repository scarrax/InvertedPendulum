from pendulum_game_controlled import classify_mode


def test_classify_mode_pure_auto():
    assert classify_mode(auto_time=40.0, manual_time=0.0) == "Auto"


def test_classify_mode_pure_manual():
    assert classify_mode(auto_time=0.0, manual_time=40.0) == "Manual"


def test_classify_mode_mixed():
    assert classify_mode(auto_time=10.0, manual_time=30.0) == "Mixed"
