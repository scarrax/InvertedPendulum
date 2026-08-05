import pandas as pd

from pendulum_game_controlled import update_leaderboard


def test_update_leaderboard_writes_mode_and_default_difficulty(tmp_path):
    filename = tmp_path / "leaderboard.csv"

    update_leaderboard(score=12.34, player_name="Alice", mode="Auto", filename=str(filename))

    df = pd.read_csv(filename)
    assert df.loc[0, "Mode"] == "Auto"
    assert df.loc[0, "Difficulty"] == "Standard"


def test_update_leaderboard_custom_difficulty(tmp_path):
    filename = tmp_path / "leaderboard.csv"

    update_leaderboard(
        score=5.0, player_name="Bob", mode="Manual", difficulty="Hard", filename=str(filename)
    )

    df = pd.read_csv(filename)
    assert df.loc[0, "Difficulty"] == "Hard"
