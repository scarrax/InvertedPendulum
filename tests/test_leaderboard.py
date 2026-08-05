import pandas as pd

from pendulum_game_controlled import update_leaderboard, _ensure_leaderboard_columns


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


def test_ensure_leaderboard_columns_adds_missing_columns():
    """
    Test that _ensure_leaderboard_columns backfills missing Mode and Difficulty columns.
    This is critical for handling legacy CSV files from before Task 3.
    """
    # Create a legacy DataFrame with only old columns
    legacy_df = pd.DataFrame({
        "Date": ["2026-08-01"],
        "Time": ["10:30:45"],
        "Name": ["Charlie"],
        "Score": [25.5],
    })

    # Ensure columns exist
    result_df = _ensure_leaderboard_columns(legacy_df)

    # Verify new columns were added with defaults
    assert "Mode" in result_df.columns
    assert "Difficulty" in result_df.columns
    assert result_df.loc[0, "Mode"] == "—"
    assert result_df.loc[0, "Difficulty"] == "Standard"

    # Verify original columns are intact
    assert result_df.loc[0, "Name"] == "Charlie"
    assert result_df.loc[0, "Score"] == 25.5


def test_ensure_leaderboard_columns_preserves_existing():
    """
    Test that _ensure_leaderboard_columns doesn't overwrite existing Mode/Difficulty values.
    """
    df = pd.DataFrame({
        "Date": ["2026-08-01"],
        "Time": ["10:30:45"],
        "Name": ["Alice"],
        "Score": [50.0],
        "Mode": ["Auto"],
        "Difficulty": ["Hard"],
    })

    result_df = _ensure_leaderboard_columns(df)

    # Verify existing values are preserved
    assert result_df.loc[0, "Mode"] == "Auto"
    assert result_df.loc[0, "Difficulty"] == "Hard"


def test_update_leaderboard_with_legacy_csv(tmp_path):
    """
    Regression test: update_leaderboard must handle legacy CSV files
    that lack Mode and Difficulty columns (created before Task 3).
    """
    filename = tmp_path / "leaderboard.csv"

    # Create a legacy CSV with only the old 4-column schema
    legacy_df = pd.DataFrame({
        "Date": ["2026-08-01", "2026-08-02"],
        "Time": ["10:30:45", "14:15:30"],
        "Name": ["Charlie", "Diana"],
        "Score": [25.5, 30.2],
    })
    legacy_df.to_csv(filename, index=False)

    # Update the leaderboard with a new entry - should not crash
    update_leaderboard(score=35.0, player_name="Eve", mode="Mixed", filename=str(filename))

    # Verify the new entry has Mode and Difficulty
    df = pd.read_csv(filename)
    eve_row = df[df["Name"] == "Eve"].iloc[0]
    assert eve_row["Mode"] == "Mixed"
    assert eve_row["Difficulty"] == "Standard"

    # Verify legacy entries are preserved (even without Mode/Difficulty initially)
    assert len(df) == 3
    assert "Charlie" in df["Name"].values
    assert "Diana" in df["Name"].values
