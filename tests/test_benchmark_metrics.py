import math

from benchmark_controllers import held_from, find_capture_envelope


def test_held_from_returns_start_time_when_held_from_the_beginning():
    t = [0.0, 0.5, 1.0, 1.5]
    theta = [0.0, 0.0, 0.0, 0.0]
    result = held_from(t, theta, tolerance_rad=math.radians(5), hold_duration=1.0)
    assert result == 0.0


def test_held_from_returns_none_when_it_only_passes_through_the_zone():
    t = [0.0, 0.5, 1.0, 1.5]
    theta = [math.radians(90), 0.0, math.radians(90), math.radians(90)]
    result = held_from(t, theta, tolerance_rad=math.radians(5), hold_duration=1.0)
    assert result is None


def test_held_from_returns_none_when_never_within_tolerance():
    t = [0.0, 0.5, 1.0]
    theta = [math.radians(90), math.radians(90), math.radians(90)]
    result = held_from(t, theta, tolerance_rad=math.radians(5), hold_duration=1.0)
    assert result is None


def test_held_from_returns_entry_time_not_confirmation_time():
    t = [0.0, 0.3, 0.6, 1.0, 1.3]
    theta = [math.radians(90), 0.0, 0.0, 0.0, 0.0]
    result = held_from(t, theta, tolerance_rad=math.radians(5), hold_duration=1.0)
    assert result == 0.3


def test_find_capture_envelope_returns_largest_success_before_first_failure():
    results = {2: True, 4: True, 6: True, 8: False, 10: False}
    assert find_capture_envelope(results) == 6


def test_find_capture_envelope_ignores_a_later_spurious_success():
    results = {2: True, 4: True, 6: False, 8: True}
    assert find_capture_envelope(results) == 4


def test_find_capture_envelope_returns_none_when_first_value_fails():
    results = {2: False, 4: True}
    assert find_capture_envelope(results) is None


def test_find_capture_envelope_returns_none_for_empty_input():
    assert find_capture_envelope({}) is None
