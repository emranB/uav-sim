"""Profile benchmarks and external cancellations cannot enter nominal denominators."""

from pathlib import Path

from testing.simulation.stereo_acceptance import nominal_development_attempt


def test_real_nominal_failure_remains_in_denominator() -> None:
    flight = {
        "expected": "reached",
        "obstacle_mode": "camera",
        "label": "stereo-all-single-01",
        "passed": False,
        "navigation_result": {"outcome": "cancelled"},
    }
    path = Path("failed.json")
    assert nominal_development_attempt(flight, path, set())
    assert not nominal_development_attempt(flight, path, {path})


def test_benchmark_profiles_and_degraded_checks_are_separate() -> None:
    for label in [
        "stereo-rtf-0.5-single-01",
        "stereo-rtf-1.0-single-01",
        "stereo-degraded-stale",
        "stereo-reciprocal-camera",
    ]:
        flight = {"expected": "reached", "obstacle_mode": "camera", "label": label}
        assert not nominal_development_attempt(flight, Path("report.json"), set())
