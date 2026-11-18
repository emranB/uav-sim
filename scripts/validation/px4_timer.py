"""Validate the pinned PX4 periodic-timer requeue patch without a simulator."""

from testing.simulation.px4_timer_regression import main

if __name__ == "__main__":
    raise SystemExit(main())
