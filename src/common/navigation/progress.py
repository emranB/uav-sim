"""Bounded goal-progress observation, independent of wall-clock APIs and transports."""

from dataclasses import dataclass, field


@dataclass
class GoalProgress:
    """Bound stalls using goal improvement and optional movement through observed cells.

    Time is monotonic wall seconds supplied by the session. Revisited cells cannot
    keep a stalled exploration alive. Without route cells, retain goal-only behavior.
    """

    best_error_m: float
    last_progress_monotonic: float
    timeout_seconds: float = 40.0
    improvement_m: float = 0.2
    initial_route_cell: tuple[int, int] | None = None
    _visited_route_cells: set[tuple[int, int]] = field(default_factory=set, init=False, repr=False)

    def __post_init__(self) -> None:
        if self.initial_route_cell is not None:
            self._visited_route_cells.add(self.initial_route_cell)

    def blocked(
        self,
        error_m: float,
        now_monotonic: float,
        observed_route_cell: tuple[int, int] | None = None,
    ) -> bool:
        """Stop after the declared interval without goal or newly traversed observed space."""
        if observed_route_cell is not None and observed_route_cell not in self._visited_route_cells:
            self._visited_route_cells.add(observed_route_cell)
            self.last_progress_monotonic = now_monotonic
        if error_m < self.best_error_m - self.improvement_m:
            self.best_error_m = error_m
            self.last_progress_monotonic = now_monotonic
        return now_monotonic - self.last_progress_monotonic >= self.timeout_seconds
