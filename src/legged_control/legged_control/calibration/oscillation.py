"""Sliding-window oscillation detector based on velocity sign reversals."""

from __future__ import annotations


class OscillationDetector:
    """Detects oscillation by counting velocity sign reversals in a time window.

    Args:
        window_sec: Length of the sliding window in seconds.
        threshold:  Number of sign reversals that triggers oscillation.
    """

    def __init__(self, window_sec: float = 1.0, threshold: int = 3) -> None:
        self._window = window_sec
        self._threshold = threshold
        self._history: list[tuple[float, float]] = []  # (timestamp, velocity)

    def update(self, t: float, velocity: float) -> None:
        """Record a new (time, velocity) sample."""
        self._history.append((t, velocity))
        cutoff = t - self._window
        self._history = [(ts, v) for ts, v in self._history if ts >= cutoff]

    def is_oscillating(self) -> bool:
        """Return True if sign reversals in the current window exceed threshold."""
        reversals = 0
        prev_sign: int | None = None
        for _, v in self._history:
            if v == 0.0:
                continue
            sign = 1 if v > 0 else -1
            if prev_sign is not None and sign != prev_sign:
                reversals += 1
            prev_sign = sign
        return reversals >= self._threshold

    def reset(self) -> None:
        self._history.clear()
