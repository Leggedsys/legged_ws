"""gait_scheduler — phase-based trot gait contact schedule.

Trot: diagonal pairs (FL+RR) and (FR+RL) alternate.
Each leg goes through a full cycle of period T seconds:
  [0, swing_ratio*T)   → swing (foot in air)
  [swing_ratio*T, T)   → stance (foot on ground)

FL and RR are offset by 0 phase; FR and RL by 0.5 (half period).
"""

from __future__ import annotations
import time


# Leg indices used throughout MPC (consistent with robot.yaml yaml order)
# FR=0, FL=1, RR=2, RL=3
LEG_NAMES = ["FR", "FL", "RR", "RL"]
LEG_IDX = {name: i for i, name in enumerate(LEG_NAMES)}

# Trot phase offsets: FR+RL in phase, FL+RR offset by 0.5
_TROT_OFFSETS = {
    "FR": 0.0,
    "FL": 0.5,
    "RR": 0.5,
    "RL": 0.0,
}


class GaitScheduler:
    """Phase-based trot scheduler.

    Args:
        period:      full gait cycle duration (s). Typical: 0.5–0.8 s.
        swing_ratio: fraction of cycle spent in swing. Typical: 0.4.
    """

    def __init__(self, period: float = 0.6, swing_ratio: float = 0.4) -> None:
        self._period = period
        self._swing_ratio = swing_ratio
        self._t0: float = time.monotonic()

    def reset(self) -> None:
        self._t0 = time.monotonic()

    def set_period(self, period: float) -> None:
        self._period = max(0.1, period)

    @property
    def period(self) -> float:
        return self._period

    def query(self, t: float | None = None) -> dict:
        """Return current contact/swing state for each leg.

        Returns a dict keyed by leg name with:
          'contact': bool   — True if stance (foot on ground)
          'phase':   float  — [0,1) within the leg's own cycle,
                              0=swing start, swing_ratio=stance start
        """
        if t is None:
            t = time.monotonic()
        elapsed = t - self._t0
        result = {}
        for leg in LEG_NAMES:
            offset = _TROT_OFFSETS[leg]
            phase = ((elapsed / self._period) + offset) % 1.0
            in_contact = phase >= self._swing_ratio
            result[leg] = {"contact": in_contact, "phase": phase}
        return result

    def contact_mask(self, t: float | None = None) -> list[bool]:
        """Return [FR, FL, RR, RL] contact booleans."""
        s = self.query(t)
        return [s[leg]["contact"] for leg in LEG_NAMES]

    def swing_phase(self, leg: str, t: float | None = None) -> float:
        """Return swing progress in [0,1] for a leg currently in swing.
        Returns 0.0 if the leg is in stance.
        """
        s = self.query(t)
        phase = s[leg]["phase"]
        if s[leg]["contact"]:
            return 0.0
        return phase / self._swing_ratio
