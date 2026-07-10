"""gait_scheduler — phase-based gait contact schedule (trot / crawl).

Each leg goes through a full cycle of period T seconds:
  [0, swing_ratio*T)   → swing (foot in air)
  [swing_ratio*T, T)   → stance (foot on ground)

trot:  diagonal pairs (FR+RL at offset 0, FL+RR at 0.5), swing_ratio ≤ 0.49.
crawl: one leg at a time in the classic creep order RL → FL → RR → FR
       (hind leg, then the front on the same side), swing_ratio ≤ 0.24 so
       three feet are ALWAYS planted — statically stable, can pause at any
       instant. The stair gait.
"""

from __future__ import annotations
import time


# Leg indices used throughout MPC (consistent with robot.yaml yaml order)
# FR=0, FL=1, RR=2, RL=3
LEG_NAMES = ["FR", "FL", "RR", "RL"]
LEG_IDX = {name: i for i, name in enumerate(LEG_NAMES)}

# Phase offsets per gait. A leg starts its swing when (t/T + offset) wraps
# past 0, i.e. at t/T = (1 − offset): crawl swings RL at 0, FL at 0.25,
# RR at 0.5, FR at 0.75.
_GAIT_OFFSETS = {
    "trot": {"FR": 0.0, "FL": 0.5, "RR": 0.5, "RL": 0.0},
    "crawl": {"RL": 0.0, "FL": 0.75, "RR": 0.5, "FR": 0.25},
}
# swing_ratio ceiling per gait: trot needs diagonal stance overlap; crawl
# needs the four swings to never overlap (< 0.25 keeps 3 feet down).
_MAX_SWING_RATIO = {"trot": 0.49, "crawl": 0.24}


class GaitScheduler:
    """Phase-based trot scheduler.

    Args:
        period:      full gait cycle duration (s). Typical: 0.5–0.8 s.
        swing_ratio: fraction of cycle spent in swing. Typical: 0.4.
    """

    def __init__(self, period: float = 0.6, swing_ratio: float = 0.4) -> None:
        self._period = period
        self._mode = "trot"
        self._swing_ratio = swing_ratio
        self._t0: float = time.monotonic()

    def reset(self) -> None:
        self._t0 = time.monotonic()

    def set_period(self, period: float) -> None:
        self._period = max(0.1, period)

    def set_swing_ratio(self, swing_ratio: float) -> None:
        """Runtime sync, same contract as set_period: the node reads the
        swing_ratio parameter every tick for the TRAJECTORY; if the
        scheduler keeps its constructor value the contact schedule and the
        foot trajectory disagree about when swing ends (foot commanded
        mid-air at 'touchdown'). Clamped per gait — trot needs diagonal
        stance overlap, crawl needs three feet always planted."""
        self._swing_ratio = float(
            min(max(swing_ratio, 0.1), _MAX_SWING_RATIO[self._mode])
        )

    def set_mode(self, mode: str) -> None:
        """Switch gait ("trot"/"crawl"). Phases jump on a switch — the node
        only applies it while the robot is standing (gait not running)."""
        if mode in _GAIT_OFFSETS and mode != self._mode:
            self._mode = mode
            self.set_swing_ratio(self._swing_ratio)  # re-clamp for new gait

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def period(self) -> float:
        return self._period

    @property
    def swing_ratio(self) -> float:
        return self._swing_ratio

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
            offset = _GAIT_OFFSETS[self._mode][leg]
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

    def stance_phase(self, leg: str, t: float | None = None) -> float:
        """Return stance progress in [0,1] for a leg currently in stance.
        Returns 0.0 if the leg is in swing.
        """
        s = self.query(t)
        phase = s[leg]["phase"]
        if not s[leg]["contact"]:
            return 0.0
        return (phase - self._swing_ratio) / (1.0 - self._swing_ratio)
