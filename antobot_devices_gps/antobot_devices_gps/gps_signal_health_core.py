"""Pure GSV signal-strength aggregation used by the ROS health node."""

from dataclasses import dataclass
import math
from typing import Dict, Iterable, List, Tuple


@dataclass(frozen=True)
class SignalHealthSnapshot:
    """A time-windowed C/N0 summary derived from GSV sentences."""

    satellites_in_view: int
    constellation_count: int
    weak_signal_count: int
    cno_mean_dbhz: float
    cno_median_dbhz: float
    cno_p10_dbhz: float


class GsvSignalAccumulator:
    """Keep the latest C/N0 values for satellites announced by GSV."""

    def __init__(self, sample_timeout_s: float = 2.0, weak_cno_dbhz: float = 25.0) -> None:
        if sample_timeout_s <= 0.0:
            raise ValueError("sample_timeout_s must be positive")
        self.sample_timeout_s = sample_timeout_s
        self.weak_cno_dbhz = weak_cno_dbhz
        self._signals: Dict[Tuple[str, str], Tuple[float, float]] = {}

    def add(self, now: float, talker: str, satellites: Iterable[Tuple[str, float]]) -> None:
        """Record finite C/N0 samples from one GSV sentence."""
        for satellite_id, cno_dbhz in satellites:
            if satellite_id and math.isfinite(cno_dbhz) and cno_dbhz >= 0.0:
                self._signals[(talker, satellite_id)] = (now, cno_dbhz)

    def snapshot(self, now: float) -> SignalHealthSnapshot | None:
        """Return fresh satellite statistics, pruning expired GSV observations."""
        self._signals = {
            key: value
            for key, value in self._signals.items()
            if now - value[0] <= self.sample_timeout_s
        }
        if not self._signals:
            return None

        values = sorted(value[1] for value in self._signals.values())
        constellation_count = len({key[0] for key in self._signals})
        return SignalHealthSnapshot(
            satellites_in_view=len(values),
            constellation_count=constellation_count,
            weak_signal_count=sum(value < self.weak_cno_dbhz for value in values),
            cno_mean_dbhz=sum(values) / len(values),
            cno_median_dbhz=self._percentile(values, 50.0),
            cno_p10_dbhz=self._percentile(values, 10.0),
        )

    @staticmethod
    def _percentile(values: List[float], percentile: float) -> float:
        if len(values) == 1:
            return values[0]
        position = (len(values) - 1) * percentile / 100.0
        lower = math.floor(position)
        upper = math.ceil(position)
        if lower == upper:
            return values[lower]
        return values[lower] + (values[upper] - values[lower]) * (position - lower)
