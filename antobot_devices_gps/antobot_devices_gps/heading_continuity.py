"""Continuity validation for moving-base GNSS headings."""


class HeadingContinuityChecker:
    """Accept a GNSS heading only after a stable sequence of samples.

    The receiver's iTOW is used rather than the local receive time. This
    makes the check resilient to serial buffering while still detecting
    missing GNSS epochs.
    """

    GPS_WEEK_MS = 604_800_000

    def __init__(
        self,
        minimum_samples=3,
        maximum_turn_rate_deg_s=120.0,
        maximum_sample_gap_s=0.5,
    ):
        if minimum_samples < 1:
            raise ValueError("minimum_samples must be at least 1")
        if maximum_turn_rate_deg_s <= 0:
            raise ValueError("maximum_turn_rate_deg_s must be positive")
        if maximum_sample_gap_s <= 0:
            raise ValueError("maximum_sample_gap_s must be positive")

        self.minimum_samples = minimum_samples
        self.maximum_turn_rate_deg_s = maximum_turn_rate_deg_s
        self.maximum_sample_gap_s = maximum_sample_gap_s
        self.reset()

    @staticmethod
    def shortest_angular_difference_deg(current_deg, previous_deg):
        """Return the signed smallest angular change in the range [-180, 180)."""
        return (current_deg - previous_deg + 180.0) % 360.0 - 180.0

    def reset(self):
        """Forget the current sequence after a source or continuity failure."""
        self._last_heading_deg = None
        self._last_itow_ms = None
        self._consecutive_samples = 0

    def check(self, heading_deg, itow_ms, source_valid):
        """Return ``(usable, reason)`` for one heading sample.

        ``source_valid`` contains the receiver/RTK checks. A discontinuity
        immediately makes the output unusable and starts a new sequence, so a
        later recovery must provide ``minimum_samples`` stable readings.
        """
        if not source_valid:
            self.reset()
            return False, "receiver quality flags are invalid"

        heading_deg = float(heading_deg) % 360.0
        itow_ms = int(itow_ms)

        if self._last_heading_deg is None:
            self._start_sequence(heading_deg, itow_ms)
            return self._sequence_status()

        interval_ms = (itow_ms - self._last_itow_ms) % self.GPS_WEEK_MS
        interval_s = interval_ms / 1000.0
        if interval_s <= 0.0 or interval_s > self.maximum_sample_gap_s:
            self._start_sequence(heading_deg, itow_ms)
            return False, "GNSS heading sample interval is discontinuous"

        delta_deg = self.shortest_angular_difference_deg(
            heading_deg, self._last_heading_deg
        )
        turn_rate_deg_s = abs(delta_deg) / interval_s
        if turn_rate_deg_s > self.maximum_turn_rate_deg_s:
            self.reset()
            return (
                False,
                "heading jump %.1f deg in %.3f s (%.1f deg/s)"
                % (delta_deg, interval_s, turn_rate_deg_s),
            )

        self._last_heading_deg = heading_deg
        self._last_itow_ms = itow_ms
        self._consecutive_samples += 1
        return self._sequence_status()

    def _start_sequence(self, heading_deg, itow_ms):
        self._last_heading_deg = heading_deg
        self._last_itow_ms = itow_ms
        self._consecutive_samples = 1

    def _sequence_status(self):
        if self._consecutive_samples >= self.minimum_samples:
            return True, "continuous"
        return (
            False,
            "collecting stable headings (%d/%d)"
            % (self._consecutive_samples, self.minimum_samples),
        )
