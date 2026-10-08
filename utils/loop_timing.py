"""
Continuous loop-timing instrumentation.

Measures how much of the timed robot periodic budget the robot code consumes,
broken down by channel (e.g. userCode vs scheduler).  Stats are recorded
every cycle and published to NetworkTables under /FrameTiming/ at 10 Hz by
default, so they are always visible (not only on overrun like the built-in
CommandScheduler Watchdog) without adding NT writes to every loop.

Timing uses FPGA timestamps (microsecond resolution on real hardware).
In simulation the values reflect simulated time, not wall-clock.
"""

import math

import ntcore
import wpilib


class LoopTimingStats:
    """Running statistics accumulator (Welford's online algorithm).

    All public values are in **milliseconds**.  Input to :meth:`record`
    is in seconds (matching ``Timer.getFPGATimestamp()`` units).
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self._count: int = 0
        self._min: float = float("inf")
        self._max: float = float("-inf")
        self._last: float = 0.0
        # Welford accumulators
        self._mean: float = 0.0
        self._m2: float = 0.0

    def record(self, duration_sec: float):
        """Record one sample.  *duration_sec* is converted to ms internally."""
        ms = duration_sec * 1000.0
        self._last = ms
        self._count += 1

        if ms < self._min:
            self._min = ms
        if ms > self._max:
            self._max = ms

        # Welford's online update
        delta = ms - self._mean
        self._mean += delta / self._count
        delta2 = ms - self._mean
        self._m2 += delta * delta2

    @property
    def count(self) -> int:
        return self._count

    @property
    def min_ms(self) -> float:
        return self._min if self._count > 0 else 0.0

    @property
    def max_ms(self) -> float:
        return self._max if self._count > 0 else 0.0

    @property
    def avg_ms(self) -> float:
        return self._mean if self._count > 0 else 0.0

    @property
    def stddev_ms(self) -> float:
        if self._count < 2:
            return 0.0
        return math.sqrt(self._m2 / (self._count - 1))

    @property
    def last_ms(self) -> float:
        return self._last


class LoopTimer:
    """Coordinator that manages named timing channels and publishes stats.

    Typical usage::

        timer = LoopTimer(budget_sec=0.020)
        timer.add_channel("userCode")
        timer.add_channel("scheduler")

        # each cycle:
        timer.start("userCode")
        ...
        timer.stop("userCode")
        timer.publish()
    """

    _STAT_NAMES = ("lastMs", "minMs", "maxMs", "avgMs", "stddevMs", "count", "budgetPct")

    def __init__(self, budget_sec: float, publish_period_sec: float = 0.1):
        self._budget_ms = budget_sec * 1000.0
        self._publish_period = publish_period_sec
        self._last_publish = -math.inf
        self._channels: dict[str, LoopTimingStats] = {}
        self._starts: dict[str, float] = {}
        self._table = ntcore.NetworkTableInstance.getDefault().getTable("FrameTiming")
        self._channel_pubs: dict[str, dict[str, ntcore.DoublePublisher]] = {}
        self._total_ms_pub = self._table.getDoubleTopic("totalMs").publish()
        self._total_pct_pub = self._table.getDoubleTopic("totalBudgetPct").publish()
        self._alert_info = wpilib.Alert("Frame Timing", wpilib.Alert.AlertType.kInfo)
        self._alert_warning = wpilib.Alert("Frame Timing", wpilib.Alert.AlertType.kWarning)
        self._alert_error = wpilib.Alert("Frame Timing", wpilib.Alert.AlertType.kError)

    def add_channel(self, name: str):
        self._channels[name] = LoopTimingStats()
        sub = self._table.getSubTable(name)
        self._channel_pubs[name] = {stat: sub.getDoubleTopic(stat).publish() for stat in self._STAT_NAMES}

    def start(self, name: str):
        self._starts[name] = wpilib.Timer.getFPGATimestamp()

    def stop(self, name: str):
        end = wpilib.Timer.getFPGATimestamp()
        begin = self._starts.pop(name, end)
        self._channels[name].record(end - begin)

    def reset_all(self):
        for stats in self._channels.values():
            stats.reset()
        self._starts.clear()

    def publish(self):
        """Update alerts every call; write NT values at the publish rate."""
        total_ms = sum(stats.last_ms for stats in self._channels.values())
        total_pct = 0.0
        if self._budget_ms > 0:
            total_pct = (total_ms / self._budget_ms) * 100.0

        now = wpilib.Timer.getFPGATimestamp()
        if now - self._last_publish >= self._publish_period:
            self._last_publish = now
            self._publish_nt(total_ms, total_pct)

        # Alert based on budget usage
        overrun = total_pct >= 100.0
        warning = total_pct >= 80.0 and not overrun
        info = not warning and not overrun

        self._alert_error.set(overrun)
        self._alert_warning.set(warning)
        self._alert_info.set(info)

        if overrun:
            self._alert_error.setText(f"Frame Timing: OVERRUN {total_pct:.0f}% ({total_ms:.1f}ms)")
        elif warning:
            self._alert_warning.setText(f"Frame Timing: {total_pct:.0f}% - possible overrun")
        else:
            self._alert_info.setText(f"Frame Timing: {total_pct:.0f}%")

    def _publish_nt(self, total_ms: float, total_pct: float):
        for name, stats in self._channels.items():
            pubs = self._channel_pubs[name]
            pubs["lastMs"].set(stats.last_ms)
            pubs["minMs"].set(stats.min_ms)
            pubs["maxMs"].set(stats.max_ms)
            pubs["avgMs"].set(stats.avg_ms)
            pubs["stddevMs"].set(stats.stddev_ms)
            pubs["count"].set(stats.count)
            if self._budget_ms > 0:
                pubs["budgetPct"].set((stats.last_ms / self._budget_ms) * 100.0)
        self._total_ms_pub.set(total_ms)
        if self._budget_ms > 0:
            self._total_pct_pub.set(total_pct)
