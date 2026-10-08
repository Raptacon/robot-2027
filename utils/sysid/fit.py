"""
A quick feedforward fit from SysId readings, without the SysId app.

The WPILib SysId app is the real tool: it reads the wpilog and shows plots
that tell you whether the data is any good. This file does the same basic
math (a least-squares fit of ``volts = kS x sign(v) + kV x v + kA x a``) so
the robot can show a rough answer on the dashboard right after a test, and
so the tests in CI can check the routines against the simulator.

Example:
    Readings from a motor with kS = 0.2, kV = 2.0 and kA = 0.5, one loop
    (0.02 s) apart, while its speed rises and falls:

    >>> import math
    >>> from utils.sysid.characterizable import Reading
    >>> from utils.sysid.fit import fit_feedforward
    >>> speed = [1.0 + 0.5 * math.sin(i / 5) for i in range(60)]
    >>> accel = [(speed[i + 1] - speed[i - 1]) / 0.04 for i in range(1, 59)]
    >>> run = [Reading(volts=0.2 + 2.0 * v + 0.5 * a, position=0.0, velocity=v) for v, a in zip(speed[1:], accel)]
    >>> fit = fit_feedforward([run], period_s=0.02)
    >>> round(fit.ks, 2), round(fit.kv, 2), round(fit.ka, 2)
    (0.2, 2.0, 0.5)
"""

import math
from collections.abc import Sequence
from dataclasses import dataclass

from utils.sysid.characterizable import Reading

MIN_SPEED = 1e-3
"""Readings slower than this (m/s or rad/s) are left out: the mechanism hasn't started moving."""


@dataclass(frozen=True)
class FeedforwardFit:
    """The result of :func:`fit_feedforward`.

    Units are volts, volts per (m/s) and volts per (m/s^2), or per rad/s and
    rad/s^2 for a mechanism that turns.

    Attributes:
        ks: Volts to overcome friction.
        kv: Volts per unit of speed.
        ka: Volts per unit of acceleration.
        r_squared: How well the line fits, from 0 (not at all) to 1 (perfect).
            Below about 0.9, look at the data in the SysId app before using it.
        samples: How many readings were used.
    """

    ks: float
    kv: float
    ka: float
    r_squared: float
    samples: int


def fit_feedforward(runs: Sequence[Sequence[Reading]], period_s: float) -> FeedforwardFit:
    """Fit kS, kV and kA to the readings from one or more SysId tests.

    Acceleration is worked out from the change in velocity between loops,
    within each run only (never across two tests).

    Args:
        runs: One list of readings per test, one reading per loop, in order.
        period_s: Time between readings, seconds.

    Returns:
        The :class:`FeedforwardFit`.

    Raises:
        ValueError: If there are too few moving readings to fit three numbers.
    """
    rows: list[tuple[float, float, float]] = []
    targets: list[float] = []
    for run in runs:
        for before, now, after in zip(run, run[1:], run[2:]):
            if abs(now.velocity) < MIN_SPEED:
                continue
            accel = (after.velocity - before.velocity) / (2.0 * period_s)
            rows.append((math.copysign(1.0, now.velocity), now.velocity, accel))
            targets.append(now.volts)
    if len(rows) < 3:
        raise ValueError(f"Need at least 3 moving readings to fit, got {len(rows)}")

    ks, kv, ka = _least_squares(rows, targets)
    predicted = [ks * s + kv * v + ka * a for s, v, a in rows]
    mean = sum(targets) / len(targets)
    total = sum((y - mean) ** 2 for y in targets)
    residual = sum((y - p) ** 2 for y, p in zip(targets, predicted))
    r_squared = 1.0 - residual / total if total > 0 else 0.0
    return FeedforwardFit(ks=ks, kv=kv, ka=ka, r_squared=r_squared, samples=len(rows))


def _least_squares(rows: list[tuple[float, float, float]], targets: list[float]) -> tuple[float, float, float]:
    """Solve the 3-unknown least-squares problem with the normal equations."""
    # Build A^T A (3x3) and A^T y (3), then solve by Gaussian elimination.
    ata = [[sum(r[i] * r[j] for r in rows) for j in range(3)] for i in range(3)]
    aty = [sum(r[i] * y for r, y in zip(rows, targets)) for i in range(3)]
    m = [ata[i] + [aty[i]] for i in range(3)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-12:
            raise ValueError("Readings don't vary enough to fit (did the mechanism move?)")
        m[col], m[pivot] = m[pivot], m[col]
        for r in range(3):
            if r != col:
                factor = m[r][col] / m[col][col]
                m[r] = [a - factor * b for a, b in zip(m[r], m[col])]
    ks, kv, ka = (m[i][3] / m[i][i] for i in range(3))
    return ks, kv, ka
