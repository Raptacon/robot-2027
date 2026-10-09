"""
A quick feedforward fit from SysId readings, without the SysId app.

The WPILib SysId app is the real tool: it reads the wpilog and shows plots
that tell you whether the data is any good. This file does the same basic
math (a least-squares fit of ``volts = kS x sign(v) + kV x v + kA x a``) so
the robot can show a rough answer on the dashboard right after a test, and
so the tests in CI can check the routines against the simulator.

Mechanisms that lift against gravity get one more number, kG:

- an **elevator** needs the same kG volts at every height:
  ``volts = kG + kS x sign(v) + kV x v + kA x a``
- an **arm** needs the most when it sticks straight out and none when it
  points straight up or down: ``volts = kG x cos(angle) + ...``. The angle
  must be in radians with 0 meaning the arm is level (horizontal).

Example:
    Readings from a motor with kS = 0.2, kV = 2.0 and kA = 0.5, one loop
    (0.02 s) apart, while its speed rises and falls:

    >>> import math
    >>> from utils.sysid.characterizable import Gravity, Reading
    >>> from utils.sysid.fit import fit_feedforward
    >>> speed = [1.0 + 0.5 * math.sin(i / 5) for i in range(60)]
    >>> accel = [(speed[i + 1] - speed[i - 1]) / 0.04 for i in range(1, 59)]
    >>> run = [Reading(volts=0.2 + 2.0 * v + 0.5 * a, position=0.0, velocity=v) for v, a in zip(speed[1:], accel)]
    >>> fit = fit_feedforward([run], period_s=0.02)
    >>> round(fit.ks, 2), round(fit.kv, 2), round(fit.ka, 2)
    (0.2, 2.0, 0.5)

    An elevator with kG = 0.6 needs runs both up and down, so kG and kS can
    be told apart:

    >>> up = [Reading(volts=0.6 + 0.2 + 2.0 * v, position=0.0, velocity=v) for v in (0.1, 0.2, 0.3, 0.4)]
    >>> down = [Reading(volts=0.6 - 0.2 - 2.0 * v, position=0.0, velocity=-v) for v in (0.1, 0.2, 0.3, 0.4)]
    >>> fit = fit_feedforward([up, down], period_s=0.02, gravity=Gravity.ELEVATOR, fit_ka=False)
    >>> round(fit.kg, 2), round(fit.ks, 2), round(fit.kv, 2)
    (0.6, 0.2, 2.0)
"""

import math
from collections.abc import Sequence
from dataclasses import dataclass

from utils.sysid.characterizable import Gravity, Reading

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
        kg: Volts to hold the mechanism up against gravity (0 when the
            mechanism has no gravity to fight, like a drivetrain).
        r_squared: How well the line fits, from 0 (not at all) to 1 (perfect).
            Below about 0.9, look at the data in the SysId app before using it.
        samples: How many readings were used.
    """

    ks: float
    kv: float
    ka: float
    kg: float
    r_squared: float
    samples: int


def fit_feedforward(
    runs: Sequence[Sequence[Reading]],
    period_s: float,
    gravity: Gravity = Gravity.NONE,
    fit_ka: bool = True,
) -> FeedforwardFit:
    """Fit kS, kV, kA (and kG for an elevator or arm) to the readings from SysId tests.

    Acceleration is worked out from the change in velocity between loops,
    within each run only (never across two tests). kS and kG can only be
    told apart for an elevator if there are runs in both directions.

    Args:
        runs: One list of readings per test, one reading per loop, in order.
        period_s: Time between readings, seconds.
        gravity: What gravity does to the mechanism (see :class:`~utils.sysid.characterizable.Gravity`).
        fit_ka: ``False`` to leave kA out (reported as 0), for a slow ramp
            where the mechanism barely accelerates and kA can't be measured.

    Returns:
        The :class:`FeedforwardFit`.

    Raises:
        ValueError: If there are too few moving readings, or they don't vary
            enough, to fit the numbers.
    """
    rows: list[list[float]] = []
    targets: list[float] = []
    for run in runs:
        for before, now, after in zip(run, run[1:], run[2:]):
            if abs(now.velocity) < MIN_SPEED:
                continue
            accel = (after.velocity - before.velocity) / (2.0 * period_s)
            row = [math.copysign(1.0, now.velocity), now.velocity]
            if fit_ka:
                row.append(accel)
            if gravity == Gravity.ELEVATOR:
                row.append(1.0)
            elif gravity == Gravity.ARM:
                row.append(math.cos(now.position))
            rows.append(row)
            targets.append(now.volts)
    unknowns = (3 if fit_ka else 2) + (0 if gravity == Gravity.NONE else 1)
    if len(rows) < unknowns:
        raise ValueError(f"Need at least {unknowns} moving readings to fit, got {len(rows)}")

    gains = _least_squares(rows, targets)
    predicted = [sum(g * x for g, x in zip(gains, row)) for row in rows]
    mean = sum(targets) / len(targets)
    total = sum((y - mean) ** 2 for y in targets)
    residual = sum((y - p) ** 2 for y, p in zip(targets, predicted))
    r_squared = 1.0 - residual / total if total > 0 else 0.0
    ks, kv = gains[0], gains[1]
    ka = gains[2] if fit_ka else 0.0
    kg = gains[-1] if gravity != Gravity.NONE else 0.0
    return FeedforwardFit(ks=ks, kv=kv, ka=ka, kg=kg, r_squared=r_squared, samples=len(rows))


def _least_squares(rows: list[list[float]], targets: list[float]) -> list[float]:
    """Solve a small least-squares problem with the normal equations and Gaussian elimination."""
    n = len(rows[0])
    ata = [[sum(r[i] * r[j] for r in rows) for j in range(n)] for i in range(n)]
    aty = [sum(r[i] * y for r, y in zip(rows, targets)) for i in range(n)]
    m = [ata[i] + [aty[i]] for i in range(n)]
    tiny = 1e-9 * max(abs(ata[i][i]) for i in range(n))
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) <= tiny:
            raise ValueError("Readings don't vary enough to fit (did the mechanism move both ways?)")
        m[col], m[pivot] = m[pivot], m[col]
        for r in range(n):
            if r != col:
                factor = m[r][col] / m[col][col]
                m[r] = [a - factor * b for a, b in zip(m[r], m[col])]
    return [m[i][n] / m[i][i] for i in range(n)]
