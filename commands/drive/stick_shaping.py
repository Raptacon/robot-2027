"""
Shaping the translate stick: deadband, response curve and slew limit.

Why shape the stick as a whole?
    InputFactory can shape each axis on its own, but doing that to the left
    stick makes a square deadband and bends diagonal moves (a 45 degree push
    comes out at a different angle). Here the stick is treated as one arrow:
    only its *length* is shaped, so the robot always goes the way the stick
    points.

What each step does:
    1. Deadband: a stick that is barely pushed (a worn stick never rests at
       exactly 0) counts as 0. Past the deadband, the length is rescaled so
       it still starts at 0 and reaches 1 at full push.
    2. Curve: length ** exponent. With 2, half push gives quarter speed, so
       small moves are easier to control while full push is still full speed.
    3. Slew: :class:`VectorSlewLimiter` stops the commanded velocity from
       changing faster than an acceleration limit, so the robot doesn't jerk
       or tip when the driver slams the stick.

Example:
    >>> from commands.drive.stick_shaping import shape_stick
    >>> shape_stick(0.05, 0.0, deadband=0.1, exponent=2.0)  # inside the deadband
    (0.0, 0.0)
    >>> x, y = shape_stick(0.7071, 0.7071, deadband=0.1, exponent=2.0)  # full diagonal
    >>> round(x, 3), round(y, 3)  # still exactly 45 degrees
    (0.707, 0.707)
"""

import math

# TODO: Move stick-vector shaping into the controller shaping factory
# (utils/input/shaping.py). Add a 2D "translate" action whose deadband and
# curve (squared, spline, segmented) come from the YAML and are tunable from
# NetworkTables, applied to the stick's length instead of each axis. Then
# TeleopDrive can use it and shape_stick can go. VectorSlewLimiter stays,
# since the factory's slew works on one axis at a time.


def shape_stick(x: float, y: float, deadband: float, exponent: float) -> tuple[float, float]:
    """Apply a round deadband and a response curve to a stick, keeping its direction.

    Args:
        x: Stick forward amount, -1 to 1.
        y: Stick left amount, -1 to 1.
        deadband: Stick length (0 to 1) below which the output is 0.
        exponent: Response curve power. 1 is linear, 2 is squared.

    Returns:
        The shaped (x, y). The length is at most 1, and the direction is the
        same as the input's.

    Example:
        Half push straight forward, squared:

        >>> from commands.drive.stick_shaping import shape_stick
        >>> x, y = shape_stick(0.55, 0.0, deadband=0.1, exponent=2.0)
        >>> round(x, 3), y
        (0.25, 0.0)
    """
    length = math.hypot(x, y)
    if length <= deadband:
        return 0.0, 0.0
    # A corner push on a square-gated stick can read a little over 1.
    clipped = min(length, 1.0)
    shaped = ((clipped - deadband) / (1.0 - deadband)) ** exponent
    return x / length * shaped, y / length * shaped


class VectorSlewLimiter:
    """Limits how fast a 2D velocity can change (an acceleration limit).

    Unlike WPILib's ``SlewRateLimiter``, which limits x and y separately,
    this limits the change of the whole velocity arrow, so a diagonal
    speeds up as quickly as a straight move and keeps its direction.

    Args:
        max_accel: The largest change per second, in the same units as the
            values per second (for m/s values, m/s^2).
        period_s: How often :meth:`calculate` is called (0.02 for our loop).

    Example:
        Limit to 5 m/s^2: one 20 ms loop can add at most 0.1 m/s.

        >>> from commands.drive.stick_shaping import VectorSlewLimiter
        >>> slew = VectorSlewLimiter(max_accel=5.0, period_s=0.02)
        >>> vx, vy = slew.calculate(3.0, 0.0)
        >>> round(vx, 6), vy
        (0.1, 0.0)
    """

    def __init__(self, max_accel: float, period_s: float = 0.02) -> None:
        self.max_accel = max_accel
        self.period_s = period_s
        self._x = 0.0
        self._y = 0.0

    def calculate(self, x: float, y: float) -> tuple[float, float]:
        """Move toward (x, y) by at most ``max_accel * period_s`` and return the new value."""
        dx = x - self._x
        dy = y - self._y
        step = math.hypot(dx, dy)
        max_step = self.max_accel * self.period_s
        if step > max_step:
            dx *= max_step / step
            dy *= max_step / step
        self._x += dx
        self._y += dy
        return self._x, self._y

    def reset(self, x: float = 0.0, y: float = 0.0) -> None:
        """Jump straight to (x, y), for example when the command restarts."""
        self._x = x
        self._y = y
