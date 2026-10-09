"""
SysId for an arm or wrist: it turns, and gravity pulls on it.

Gravity pulls hardest when the arm sticks straight out and not at all when
it points straight up or down, so the fit measures kG along with kS, kV
and kA. For that to work:

- the position must be in **radians with 0 meaning level** (horizontal), so
  set the arm's encoder offset first, and
- the arm must have **limits** a little inside its hard stops. Every test
  stops when it reaches one.

The default settings are gentle (2 V step, 6 V most) because an arm can
hurt someone. Raise them on the dashboard once you've seen it move.

Example:
    >>> import math
    >>> import commands2
    >>> from utils.sysid.characterizable import Gravity, Reading
    >>> from utils.sysid.mech.arm import arm
    >>> pivot = arm(
    ...     "pivot",
    ...     subsystem=commands2.Subsystem(),
    ...     set_voltage=lambda volts: None,
    ...     read=lambda: Reading(volts=0.0, position=0.0, velocity=0.0),
    ...     min_rad=math.radians(-80),
    ...     max_rad=math.radians(80),
    ... )
    >>> pivot.gravity == Gravity.ARM
    True
"""

from collections.abc import Callable

import commands2

from utils.sysid.characterizable import Characterizable, Gravity, Reading
from utils.sysid.mech._checks import check_limits
from utils.sysid.settings import SysIdSettings

ARM_SYSID = SysIdSettings(ramp_volts_per_s=0.5, step_volts=2.0, timeout_s=4.0, dynamic_timeout_s=1.0, max_volts=6.0)
"""Default arm settings: gentle, because an arm moves fast and can hit someone."""


def arm(
    name: str,
    subsystem: commands2.Subsystem,
    set_voltage: Callable[[float], None],
    read: Callable[[], Reading],
    min_rad: float,
    max_rad: float,
    settings: SysIdSettings = ARM_SYSID,
) -> Characterizable:
    """Describe an arm for SysId.

    Args:
        name: Short name for the dashboard and log, like ``"pivot"``.
        subsystem: The subsystem the motors belong to.
        set_voltage: Apply this many volts.
        read: Return a :class:`~utils.sysid.characterizable.Reading` in radians
            (0 = level) and rad/s.
        min_rad: Reverse tests stop here, radians. Required.
        max_rad: Forward tests stop here, radians. Required.
        settings: How hard to push it (:data:`ARM_SYSID` by default).

    Returns:
        The :class:`~utils.sysid.characterizable.Characterizable`.

    Raises:
        ValueError: If the limits are backwards or look like degrees.
    """
    check_limits(min_rad, max_rad, angular=True)
    return Characterizable(
        name,
        subsystem,
        set_voltage,
        read,
        angular=True,
        settings=settings,
        gravity=Gravity.ARM,
        min_position=min_rad,
        max_position=max_rad,
    )
