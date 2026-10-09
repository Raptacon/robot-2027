"""
SysId for something that turns where gravity doesn't matter (turrets, swerve steering).

Positions are radians and speeds rad/s. If it has hard stops (a turret),
give limits a little inside them; if it turns forever (swerve steering),
leave them out. For something gravity pulls on, use
:mod:`utils.sysid.mech.arm` instead.

Example:
    >>> import math
    >>> import commands2
    >>> from utils.sysid.characterizable import Reading
    >>> from utils.sysid.mech.rotary import rotary
    >>> turret = rotary(
    ...     "turret",
    ...     subsystem=commands2.Subsystem(),
    ...     set_voltage=lambda volts: None,
    ...     read=lambda: Reading(volts=0.0, position=0.0, velocity=0.0),
    ...     min_rad=math.radians(-170),
    ...     max_rad=math.radians(170),
    ... )
    >>> round(turret.max_position, 3)
    2.967
"""

from collections.abc import Callable

import commands2

from utils.sysid.characterizable import Characterizable, Gravity, Reading
from utils.sysid.mech._checks import check_limits
from utils.sysid.settings import SysIdSettings

ROTARY_SYSID = SysIdSettings(ramp_volts_per_s=1.0, step_volts=4.0, timeout_s=6.0, dynamic_timeout_s=2.0)
"""Default rotary settings (the same as swerve steering)."""


def rotary(
    name: str,
    subsystem: commands2.Subsystem,
    set_voltage: Callable[[float], None],
    read: Callable[[], Reading],
    min_rad: float | None = None,
    max_rad: float | None = None,
    settings: SysIdSettings = ROTARY_SYSID,
) -> Characterizable:
    """Describe a rotary mechanism for SysId.

    Args:
        name: Short name for the dashboard and log, like ``"turret"``.
        subsystem: The subsystem the motors belong to.
        set_voltage: Apply this many volts.
        read: Return a :class:`~utils.sysid.characterizable.Reading` in radians and rad/s.
        min_rad: Reverse tests stop here, radians. ``None`` if it turns forever.
        max_rad: Forward tests stop here, radians. ``None`` if it turns forever.
        settings: How hard to push it (:data:`ROTARY_SYSID` by default).

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
        gravity=Gravity.NONE,
        min_position=min_rad,
        max_position=max_rad,
    )
