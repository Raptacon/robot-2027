"""
SysId for a flywheel: anything that spins freely (shooter wheels, rollers, intakes).

A flywheel can't run into anything, so it has no limits and can take
WPILib's normal 7 V step. Positions are radians and speeds rad/s.

Example:
    >>> import commands2
    >>> from utils.sysid.characterizable import Reading
    >>> from utils.sysid.mech.flywheel import flywheel
    >>> shooter = flywheel(
    ...     "shooter",
    ...     subsystem=commands2.Subsystem(),
    ...     set_voltage=lambda volts: None,
    ...     read=lambda: Reading(volts=0.0, position=0.0, velocity=0.0),
    ... )
    >>> shooter.angular, shooter.settings.step_volts
    (True, 7.0)
"""

from collections.abc import Callable

import commands2

from utils.sysid.characterizable import Characterizable, Gravity, Reading
from utils.sysid.settings import SysIdSettings

FLYWHEEL_SYSID = SysIdSettings(ramp_volts_per_s=1.0, step_volts=7.0, timeout_s=10.0, dynamic_timeout_s=3.0)
"""Default flywheel settings: WPILib's own defaults, since nothing can crash."""


def flywheel(
    name: str,
    subsystem: commands2.Subsystem,
    set_voltage: Callable[[float], None],
    read: Callable[[], Reading],
    settings: SysIdSettings = FLYWHEEL_SYSID,
) -> Characterizable:
    """Describe a flywheel for SysId.

    Args:
        name: Short name for the dashboard and log, like ``"shooter"``.
        subsystem: The subsystem the motors belong to.
        set_voltage: Apply this many volts.
        read: Return a :class:`~utils.sysid.characterizable.Reading` in radians and rad/s.
        settings: How hard to push it (:data:`FLYWHEEL_SYSID` by default).

    Returns:
        The :class:`~utils.sysid.characterizable.Characterizable`.
    """
    return Characterizable(name, subsystem, set_voltage, read, angular=True, settings=settings, gravity=Gravity.NONE)
