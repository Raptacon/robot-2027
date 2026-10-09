"""
SysId for a drivetrain: wheels that carry the robot across the floor.

Unlike a flywheel, the drive wheels carry the robot's weight. A big voltage
step makes them slip and sags the battery, so the default step is 4 V, and
the robot needs room, so every test stops after ``max_travel_m`` meters.
Positions are meters and speeds m/s. Run it on carpet.

Example:
    >>> import commands2
    >>> from utils.sysid.characterizable import Reading
    >>> from utils.sysid.mech.drivetrain import drivetrain
    >>> drive = drivetrain(
    ...     "drive",
    ...     subsystem=commands2.Subsystem(),
    ...     set_voltage=lambda volts: None,
    ...     read=lambda: Reading(volts=0.0, position=0.0, velocity=0.0),
    ...     max_travel_m=5.0,
    ... )
    >>> drive.max_travel, drive.settings.step_volts
    (5.0, 4.0)
"""

from collections.abc import Callable

import commands2

from utils.sysid.characterizable import Characterizable, Gravity, Reading
from utils.sysid.settings import SysIdSettings

DRIVETRAIN_SYSID = SysIdSettings(ramp_volts_per_s=1.0, step_volts=4.0, timeout_s=5.0, dynamic_timeout_s=2.0)
"""Default drivetrain settings. The 4 V step keeps the wheels from slipping
(CTRE's swerve example uses 4 V too)."""


def drivetrain(
    name: str,
    subsystem: commands2.Subsystem,
    set_voltage: Callable[[float], None],
    read: Callable[[], Reading],
    max_travel_m: float,
    settings: SysIdSettings = DRIVETRAIN_SYSID,
) -> Characterizable:
    """Describe a drivetrain for SysId.

    Args:
        name: Short name for the dashboard and log, like ``"drive"``.
        subsystem: The drivetrain subsystem.
        set_voltage: Apply this many volts to every drive motor.
        read: Return a :class:`~utils.sysid.characterizable.Reading` in meters and m/s.
        max_travel_m: Each test stops after the robot moves this far, meters.
            Measure the clear carpet you have and leave a meter spare.
        settings: How hard to push it (:data:`DRIVETRAIN_SYSID` by default).

    Returns:
        The :class:`~utils.sysid.characterizable.Characterizable`.

    Raises:
        ValueError: If ``max_travel_m`` isn't more than 0.
    """
    if max_travel_m <= 0:
        raise ValueError(f"max_travel_m must be more than 0, got {max_travel_m}")
    return Characterizable(
        name,
        subsystem,
        set_voltage,
        read,
        angular=False,
        settings=settings,
        gravity=Gravity.NONE,
        max_travel=max_travel_m,
    )
