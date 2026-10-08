"""
Describe a mechanism so the SysId tool can test it.

A mechanism only needs to say three things:

- how to apply a voltage to it (``set_voltage``),
- how to read it back (``read``: applied volts, position and velocity),
- which subsystem it belongs to, so the tests take control of it.

Everything else (the four tests, logging, the dashboard chooser) is shared.

Units:
    A mechanism that moves in a line (a drive wheel, an elevator) reports
    meters and m/s. One that turns (a steer motor, a flywheel, an arm)
    reports radians and rad/s and sets ``angular=True``.

Example:
    >>> import commands2
    >>> from utils.sysid.characterizable import Characterizable, Reading
    >>> from utils.sysid.settings import SysIdSettings
    >>> volts_applied = []
    >>> flywheel = Characterizable(
    ...     name="flywheel",
    ...     subsystem=commands2.Subsystem(),
    ...     set_voltage=volts_applied.append,
    ...     read=lambda: Reading(volts=0.0, position=0.0, velocity=0.0),
    ...     angular=True,
    ...     settings=SysIdSettings(step_volts=4.0),
    ... )
    >>> flywheel.settings.step_volts
    4.0
"""

from collections.abc import Callable
from dataclasses import dataclass

import commands2

from utils.sysid.settings import SysIdSettings


@dataclass(frozen=True)
class Reading:
    """One loop's measurement of a mechanism.

    Attributes:
        volts: The voltage the motor actually applied, volts.
        position: Meters, or radians if the mechanism is angular.
        velocity: m/s, or rad/s if the mechanism is angular.
    """

    volts: float
    position: float
    velocity: float


@dataclass(frozen=True)
class Characterizable:
    """A mechanism the SysId tool can test.

    Attributes:
        name: Short name shown on the dashboard and in the log, like ``"drive"``.
        subsystem: The subsystem the motors belong to. The tests require it,
            so nothing else drives the motors while they run.
        set_voltage: Apply this many volts to the motors. Called every loop.
        read: Return this loop's :class:`Reading`.
        angular: ``True`` if position is in radians (it turns), ``False`` if
            in meters (it moves in a line).
        settings: The :class:`SysIdSettings` for this mechanism.
    """

    name: str
    subsystem: commands2.Subsystem
    set_voltage: Callable[[float], None]
    read: Callable[[], Reading]
    angular: bool = False
    settings: SysIdSettings = SysIdSettings()
