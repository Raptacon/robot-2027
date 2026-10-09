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
from enum import Enum

import commands2

from utils.sysid.settings import SysIdSettings


class Gravity(Enum):
    """What gravity does to a mechanism, so the fit can measure kG.

    - ``NONE``: gravity doesn't push it along its travel (a drivetrain, a
      flywheel, a turret, swerve steering).
    - ``ELEVATOR``: gravity pulls with the same force everywhere (an
      elevator, a linear lift). Run tests both up and down.
    - ``ARM``: gravity pulls hardest when the arm sticks straight out. The
      position must be in radians with 0 meaning level (horizontal), so set
      the arm's encoder offset before characterizing it.
    """

    NONE = "none"
    ELEVATOR = "elevator"
    ARM = "arm"


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
        gravity: What gravity does to it (:class:`Gravity`). Swerve drive and
            steer are ``Gravity.NONE``.
        min_position: Reverse tests stop when the position gets this low
            (meters or radians). Set it a little inside the mechanism's hard
            stop. ``None`` for something that can turn forever, like a wheel.
        max_position: Forward tests stop when the position gets this high.
        max_travel: Every test stops once the mechanism has moved this far
            from where the test started (meters or radians), whichever way
            it went. Used for a drivetrain, which has no fixed position but
            only so much room. ``None`` for no travel limit.
    """

    name: str
    subsystem: commands2.Subsystem
    set_voltage: Callable[[float], None]
    read: Callable[[], Reading]
    angular: bool = False
    settings: SysIdSettings = SysIdSettings()
    gravity: Gravity = Gravity.NONE
    min_position: float | None = None
    max_position: float | None = None
    max_travel: float | None = None

    def past_limit(self, position: float, forward: bool) -> bool:
        """``True`` if a test moving forward (or in reverse) has reached its position limit.

        Args:
            position: The current position, meters or radians.
            forward: ``True`` for a forward (positive volts) test.

        Example:
            >>> import commands2
            >>> from utils.sysid.characterizable import Characterizable, Reading
            >>> arm = Characterizable(
            ...     name="arm",
            ...     subsystem=commands2.Subsystem(),
            ...     set_voltage=lambda volts: None,
            ...     read=lambda: Reading(0.0, 0.0, 0.0),
            ...     min_position=-0.2,
            ...     max_position=1.4,
            ... )
            >>> arm.past_limit(1.5, forward=True), arm.past_limit(1.5, forward=False)
            (True, False)
        """
        if forward:
            return self.max_position is not None and position >= self.max_position
        return self.min_position is not None and position <= self.min_position

    def travelled_too_far(self, position: float, start: float) -> bool:
        """``True`` if the mechanism has moved :attr:`max_travel` or more since ``start``.

        Args:
            position: The current position, meters or radians.
            start: The position when the test started.

        Example:
            >>> import commands2
            >>> from utils.sysid.characterizable import Characterizable, Reading
            >>> drive = Characterizable(
            ...     name="drive",
            ...     subsystem=commands2.Subsystem(),
            ...     set_voltage=lambda volts: None,
            ...     read=lambda: Reading(0.0, 0.0, 0.0),
            ...     max_travel=4.0,
            ... )
            >>> drive.travelled_too_far(10.0, start=7.0), drive.travelled_too_far(2.5, start=7.0)
            (False, True)
        """
        return self.max_travel is not None and abs(position - start) >= self.max_travel
