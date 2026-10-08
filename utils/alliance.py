"""
Which alliance we are on, the same way on RobotPy 2026 and 2027.

The driver station (or the FMS at an event) tells the robot whether it is
red or blue. Field-relative driving uses it: "forward" on the stick means
away from *our* alliance wall, which is +x on the field for blue and -x for
red (WPILib puts the field origin in the blue corner).

Example:
    >>> from utils.alliance import is_red_alliance
    >>> is_red_alliance()  # no driver station in this example, so blue
    False
"""

import wpilib

try:  # RobotPy 2026
    _RED = wpilib.DriverStation.Alliance.kRed
    _get_alliance = wpilib.DriverStation.getAlliance
except AttributeError:  # RobotPy 2027 moved it to MatchState
    _RED = wpilib.Alliance.RED  # pyright: ignore[reportAttributeAccessIssue]
    _get_alliance = wpilib.MatchState.getAlliance  # pyright: ignore[reportAttributeAccessIssue]


def is_red_alliance() -> bool:
    """Return ``True`` on the red alliance, ``False`` on blue or when the alliance isn't known yet.

    Before the driver station connects, the alliance is unknown. We treat
    that as blue, so the robot still drives sensibly on the practice field.
    """
    return _get_alliance() == _RED
