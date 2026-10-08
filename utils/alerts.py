"""
Dashboard alerts that work on RobotPy 2026 and 2027.

WPILib's ``Alert`` shows a message on the dashboard (Elastic and
AdvantageScope both show them) while ``set(True)``. The urgency names
changed in 2027 (``kError`` became ``HIGH``), so make alerts with
:func:`error_alert` and :func:`warning_alert` instead of calling ``Alert``
directly.

Example:
    >>> from utils.alerts import error_alert
    >>> alert = error_alert("Front-left absolute encoder not responding")
    >>> alert.set(True)   # shows on the dashboard
    >>> alert.set(False)  # hides it again
"""

import wpilib

_levels = getattr(wpilib.Alert, "Level", None)
if _levels is not None:  # RobotPy 2027
    _ERROR, _WARNING = _levels.HIGH, _levels.MEDIUM
else:  # RobotPy 2026
    _ERROR, _WARNING = wpilib.Alert.AlertType.kError, wpilib.Alert.AlertType.kWarning


def error_alert(text: str) -> wpilib.Alert:
    """An alert for something that stops part of the robot from working."""
    return wpilib.Alert(text, _ERROR)


def warning_alert(text: str) -> wpilib.Alert:
    """An alert for something that still works but needs a look."""
    return wpilib.Alert(text, _WARNING)
