"""
SysId for an elevator or lift: it moves in a line, and gravity pulls it down.

Gravity pulls with the same force at every height, so the fit measures kG
along with kS, kV and kA. Run the tests both up and down ("all four tests"
does): kG and kS can only be told apart with both. The elevator must have
**limits** a little inside its top and bottom; every test stops when it
reaches one. Positions are meters and speeds m/s.

The default settings are gentle (2 V step, 6 V most). Raise them on the
dashboard once you've seen it move.

Example:
    >>> import commands2
    >>> from utils.sysid.characterizable import Gravity, Reading
    >>> from utils.sysid.mech.elevator import elevator
    >>> lift = elevator(
    ...     "lift",
    ...     subsystem=commands2.Subsystem(),
    ...     set_voltage=lambda volts: None,
    ...     read=lambda: Reading(volts=0.0, position=0.0, velocity=0.0),
    ...     min_m=0.05,
    ...     max_m=1.2,
    ... )
    >>> lift.gravity == Gravity.ELEVATOR, lift.max_position
    (True, 1.2)
"""

from collections.abc import Callable

import commands2

from utils.sysid.characterizable import Characterizable, Gravity, Reading
from utils.sysid.mech._checks import check_limits
from utils.sysid.settings import SysIdSettings

ELEVATOR_SYSID = SysIdSettings(
    ramp_volts_per_s=0.5, step_volts=2.0, timeout_s=4.0, dynamic_timeout_s=1.0, max_volts=6.0
)
"""Default elevator settings: gentle, since it runs into its own top and bottom."""


def elevator(
    name: str,
    subsystem: commands2.Subsystem,
    set_voltage: Callable[[float], None],
    read: Callable[[], Reading],
    min_m: float,
    max_m: float,
    settings: SysIdSettings = ELEVATOR_SYSID,
) -> Characterizable:
    """Describe an elevator for SysId.

    Args:
        name: Short name for the dashboard and log, like ``"lift"``.
        subsystem: The subsystem the motors belong to.
        set_voltage: Apply this many volts.
        read: Return a :class:`~utils.sysid.characterizable.Reading` in meters and m/s.
        min_m: Down tests stop here, meters. Required.
        max_m: Up tests stop here, meters. Required.
        settings: How hard to push it (:data:`ELEVATOR_SYSID` by default).

    Returns:
        The :class:`~utils.sysid.characterizable.Characterizable`.

    Raises:
        ValueError: If the limits are backwards.
    """
    check_limits(min_m, max_m, angular=False)
    return Characterizable(
        name,
        subsystem,
        set_voltage,
        read,
        angular=False,
        settings=settings,
        gravity=Gravity.ELEVATOR,
        min_position=min_m,
        max_position=max_m,
    )
