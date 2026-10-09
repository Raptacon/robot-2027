"""
How hard and how long the SysId tests push a mechanism.

This file has no WPILib imports, so robot config files can use
:class:`SysIdSettings` and still load on any RobotPy version.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class SysIdSettings:
    """How hard and how long the SysId tests push a mechanism.

    Start gentle. A quasistatic test reaches ``ramp_volts_per_s x timeout_s``
    volts by the end, and a dynamic test jumps straight to ``step_volts``.

    Attributes:
        ramp_volts_per_s: How fast the quasistatic test raises the voltage,
            volts per second. 1 V/s is WPILib's default.
        step_volts: The voltage the dynamic test jumps to, volts.
        timeout_s: The quasistatic tests stop after this many seconds, even
            if nobody disables the robot.
        dynamic_timeout_s: The dynamic tests stop after this many seconds.
            They reach full speed quickly, so they need less time (and,
            for a drivetrain, less room).
        max_volts: Safety limit: the tool never applies more than this, volts.
        settle_s: Seconds to hold 0 V before each test, so the mechanism
            starts still (and, for the drive, the wheels finish pointing
            straight).

    Example:
        >>> from utils.sysid.settings import SysIdSettings
        >>> s = SysIdSettings(ramp_volts_per_s=1.0, timeout_s=6.0)
        >>> s.ramp_volts_per_s * s.timeout_s  # volts at the end of the quasistatic test
        6.0
    """

    ramp_volts_per_s: float = 1.0
    step_volts: float = 7.0
    timeout_s: float = 10.0
    dynamic_timeout_s: float = 3.0
    max_volts: float = 12.0
    settle_s: float = 0.5
