"""
Swerve module presets: the gear ratios and directions built into each module.

What is this file for?
    A swerve module is one wheel corner of the robot. It has two motors: a
    *drive* motor that spins the wheel and a *steer* motor that turns the wheel
    to point in a direction. Gears sit between each motor and the wheel, so
    the motor turns many times for each wheel turn. How many times depends on
    which module we bought (MK4i or MK4) and which gear set (L1 to L4) is
    inside it.

    A *preset* holds those built-in facts, so we never type a gear ratio by
    hand. Things that change from robot to robot (CAN IDs, encoder offsets,
    wheel size) live in ``config/robots/`` instead.

How do I use it?
    Pick a preset in the robot's config file:

    >>> from config.module_presets import MK4I_L2
    >>> MK4I_L2.drive_ratio  # drive motor turns per wheel turn
    6.75

    Or look one up by name:

    >>> from config.module_presets import PRESETS
    >>> PRESETS["MK4i_L3"].drive_ratio
    6.12

Where do the numbers come from?
    The SDS MK4i and MK4 product pages. See
    doc/swerve/research-and-review.md section 1 for the table and links.
"""

import math
from dataclasses import dataclass

NOMINAL_WHEEL_RADIUS_M = 0.0508
"""Radius of a new 4 inch wheel, in meters (2 in = 0.0508 m).

Tread wears down, so a real robot should use the radius measured in the
wheel radius test (milestone M6) instead of this number.
"""


@dataclass(frozen=True)
class MotorSpec:
    """The facts about a motor that the config needs.

    Attributes:
        name: A label for logs and dashboards, like ``"NEO"``.
        free_speed_rpm: How fast the motor spins with nothing attached, in
            rotations per minute. Found on the motor's spec sheet.

    Example:
        >>> from config.module_presets import NEO
        >>> NEO.free_speed_rpm
        5676.0
        >>> round(NEO.free_speed_rps, 1)  # rotations per second
        94.6
    """

    name: str
    free_speed_rpm: float

    @property
    def free_speed_rps(self) -> float:
        """Free speed in rotations per second (RPM divided by 60)."""
        return self.free_speed_rpm / 60.0


NEO = MotorSpec("NEO", 5676.0)
"""REV NEO brushless motor, the motor we use for both drive and steer."""


@dataclass(frozen=True)
class ModulePreset:
    """Gear ratios and motor directions fixed by one module design.

    ``frozen=True`` means a preset can't be changed after it is made. That
    protects us from a bug we had in robot-2026, where code changed a shared
    constants object and every module ended up with the same offset.

    Attributes:
        name: Preset name, like ``"MK4i_L2"``. Used to look it up in
            :data:`PRESETS`.
        drive_ratio: Drive motor rotations for one wheel rotation. A bigger
            number means a slower, stronger wheel.
        steer_ratio: Steer motor rotations for one full turn of the wheel's
            direction (360 degrees).
        steer_inverted: ``True`` if the steer motor has to be reversed so a
            positive command turns the wheel counterclockwise when seen from
            above. MK4i and MK4 steer in opposite directions.
        encoder_inverted: ``True`` if the absolute encoder counts the wrong
            way and has to be reversed. We confirm this on blocks at M5.

    Example:
        How far does the robot move for one turn of the drive motor?

        >>> from config.module_presets import MK4I_L2, NOMINAL_WHEEL_RADIUS_M
        >>> meters = MK4I_L2.drive_m_per_motor_rot(NOMINAL_WHEEL_RADIUS_M)
        >>> round(meters * 100, 2)  # in centimeters
        4.73
    """

    name: str
    drive_ratio: float
    steer_ratio: float
    steer_inverted: bool
    encoder_inverted: bool

    def drive_m_per_motor_rot(self, wheel_radius_m: float) -> float:
        """Distance the wheel rolls, in meters, for one drive motor rotation.

        One wheel turn rolls the wheel's circumference (2 x pi x radius), and
        one wheel turn takes ``drive_ratio`` motor turns, so one motor turn
        rolls circumference / drive_ratio.

        Args:
            wheel_radius_m: Wheel radius in meters. Use the robot's measured
                value.

        Returns:
            Meters of travel per motor rotation. The SparkMax uses this as its
            position conversion factor so its encoder reads in meters.
        """
        return 2.0 * math.pi * wheel_radius_m / self.drive_ratio

    @property
    def steer_rad_per_motor_rot(self) -> float:
        """How far the wheel's direction turns, in radians, for one steer motor rotation.

        A full turn of the wheel's direction is 2 x pi radians (360 degrees),
        and it takes ``steer_ratio`` motor turns.

        Example:
            >>> import math
            >>> from config.module_presets import MK4I_L2
            >>> round(math.degrees(MK4I_L2.steer_rad_per_motor_rot), 1)
            16.8
        """
        return 2.0 * math.pi / self.steer_ratio

    def free_speed_mps(self, motor: MotorSpec, wheel_radius_m: float) -> float:
        """Top wheel speed, in meters per second, if the motor ran at free speed.

        This is a theoretical number: friction, battery sag and robot weight
        mean the real top speed is usually 80 to 90 percent of it.

        Args:
            motor: The drive motor, like :data:`NEO`.
            wheel_radius_m: Wheel radius in meters.

        Example:
            >>> from config.module_presets import MK4I_L2, NEO, NOMINAL_WHEEL_RADIUS_M
            >>> round(MK4I_L2.free_speed_mps(NEO, NOMINAL_WHEEL_RADIUS_M), 2)
            4.47
        """
        return motor.free_speed_rps * self.drive_m_per_motor_rot(wheel_radius_m)


_MK4I_STEER = 150.0 / 7.0  # about 21.43 motor turns per wheel turn
_MK4_STEER = 12.8

# MK4i modules. The "L" number is the gear set: L1 is slowest, L3 fastest.
MK4I_L1 = ModulePreset("MK4i_L1", 8.14, _MK4I_STEER, steer_inverted=True, encoder_inverted=False)
MK4I_L2 = ModulePreset("MK4i_L2", 6.75, _MK4I_STEER, steer_inverted=True, encoder_inverted=False)
MK4I_L3 = ModulePreset("MK4i_L3", 6.12, _MK4I_STEER, steer_inverted=True, encoder_inverted=False)

# MK4 modules (not the "i" version). Same drive gears plus L4, different steering.
MK4_L1 = ModulePreset("MK4_L1", 8.14, _MK4_STEER, steer_inverted=False, encoder_inverted=False)
MK4_L2 = ModulePreset("MK4_L2", 6.75, _MK4_STEER, steer_inverted=False, encoder_inverted=False)
MK4_L3 = ModulePreset("MK4_L3", 6.12, _MK4_STEER, steer_inverted=False, encoder_inverted=False)
MK4_L4 = ModulePreset("MK4_L4", 5.14, _MK4_STEER, steer_inverted=False, encoder_inverted=False)

PRESETS: dict[str, ModulePreset] = {p.name: p for p in (MK4I_L1, MK4I_L2, MK4I_L3, MK4_L1, MK4_L2, MK4_L3, MK4_L4)}
"""Every preset, looked up by its name (for example ``PRESETS["MK4i_L2"]``)."""

DEFAULT_PRESET = MK4I_L2
"""The modules on our robots unless a robot config says otherwise."""
