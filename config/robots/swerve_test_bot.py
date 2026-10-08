"""
Swerve test bot: the robot-2026 chassis on SDS MK4i L2 modules.

This is the file to edit when something about this robot's drive changes.
Each value below says where it came from and how to re-measure it.

Common changes:
    Swapped the gears to L3?
        Change ``preset=MK4I_L2`` to ``preset=MK4I_L3`` (and import it).
    Recalibrated an encoder?
        Update its number in ``offsets_rot`` and add a line to
        doc/swerve/calibration-log.md in the same commit.
    Measured the wheel radius?
        Replace ``NOMINAL_WHEEL_RADIUS_M`` with the measured number, in
        meters, and log it in doc/swerve/calibration-log.md.

The config tests (tests/drivetrain/test_swerve_config.py) run on every pull
request and catch common mistakes like a repeated CAN ID or an offset out of
range. Run them with ``python -m robotpy test``.

Where the current values came from:
    Module positions, CAN IDs, offsets and inversions are copied from
    robot-2026 (constants/robot_geometry.py, config.py and
    constants/swerve_constants.py). The offsets still need checking with the
    calibration steps in doc/swerve/tuning-and-calibration.md at M5.
"""

from config.module_presets import MK4I_L2, NOMINAL_WHEEL_RADIUS_M
from config.robot_config import RobotConfig, mirrored_corners

# Meters per inch. Robot drawings are in inches; the code uses meters.
# (Config imports no WPILib on purpose, so it loads on any RobotPy version.)
INCH = 0.0254

CONFIG = RobotConfig(
    name="swerve_test_bot",
    # SDS MK4i modules with the L2 gear set.
    preset=MK4I_L2,
    corners=mirrored_corners(
        # Front-left wheel center: 10.39 in forward and 11.30 in left of the
        # robot's center. The other three corners are mirrored from it.
        x_m=10.39 * INCH,
        y_m=11.30 * INCH,
        # Front-left drive motor is CAN ID 50; IDs then go up by one per
        # device: FL 50-52, FR 53-55, BL 56-58, BR 59-61.
        first_can_id=50,
        # Encoder offsets in rotations (degrees / 360), in FL, FR, BL, BR order.
        # Values above 0.5 are wrapped, so 323.6 deg becomes -36.4 deg.
        offsets_rot=(
            10.283203125 / 360.0,
            323.6148 / 360.0,
            347.2737188 / 360.0,
            83.2865625 / 360.0,
        ),
        # All four drive motors are reversed on this robot (FL, FR, BL, BR).
        drive_inverted=(True, True, True, True),
    ),
    # A new 4 in wheel. Replace with the measured radius after the M6 test.
    wheel_radius_m=NOMINAL_WHEEL_RADIUS_M,
    # No CAN gyro picked yet, so there is no gyro CAN ID.
    gyro_can_id=None,
)
"""The swerve test bot's config. Registered in :mod:`config.loader` as ``"swerve_test_bot"``."""
