"""
Swerve test bot: the robot-2026 chassis on SDS MK4i L2 modules.

Positions, CAN IDs, offsets and inversions are carried over from robot-2026
(constants/robot_geometry.py, config.py, constants/swerve_constants.py).
Re-check the offsets with the calibration steps in
doc/swerve/tuning-and-calibration.md at M5, and log every change in
doc/swerve/calibration-log.md.
"""

from config.module_presets import MK4I_L2, NOMINAL_WHEEL_RADIUS_M
from config.robot_config import RobotConfig, mirrored_corners

# Plain Python on purpose: config imports no WPILib, so it loads on any RobotPy version.
INCH = 0.0254

CONFIG = RobotConfig(
    name="swerve_test_bot",
    preset=MK4I_L2,
    corners=mirrored_corners(
        x_m=10.39 * INCH,
        y_m=11.30 * INCH,
        first_can_id=50,
        # FL, FR, BL, BR in rotations; robot-2026 values (wrapped to [-0.5, 0.5))
        offsets_rot=(
            10.283203125 / 360.0,
            323.6148 / 360.0,
            347.2737188 / 360.0,
            83.2865625 / 360.0,
        ),
        drive_inverted=(True, True, True, True),
    ),
    # Nominal until the M6 wheel radius test on carpet.
    wheel_radius_m=NOMINAL_WHEEL_RADIUS_M,
    # NavX on SPI, so no CAN ID.
    gyro_can_id=None,
)
