"""
Build the real drivetrain from the robot config: SPARK MAX modules, CANcoders and the gyro.

``robot.py`` calls :func:`build_drivetrain` on the robot. In simulation it
uses :class:`~subsystem.drivetrain.drivetrain_sim.DrivetrainSim` instead.

Which gyro?
    On SystemCore, the built-in IMU (:mod:`subsystem.drivetrain.io.gyro_io_onboard`).
    On a roboRIO, a NavX on the MXP port (:mod:`subsystem.drivetrain.io.gyro_io_navx`).
    The choice is made from the WPILib version, so there is nothing to set.

Example:
    In ``robot.py`` (needs the robot's CAN devices, so not run here):

    .. code-block:: python

        from subsystem.drivetrain.drivetrain_hardware import build_drivetrain

        self.drivetrain = build_drivetrain(self.robot_config)
"""

from config.robot_config import RobotConfig
from subsystem.drivetrain.drivetrain import Drivetrain
from subsystem.drivetrain.io.abs_encoder_cancoder import AbsoluteEncoderIOCANcoder
from subsystem.drivetrain.io.gyro_io import GyroIO
from subsystem.drivetrain.io.gyro_io_onboard import has_onboard_imu
from subsystem.drivetrain.io.module_io_spark import ModuleIOSpark
from subsystem.drivetrain.module import SwerveModule


def build_gyro(config: RobotConfig) -> GyroIO:
    """Make the gyro for this controller: SystemCore's IMU, or a NavX on a roboRIO.

    Args:
        config: The robot config.

    Returns:
        The gyro IO.
    """
    if has_onboard_imu():
        from subsystem.drivetrain.io.gyro_io_onboard import GyroIOOnboard

        return GyroIOOnboard(config)
    from subsystem.drivetrain.io.gyro_io_navx import GyroIONavX

    return GyroIONavX(config)


def build_drivetrain(config: RobotConfig, loop_period_s: float = 0.02, gyro: GyroIO | None = None) -> Drivetrain:
    """Build the real swerve drivetrain.

    Args:
        config: The robot config (CAN IDs, offsets, gearing, SPARK settings).
        loop_period_s: The robot loop period in seconds.
        gyro: The gyro to use, or ``None`` for this controller's usual one
            (see :func:`build_gyro`). Tests pass a simulated gyro.

    Returns:
        The drivetrain subsystem, with one module per corner.
    """
    modules = [
        SwerveModule(corner, config, ModuleIOSpark(config, corner), AbsoluteEncoderIOCANcoder(config, corner))
        for corner in config.corners
    ]
    return Drivetrain(config, modules, gyro or build_gyro(config), loop_period_s=loop_period_s)
