"""
The roboRIO gyro: a Studica NavX on the roboRIO's MXP port.

This is the only drivetrain file that talks to the NavX library. On
SystemCore we use its built-in IMU instead
(:mod:`subsystem.drivetrain.io.gyro_io_onboard`).

The NavX counts clockwise as positive and in degrees; WPILib (and our
:class:`~subsystem.drivetrain.io.gyro_io.GyroInputs`) count
counterclockwise as positive and in radians, so the readings are flipped
and converted here.

Example:
    In ``robot.py`` on a roboRIO (needs the NavX, so not run here):

    .. code-block:: python

        from subsystem.drivetrain.io.gyro_io_navx import GyroIONavX

        gyro = GyroIONavX(CONFIG)
"""

import math

import navx

from config.robot_config import RobotConfig
from subsystem.drivetrain.io.gyro_io import GyroInputs, GyroIO


class GyroIONavX(GyroIO):
    """A :class:`~subsystem.drivetrain.io.gyro_io.GyroIO` for a NavX on the MXP SPI port.

    Args:
        config: The robot config. ``gyro_inverted`` flips the direction if
            the NavX is mounted upside down.

    Attributes:
        ahrs: The NavX library's ``AHRS`` object.
    """

    def __init__(self, config: RobotConfig) -> None:
        self.ahrs = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)
        # NavX is clockwise-positive; flip it unless the config says the
        # gyro is mounted so it already reads the other way.
        self._sign = 1.0 if config.gyro_inverted else -1.0

    def update_inputs(self, inputs: GyroInputs) -> None:
        inputs.connected = self.ahrs.isConnected()
        # getAngle() keeps counting past 360 degrees, so the yaw never jumps.
        inputs.yaw_rad = self._sign * math.radians(self.ahrs.getAngle())
        inputs.yaw_rate_rad_per_s = self._sign * math.radians(self.ahrs.getRate())
