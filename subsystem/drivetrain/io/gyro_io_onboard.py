"""
The SystemCore gyro: the IMU built into SystemCore (RobotPy 2027 and later).

On a roboRIO there is no built-in IMU; we use a NavX there
(:mod:`subsystem.drivetrain.io.gyro_io_navx`).

The IMU's yaw goes from -pi to pi and jumps when it passes pi. Odometry
wants an angle that keeps counting (two full turns = 4 x pi), so this
class adds up the change each loop instead of passing the jump through.

Example:
    In ``robot.py`` on SystemCore (needs the hardware, so not run here):

    .. code-block:: python

        from subsystem.drivetrain.io.gyro_io_onboard import GyroIOOnboard

        gyro = GyroIOOnboard(CONFIG)
"""

import math

import wpilib

from config.robot_config import RobotConfig
from subsystem.drivetrain.io.gyro_io import GyroInputs, GyroIO


def has_onboard_imu() -> bool:
    """Return ``True`` if this WPILib has SystemCore's ``OnboardIMU`` (RobotPy 2027 and later)."""
    return hasattr(wpilib, "OnboardIMU")


class YawUnwrapper:
    """Turns a yaw that jumps at +/- pi into one that keeps counting.

    Example:
        >>> import math
        >>> from subsystem.drivetrain.io.gyro_io_onboard import YawUnwrapper
        >>> unwrap = YawUnwrapper()
        >>> round(math.degrees(unwrap.update(math.radians(170))))
        170
        >>> round(math.degrees(unwrap.update(math.radians(-170))))  # passed 180, kept going
        190
    """

    def __init__(self) -> None:
        self._last: float | None = None
        self._total = 0.0

    def update(self, wrapped_rad: float) -> float:
        """Take this loop's wrapped yaw (radians) and return the continuous yaw."""
        if self._last is None:
            self._total = wrapped_rad
        else:
            self._total += math.remainder(wrapped_rad - self._last, math.tau)
        self._last = wrapped_rad
        return self._total


class GyroIOOnboard(GyroIO):
    """A :class:`~subsystem.drivetrain.io.gyro_io.GyroIO` for SystemCore's built-in IMU.

    Args:
        config: The robot config. ``systemcore_imu_mount`` says how SystemCore
            is mounted; ``gyro_inverted`` flips the direction.

    Attributes:
        imu: WPILib's ``OnboardIMU``.
    """

    def __init__(self, config: RobotConfig) -> None:
        orientation = getattr(wpilib.OnboardIMU.MountOrientation, config.systemcore_imu_mount.upper())  # pyright: ignore[reportAttributeAccessIssue]
        self.imu = wpilib.OnboardIMU(orientation)  # pyright: ignore[reportAttributeAccessIssue]
        self._sign = -1.0 if config.gyro_inverted else 1.0
        self._unwrap = YawUnwrapper()

    def update_inputs(self, inputs: GyroInputs) -> None:
        # The IMU is part of SystemCore itself, so it is always there.
        inputs.connected = True
        inputs.yaw_rad = self._sign * self._unwrap.update(self.imu.getYaw())
        inputs.yaw_rate_rad_per_s = self._sign * self.imu.getGyroRateZ()
