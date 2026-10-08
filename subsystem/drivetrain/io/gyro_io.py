"""
The interface for the gyro, which measures which way the robot is facing.

Odometry (tracking where the robot is on the field) needs the robot's heading,
and field-relative driving needs it too. The gyro gives the heading as *yaw*:
the angle the robot has turned, counterclockwise-positive when seen from
above.

Which gyro?
    The team hasn't picked the final gyro yet. Candidates are SystemCore's
    built-in IMU and a navX3 on CAN. Because everything uses this interface,
    switching gyros means adding one implementation file and changing the
    robot config, not changing the drive code.

Example:
    >>> from subsystem.drivetrain.io.gyro_io import GyroInputs
    >>> from subsystem.drivetrain.io.gyro_io_sim import GyroIOSim
    >>> gyro = GyroIOSim()
    >>> gyro.step(yaw_rate_rad_per_s=1.0, dt_s=0.5)  # spin at 1 rad/s for half a second
    >>> inputs = GyroInputs()
    >>> gyro.update_inputs(inputs)
    >>> inputs.yaw_rad
    0.5
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass
class GyroInputs:
    """One loop's readings from the gyro.

    Attributes:
        connected: ``False`` if the gyro isn't answering. Odometry falls back
            to wheel-only rotation when this is ``False``.
        yaw_rad: Total angle turned since boot, in radians, counterclockwise
            positive. It keeps counting past a full turn (two full turns
            reads 4 x pi), so it never jumps.
        yaw_rate_rad_per_s: How fast the robot is turning, in radians per second.
    """

    connected: bool = False
    yaw_rad: float = 0.0
    yaw_rate_rad_per_s: float = 0.0


class GyroIO(ABC):
    """What the drive code can ask of a gyro.

    Implementations don't reset or offset the yaw: the drivetrain keeps track
    of "which way is forward for the driver" itself, so odometry and the gyro
    never disagree about a hidden offset.
    """

    @abstractmethod
    def update_inputs(self, inputs: GyroInputs) -> None:
        """Read the gyro and write the values into ``inputs``. Call once per loop."""
