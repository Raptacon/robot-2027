"""
X-lock: point all four wheels at the robot's center so it is hard to push.

Bound to a button with ``whileTrue``: the wheels stay in an X while the
button is held, then the drivetrain's default command (teleop) takes over
again.

Example:
    >>> from config.robots.swerve_test_bot import CONFIG
    >>> from commands.drive.x_lock import XLock
    >>> from subsystem.drivetrain.drivetrain_sim import DrivetrainSim
    >>> sim = DrivetrainSim(CONFIG)
    >>> lock = XLock(sim.drivetrain)
    >>> for _ in range(25):
    ...     sim.drivetrain.periodic()
    ...     lock.execute()
    >>> import math
    >>> front_left = CONFIG.corners[0]
    >>> toward_center = math.atan2(front_left.y_m, front_left.x_m)  # 47 degrees on this robot
    >>> abs(sim.drivetrain.modules[0].angle_rad - toward_center) < math.radians(1)
    True
"""

import commands2

from subsystem.drivetrain.drivetrain import Drivetrain


class XLock(commands2.Command):
    """Holds the wheels in an X until the command ends.

    Args:
        drivetrain: The drivetrain to lock.
    """

    def __init__(self, drivetrain: Drivetrain) -> None:
        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)

    def execute(self) -> None:
        self.drivetrain.lock_wheels_x()
