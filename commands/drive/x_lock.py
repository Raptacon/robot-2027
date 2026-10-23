"""
X-lock: point all four wheels at the robot's center so it is hard to push.

Bound to a button with ``whileTrue``: the wheels stay in an X while the
button is held, then the drivetrain's default command (teleop) takes over
again.

Example:
    Bound to a button in ``commands/drive/bindings.py`` (tests/drivetrain/test_teleop.py
    runs it on the simulated robot):

    .. code-block:: python

        factory.getButton("drivetrain.x_lock").whileTrue(XLock(drivetrain))
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
