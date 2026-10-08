"""
Module check: point every wheel at 0, 90 and 180 degrees in turn, for bring-up on blocks.

Run it in **test mode** with the robot on blocks. Every wheel should point:

1. straight forward (0 degrees),
2. to the robot's left (90 degrees),
3. straight backward (180 degrees),

holding each for :data:`HOLD_S` seconds, then starting again. A wheel that
points the wrong way has a bad offset, a reversed encoder (preset
``encoder_inverted``) or a reversed steer motor (preset ``steer_inverted``).
Lay a straightedge across both wheels on each side to check they are square.

The wheels only turn; they never roll. They point exactly where asked
(no "short way" turning), so a wheel 180 degrees off shows up. At 0 degrees
also check the bevel gears all face the same side
(doc/swerve/tuning-and-calibration.md section 3).

Example:
    >>> from config.robots.swerve_test_bot import CONFIG
    >>> from commands.drive.module_check import ModuleCheck
    >>> from subsystem.drivetrain.drivetrain_sim import DrivetrainSim
    >>> sim = DrivetrainSim(CONFIG)
    >>> check = ModuleCheck(sim.drivetrain)
    >>> check.initialize()
    >>> check.target_degrees(elapsed_s=4.0)  # second step
    90.0
"""

import math

import commands2
import wpilib

from subsystem.drivetrain.drivetrain import Drivetrain

STEPS_DEG = (0.0, 90.0, 180.0)
"""The wheel directions to check, in degrees (counterclockwise from forward)."""

HOLD_S = 3.0
"""How long to hold each direction, in seconds."""


class ModuleCheck(commands2.Command):
    """Points every wheel at each of :data:`STEPS_DEG`, :data:`HOLD_S` seconds each, over and over.

    Args:
        drivetrain: The drivetrain to check.
    """

    def __init__(self, drivetrain: Drivetrain) -> None:
        super().__init__()
        self.drivetrain = drivetrain
        self._timer = wpilib.Timer()
        self.addRequirements(drivetrain)

    def target_degrees(self, elapsed_s: float) -> float:
        """The direction (degrees) the wheels should point ``elapsed_s`` seconds after starting."""
        return STEPS_DEG[int(elapsed_s // HOLD_S) % len(STEPS_DEG)]

    def initialize(self) -> None:
        self._timer.restart()

    def execute(self) -> None:
        angle = math.radians(self.target_degrees(self._timer.get()))
        for module in self.drivetrain.modules:
            module.set_angle(angle, short_way=False)

    def end(self, interrupted: bool) -> None:
        self.drivetrain.stop()

    def runsWhenDisabled(self) -> bool:
        return False
