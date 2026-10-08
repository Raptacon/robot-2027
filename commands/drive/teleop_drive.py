"""
TeleopDrive: drive the swerve from the driver's sticks. This is the drivetrain's default command.

What the driver gets:
    - Field-relative driving: pushing the left stick forward always moves
      the robot away from the driver (away from our alliance wall), whichever
      way the robot faces. Works on both alliances.
    - Rotate with the right stick. Let go and :class:`~commands.drive.heading_lock.HeadingLock`
      holds the heading.
    - Heading snaps (D-pad): face away from / left of / toward / right of the
      driver. Call :meth:`TeleopDrive.snap_heading`.
    - Heading re-zero: point the robot away from the driver and call
      :meth:`TeleopDrive.reset_heading`.
    - Robot-relative while a button is held: forward on the stick means the
      robot's own front (handy for lining up with something on the robot).
    - Slow mode: :meth:`TeleopDrive.toggle_slow` scales speed and spin down.

The buttons are connected in :mod:`commands.drive.bindings`. This class only
needs functions that return the stick values, so tests can drive it
without a controller.

Example (in simulation, no controller):
    >>> from config.robots.swerve_test_bot import CONFIG
    >>> from commands.drive.teleop_drive import TeleopDrive
    >>> from subsystem.drivetrain.drivetrain_sim import DrivetrainSim
    >>> sim = DrivetrainSim(CONFIG)
    >>> teleop = TeleopDrive(
    ...     sim.drivetrain,
    ...     forward=lambda: 1.0,  # stick fully forward
    ...     left=lambda: 0.0,
    ...     rotate=lambda: 0.0,
    ...     is_red=lambda: False,
    ... )
    >>> teleop.initialize()
    >>> for _ in range(50):  # one second
    ...     sim.drivetrain.periodic()
    ...     teleop.execute()
    >>> sim.drivetrain.pose.X() > 1.5  # moving away from the blue wall
    True
"""

import math
from collections.abc import Callable
from dataclasses import dataclass

import commands2

from commands.drive.heading_lock import HeadingLock
from commands.drive.stick_shaping import VectorSlewLimiter, shape_stick
from subsystem.drivetrain.drivetrain import Drivetrain
from subsystem.drivetrain.swerve_math import ChassisSpeeds
from utils.alliance import is_red_alliance


@dataclass
class TeleopSettings:
    """Driver feel settings for :class:`TeleopDrive`. Change these to suit the driver.

    Attributes:
        deadband: Left stick length (0 to 1) that counts as "not pushed".
        exponent: Translate response curve. 1 = linear, 2 = squared (finer
            control at low speed).
        max_speed_fraction: Top translate speed as a fraction of the
            drivetrain's free speed (1.0 = as fast as the motors go).
        max_spin_fraction: Top spin rate as a fraction of the drivetrain's
            maximum (``RobotConfig.max_angular_speed_rad_per_s``).
        max_accel_mps2: How quickly the commanded velocity can change, m/s^2.
            Lower is smoother; too low feels sluggish.
        slow_scale: Speed and spin multiplier in slow mode.
        heading_kp: Heading lock strength, (rad/s) per radian of error.
        heading_max_spin_fraction: Heading lock's top spin rate, as a
            fraction of the drivetrain's maximum.

    Example:
        A gentler setup for a new driver:

        >>> from commands.drive.teleop_drive import TeleopSettings
        >>> settings = TeleopSettings(max_speed_fraction=0.6, slow_scale=0.25)
        >>> settings.max_speed_fraction
        0.6
    """

    deadband: float = 0.1
    exponent: float = 2.0
    max_speed_fraction: float = 1.0
    max_spin_fraction: float = 0.75
    max_accel_mps2: float = 8.0
    slow_scale: float = 0.35
    heading_kp: float = 5.0
    heading_max_spin_fraction: float = 0.5


class TeleopDrive(commands2.Command):
    """Drives the robot from the driver's sticks. See the module docs for what it does.

    Args:
        drivetrain: The drivetrain to drive.
        forward: Returns the translate stick's forward amount, -1 to 1
            (+1 = fully away from the driver).
        left: Returns the translate stick's left amount, -1 to 1 (+1 = fully left).
        rotate: Returns the rotate stick, -1 to 1 (+1 = spin counterclockwise),
            already through its deadband.
        robot_relative: Returns ``True`` while the driver wants robot-relative
            driving. Defaults to never.
        is_red: Returns ``True`` on the red alliance. Defaults to asking the
            driver station.
        settings: Driver feel settings.

    Attributes:
        slow: ``True`` while slow mode is on.
        heading_lock: The heading lock, for tests and the dashboard.
    """

    def __init__(
        self,
        drivetrain: Drivetrain,
        forward: Callable[[], float],
        left: Callable[[], float],
        rotate: Callable[[], float],
        robot_relative: Callable[[], bool] = lambda: False,
        is_red: Callable[[], bool] = is_red_alliance,
        settings: TeleopSettings | None = None,
    ) -> None:
        super().__init__()
        self.drivetrain = drivetrain
        self._forward = forward
        self._left = left
        self._rotate = rotate
        self._robot_relative = robot_relative
        self._is_red = is_red
        self.settings = settings or TeleopSettings()
        self.slow = False
        config = drivetrain.config
        self.heading_lock = HeadingLock(
            kp=self.settings.heading_kp,
            max_omega_rad_per_s=config.max_angular_speed_rad_per_s * self.settings.heading_max_spin_fraction,
        )
        self._slew = VectorSlewLimiter(self.settings.max_accel_mps2, drivetrain.loop_period_s)
        self.addRequirements(drivetrain)

    # -- Driver actions (bound to buttons in commands.drive.bindings) ----------

    def toggle_slow(self) -> None:
        """Turn slow mode on or off."""
        self.slow = not self.slow

    def snap_heading(self, driver_degrees: float) -> None:
        """Turn to face a direction measured from the driver's point of view.

        Args:
            driver_degrees: 0 = away from the driver, 90 = the driver's left,
                180 = toward the driver, 270 = the driver's right.
        """
        self.heading_lock.set_target(self._driver_to_field_rad(math.radians(driver_degrees)))

    def reset_heading(self) -> None:
        """Re-zero field-relative driving: "the robot is facing away from me now"."""
        self.drivetrain.reset_heading(self._driver_to_field_rad(0.0))
        self.heading_lock.clear()

    # -- Command ----------------------------------------------------------------

    def initialize(self) -> None:
        # Start from rest and capture a fresh heading (after X-lock, for example).
        self._slew.reset()
        self.heading_lock.clear()

    def execute(self) -> None:
        settings = self.settings
        config = self.drivetrain.config
        scale = settings.slow_scale if self.slow else 1.0

        forward, left = shape_stick(self._forward(), self._left(), settings.deadband, settings.exponent)
        max_speed = config.free_speed_mps * settings.max_speed_fraction * scale
        vx, vy = forward * max_speed, left * max_speed

        robot_relative = self._robot_relative()
        if not robot_relative and self._is_red():
            # The red driver faces -x on the field, so "away from me" is -x.
            vx, vy = -vx, -vy
        vx, vy = self._slew.calculate(vx, vy)

        spin = self._rotate() * config.max_angular_speed_rad_per_s * settings.max_spin_fraction * scale
        if robot_relative:
            self.heading_lock.clear()
            omega = spin
        else:
            omega = self.heading_lock.calculate(
                spin, self.drivetrain.heading_rad, self.drivetrain.measured_speeds.omega_rad_per_s
            )

        self.drivetrain.drive(ChassisSpeeds(vx, vy, omega), field_relative=not robot_relative)

    def end(self, interrupted: bool) -> None:
        self.drivetrain.stop()

    def _driver_to_field_rad(self, driver_rad: float) -> float:
        """Turn a heading seen from the driver's wall into a field heading."""
        return driver_rad + (math.pi if self._is_red() else 0.0)
