"""
Heading lock: keep the robot facing one way while the driver isn't turning.

Without it, a robot driving across the field slowly turns (one wheel grips
a little better, a bump, a defender) and the driver has to keep correcting.
With it, when the rotate stick is let go the robot remembers which way it
faces and turns itself back if it gets pushed.

How it works:
    - Rotate stick pushed: the driver is in charge. The lock is cleared and
      the stick's spin rate is used as-is.
    - Stick just let go: wait until the robot has stopped turning, then
      remember that heading as the target. (Grabbing it while still spinning
      would make the robot swing back.)
    - Target set: turn toward it at ``kp`` rad/s for every radian of error,
      up to ``max_omega_rad_per_s``. Inside ``tolerance_rad`` it sends 0, so
      the wheels don't twitch.
    - :meth:`HeadingLock.set_target` jumps the target, which is how the
      D-pad heading snaps work.

Example:
    >>> import math
    >>> from commands.drive.heading_lock import HeadingLock
    >>> lock = HeadingLock(kp=4.0, max_omega_rad_per_s=6.0)
    >>> lock.set_target(math.radians(90))
    >>> round(lock.calculate(0.0, heading_rad=0.0, measured_omega_rad_per_s=0.0), 3)  # turn left
    6.0
    >>> lock.calculate(2.0, heading_rad=0.0, measured_omega_rad_per_s=0.0)  # driver turns: stick wins
    2.0
    >>> lock.target_rad is None
    True
"""

import math


class HeadingLock:
    """Turns a rotate-stick value into a spin rate, holding heading when the stick is released.

    Args:
        kp: Spin rate per radian of heading error, in (rad/s) per rad.
        max_omega_rad_per_s: The fastest it will turn to get back on target.
        tolerance_rad: Errors smaller than this count as on target.
        settle_rad_per_s: The robot counts as no longer turning below this
            spin rate, so the heading can be captured.

    Attributes:
        target_rad: The heading being held (field-relative, radians), or
            ``None`` while the driver is turning or the robot is settling.
    """

    def __init__(
        self,
        kp: float = 5.0,
        max_omega_rad_per_s: float = 6.0,
        tolerance_rad: float = math.radians(0.5),
        settle_rad_per_s: float = 0.3,
    ) -> None:
        self.kp = kp
        self.max_omega_rad_per_s = max_omega_rad_per_s
        self.tolerance_rad = tolerance_rad
        self.settle_rad_per_s = settle_rad_per_s
        self.target_rad: float | None = None

    def set_target(self, heading_rad: float) -> None:
        """Hold this field heading (radians) until the driver turns."""
        self.target_rad = heading_rad

    def clear(self) -> None:
        """Forget the target. A new one is captured once the robot stops turning."""
        self.target_rad = None

    def calculate(self, requested_omega_rad_per_s: float, heading_rad: float, measured_omega_rad_per_s: float) -> float:
        """Work out the spin rate to command this loop.

        Args:
            requested_omega_rad_per_s: The driver's rotate stick as a spin
                rate, already through its deadband (0 when released).
            heading_rad: The robot's heading now (from the drivetrain).
            measured_omega_rad_per_s: How fast the robot is turning now.

        Returns:
            The spin rate to send to the drivetrain, rad/s, counterclockwise positive.
        """
        if requested_omega_rad_per_s != 0.0:
            self.target_rad = None
            return requested_omega_rad_per_s
        if self.target_rad is None:
            if abs(measured_omega_rad_per_s) < self.settle_rad_per_s:
                self.target_rad = heading_rad
            return 0.0
        error = math.remainder(self.target_rad - heading_rad, math.tau)
        if abs(error) < self.tolerance_rad:
            return 0.0
        return max(-self.max_omega_rad_per_s, min(self.max_omega_rad_per_s, self.kp * error))
