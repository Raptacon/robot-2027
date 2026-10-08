"""
The math that turns "drive the robot like this" into "point and spin each wheel like this".

This is plain Python with no WPILib, so it is the same on every RobotPy
version and easy to test. Each function matches a section of the Student
Swerve Theory Manual (doc/swerve/theory-manual.md).

The words used here:
    Chassis speeds: how the whole robot should move, as forward speed
        (``vx``), leftward speed (``vy``) and spin rate (``omega``,
        counterclockwise positive). Robot-relative unless a function says
        field-relative.
    Module target: what one wheel should do: a speed in m/s and a direction
        in radians (0 = toward the robot's front, counterclockwise positive).

The usual order each loop:
    1. :func:`field_to_robot` if the driver is driving field-relative.
    2. :func:`discretize`, so the robot doesn't drift sideways while it spins.
    3. :func:`inverse_kinematics` to get the four module targets.
    4. :func:`desaturate`, so no wheel is asked to go faster than it can.
    5. In each module: :func:`optimize`, then :func:`cosine_scale`.

Example:
    Drive forward at 1 m/s on a robot with wheels 0.3 m from center:

    >>> from subsystem.drivetrain.swerve_math import ChassisSpeeds, inverse_kinematics
    >>> corners = [(0.3, 0.3), (0.3, -0.3), (-0.3, 0.3), (-0.3, -0.3)]
    >>> targets = inverse_kinematics(ChassisSpeeds(1.0, 0.0, 0.0), corners)
    >>> [(t.speed_mps, t.angle_rad) for t in targets]  # all wheels forward at 1 m/s
    [(1.0, 0.0), (1.0, 0.0), (1.0, 0.0), (1.0, 0.0)]
"""

import math
from collections.abc import Sequence
from dataclasses import dataclass

# Below this speed (m/s) a wheel is treated as stopped and keeps its angle.
STOPPED_SPEED_MPS = 1e-3


@dataclass(frozen=True)
class ChassisSpeeds:
    """How the whole robot should move.

    Attributes:
        vx_mps: Forward speed in meters per second.
        vy_mps: Leftward speed in meters per second.
        omega_rad_per_s: Spin rate in radians per second, counterclockwise positive.
    """

    vx_mps: float = 0.0
    vy_mps: float = 0.0
    omega_rad_per_s: float = 0.0


@dataclass(frozen=True)
class ModuleTarget:
    """What one module should do.

    Attributes:
        speed_mps: Wheel speed in meters per second. Negative means roll backward.
        angle_rad: Wheel direction in radians, 0 toward the robot's front,
            counterclockwise positive.
    """

    speed_mps: float = 0.0
    angle_rad: float = 0.0


@dataclass(frozen=True)
class Pose:
    """A position and heading on the field: x and y in meters, heading in radians."""

    x_m: float = 0.0
    y_m: float = 0.0
    heading_rad: float = 0.0


def field_to_robot(speeds: ChassisSpeeds, robot_heading_rad: float) -> ChassisSpeeds:
    """Turn field-relative speeds into robot-relative speeds.

    With field-relative driving, pushing the stick "away from the driver"
    always moves the robot that way on the field, whichever way the robot
    faces. The wheels only understand robot-relative directions, so the
    speed arrow is rotated backward by the robot's heading.

    Args:
        speeds: Field-relative speeds (vx toward the far wall, vy to the left).
        robot_heading_rad: Which way the robot faces on the field.

    Example:
        Robot facing left (90 degrees), driver pushes "forward" on the field.
        To the robot, that is to its right:

        >>> import math
        >>> from subsystem.drivetrain.swerve_math import ChassisSpeeds, field_to_robot
        >>> s = field_to_robot(ChassisSpeeds(1.0, 0.0, 0.0), math.pi / 2)
        >>> round(s.vx_mps, 6), round(s.vy_mps, 6)
        (0.0, -1.0)
    """
    cos_h = math.cos(robot_heading_rad)
    sin_h = math.sin(robot_heading_rad)
    return ChassisSpeeds(
        vx_mps=speeds.vx_mps * cos_h + speeds.vy_mps * sin_h,
        vy_mps=-speeds.vx_mps * sin_h + speeds.vy_mps * cos_h,
        omega_rad_per_s=speeds.omega_rad_per_s,
    )


def discretize(speeds: ChassisSpeeds, dt_s: float) -> ChassisSpeeds:
    """Correct speeds so driving and spinning at the same time goes straight.

    The robot holds each command for one loop (``dt_s``). If it drives
    forward while spinning, "forward" keeps turning during that loop, so the
    robot curves off to the side. This works out the slightly different
    speeds that end the loop exactly where the original speeds meant to go.
    Theory manual section 4.

    Args:
        speeds: The robot-relative speeds we want.
        dt_s: The loop period in seconds (0.02 for our 50 Hz loop).

    Example:
        Without spinning, nothing changes:

        >>> from subsystem.drivetrain.swerve_math import ChassisSpeeds, discretize
        >>> discretize(ChassisSpeeds(1.0, 0.0, 0.0), 0.02)
        ChassisSpeeds(vx_mps=1.0, vy_mps=0.0, omega_rad_per_s=0.0)
    """
    # The pose we want to reach after one loop, relative to where we start...
    dx = speeds.vx_mps * dt_s
    dy = speeds.vy_mps * dt_s
    dtheta = speeds.omega_rad_per_s * dt_s
    # ...and the constant-speed "twist" that reaches it (the pose log).
    half = dtheta / 2.0
    cos_minus_one = math.cos(dtheta) - 1.0
    if abs(cos_minus_one) < 1e-9:
        half_over_tan = 1.0 - dtheta * dtheta / 12.0
    else:
        half_over_tan = -(half * math.sin(dtheta)) / cos_minus_one
    twist_x = dx * half_over_tan + dy * half
    twist_y = -dx * half + dy * half_over_tan
    return ChassisSpeeds(twist_x / dt_s, twist_y / dt_s, speeds.omega_rad_per_s)


def inverse_kinematics(speeds: ChassisSpeeds, module_xy_m: Sequence[tuple[float, float]]) -> list[ModuleTarget]:
    """Work out each wheel's speed and direction from the robot's speeds.

    Each wheel moves with the robot plus a sideways push from the spin: a
    wheel at (x, y) gets an extra (-omega * y, omega * x). Theory manual
    section 3.

    Args:
        speeds: Robot-relative speeds.
        module_xy_m: Each module's (x, y) position from the robot's center,
            in the usual FL, FR, BL, BR order.

    Returns:
        One target per module, in the same order. A wheel that should not move
        gets speed 0 and angle 0; the module code keeps its current angle then.

    Example:
        Spinning in place: every wheel points along the circle around the center.

        >>> import math
        >>> from subsystem.drivetrain.swerve_math import ChassisSpeeds, inverse_kinematics
        >>> fl = inverse_kinematics(ChassisSpeeds(0.0, 0.0, 1.0), [(0.3, 0.3)])[0]
        >>> round(fl.speed_mps, 3), round(math.degrees(fl.angle_rad))
        (0.424, 135)
    """
    targets = []
    for x, y in module_xy_m:
        wheel_vx = speeds.vx_mps - speeds.omega_rad_per_s * y
        wheel_vy = speeds.vy_mps + speeds.omega_rad_per_s * x
        speed = math.hypot(wheel_vx, wheel_vy)
        angle = math.atan2(wheel_vy, wheel_vx) if speed > 1e-9 else 0.0
        targets.append(ModuleTarget(speed, angle))
    return targets


def forward_kinematics(targets: Sequence[ModuleTarget], module_xy_m: Sequence[tuple[float, float]]) -> ChassisSpeeds:
    """Work out the robot's speeds from what the wheels are doing.

    The opposite of :func:`inverse_kinematics`. With four wheels there are
    more measurements than unknowns, so this finds the robot speeds that fit
    all four wheels best (least squares). If the wheels disagree, for example
    because one is slipping, the answer is a compromise.

    Args:
        targets: Each wheel's measured speed and direction, FL, FR, BL, BR.
        module_xy_m: Each module's (x, y) position, same order.

    Example:
        >>> from subsystem.drivetrain.swerve_math import (
        ...     ChassisSpeeds, forward_kinematics, inverse_kinematics)
        >>> corners = [(0.3, 0.3), (0.3, -0.3), (-0.3, 0.3), (-0.3, -0.3)]
        >>> wheels = inverse_kinematics(ChassisSpeeds(1.0, 0.5, 2.0), corners)
        >>> s = forward_kinematics(wheels, corners)
        >>> round(s.vx_mps, 6), round(s.vy_mps, 6), round(s.omega_rad_per_s, 6)
        (1.0, 0.5, 2.0)
    """
    # Each wheel gives two equations: wheel_vx = vx - omega*y, wheel_vy = vy + omega*x.
    # Solve the 3x3 "normal equations" of that least-squares problem.
    n = len(module_xy_m)
    sum_x = sum(x for x, _ in module_xy_m)
    sum_y = sum(y for _, y in module_xy_m)
    sum_r2 = sum(x * x + y * y for x, y in module_xy_m)
    b_vx = b_vy = b_w = 0.0
    for t, (x, y) in zip(targets, module_xy_m):
        wvx = t.speed_mps * math.cos(t.angle_rad)
        wvy = t.speed_mps * math.sin(t.angle_rad)
        b_vx += wvx
        b_vy += wvy
        b_w += -y * wvx + x * wvy
    a = [[n, 0.0, -sum_y], [0.0, n, sum_x], [-sum_y, sum_x, sum_r2]]
    vx, vy, omega = _solve3(a, [b_vx, b_vy, b_w])
    return ChassisSpeeds(vx, vy, omega)


def desaturate(targets: Sequence[ModuleTarget], max_speed_mps: float) -> list[ModuleTarget]:
    """Slow every wheel down by the same amount if any wheel is asked to go too fast.

    Scaling all four wheels together keeps the robot moving in the right
    direction, just slower. Clipping only the fast wheel would bend the path.
    Theory manual section 3.

    Example:
        >>> from subsystem.drivetrain.swerve_math import ModuleTarget, desaturate
        >>> out = desaturate([ModuleTarget(6.0, 0.0), ModuleTarget(3.0, 0.0)], 4.5)
        >>> [t.speed_mps for t in out]
        [4.5, 2.25]
    """
    fastest = max((abs(t.speed_mps) for t in targets), default=0.0)
    if fastest <= max_speed_mps:
        return list(targets)
    scale = max_speed_mps / fastest
    return [ModuleTarget(t.speed_mps * scale, t.angle_rad) for t in targets]


def optimize(target: ModuleTarget, current_angle_rad: float) -> ModuleTarget:
    """Never turn a wheel more than 90 degrees: spin it backward instead.

    Pointing a wheel at 180 degrees and rolling forward is the same as
    leaving it at 0 degrees and rolling backward, and much quicker.

    Args:
        target: Where we want the wheel to point and how fast.
        current_angle_rad: Where the wheel points now (the steer motor's
            encoder, the same angle the steer controller uses).

    Example:
        >>> import math
        >>> from subsystem.drivetrain.swerve_math import ModuleTarget, optimize
        >>> t = optimize(ModuleTarget(1.0, math.radians(170)), current_angle_rad=0.0)
        >>> t.speed_mps, round(math.degrees(t.angle_rad))
        (-1.0, -10)
    """
    delta = math.remainder(target.angle_rad - current_angle_rad, math.tau)
    if abs(delta) > math.pi / 2:
        return ModuleTarget(-target.speed_mps, math.remainder(target.angle_rad + math.pi, math.tau))
    return target


def cosine_scale(target: ModuleTarget, current_angle_rad: float) -> ModuleTarget:
    """Slow the wheel while it is still turning toward its target direction.

    A wheel pointing 60 degrees away from where it should only pushes the
    robot the right way at cos(60) = half speed, and pushes it sideways the
    rest. Scaling the speed by that cosine cuts the sideways push.

    Example:
        >>> import math
        >>> from subsystem.drivetrain.swerve_math import ModuleTarget, cosine_scale
        >>> round(cosine_scale(ModuleTarget(2.0, 0.0), math.radians(60)).speed_mps, 6)
        1.0
    """
    return ModuleTarget(target.speed_mps * math.cos(target.angle_rad - current_angle_rad), target.angle_rad)


def pose_exp(pose: Pose, speeds: ChassisSpeeds, dt_s: float) -> Pose:
    """Move a pose by robot-relative speeds held for ``dt_s`` seconds, following the true arc.

    Used by the simulator's ground truth. (Odometry on the robot uses
    WPILib's pose estimator, which does the same math.)

    Example:
        Drive forward 1 m/s for 1 s while facing left: the robot moves +y.

        >>> import math
        >>> from subsystem.drivetrain.swerve_math import ChassisSpeeds, Pose, pose_exp
        >>> p = pose_exp(Pose(0.0, 0.0, math.pi / 2), ChassisSpeeds(1.0, 0.0, 0.0), 1.0)
        >>> round(p.x_m, 6), round(p.y_m, 6)
        (0.0, 1.0)
    """
    dx = speeds.vx_mps * dt_s
    dy = speeds.vy_mps * dt_s
    dtheta = speeds.omega_rad_per_s * dt_s
    if abs(dtheta) < 1e-9:
        s, c = 1.0 - dtheta * dtheta / 6.0, 0.5 * dtheta
    else:
        s, c = math.sin(dtheta) / dtheta, (1.0 - math.cos(dtheta)) / dtheta
    local_x = dx * s - dy * c
    local_y = dx * c + dy * s
    cos_h, sin_h = math.cos(pose.heading_rad), math.sin(pose.heading_rad)
    return Pose(
        pose.x_m + local_x * cos_h - local_y * sin_h,
        pose.y_m + local_x * sin_h + local_y * cos_h,
        pose.heading_rad + dtheta,
    )


def _solve3(a: list[list[float]], b: list[float]) -> tuple[float, float, float]:
    """Solve a 3x3 linear system with Cramer's rule."""

    def det(m: list[list[float]]) -> float:
        return (
            m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
        )

    d = det(a)
    out = []
    for col in range(3):
        m = [row[:] for row in a]
        for r in range(3):
            m[r][col] = b[r]
        out.append(det(m) / d)
    return out[0], out[1], out[2]
