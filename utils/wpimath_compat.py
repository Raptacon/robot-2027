# pyright: reportAttributeAccessIssue=false
# (pyright only sees one RobotPy version, so the other branch looks wrong to it)
"""
One place to import WPILib math classes that moved or were renamed in 2027.

Why does this exist?
    RobotPy 2027 moved everything in ``wpimath.geometry``, ``wpimath.kinematics``
    and ``wpimath.estimator`` up into ``wpimath``, and renamed a few classes
    (``SwerveModuleState`` became ``SwerveModuleVelocity``, ``ChassisSpeeds``
    became ``ChassisVelocities``). We run 2026 now and move to 2027 for
    SystemCore, so code imports these names from here instead, and only
    this file knows the difference.

    When the team is fully on 2027, the 2026 branch below can be deleted.

Example:
    >>> import math
    >>> from utils.wpimath_compat import Rotation2d, make_module_velocity, module_velocity_speed
    >>> state = make_module_velocity(1.5, math.pi / 2)
    >>> module_velocity_speed(state), round(state.angle.degrees())
    (1.5, 90)
"""

try:  # RobotPy 2026 and earlier
    from wpimath.estimator import SwerveDrive4PoseEstimator
    from wpimath.geometry import Pose2d, Rotation2d, Translation2d
    from wpimath.kinematics import ChassisSpeeds as ChassisVelocities
    from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry, SwerveModulePosition
    from wpimath.kinematics import SwerveModuleState as SwerveModuleVelocity

    WPILIB_2027 = False
except ImportError:  # RobotPy 2027 and later
    from wpimath import (
        ChassisVelocities,
        Pose2d,
        Rotation2d,
        SwerveDrive4Kinematics,
        SwerveDrive4Odometry,
        SwerveDrive4PoseEstimator,
        SwerveModulePosition,
        SwerveModuleVelocity,
        Translation2d,
    )

    WPILIB_2027 = True

__all__ = [
    "ChassisVelocities",
    "Pose2d",
    "Rotation2d",
    "SwerveDrive4Kinematics",
    "SwerveDrive4Odometry",
    "SwerveDrive4PoseEstimator",
    "SwerveModulePosition",
    "SwerveModuleVelocity",
    "Translation2d",
    "WPILIB_2027",
    "make_module_velocity",
    "module_velocity_speed",
]


def make_module_velocity(speed_mps: float, angle_rad: float) -> SwerveModuleVelocity:
    """Build a WPILib module velocity (``SwerveModuleState`` in 2026) for logging or PathPlanner."""
    return SwerveModuleVelocity(speed_mps, Rotation2d(angle_rad))


def module_velocity_speed(state: SwerveModuleVelocity) -> float:
    """Read the speed from a WPILib module velocity (the field is ``speed`` in 2026, ``velocity`` in 2027)."""
    return state.velocity if WPILIB_2027 else state.speed  # type: ignore[attr-defined]
