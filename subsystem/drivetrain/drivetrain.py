"""
The swerve drivetrain subsystem: four modules, a gyro, and the robot's position on the field.

What does it do each loop?
    :meth:`Drivetrain.periodic` (run automatically by the command scheduler)
    reads the gyro and all four modules, then updates *odometry*: where the
    robot is on the field, worked out from how far each wheel rolled and
    which way the robot faces. Everything is logged under ``/Drive/`` so
    AdvantageScope can draw the robot and its wheels.

How do I drive it?
    Commands call :meth:`Drivetrain.drive` with the speeds they want. It
    handles field-relative driving, discretizing, kinematics and
    desaturating (see :mod:`subsystem.drivetrain.swerve_math`), and sends
    each module its target.

Example (in simulation):
    >>> from config.robots.swerve_test_bot import CONFIG
    >>> from subsystem.drivetrain.drivetrain_sim import DrivetrainSim
    >>> from subsystem.drivetrain.swerve_math import ChassisSpeeds
    >>> sim = DrivetrainSim(CONFIG)
    >>> drive = sim.drivetrain
    >>> for _ in range(50):  # one second of 20 ms loops
    ...     drive.periodic()
    ...     drive.drive(ChassisSpeeds(vx_mps=1.0), field_relative=True)
    >>> 0.8 < drive.pose.X() < 1.0  # about 1 m forward (it had to speed up first)
    True
"""

from collections.abc import Sequence

import commands2
import ntcore
import wpilib

from config.robot_config import RobotConfig
from subsystem.drivetrain.io.gyro_io import GyroInputs, GyroIO
from subsystem.drivetrain.module import SwerveModule
from subsystem.drivetrain.swerve_math import (
    ChassisSpeeds,
    desaturate,
    discretize,
    field_to_robot,
    forward_kinematics,
    inverse_kinematics,
)
from utils.alerts import warning_alert
from utils.inputs_publisher import InputsPublisher
from utils.wpimath_compat import (
    ChassisVelocities,
    Pose2d,
    Rotation2d,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveDrive4PoseEstimator,
    SwerveModulePosition,
    SwerveModuleVelocity,
    Translation2d,
    make_module_velocity,
)


class Drivetrain(commands2.Subsystem):
    """The swerve drive. Build it with real IO (milestone M5) or with
    :class:`~subsystem.drivetrain.drivetrain_sim.DrivetrainSim`.

    Args:
        config: The robot config.
        modules: The four modules in FL, FR, BL, BR order.
        gyro: The gyro IO.
        loop_period_s: How long each loop is, used by :meth:`drive` to discretize.

    Attributes:
        gyro_inputs: This loop's gyro readings.
        commanded: The robot-relative speeds last sent to the modules.
    """

    def __init__(
        self,
        config: RobotConfig,
        modules: Sequence[SwerveModule],
        gyro: GyroIO,
        loop_period_s: float = 0.02,
    ) -> None:
        super().__init__()
        if len(modules) != 4:
            raise ValueError("A swerve drivetrain needs exactly 4 modules (FL, FR, BL, BR)")
        self.config = config
        self.modules = list(modules)
        self.gyro = gyro
        self.gyro_inputs = GyroInputs()
        self.loop_period_s = loop_period_s
        self.commanded = ChassisSpeeds()
        self._module_xy = [(c.x_m, c.y_m) for c in config.corners]

        # Heading used by odometry. It follows the gyro, and if the gyro drops
        # out it carries on from the wheels so the pose never jumps.
        self._heading_rad = 0.0
        self._gyro_offset_rad = 0.0
        self._gyro_was_connected = False

        self._kinematics = SwerveDrive4Kinematics(*[Translation2d(x, y) for x, y in self._module_xy])
        start = Rotation2d(0.0)
        positions = self._module_positions()
        self._estimator = SwerveDrive4PoseEstimator(self._kinematics, start, positions, Pose2d())
        self._odometry = SwerveDrive4Odometry(self._kinematics, start, positions, Pose2d())

        nt = ntcore.NetworkTableInstance.getDefault()
        self._gyro_log = InputsPublisher("/Drive/Gyro", GyroInputs)
        self._pose_pub = nt.getStructTopic("/Drive/Pose", Pose2d).publish()
        self._odometry_pose_pub = nt.getStructTopic("/Drive/OdometryPose", Pose2d).publish()
        self._measured_states_pub = nt.getStructArrayTopic("/Drive/MeasuredStates", SwerveModuleVelocity).publish()
        self._setpoint_states_pub = nt.getStructArrayTopic("/Drive/SetpointStates", SwerveModuleVelocity).publish()
        self._measured_speeds_pub = nt.getStructTopic("/Drive/MeasuredSpeeds", ChassisVelocities).publish()
        self._commanded_speeds_pub = nt.getStructTopic("/Drive/CommandedSpeeds", ChassisVelocities).publish()
        self._healthy_pub = nt.getBooleanTopic("/Drive/Health/allModulesHealthy").publish()
        self._gyro_alert = warning_alert("Gyro not responding: heading is coming from the wheels only")

    # -- Each loop ----------------------------------------------------------

    def periodic(self) -> None:
        """Read every sensor, update odometry, and log. Runs automatically each loop."""
        disabled = wpilib.RobotState.isDisabled()
        for module in self.modules:
            module.periodic(disabled)
        # Read the gyro after the modules. On the robot the order doesn't
        # matter; in simulation it lets the gyro see this loop's wheel motion.
        self.gyro.update_inputs(self.gyro_inputs)
        self._gyro_log.publish(self.gyro_inputs)

        self._update_heading()
        rotation = Rotation2d(self._heading_rad)
        positions = self._module_positions()
        self._estimator.update(rotation, positions)
        self._odometry.update(rotation, positions)
        self._log()

    def _update_heading(self) -> None:
        if self.gyro_inputs.connected:
            if not self._gyro_was_connected:
                # Gyro just came (back) online: line it up with where we are.
                self._gyro_offset_rad = self._heading_rad - self.gyro_inputs.yaw_rad
            self._heading_rad = self.gyro_inputs.yaw_rad + self._gyro_offset_rad
        else:
            # No gyro: use the spin rate the wheels report.
            self._heading_rad += self.measured_speeds.omega_rad_per_s * self.loop_period_s
        self._gyro_was_connected = self.gyro_inputs.connected
        self._gyro_alert.set(not self.gyro_inputs.connected)

    # -- Commands -----------------------------------------------------------

    def drive(self, speeds: ChassisSpeeds, field_relative: bool = False) -> None:
        """Drive the robot.

        Args:
            speeds: How the robot should move. vx is forward, vy is left,
                omega is counterclockwise spin, in m/s and rad/s.
            field_relative: ``True`` if vx and vy are relative to the field
                (vx = away from the blue alliance wall) instead of the robot.
                The driver-facing direction for red alliance is handled by
                the teleop command (milestone M4).
        """
        if field_relative:
            speeds = field_to_robot(speeds, self.heading_rad)
        self.commanded = speeds
        speeds = discretize(speeds, self.loop_period_s)
        targets = inverse_kinematics(speeds, self._module_xy)
        targets = desaturate(targets, self.config.free_speed_mps)
        for module, target in zip(self.modules, targets):
            module.set_target(target)

    def stop(self) -> None:
        """Stop all four modules."""
        self.commanded = ChassisSpeeds()
        for module in self.modules:
            module.stop()

    def reset_pose(self, pose: Pose2d) -> None:
        """Tell odometry where the robot is (for example, at the start of autonomous)."""
        self._heading_rad = pose.rotation().radians()
        self._gyro_offset_rad = self._heading_rad - self.gyro_inputs.yaw_rad
        rotation = Rotation2d(self._heading_rad)
        positions = self._module_positions()
        self._estimator.resetPosition(rotation, positions, pose)
        self._odometry.resetPosition(rotation, positions, pose)

    # -- Readings -----------------------------------------------------------

    @property
    def pose(self) -> Pose2d:
        """Where the robot is on the field (pose estimator: odometry plus vision from M7)."""
        return self._estimator.getEstimatedPosition()

    @property
    def odometry_pose(self) -> Pose2d:
        """Where wheels and gyro alone say the robot is, for comparing with :attr:`pose`."""
        return self._odometry.getPose()

    @property
    def heading_rad(self) -> float:
        """Which way the robot faces on the field, radians, counterclockwise from +x."""
        return self._heading_rad

    @property
    def measured_speeds(self) -> ChassisSpeeds:
        """Robot-relative speeds worked out from the wheels (forward kinematics)."""
        return forward_kinematics([m.measured for m in self.modules], self._module_xy)

    @property
    def all_modules_healthy(self) -> bool:
        """``True`` when every module is seeded and answering."""
        return all(m.healthy for m in self.modules)

    # -- Internals ------------------------------------------------------------

    def _module_positions(
        self,
    ) -> tuple[SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition]:
        fl, fr, bl, br = (SwerveModulePosition(m.distance_m, Rotation2d(m.angle_rad)) for m in self.modules)
        return (fl, fr, bl, br)

    def _log(self) -> None:
        self._pose_pub.set(self.pose)
        self._odometry_pose_pub.set(self.odometry_pose)
        self._measured_states_pub.set(
            [make_module_velocity(m.measured.speed_mps, m.measured.angle_rad) for m in self.modules]
        )
        self._setpoint_states_pub.set(
            [make_module_velocity(m.setpoint.speed_mps, m.setpoint.angle_rad) for m in self.modules]
        )
        measured = self.measured_speeds
        self._measured_speeds_pub.set(ChassisVelocities(measured.vx_mps, measured.vy_mps, measured.omega_rad_per_s))
        c = self.commanded
        self._commanded_speeds_pub.set(ChassisVelocities(c.vx_mps, c.vy_mps, c.omega_rad_per_s))
        self._healthy_pub.set(self.all_modules_healthy)
