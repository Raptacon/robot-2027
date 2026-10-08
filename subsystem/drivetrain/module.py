"""
One swerve module: turns a wheel target into motor commands, and checks its own health.

What does a module do each loop?
    1. :meth:`SwerveModule.periodic` reads the motors and the absolute
       encoder once, logs them, and handles *seeding* (below).
    2. The drivetrain calls :meth:`SwerveModule.set_target` with a speed and
       direction. The module optimizes it (never turn more than 90 degrees),
       slows it while the wheel is still turning (cosine scaling), works out
       the feedforward voltage, and sends it to the motor controllers.

Seeding
    The steer motor's encoder reads 0 at power-on wherever the wheel points.
    The absolute encoder knows the real direction, so on the first loop it
    answers, the module copies its angle into the steer encoder. After that
    only the steer encoder is used for steering and odometry (one angle
    source). While the robot is disabled and the wheel is still, the module
    re-seeds if the two disagree by more than 2 degrees.

    If the absolute encoder never answers, the module can't know where its
    wheel points. After :data:`SEED_TIMEOUT_LOOPS` it sets
    ``encoder_failed``, shows a dashboard alert, and refuses to drive that
    wheel, rather than driving it pointed the wrong way.

Example:
    >>> from config.robots.swerve_test_bot import CONFIG
    >>> from subsystem.drivetrain.io.abs_encoder_sim import AbsoluteEncoderIOSim
    >>> from subsystem.drivetrain.io.module_io_sim import ModuleIOSim, ModuleSimOptions
    >>> from subsystem.drivetrain.module import SwerveModule
    >>> from subsystem.drivetrain.swerve_math import ModuleTarget
    >>> corner = CONFIG.corners[0]
    >>> io = ModuleIOSim(CONFIG, ModuleSimOptions(initial_steer_angle_rad=0.7))
    >>> module = SwerveModule(corner, CONFIG, io, AbsoluteEncoderIOSim.for_module(io, corner))
    >>> module.periodic(disabled=True)  # first loop: seeds from the absolute encoder
    >>> module.seeded, round(module.angle_rad, 3)
    (True, 0.7)
    >>> module.set_target(ModuleTarget(speed_mps=1.0, angle_rad=0.0))
"""

import math

from config.robot_config import CornerConfig, RobotConfig
from subsystem.drivetrain.io.abs_encoder_io import AbsoluteEncoderInputs, AbsoluteEncoderIO
from subsystem.drivetrain.io.module_io import ModuleInputs, ModuleIO
from subsystem.drivetrain.swerve_math import STOPPED_SPEED_MPS, ModuleTarget, cosine_scale, optimize
from utils.alerts import error_alert
from utils.inputs_publisher import InputsPublisher

SEED_TIMEOUT_LOOPS = 50
"""Loops (1 second at 50 Hz) to wait for the absolute encoder before flagging it as failed."""

RESEED_TOLERANCE_RAD = math.radians(2.0)
"""Re-seed while disabled if the two angle sources disagree by more than this."""

STILL_RAD_PER_S = 0.05
"""The wheel counts as still (safe to re-seed) when it turns slower than this."""


class SwerveModule:
    """The logic for one module, on top of its :class:`~subsystem.drivetrain.io.module_io.ModuleIO`.

    Args:
        corner: This module's corner config (name, position, offset).
        config: The whole robot config (gearing, feedforward).
        io: The module's motors (real or simulated).
        encoder: The module's absolute encoder (real or simulated).
        log_prefix: NetworkTables folder for this module's logged inputs.

    Attributes:
        inputs: This loop's motor readings. Read-only for other code.
        encoder_inputs: This loop's absolute encoder readings.
        seeded: ``True`` once the steer encoder has been set from the absolute encoder.
        encoder_failed: ``True`` if the absolute encoder never answered.
        reseed_count: How many times the module re-seeded while disabled. If
            this keeps going up, check the encoder magnet and wiring.
        angle_disagreement_rad: Absolute encoder angle minus steer encoder
            angle, last time both were valid.
        setpoint: The target last sent to the motors, after optimizing.
    """

    def __init__(
        self,
        corner: CornerConfig,
        config: RobotConfig,
        io: ModuleIO,
        encoder: AbsoluteEncoderIO,
        log_prefix: str = "/Drive/Module",
    ) -> None:
        self.corner = corner
        self.config = config
        self.io = io
        self.encoder = encoder
        self.inputs = ModuleInputs()
        self.encoder_inputs = AbsoluteEncoderInputs()
        self.seeded = False
        self.encoder_failed = False
        self.reseed_count = 0
        self.angle_disagreement_rad = 0.0
        self.setpoint = ModuleTarget()
        self._unseeded_loops = 0
        self._inputs_log = InputsPublisher(f"{log_prefix}/{corner.name}", ModuleInputs)
        self._encoder_log = InputsPublisher(f"{log_prefix}/{corner.name}/absEncoder", AbsoluteEncoderInputs)
        self._encoder_alert = error_alert(f"Swerve {corner.name}: absolute encoder not responding, wheel disabled")

    # -- Each loop ----------------------------------------------------------

    def periodic(self, disabled: bool) -> None:
        """Read sensors, log them, and seed the steer encoder if needed. Call once per loop.

        Args:
            disabled: ``True`` while the robot is disabled. Re-seeding only
                happens then, so the wheel never jumps while driving.
        """
        self.io.update_inputs(self.inputs)
        self.encoder.update_inputs(self.encoder_inputs)
        self._inputs_log.publish(self.inputs)
        self._encoder_log.publish(self.encoder_inputs)

        if not self.seeded:
            self._try_first_seed()
        elif self.encoder_inputs.connected:
            self.angle_disagreement_rad = math.remainder(
                self.encoder_inputs.angle_rad - self.inputs.steer_angle_rad, math.tau
            )
            still = abs(self.inputs.steer_velocity_rad_per_s) < STILL_RAD_PER_S
            if disabled and still and abs(self.angle_disagreement_rad) > RESEED_TOLERANCE_RAD:
                self._seed()
                self.reseed_count += 1

    def _try_first_seed(self) -> None:
        if self.encoder_inputs.connected:
            self._seed()
            self.encoder_failed = False
            self._encoder_alert.set(False)
            return
        self._unseeded_loops += 1
        if self._unseeded_loops >= SEED_TIMEOUT_LOOPS and not self.encoder_failed:
            self.encoder_failed = True
            self._encoder_alert.set(True)

    def _seed(self) -> None:
        self.io.reset_steer_angle(self.encoder_inputs.angle_rad)
        # The readings from this loop were taken before the reset, so fix the
        # angle here too; next loop the steer encoder reports it itself.
        self.inputs.steer_angle_rad = self.encoder_inputs.angle_rad
        self.angle_disagreement_rad = 0.0
        self.seeded = True

    # -- Commands -----------------------------------------------------------

    def set_target(self, target: ModuleTarget) -> None:
        """Drive the wheel at a speed and direction.

        A wheel that isn't seeded yet (or whose encoder failed) is stopped
        instead, because we don't know which way it points.

        When the target speed is about 0, the wheel keeps pointing where it
        was instead of snapping back to 0 degrees.
        """
        if not self.seeded:
            self.setpoint = ModuleTarget()
            self.io.stop()
            return
        current = self.inputs.steer_angle_rad
        if abs(target.speed_mps) < STOPPED_SPEED_MPS:
            target = ModuleTarget(0.0, self.setpoint.angle_rad)
        target = optimize(target, current)
        target = cosine_scale(target, current)
        self.setpoint = target
        self.io.set_steer_angle(target.angle_rad)
        self.io.set_drive_velocity(target.speed_mps, self.config.drive_feedforward_volts(target.speed_mps))

    def set_angle(self, angle_rad: float) -> None:
        """Point the wheel at ``angle_rad`` without rolling it (used by X-lock).

        :meth:`set_target` keeps the old angle when the speed is 0, so this is
        the way to turn a wheel that isn't driving. It still takes the short
        way round (the wheel may end up pointing the opposite way, which is
        the same line).

        Args:
            angle_rad: Direction to point, radians, 0 toward the robot's front.
        """
        if not self.seeded:
            self.setpoint = ModuleTarget()
            self.io.stop()
            return
        target = optimize(ModuleTarget(0.0, angle_rad), self.inputs.steer_angle_rad)
        self.setpoint = ModuleTarget(0.0, target.angle_rad)
        self.io.set_steer_angle(target.angle_rad)
        self.io.set_drive_velocity(0.0, self.config.drive_feedforward_volts(0.0))

    def stop(self) -> None:
        """Stop both motors."""
        self.setpoint = ModuleTarget(0.0, self.setpoint.angle_rad)
        self.io.stop()

    # -- Readings for the drivetrain ------------------------------------------

    @property
    def angle_rad(self) -> float:
        """Which way the wheel points (from the steer encoder), radians."""
        return self.inputs.steer_angle_rad

    @property
    def measured(self) -> ModuleTarget:
        """The wheel's measured speed and direction, for logging and forward kinematics."""
        return ModuleTarget(self.inputs.drive_velocity_mps, self.inputs.steer_angle_rad)

    @property
    def distance_m(self) -> float:
        """Total distance the wheel has rolled, meters (for odometry)."""
        return self.inputs.drive_position_m

    @property
    def healthy(self) -> bool:
        """``True`` when the module is seeded and both motor controllers answer."""
        return self.seeded and self.inputs.drive_connected and self.inputs.steer_connected
