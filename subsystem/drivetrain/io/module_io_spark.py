"""
The real module IO: two SPARK MAX controllers with NEO motors.

This is the only drivetrain file that talks to REV's library. Everything
else uses :class:`~subsystem.drivetrain.io.module_io.ModuleIO`, so the
module and drivetrain code is the same on the robot and in the simulator.

What happens at boot:
    Each controller is reset to its factory defaults and then sent every
    setting we use (brake mode, current limit, 12 V compensation, encoder
    units, PID gains, how often to report), without writing to its flash.
    That way a controller swapped in at an event, or one someone changed
    in the REV Hardware Client, behaves exactly like the old one.

Units:
    The encoders are set up to count in our units, so the rest of the code
    never sees motor rotations or RPM:

    - drive position in meters and speed in m/s (gear ratio and wheel size
      from the robot config);
    - steer angle in radians and speed in rad/s.

    The steer controller's angle loop wraps at +/- pi, so asking for 179
    degrees from -179 degrees turns 2 degrees, not 358.

Example:
    In ``robot.py`` on the real robot (needs CAN hardware, so not run here):

    .. code-block:: python

        from subsystem.drivetrain.io.module_io_spark import ModuleIOSpark

        front_left = ModuleIOSpark(CONFIG, CONFIG.corners[0])
"""

import logging
import math

import rev

from config.robot_config import CornerConfig, RobotConfig
from subsystem.drivetrain.io.module_io import ModuleInputs, ModuleIO
from utils.alerts import error_alert

log = logging.getLogger(__name__)

_RARELY_MS = 500
"""Report period (ms) for values we don't use, to save CAN bandwidth."""


def make_spark_max(can_id: int) -> rev.SparkMax:
    """Create a SPARK MAX for a brushless NEO on the default CAN bus.

    RobotPy 2027 adds a CAN bus number as the first argument (SystemCore
    has several buses); 2026 has only one bus. This works on both.

    Args:
        can_id: The controller's CAN ID.
    """
    brushless = rev.SparkLowLevel.MotorType.kBrushless
    try:
        return rev.SparkMax(can_id, brushless)  # pyright: ignore[reportCallIssue]
    except TypeError:
        return rev.SparkMax(0, can_id, brushless)  # pyright: ignore[reportCallIssue]


class ModuleIOSpark(ModuleIO):
    """A :class:`~subsystem.drivetrain.io.module_io.ModuleIO` for SPARK MAX + NEO drive and steer.

    Args:
        config: The robot config (gearing, wheel size, SPARK settings).
        corner: This module's corner (CAN IDs and drive inversion).

    Attributes:
        drive: The drive SPARK MAX.
        steer: The steer SPARK MAX.
        configured: ``True`` if both controllers accepted their settings at boot.
    """

    def __init__(self, config: RobotConfig, corner: CornerConfig) -> None:
        self.drive = make_spark_max(corner.drive_can_id)
        self.steer = make_spark_max(corner.steer_can_id)
        self._drive_encoder = self.drive.getEncoder()
        self._steer_encoder = self.steer.getEncoder()
        self._drive_loop = self.drive.getClosedLoopController()
        self._steer_loop = self.steer.getClosedLoopController()

        ok_drive = self._apply(self.drive, drive_config(config, corner), f"{corner.name} drive")
        ok_steer = self._apply(self.steer, steer_config(config), f"{corner.name} steer")
        self.configured = ok_drive and ok_steer
        self._config_alert = error_alert(f"Swerve {corner.name}: a SPARK MAX rejected its settings, check CAN")
        self._config_alert.set(not self.configured)

    @staticmethod
    def _apply(spark: rev.SparkMax, cfg: rev.SparkMaxConfig, what: str) -> bool:
        error = spark.configure(cfg, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kNoPersistParameters)
        if error != rev.REVLibError.kOk:
            log.error("%s SPARK MAX configure failed: %s", what, error)
            return False
        return True

    # -- ModuleIO --------------------------------------------------------------

    def update_inputs(self, inputs: ModuleInputs) -> None:
        position, position_ok = _read(self._drive_encoder.getPosition())
        inputs.drive_position_m = position
        inputs.drive_velocity_mps, _ = _read(self._drive_encoder.getVelocity())
        inputs.drive_applied_volts = _volts(self.drive)
        inputs.drive_current_amps, _ = _read(self.drive.getOutputCurrent())
        inputs.drive_temp_c, _ = _read(self.drive.getMotorTemperature())
        inputs.drive_connected = position_ok and self.drive.getLastError() == rev.REVLibError.kOk

        angle, angle_ok = _read(self._steer_encoder.getPosition())
        inputs.steer_angle_rad = math.remainder(angle, math.tau)
        inputs.steer_velocity_rad_per_s, _ = _read(self._steer_encoder.getVelocity())
        inputs.steer_applied_volts = _volts(self.steer)
        inputs.steer_current_amps, _ = _read(self.steer.getOutputCurrent())
        inputs.steer_temp_c, _ = _read(self.steer.getMotorTemperature())
        inputs.steer_connected = angle_ok and self.steer.getLastError() == rev.REVLibError.kOk

    def set_drive_velocity(self, velocity_mps: float, feedforward_volts: float = 0.0) -> None:
        self._drive_loop.setSetpoint(
            velocity_mps,
            rev.SparkBase.ControlType.kVelocity,
            rev.ClosedLoopSlot.kSlot0,
            feedforward_volts,
            rev.SparkClosedLoopController.ArbFFUnits.kVoltage,
        )

    def set_drive_voltage(self, volts: float) -> None:
        self.drive.setVoltage(volts)

    def set_steer_angle(self, angle_rad: float) -> None:
        self._steer_loop.setSetpoint(angle_rad, rev.SparkBase.ControlType.kPosition)

    def set_steer_voltage(self, volts: float) -> None:
        self.steer.setVoltage(volts)

    def reset_steer_angle(self, angle_rad: float) -> None:
        self._steer_encoder.setPosition(angle_rad)


def _read(value) -> tuple[float, bool]:
    """Read a REV value as ``(number, is_fresh)`` on both RobotPy versions.

    RobotPy 2026 returns plain numbers. 2027 returns a ``Signal`` that also
    says whether the value arrived recently.
    """
    if hasattr(value, "get"):
        return float(value.get()), bool(value.isValid())
    return float(value), True


def _volts(spark: rev.SparkMax) -> float:
    """The voltage a controller is applying: output fraction times battery voltage."""
    output, _ = _read(spark.getAppliedOutput())
    battery, _ = _read(spark.getBusVoltage())
    return output * battery


def drive_config(config: RobotConfig, corner: CornerConfig) -> rev.SparkMaxConfig:
    """Build the drive SPARK MAX settings for one corner.

    Args:
        config: The robot config.
        corner: The corner (for its drive inversion).

    Returns:
        A ``SparkMaxConfig`` ready for ``SparkMax.configure``.
    """
    settings = config.spark
    m_per_rot = config.drive_m_per_motor_rot
    cfg = rev.SparkMaxConfig()
    cfg.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    cfg.inverted(corner.drive_inverted)
    cfg.smartCurrentLimit(settings.drive_current_limit_amps)
    cfg.voltageCompensation(12.0)
    # Meters and m/s instead of rotations and RPM. A short velocity filter
    # (16 ms x 2 samples instead of 32 ms x 8) cuts about 100 ms of lag.
    cfg.encoder.positionConversionFactor(m_per_rot).velocityConversionFactor(m_per_rot / 60.0)
    cfg.encoder.uvwMeasurementPeriod(16).uvwAverageDepth(2)
    cfg.closedLoop.pid(settings.drive_kp, 0.0, 0.0)
    _set_report_periods(cfg, settings.encoder_period_ms)
    return cfg


def steer_config(config: RobotConfig) -> rev.SparkMaxConfig:
    """Build the steer SPARK MAX settings (the same for every corner).

    Args:
        config: The robot config.

    Returns:
        A ``SparkMaxConfig`` ready for ``SparkMax.configure``.
    """
    settings = config.spark
    rad_per_rot = config.preset.steer_rad_per_motor_rot
    cfg = rev.SparkMaxConfig()
    cfg.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
    cfg.inverted(config.preset.steer_inverted)
    cfg.smartCurrentLimit(settings.steer_current_limit_amps)
    cfg.voltageCompensation(12.0)
    cfg.encoder.positionConversionFactor(rad_per_rot).velocityConversionFactor(rad_per_rot / 60.0)
    cfg.closedLoop.pid(settings.steer_kp, 0.0, settings.steer_kd)
    cfg.closedLoop.positionWrappingEnabled(True).positionWrappingInputRange(-math.pi, math.pi)
    _set_report_periods(cfg, settings.encoder_period_ms)
    return cfg


def _set_report_periods(cfg: rev.SparkMaxConfig, encoder_period_ms: int) -> None:
    """Report the encoder every loop and slow down the values we never read."""
    signals = cfg.signals
    signals.primaryEncoderPositionPeriodMs(encoder_period_ms)
    signals.primaryEncoderVelocityPeriodMs(encoder_period_ms)
    signals.appliedOutputPeriodMs(encoder_period_ms)
    signals.outputCurrentPeriodMs(encoder_period_ms)
    signals.analogPositionPeriodMs(_RARELY_MS)
    signals.analogVelocityPeriodMs(_RARELY_MS)
    signals.analogVoltagePeriodMs(_RARELY_MS)
    signals.absoluteEncoderPositionPeriodMs(_RARELY_MS)
    signals.absoluteEncoderVelocityPeriodMs(_RARELY_MS)
