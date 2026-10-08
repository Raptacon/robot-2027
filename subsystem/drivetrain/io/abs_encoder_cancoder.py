"""
The real absolute encoder: a CTRE CANcoder on each module.

This is the only drivetrain file that talks to CTRE's Phoenix 6 library.

How the offset works:
    The CANcoder's own magnet offset is set to 0, and our offset from the
    robot config is added in code. That keeps every calibration value in
    git (``encoder_offset_rot`` in ``config/robots/<robot>.py``) instead of
    hidden inside the encoder, so a swapped encoder only needs its CAN ID set.

    ``raw_rot`` is the encoder's reading before the offset. To calibrate,
    point the wheel straight forward and set the offset to minus that
    reading (doc/swerve/tuning-and-calibration.md section 3).

Direction:
    The preset's ``encoder_inverted`` says which way the encoder counts on
    that module type. We set it so counterclockwise (seen from above) is
    positive, matching the steer motor.

Example:
    In ``robot.py`` on the real robot (needs CAN hardware, so not run here):

    .. code-block:: python

        from subsystem.drivetrain.io.abs_encoder_cancoder import AbsoluteEncoderIOCANcoder

        encoder = AbsoluteEncoderIOCANcoder(CONFIG, CONFIG.corners[0])
"""

import logging
import math

from phoenix6 import CANBus
from phoenix6.configs import CANcoderConfiguration
from phoenix6.hardware import CANcoder
from phoenix6.signals import SensorDirectionValue

from config.robot_config import CornerConfig, RobotConfig, wrap_rotations
from subsystem.drivetrain.io.abs_encoder_io import AbsoluteEncoderInputs, AbsoluteEncoderIO
from utils.alerts import error_alert

log = logging.getLogger(__name__)

UPDATE_HZ = 50.0
"""How often the CANcoder sends its absolute position (once per 20 ms loop)."""

CONFIG_TIMEOUT_S = 0.25
"""How long to wait for the CANcoder to accept its settings at boot."""


class AbsoluteEncoderIOCANcoder(AbsoluteEncoderIO):
    """An :class:`~subsystem.drivetrain.io.abs_encoder_io.AbsoluteEncoderIO` for a CANcoder.

    Args:
        config: The robot config (CAN bus and the preset's encoder direction).
        corner: This module's corner (encoder CAN ID and offset).

    Attributes:
        encoder: The Phoenix 6 ``CANcoder``.
        configured: ``True`` if the encoder accepted its settings at boot.
    """

    def __init__(self, config: RobotConfig, corner: CornerConfig) -> None:
        self.encoder = CANcoder(corner.encoder_can_id, CANBus(config.can_bus))
        self._offset_rot = corner.encoder_offset_rot

        cfg = CANcoderConfiguration()
        cfg.magnet_sensor.magnet_offset = 0.0  # our offset is added in code
        cfg.magnet_sensor.absolute_sensor_discontinuity_point = 0.5  # read -0.5 to 0.5 rotations
        cfg.magnet_sensor.sensor_direction = (
            SensorDirectionValue.CLOCKWISE_POSITIVE
            if config.preset.encoder_inverted
            else SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        )
        status = self.encoder.configurator.apply(cfg, CONFIG_TIMEOUT_S)
        self.configured = status.is_ok()
        if not self.configured:
            log.error("%s CANcoder configure failed: %s", corner.name, status)
        self._config_alert = error_alert(f"Swerve {corner.name}: CANcoder rejected its settings, check CAN")
        self._config_alert.set(not self.configured)

        self._position = self.encoder.get_absolute_position()
        self._position.set_update_frequency(UPDATE_HZ)
        # Turn off the CANcoder's other messages to save CAN bandwidth.
        self.encoder.optimize_bus_utilization()

    def update_inputs(self, inputs: AbsoluteEncoderInputs) -> None:
        status = self._position.refresh().status
        inputs.connected = status.is_ok()
        raw = float(self._position.value)
        inputs.raw_rot = raw
        inputs.angle_rad = wrap_rotations(raw + self._offset_rot) * math.tau
