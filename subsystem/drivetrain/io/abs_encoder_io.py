"""
The interface for a module's absolute encoder.

Why do we need an absolute encoder?
    The steer motor has its own encoder, but it starts at 0 every time the
    robot turns on, wherever the wheel happens to point. An *absolute*
    encoder remembers the wheel's real direction even after power-off. At
    startup the module reads the absolute encoder once and copies the angle
    into the steer motor's encoder (see
    :meth:`~subsystem.drivetrain.io.module_io.ModuleIO.reset_steer_angle`).

    After that, the drive code uses only the steer motor's encoder. The
    absolute encoder is still read every loop so we can notice if the two
    disagree.

Example:
    >>> from subsystem.drivetrain.io.abs_encoder_io import AbsoluteEncoderInputs
    >>> from subsystem.drivetrain.io.abs_encoder_sim import AbsoluteEncoderIOSim
    >>> encoder = AbsoluteEncoderIOSim(true_angle_rad=lambda: 1.0, offset_rot=0.0)
    >>> inputs = AbsoluteEncoderInputs()
    >>> encoder.update_inputs(inputs)
    >>> inputs.connected, round(inputs.angle_rad, 6)
    (True, 1.0)
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass
class AbsoluteEncoderInputs:
    """One loop's readings from an absolute encoder.

    Attributes:
        connected: ``False`` if the encoder isn't answering, or hasn't sent a
            fresh reading yet after boot. Never seed the steer motor from a
            disconnected encoder.
        raw_rot: The encoder's reading before our offset is added, in
            rotations. This is the number to write down when calibrating
            offsets (doc/swerve/tuning-and-calibration.md section 3).
        angle_rad: Which way the wheel points with the offset added, in
            radians from -pi to pi. 0 means straight forward.
    """

    connected: bool = False
    raw_rot: float = 0.0
    angle_rad: float = 0.0


class AbsoluteEncoderIO(ABC):
    """What the module code can ask of an absolute encoder.

    Implementations add the corner's ``encoder_offset_rot`` from the robot
    config, so ``angle_rad`` is already "0 means forward".
    """

    @abstractmethod
    def update_inputs(self, inputs: AbsoluteEncoderInputs) -> None:
        """Read the encoder and write the values into ``inputs``. Call once per loop."""
