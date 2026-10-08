"""
A simulated absolute encoder.

It reads the wheel's true direction from the simulated module and reports it
the way a real absolute encoder would: as a raw reading in rotations, with our
calibration offset added on top. Options copy real failures so tests can check
the module code copes with them.

Example:
    An encoder that takes 3 loops to send its first reading after boot:

    >>> from subsystem.drivetrain.io.abs_encoder_io import AbsoluteEncoderInputs
    >>> from subsystem.drivetrain.io.abs_encoder_sim import AbsoluteEncoderIOSim
    >>> encoder = AbsoluteEncoderIOSim(true_angle_rad=lambda: 0.0, offset_rot=0.25, boot_delay_loops=3)
    >>> inputs = AbsoluteEncoderInputs()
    >>> [encoder.update_inputs(inputs) or inputs.connected for _ in range(4)]
    [False, False, False, True]
"""

import math
from collections import deque
from collections.abc import Callable

from config.robot_config import CornerConfig, wrap_rotations
from subsystem.drivetrain.io.abs_encoder_io import AbsoluteEncoderInputs, AbsoluteEncoderIO
from subsystem.drivetrain.io.module_io_sim import ModuleIOSim


class AbsoluteEncoderIOSim(AbsoluteEncoderIO):
    """An :class:`~subsystem.drivetrain.io.abs_encoder_io.AbsoluteEncoderIO` for simulation.

    Args:
        true_angle_rad: A function that returns where the wheel really points,
            in radians. Usually ``module_sim.true_steer_angle_rad`` (see
            :meth:`for_module`).
        offset_rot: The calibration offset from the robot config, in rotations.
        offset_error_rot: How wrong that offset is, in rotations. A real
            encoder whose magnet slipped would look like this. Default 0.
        latency_loops: How many loops late each reading arrives.
        boot_delay_loops: How many loops after boot before the first good
            reading. Until then ``connected`` is ``False``.
        never_connects: ``True`` to simulate an encoder that is unplugged or
            dead: ``connected`` stays ``False`` forever.
    """

    def __init__(
        self,
        true_angle_rad: Callable[[], float],
        offset_rot: float,
        offset_error_rot: float = 0.0,
        latency_loops: int = 0,
        boot_delay_loops: int = 0,
        never_connects: bool = False,
    ) -> None:
        self._true_angle_rad = true_angle_rad
        self._offset_rot = offset_rot
        self._offset_error_rot = offset_error_rot
        self._boot_delay_loops = boot_delay_loops
        self._never_connects = never_connects
        self._loops = 0
        # Start with blank readings so the first real one arrives `latency_loops` loops late.
        self._history: deque[AbsoluteEncoderInputs] = deque(
            [AbsoluteEncoderInputs() for _ in range(latency_loops)], maxlen=latency_loops + 1
        )

    @classmethod
    def for_module(cls, module: ModuleIOSim, corner: CornerConfig, **options) -> "AbsoluteEncoderIOSim":
        """Make the encoder for a simulated module, using the corner's offset.

        Args:
            module: The simulated module the encoder is mounted on.
            corner: That module's config (for ``encoder_offset_rot``).
            **options: Any of the other constructor options, like
                ``boot_delay_loops=5``.
        """
        return cls(lambda: module.true_steer_angle_rad, corner.encoder_offset_rot, **options)

    def update_inputs(self, inputs: AbsoluteEncoderInputs) -> None:
        self._loops += 1
        connected = not self._never_connects and self._loops > self._boot_delay_loops
        true_rot = self._true_angle_rad() / math.tau
        # The raw reading is what makes "raw + offset = true angle" (plus any offset error).
        raw_rot = wrap_rotations(true_rot - self._offset_rot + self._offset_error_rot)
        angle_rad = wrap_rotations(raw_rot + self._offset_rot) * math.tau
        if connected:
            reading = AbsoluteEncoderInputs(connected=True, raw_rot=raw_rot, angle_rad=angle_rad)
        else:
            reading = AbsoluteEncoderInputs(connected=False)
        self._history.append(reading)
        delayed = self._history[0]
        inputs.connected = delayed.connected
        inputs.raw_rot = delayed.raw_rot
        inputs.angle_rad = delayed.angle_rad
