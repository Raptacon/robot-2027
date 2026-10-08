"""
Calibrate the absolute encoder offsets: point the wheels forward, press a button, copy the numbers.

How to calibrate (doc/swerve/tuning-and-calibration.md section 3):

1. Robot on blocks and **disabled**.
2. Turn each wheel by hand until it points straight forward, with the
   bevel gears all facing the same side. Lay a straightedge along both
   wheels on each side to get them square.
3. On the dashboard, press ``Characterization/Calibrate offsets``. It works
   while disabled and never moves a motor.
4. Copy ``/Characterization/Offsets/python`` into ``offsets_rot`` in the
   robot's config file (it is already in the file's format), and add a line
   to doc/swerve/calibration-log.md in the same commit.
5. Deploy and run the module check (test mode) to confirm.

The math: the module reads ``angle = raw + offset``. With the wheel
pointing forward the angle should be 0, so ``offset = -raw``.

Example:
    >>> from commands.drive.calibrate_offsets import offsets_from_raw, offsets_python
    >>> offsets = offsets_from_raw([0.25, -0.1, 0.0, 0.4])
    >>> [round(offset, 3) for offset in offsets]
    [-0.25, 0.1, 0.0, -0.4]
    >>> print(offsets_python(offsets, ["frontLeft", "frontRight", "backLeft", "backRight"]))
    offsets_rot=(
        -90.0 / 360.0,  # frontLeft
        36.0 / 360.0,  # frontRight
        0.0 / 360.0,  # backLeft
        -144.0 / 360.0,  # backRight
    ),
"""

import logging
from collections.abc import Sequence

import commands2
import ntcore

from config.robot_config import wrap_rotations
from subsystem.drivetrain.drivetrain import Drivetrain

log = logging.getLogger(__name__)


def offsets_from_raw(raw_rot: Sequence[float]) -> tuple[float, ...]:
    """The offsets that make each raw reading read 0 (wheel straight forward).

    Args:
        raw_rot: Each encoder's raw reading, rotations, with its wheel pointing forward.

    Returns:
        The offsets in rotations, wrapped to [-0.5, 0.5).
    """
    return tuple(wrap_rotations(-raw) for raw in raw_rot)


def offsets_python(offsets_rot: Sequence[float], names: Sequence[str]) -> str:
    """Format offsets as the ``offsets_rot=(...)`` lines for a robot config file.

    The numbers are written as degrees / 360, like the config files, so
    they are easy to compare with what a wheel looks like.

    Args:
        offsets_rot: The offsets, rotations.
        names: The corner names, for the comments.

    Returns:
        Python source text to paste into the config.
    """
    lines = ["offsets_rot=("]
    for offset, name in zip(offsets_rot, names):
        lines.append(f"    {round(offset * 360.0, 4)} / 360.0,  # {name}")
    lines.append("),")
    return "\n".join(lines)


class CalibrateOffsets(commands2.Command):
    """Read every absolute encoder and publish the offsets that make the wheels read 0.

    Runs once and finishes. It works while the robot is disabled and doesn't
    move anything, so it doesn't take over the drivetrain.

    Args:
        drivetrain: The drivetrain.

    Attributes:
        offsets_rot: The offsets from the last run, rotations, or ``None``.
    """

    def __init__(self, drivetrain: Drivetrain) -> None:
        super().__init__()
        self.drivetrain = drivetrain
        self.offsets_rot: tuple[float, ...] | None = None
        table = ntcore.NetworkTableInstance.getDefault().getTable("/Characterization/Offsets")
        self._corner_pubs = [table.getDoubleTopic(f"{c.name}_rot").publish() for c in drivetrain.config.corners]
        self._python_pub = table.getStringTopic("python").publish()
        self._status_pub = table.getStringTopic("status").publish()
        self.setName("Calibrate offsets")

    def initialize(self) -> None:
        modules = self.drivetrain.modules
        missing = [m.corner.name for m in modules if not m.encoder_inputs.connected]
        if missing:
            message = f"Not calibrated: no reading from {', '.join(missing)}"
            log.error(message)
            self._status_pub.set(message)
            return
        self.offsets_rot = offsets_from_raw([m.encoder_inputs.raw_rot for m in modules])
        text = offsets_python(self.offsets_rot, [m.corner.name for m in modules])
        for pub, offset in zip(self._corner_pubs, self.offsets_rot):
            pub.set(offset)
        self._python_pub.set(text)
        self._status_pub.set("Calibrated: copy /Characterization/Offsets/python into the robot config")
        log.info("New encoder offsets (paste into the robot config):\n%s", text)

    def isFinished(self) -> bool:
        return True

    def runsWhenDisabled(self) -> bool:
        return True
