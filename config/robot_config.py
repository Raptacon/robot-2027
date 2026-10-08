"""
Per-robot swerve config types. Each robot is one module in config/robots/.

Everything is frozen: a config is built once at import and never changed at
runtime. Units are SI (meters, radians) except encoder offsets, which are in
rotations to match what calibration tools print.
"""

import math
from dataclasses import dataclass

from config.module_presets import NEO, MotorSpec, ModulePreset

# WPILib order for kinematics arrays.
CORNER_NAMES = ("frontLeft", "frontRight", "backLeft", "backRight")


@dataclass(frozen=True)
class CornerConfig:
    """One module position on the robot.

    x_m, y_m: wheel contact point from robot center (+x forward, +y left).
    encoder_offset_rot: added to the raw absolute reading so 0 means the
        wheel points forward. Kept in [-0.5, 0.5).
    """

    name: str
    x_m: float
    y_m: float
    drive_can_id: int
    steer_can_id: int
    encoder_can_id: int
    encoder_offset_rot: float
    drive_inverted: bool

    @property
    def can_ids(self) -> tuple[int, int, int]:
        return (self.drive_can_id, self.steer_can_id, self.encoder_can_id)


@dataclass(frozen=True)
class RobotConfig:
    name: str
    preset: ModulePreset
    corners: tuple[CornerConfig, CornerConfig, CornerConfig, CornerConfig]
    wheel_radius_m: float
    drive_motor: MotorSpec = NEO
    steer_motor: MotorSpec = NEO
    gyro_can_id: int | None = None
    gyro_inverted: bool = False
    # Empty means the controller's default bus. SystemCore bus assignment is M8.
    can_bus: str = ""

    @property
    def drive_m_per_motor_rot(self) -> float:
        return self.preset.drive_m_per_motor_rot(self.wheel_radius_m)

    @property
    def free_speed_mps(self) -> float:
        return self.preset.free_speed_mps(self.drive_motor, self.wheel_radius_m)

    @property
    def drive_base_radius_m(self) -> float:
        """Distance from robot center to the farthest module."""
        return max(math.hypot(c.x_m, c.y_m) for c in self.corners)

    @property
    def max_angular_speed_rad_per_s(self) -> float:
        """Theoretical spin rate at free speed: v_max / r."""
        return self.free_speed_mps / self.drive_base_radius_m

    def all_can_ids(self) -> list[int]:
        ids = [i for c in self.corners for i in c.can_ids]
        if self.gyro_can_id is not None:
            ids.append(self.gyro_can_id)
        return ids


def wrap_rotations(rot: float) -> float:
    """Wrap an angle in rotations to [-0.5, 0.5)."""
    return (rot + 0.5) % 1.0 - 0.5


def mirrored_corners(
    x_m: float,
    y_m: float,
    first_can_id: int,
    offsets_rot: tuple[float, float, float, float],
    drive_inverted: tuple[bool, bool, bool, bool],
) -> tuple[CornerConfig, CornerConfig, CornerConfig, CornerConfig]:
    """Build FL, FR, BL, BR corners mirrored from the front-left position.

    Works for any rectangular layout (x_m and y_m can differ). For other
    layouts, list the four CornerConfig values directly.

    CAN IDs follow the team convention: 3 consecutive IDs per module (drive,
    steer, encoder) starting at first_can_id, in FL, FR, BL, BR order.
    """
    signs = ((1, 1), (1, -1), (-1, 1), (-1, -1))
    corners = []
    for i, (name, (sx, sy)) in enumerate(zip(CORNER_NAMES, signs)):
        base = first_can_id + 3 * i
        corners.append(
            CornerConfig(
                name=name,
                x_m=sx * x_m,
                y_m=sy * y_m,
                drive_can_id=base,
                steer_can_id=base + 1,
                encoder_can_id=base + 2,
                encoder_offset_rot=wrap_rotations(offsets_rot[i]),
                drive_inverted=drive_inverted[i],
            )
        )
    return (corners[0], corners[1], corners[2], corners[3])
