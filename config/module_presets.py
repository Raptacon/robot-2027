"""
Swerve module presets: gearing and direction for each SDS module and ratio.

A preset holds what is fixed by the module hardware. Per-robot values (CAN
IDs, offsets, drive inversion, wheel radius) live in config/robots/.

Ratios are motor rotations per output rotation. Sources: SDS MK4i and MK4
product pages; see doc/swerve/research-and-review.md section 1.
"""

import math
from dataclasses import dataclass

# 4 in nominal wheel. Real robots use a measured radius (M6 wheel radius test).
NOMINAL_WHEEL_RADIUS_M = 0.0508


@dataclass(frozen=True)
class MotorSpec:
    """The motor facts the config needs. The sim maps this to a DCMotor."""

    name: str
    free_speed_rpm: float

    @property
    def free_speed_rps(self) -> float:
        """Free speed in motor rotations per second."""
        return self.free_speed_rpm / 60.0


NEO = MotorSpec("NEO", 5676.0)


@dataclass(frozen=True)
class ModulePreset:
    """Gearing and directions fixed by the module design.

    steer_inverted: the steer motor must be inverted so the wheel turns CCW
        positive (seen from above) for a positive motor command.
    encoder_inverted: the absolute encoder reads CW positive and must be
        inverted to match. Checked on blocks at M5.
    """

    name: str
    drive_ratio: float
    steer_ratio: float
    steer_inverted: bool
    encoder_inverted: bool

    def drive_m_per_motor_rot(self, wheel_radius_m: float) -> float:
        """Wheel travel in meters for one drive motor rotation."""
        return 2.0 * math.pi * wheel_radius_m / self.drive_ratio

    @property
    def steer_rad_per_motor_rot(self) -> float:
        """Wheel angle change in radians for one steer motor rotation."""
        return 2.0 * math.pi / self.steer_ratio

    def free_speed_mps(self, motor: MotorSpec, wheel_radius_m: float) -> float:
        """Theoretical top wheel speed. Real top speed is about 80 to 90% of this."""
        return motor.free_speed_rps * self.drive_m_per_motor_rot(wheel_radius_m)


_MK4I_STEER = 150.0 / 7.0
_MK4_STEER = 12.8

MK4I_L1 = ModulePreset("MK4i_L1", 8.14, _MK4I_STEER, steer_inverted=True, encoder_inverted=False)
MK4I_L2 = ModulePreset("MK4i_L2", 6.75, _MK4I_STEER, steer_inverted=True, encoder_inverted=False)
MK4I_L3 = ModulePreset("MK4i_L3", 6.12, _MK4I_STEER, steer_inverted=True, encoder_inverted=False)

MK4_L1 = ModulePreset("MK4_L1", 8.14, _MK4_STEER, steer_inverted=False, encoder_inverted=False)
MK4_L2 = ModulePreset("MK4_L2", 6.75, _MK4_STEER, steer_inverted=False, encoder_inverted=False)
MK4_L3 = ModulePreset("MK4_L3", 6.12, _MK4_STEER, steer_inverted=False, encoder_inverted=False)
MK4_L4 = ModulePreset("MK4_L4", 5.14, _MK4_STEER, steer_inverted=False, encoder_inverted=False)

PRESETS: dict[str, ModulePreset] = {p.name: p for p in (MK4I_L1, MK4I_L2, MK4I_L3, MK4_L1, MK4_L2, MK4_L3, MK4_L4)}

DEFAULT_PRESET = MK4I_L2
