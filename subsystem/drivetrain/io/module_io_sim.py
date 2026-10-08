"""
A simulated swerve module, good enough to check the drive code in CI.

What does it simulate?
    Two motors (drive and steer) using :class:`~subsystem.drivetrain.io.sim_motor.SimMotor`,
    plus the motor controllers' built-in speed and position loops, which run
    every millisecond like a real SparkMax does.

    It also copies the real problems the robot-2026 review found, so tests can
    check the drive code handles them (see :class:`ModuleSimOptions`):

    - The steer motor's encoder reads 0 at power-on wherever the wheel points,
      so the code must seed it from the absolute encoder.
    - Sensor readings can arrive a few loops late.
    - The real gears or wheel can differ from what the config says.

Example:
    >>> from config.robots.swerve_test_bot import CONFIG
    >>> from subsystem.drivetrain.io.module_io import ModuleInputs
    >>> from subsystem.drivetrain.io.module_io_sim import ModuleIOSim
    >>> io = ModuleIOSim(CONFIG)
    >>> inputs = ModuleInputs()
    >>> io.set_steer_angle(1.0)
    >>> for _ in range(25):  # half a second of 20 ms loops
    ...     io.update_inputs(inputs)
    >>> round(inputs.steer_angle_rad, 2)
    1.0
"""

import math
from collections import deque
from dataclasses import dataclass

from config.module_presets import ModulePreset
from config.robot_config import RobotConfig
from subsystem.drivetrain.io.module_io import ModuleInputs, ModuleIO
from subsystem.drivetrain.io.sim_motor import NOMINAL_VOLTS, SimMotor


@dataclass(frozen=True)
class ModuleSimOptions:
    """Settings for one simulated module. The defaults are a healthy module.

    The motor time constants and gains are placeholders until SysId
    (milestone M6) gives measured kV and kA for our robot.

    Attributes:
        latency_loops: How many loops late the sensor readings arrive. 0 means
            the readings are from this loop.
        initial_steer_angle_rad: Which way the wheel really points at power-on.
            The steer encoder still reads 0, like on the real robot.
        actual_preset: The gears really in the module, if different from the
            config. Use this to test what happens with the wrong preset.
        actual_wheel_radius_m: The real wheel radius, if different from the
            config (for example, worn tread).
        drive_tau_s: How quickly the drive motor reaches a new speed, in
            seconds (kA / kV). Includes the robot's weight.
        steer_tau_s: How quickly the steer motor reaches a new speed.
        drive_kp: Drive speed loop gain, volts per (m/s) of error.
        steer_kp: Steer position loop gain, volts per radian of error.
        steer_kd: Steer damping, volts per (rad/s).
        loop_period_s: Time simulated by each :meth:`ModuleIOSim.update_inputs` call.
        substeps: Physics steps per loop. 20 steps of 1 ms match a SparkMax's
            1 kHz control loop.
    """

    latency_loops: int = 0
    initial_steer_angle_rad: float = 0.0
    actual_preset: ModulePreset | None = None
    actual_wheel_radius_m: float | None = None
    drive_tau_s: float = 0.15
    steer_tau_s: float = 0.01
    drive_kp: float = 4.0
    steer_kp: float = 40.0
    steer_kd: float = 0.0
    loop_period_s: float = 0.02
    substeps: int = 20


class ModuleIOSim(ModuleIO):
    """A :class:`~subsystem.drivetrain.io.module_io.ModuleIO` that runs a physics model.

    Each call to :meth:`update_inputs` moves time forward by one loop (20 ms)
    and then reads the simulated sensors, the same order the real robot sees.

    Args:
        config: The robot config. Gears and wheel size come from here, the
            same as they would for the real module.
        options: Simulation settings. Leave out for a healthy module.
    """

    def __init__(self, config: RobotConfig, options: ModuleSimOptions | None = None) -> None:
        self.options = options or ModuleSimOptions()
        opts = self.options
        preset = config.preset
        actual = opts.actual_preset or preset
        actual_radius = opts.actual_wheel_radius_m or config.wheel_radius_m

        # What the motor controller reports (from the config), and what really happens.
        self._drive_m_per_rot = preset.drive_m_per_motor_rot(config.wheel_radius_m)
        self._true_drive_m_per_rot = actual.drive_m_per_motor_rot(actual_radius)
        self._steer_rad_per_rot = preset.steer_rad_per_motor_rot
        self._true_steer_rad_per_rot = actual.steer_rad_per_motor_rot

        drive_kv = NOMINAL_VOLTS / config.drive_motor.free_speed_rps
        steer_kv = NOMINAL_VOLTS / config.steer_motor.free_speed_rps
        self._drive = SimMotor(kv=drive_kv, ka=drive_kv * opts.drive_tau_s)
        self._steer = SimMotor(kv=steer_kv, ka=steer_kv * opts.steer_tau_s)

        # Put the wheel at its starting angle. The steer encoder zeroes here,
        # so it reads 0 even though the wheel may not point forward.
        self._steer.position = opts.initial_steer_angle_rad / self._true_steer_rad_per_rot
        self._steer_encoder_zero_rot = self._steer.position

        self._drive_mode = "volts"
        self._drive_target = 0.0
        self._drive_ff_volts = 0.0
        self._steer_mode = "volts"
        self._steer_target = 0.0

        # Start with blank readings so the first real one arrives `latency_loops` loops late.
        self._history: deque[ModuleInputs] = deque(
            [ModuleInputs() for _ in range(opts.latency_loops)], maxlen=opts.latency_loops + 1
        )

    # -- ModuleIO -----------------------------------------------------------

    def update_inputs(self, inputs: ModuleInputs) -> None:
        """Simulate one loop, then copy the (possibly delayed) readings into ``inputs``."""
        dt = self.options.loop_period_s / self.options.substeps
        for _ in range(self.options.substeps):
            self._drive.step(self._drive_volts(), dt)
            self._steer.step(self._steer_volts(), dt)

        self._history.append(self._read_sensors())
        delayed = self._history[0]
        for name, value in vars(delayed).items():
            setattr(inputs, name, value)

    def set_drive_velocity(self, velocity_mps: float, feedforward_volts: float = 0.0) -> None:
        self._drive_mode = "velocity"
        self._drive_target = velocity_mps
        self._drive_ff_volts = feedforward_volts

    def set_drive_voltage(self, volts: float) -> None:
        self._drive_mode = "volts"
        self._drive_target = volts

    def set_steer_angle(self, angle_rad: float) -> None:
        self._steer_mode = "position"
        self._steer_target = angle_rad

    def set_steer_voltage(self, volts: float) -> None:
        self._steer_mode = "volts"
        self._steer_target = volts

    def reset_steer_angle(self, angle_rad: float) -> None:
        # Shift the encoder's zero so it reads angle_rad right now.
        self._steer_encoder_zero_rot = self._steer.position - angle_rad / self._steer_rad_per_rot

    # -- Ground truth, for other sims and for tests ---------------------------

    @property
    def true_steer_angle_rad(self) -> float:
        """Where the wheel really points, from -pi to pi (what an absolute encoder would see)."""
        return math.remainder(self._steer.position * self._true_steer_rad_per_rot, math.tau)

    @property
    def true_drive_position_m(self) -> float:
        """How far the wheel has really rolled, in meters."""
        return self._drive.position * self._true_drive_m_per_rot

    @property
    def true_drive_velocity_mps(self) -> float:
        """How fast the wheel is really rolling, in meters per second."""
        return self._drive.velocity * self._true_drive_m_per_rot

    # -- Internals ------------------------------------------------------------

    def _measured_steer_angle_rad(self) -> float:
        return math.remainder((self._steer.position - self._steer_encoder_zero_rot) * self._steer_rad_per_rot, math.tau)

    def _drive_volts(self) -> float:
        if self._drive_mode == "volts":
            return self._drive_target
        measured_mps = self._drive.velocity * self._drive_m_per_rot
        return self._drive_ff_volts + self.options.drive_kp * (self._drive_target - measured_mps)

    def _steer_volts(self) -> float:
        if self._steer_mode == "volts":
            return self._steer_target
        # Shortest way around: the error is always between -pi and pi.
        error = math.remainder(self._steer_target - self._measured_steer_angle_rad(), math.tau)
        rate = self._steer.velocity * self._steer_rad_per_rot
        return self.options.steer_kp * error - self.options.steer_kd * rate

    def _read_sensors(self) -> ModuleInputs:
        return ModuleInputs(
            drive_connected=True,
            drive_position_m=self._drive.position * self._drive_m_per_rot,
            drive_velocity_mps=self._drive.velocity * self._drive_m_per_rot,
            drive_applied_volts=self._drive.applied_volts,
            drive_current_amps=self._drive.current_amps,
            drive_temp_c=25.0,
            steer_connected=True,
            steer_angle_rad=self._measured_steer_angle_rad(),
            steer_velocity_rad_per_s=self._steer.velocity * self._steer_rad_per_rot,
            steer_applied_volts=self._steer.applied_volts,
            steer_current_amps=self._steer.current_amps,
            steer_temp_c=25.0,
        )
