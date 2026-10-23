"""
The swerve drivetrain's physics, connected to the vendor simulators.

What it does, every time it is stepped:

1. Reads the volts each SPARK MAX is applying (``rev.SparkMaxSim``), after
   the controller's own speed or angle loop has run with the gains from the
   robot config.
2. Moves each motor's :class:`~sim.motor_plant.MotorPlant` with those volts,
   in 1 ms steps (a SPARK MAX runs its loop at 1 kHz).
3. Tells each SPARK MAX how fast its motor turned, so its encoder counts.
4. Works out how the robot really moved from each wheel's real speed and
   angle, and writes that to the CANcoders and the gyro.
5. Sags the battery voltage from the current the motors draw.

The robot code never sees any of this: it only sees its own SPARK MAX,
CANcoder and gyro objects reading back the values written here.

Real problems you can give the simulated robot (see :class:`CornerSim`):
which way each wheel points at power-on, gears or a wheel that don't match
the config, an absolute encoder that is a little off, gyro drift, and an
unplugged gyro (:attr:`SwervePhysics.gyro_connected`).

Example:
    In a test (needs the vendor libraries, so not run here):

    .. code-block:: python

        from sim.swerve_physics import CornerSim, SwervePhysics
        from subsystem.drivetrain.drivetrain_hardware import build_drivetrain

        drivetrain = build_drivetrain(CONFIG)
        physics = SwervePhysics(drivetrain, corners={"frontLeft": CornerSim(initial_angle_rad=1.0)})
        physics.step(0.02)  # 20 ms of physics
        drivetrain.periodic()  # the robot code reads the simulated sensors
        physics.true_pose  # where the robot really is
"""

import math
import time
from collections.abc import Mapping
from dataclasses import dataclass

import rev
from wpilib.simulation import BatterySim, RoboRioSim

from config.module_presets import ModulePreset
from config.robot_config import RobotConfig, wrap_rotations
from sim.motor_plant import NOMINAL_VOLTS, MotorPlant
from subsystem.drivetrain.drivetrain import Drivetrain
from subsystem.drivetrain.swerve_math import ModuleTarget, Pose, forward_kinematics, pose_exp

try:  # RobotPy 2026 and earlier
    from wpimath.system.plant import DCMotor
except ImportError:  # RobotPy 2027 moved it
    from wpimath import DCMotor  # pyright: ignore[reportAttributeAccessIssue]

SUBSTEP_S = 0.001
"""Physics step, seconds. A SPARK MAX runs its control loop every 1 ms."""

DRIVE_TAU_S = 0.15
"""How quickly a drive motor reaches a new speed (kA / kV), seconds, robot weight included.

A guess for a ~50 kg robot until the drive SysId test measures kV and kA.
"""

STEER_TAU_S = 0.01
"""How quickly a steer motor reaches a new speed, seconds. A guess until steer SysId runs."""

NAVX_SIM_DEVICE = "navX-Sensor[4]"
"""The NavX's name in WPILib's simulator (the MXP SPI port)."""


@dataclass(frozen=True)
class CornerSim:
    """How one simulated module really is. The defaults are a healthy module.

    Attributes:
        initial_angle_rad: Which way the wheel really points at power-on.
            The steer motor's encoder still reads 0, like on the real robot.
        actual_preset: The gears really in the module, if different from
            the config. Use this to test what happens with the wrong preset.
        actual_wheel_radius_m: The real wheel radius, if different from the
            config (for example, worn tread).
        encoder_offset_error_rot: How far off the absolute encoder's offset
            really is, in rotations. The CANcoder reads the true angle plus
            this.
    """

    initial_angle_rad: float = 0.0
    actual_preset: ModulePreset | None = None
    actual_wheel_radius_m: float | None = None
    encoder_offset_error_rot: float = 0.0


class _ModulePhysics:
    """One module's two motors, their SPARK MAX simulators and its CANcoder simulator."""

    def __init__(self, module, config: RobotConfig, corner_sim: CornerSim) -> None:
        self.corner = module.corner
        self.corner_sim = corner_sim
        io = module.io
        self.drive_sim = rev.SparkMaxSim(io.drive, DCMotor.NEO(1))
        self.steer_sim = rev.SparkMaxSim(io.steer, DCMotor.NEO(1))
        self.encoder_sim = module.encoder.encoder.sim_state
        self._encoder_reading = module.encoder.encoder.get_absolute_position(refresh=False)
        self._written_raw_rot = 0.0

        preset = config.preset
        actual = corner_sim.actual_preset or preset
        radius = corner_sim.actual_wheel_radius_m or config.wheel_radius_m
        # What the SPARK MAX thinks one motor turn is (from the config) and what it really is.
        self.drive_m_per_rot = preset.drive_m_per_motor_rot(config.wheel_radius_m)
        self.true_drive_m_per_rot = actual.drive_m_per_motor_rot(radius)
        self.steer_rad_per_rot = preset.steer_rad_per_motor_rot
        self.true_steer_rad_per_rot = actual.steer_rad_per_motor_rot

        ff = config.drive_feedforward
        self.drive = _plant(
            ff.ks_volts,
            ff.kv_volts_per_mps,
            ff.ka_volts_per_mps2,
            self.drive_m_per_rot,
            config.drive_motor.free_speed_rps,
            DRIVE_TAU_S,
        )
        sff = config.steer_feedforward
        self.steer = _plant(
            sff.ks_volts,
            sff.kv_volts_per_rad_per_s,
            sff.ka_volts_per_rad_per_s2,
            self.steer_rad_per_rot,
            config.steer_motor.free_speed_rps,
            STEER_TAU_S,
        )
        self.steer.position = corner_sim.initial_angle_rad / self.true_steer_rad_per_rot
        self.write_encoder()

    def substep(self, vbus: float, dt_s: float) -> None:
        """Run both motors for one short step at battery voltage ``vbus``."""
        self.drive.step(self.drive_sim.getAppliedOutput() * vbus, dt_s)
        self.steer.step(self.steer_sim.getAppliedOutput() * vbus, dt_s)
        # The SPARK MAX simulator wants the speed in its own (converted) units.
        _keep_encoder_resets(self.drive_sim)
        _keep_encoder_resets(self.steer_sim)
        self.drive_sim.iterate(self.drive.velocity * self.drive_m_per_rot, vbus, dt_s)
        self.steer_sim.iterate(self.steer.velocity * self.steer_rad_per_rot, vbus, dt_s)
        # The REV simulator's own current estimate doesn't know our motor
        # model (it can come back NaN), so report the plant's current instead.
        self.drive_sim.setMotorCurrent(self.drive.current_amps)
        self.steer_sim.setMotorCurrent(self.steer.current_amps)

    def write_encoder(self) -> None:
        """Tell the CANcoder where the wheel really points (before the config offset)."""
        true_rot = self.true_angle_rad / math.tau
        raw = wrap_rotations(true_rot - self.corner.encoder_offset_rot + self.corner_sim.encoder_offset_error_rot)
        self.encoder_sim.set_raw_position(raw)
        self._written_raw_rot = raw

    def encoder_caught_up(self) -> bool:
        """``True`` once the CANcoder reports the last position written to it."""
        reading = self._encoder_reading.refresh()
        return reading.status.is_ok() and abs(math.remainder(float(reading.value) - self._written_raw_rot, 1.0)) < 1e-3

    @property
    def true_angle_rad(self) -> float:
        """Where the wheel really points, -pi to pi, counterclockwise from forward."""
        return math.remainder(self.steer.position * self.true_steer_rad_per_rot, math.tau)

    @property
    def true_distance_m(self) -> float:
        """How far the wheel has really rolled, meters."""
        return self.drive.position * self.true_drive_m_per_rot

    @property
    def currents_amps(self) -> tuple[float, float]:
        """Drive and steer motor current, amps."""
        return abs(self.drive.current_amps), abs(self.steer.current_amps)


class SwervePhysics:
    """The simulated swerve drivetrain: motors, wheels, CANcoders, gyro and battery.

    Build it after the drivetrain, from the real devices the drivetrain made
    (``ModuleIOSpark``, ``AbsoluteEncoderIOCANcoder`` and ``GyroIONavX`` or
    ``GyroIOOnboard``).

    Args:
        drivetrain: The drivetrain the robot code built.
        corners: Optional :class:`CornerSim` per corner name, like
            ``{"frontLeft": CornerSim(initial_angle_rad=1.0)}``.
        gyro_drift_rad_per_s: Gyro drift, radians per second. 1 degree per
            minute is about 0.0003 rad/s.
        battery: ``True`` to sag the battery voltage as the motors draw
            current; ``False`` keeps it at 12 V.

    Attributes:
        true_pose: Where the robot really is. Odometry only knows the config,
            so a test that gives the sim the wrong gears sees the two drift
            apart.
        true_heading_rad: Which way the robot really faces, continuous
            (two turns left is 4 pi).
        gyro_connected: Set to ``False`` to unplug the gyro.
        battery_volts: The battery voltage this step.
    """

    def __init__(
        self,
        drivetrain: Drivetrain,
        corners: Mapping[str, CornerSim] | None = None,
        gyro_drift_rad_per_s: float = 0.0,
        battery: bool = True,
    ) -> None:
        corners = corners or {}
        config = drivetrain.config
        self.modules = [_ModulePhysics(m, config, corners.get(m.corner.name, CornerSim())) for m in drivetrain.modules]
        self._module_xy = [(m.corner.x_m, m.corner.y_m) for m in self.modules]
        self._last_distances = [m.true_distance_m for m in self.modules]
        self._gyro = _GyroWriter(drivetrain.gyro)
        self.gyro_drift_rad_per_s = gyro_drift_rad_per_s
        self.gyro_connected = True
        self.battery = battery
        self.battery_volts = NOMINAL_VOLTS
        self.true_pose = Pose()
        self.true_heading_rad = 0.0
        self._drift_rad = 0.0
        self._turned_rad = 0.0
        self._sensors(dt_s=0.0)

    def step(self, dt_s: float) -> None:
        """Move the simulated robot forward by ``dt_s`` seconds.

        Args:
            dt_s: Time to simulate, seconds (``robotpy sim`` passes about 0.02).
        """
        substeps = max(1, round(dt_s / SUBSTEP_S))
        sub_dt = dt_s / substeps
        for _ in range(substeps):
            for module in self.modules:
                module.substep(self.battery_volts, sub_dt)
        self._move_robot()
        self._update_battery()
        self._sensors(dt_s)

    def _move_robot(self) -> None:
        distances = [m.true_distance_m for m in self.modules]
        # Each wheel's real distance this step, treated as a "speed over one
        # second": forward kinematics then gives the robot's real motion.
        moves = [
            ModuleTarget(d - last, m.true_angle_rad)
            for d, last, m in zip(distances, self._last_distances, self.modules)
        ]
        self._last_distances = distances
        motion = forward_kinematics(moves, self._module_xy)
        self.true_pose = pose_exp(self.true_pose, motion, 1.0)
        self.true_heading_rad += motion.omega_rad_per_s
        self._turned_rad = motion.omega_rad_per_s

    def _update_battery(self) -> None:
        if not self.battery:
            return
        currents = [amps for m in self.modules for amps in m.currents_amps]
        self.battery_volts = BatterySim.calculate(currents)
        RoboRioSim.setVInVoltage(self.battery_volts)

    def _sensors(self, dt_s: float) -> None:
        for module in self.modules:
            module.write_encoder()
        self._drift_rad += self.gyro_drift_rad_per_s * dt_s
        rate = self._turned_rad / dt_s + self.gyro_drift_rad_per_s if dt_s > 0 else 0.0
        self._gyro.write(self.true_heading_rad + self._drift_rad, rate, self.gyro_connected)

    def wait_for_sensors(self, encoders: bool = False, timeout_s: float = 0.25) -> bool:
        """Wait (in real time) until the gyro, and optionally the CANcoders, report what was written.

        The NavX and CANcoder simulators deliver new readings from their own
        threads a few milliseconds later, like real CAN and SPI messages.
        ``robotpy sim`` runs in real time so it never needs this, but a test
        that runs faster than real time calls it after :meth:`step` so the
        robot code reads this step's values.

        Args:
            encoders: Also wait for the CANcoders. Only needed while the robot
                is disabled (seeding and re-seeding use them).
            timeout_s: Give up after this long, seconds.

        Returns:
            ``True`` if every sensor caught up in time.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            gyro_ok = not self.gyro_connected or self._gyro.caught_up()
            if gyro_ok and (not encoders or all(m.encoder_caught_up() for m in self.modules)):
                return True
            time.sleep(0.001)
        return False


class _GyroWriter:
    """Writes the robot's heading into whichever gyro simulator the robot code is reading.

    Args:
        gyro: The drivetrain's gyro IO: ``GyroIONavX`` (the NavX simulator,
            clockwise-positive degrees) or ``GyroIOOnboard`` (SystemCore's
            IMU simulator, RobotPy 2027).
    """

    def __init__(self, gyro) -> None:
        # Both gyro IOs flip by ``_sign`` when reading; flip by the same sign
        # when writing so the robot code reads the true heading.
        self._sign = gyro._sign
        self._written_deg = 0.0
        if hasattr(gyro, "ahrs"):
            from wpilib.simulation import SimDeviceSim

            device = SimDeviceSim(NAVX_SIM_DEVICE)
            self._connected = device.getBoolean("Connected")
            self._yaw = device.getDouble("Yaw")
            self._rate = device.getDouble("Rate")
            self._ahrs = gyro.ahrs
            self._imu = None
        else:
            from wpilib.simulation import OnboardIMUSim  # pyright: ignore[reportAttributeAccessIssue]

            self._imu = OnboardIMUSim()

    def write(self, heading_rad: float, rate_rad_per_s: float, connected: bool) -> None:
        if self._imu is not None:
            self._imu.setYaw(math.remainder(self._sign * heading_rad, math.tau))
            self._imu.setGyroRateZ(self._sign * rate_rad_per_s)
            return
        self._connected.set(connected)
        if connected:
            # The NavX reports yaw from -180 to 180 degrees.
            self._written_deg = math.degrees(math.remainder(self._sign * heading_rad, math.tau))
            self._yaw.set(self._written_deg)
            self._rate.set(math.degrees(self._sign * rate_rad_per_s))

    def caught_up(self) -> bool:
        """``True`` once the gyro reports the last heading written to it."""
        if self._imu is not None:
            return True  # SystemCore's IMU simulator answers straight away
        return abs(math.remainder(self._ahrs.getYaw() - self._written_deg, 360.0)) < 0.01


def _keep_encoder_resets(spark_sim: rev.SparkMaxSim) -> None:
    """Carry an encoder reset from the robot code into the SPARK MAX simulator.

    When the robot code calls ``encoder.setPosition()`` (seeding the steer
    angle, say), REV's simulator changes the encoder reading but keeps
    counting from its own old position on the next ``iterate()``, which
    would undo the reset. Copy the new reading into the simulator first.
    """
    encoder_position = spark_sim.getRelativeEncoderSim().getPosition()
    if encoder_position != spark_sim.getPosition():
        spark_sim.setPosition(encoder_position)


def _plant(
    ks: float,
    kv_per_unit: float | None,
    ka_per_unit: float | None,
    units_per_rot: float,
    free_speed_rps: float,
    tau_s: float,
) -> MotorPlant:
    """Build a motor's physics from measured SysId numbers, or from its free speed until then.

    Args:
        ks: Measured kS, volts.
        kv_per_unit: Measured kV per m/s (drive) or per rad/s (steer), or ``None``.
        ka_per_unit: Measured kA per m/s^2 or rad/s^2, or ``None``.
        units_per_rot: Meters or radians per motor rotation.
        free_speed_rps: The motor's free speed, rotations per second.
        tau_s: Time constant (kA / kV) to use when kA isn't measured.

    Returns:
        A :class:`~sim.motor_plant.MotorPlant` working in motor rotations.
    """
    if kv_per_unit is not None and ka_per_unit is not None:
        # SysId numbers are per meter (or radian) as the encoder reports it;
        # the plant works per motor rotation.
        return MotorPlant(kv=kv_per_unit * units_per_rot, ka=ka_per_unit * units_per_rot, ks=ks)
    kv = NOMINAL_VOLTS / free_speed_rps
    return MotorPlant(kv=kv, ka=kv * tau_s)
