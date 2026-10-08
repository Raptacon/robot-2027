"""
Types that describe one robot's swerve drive.

What is this file for?
    :mod:`config.module_presets` says what is true for every module of a kind.
    This file says what is true for *one robot*: where each module sits, the
    CAN ID of each motor and encoder, each encoder's offset, and the measured
    wheel size.

    Each robot gets its own file in ``config/robots/`` that builds a
    :class:`RobotConfig`. See ``config/robots/swerve_test_bot.py`` for a
    complete example.

Units:
    Distances are in meters and angles in radians, like the rest of WPILib.
    The one exception is encoder offsets, which are in *rotations*
    (1 rotation = 360 degrees), because that is what calibration tools print.

Coordinates:
    Positions are measured from the center of the robot. +x points toward the
    front of the robot and +y points to the robot's left. So the front-left
    module has a positive x and a positive y.

Why frozen?
    Every class here uses ``@dataclass(frozen=True)``, so a config can't be
    changed while the robot runs. To change a value, edit the robot's config
    file and commit it to git. That way the code on the robot always matches
    what is in git.
"""

import math
from dataclasses import dataclass

from config.module_presets import NEO, MotorSpec, ModulePreset
from utils.sysid.settings import SysIdSettings

CORNER_NAMES = ("frontLeft", "frontRight", "backLeft", "backRight")
"""Module names in the order WPILib expects: front-left, front-right, back-left, back-right.

Every list of four modules in our code uses this order.
"""


@dataclass(frozen=True)
class CornerConfig:
    """Settings for the module at one corner of the robot.

    You usually don't build these one at a time; :func:`mirrored_corners`
    builds all four. Build them by hand only if the modules aren't in a
    rectangle.

    Attributes:
        name: One of :data:`CORNER_NAMES`, like ``"frontLeft"``.
        x_m: How far the wheel is in front of the robot's center, in meters.
            Negative means behind the center.
        y_m: How far the wheel is to the left of the robot's center, in
            meters. Negative means to the right.
        drive_can_id: CAN ID of the drive motor's SparkMax.
        steer_can_id: CAN ID of the steer motor's SparkMax.
        encoder_can_id: CAN ID of the absolute encoder that tells us which way
            the wheel points.
        encoder_offset_rot: The number added to the encoder's raw reading so
            that 0 means "wheel pointing straight forward". In rotations,
            between -0.5 and 0.5. Measured with the steps in
            doc/swerve/tuning-and-calibration.md section 3.
        drive_inverted: ``True`` if the drive motor has to be reversed so a
            positive command rolls the wheel forward.

    Example:
        A module 0.3 m forward and 0.3 m left of center, with CAN IDs 50 to 52:

        >>> from config.robot_config import CornerConfig
        >>> fl = CornerConfig(
        ...     name="frontLeft",
        ...     x_m=0.3,
        ...     y_m=0.3,
        ...     drive_can_id=50,
        ...     steer_can_id=51,
        ...     encoder_can_id=52,
        ...     encoder_offset_rot=0.125,
        ...     drive_inverted=False,
        ... )
        >>> fl.can_ids
        (50, 51, 52)
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
        """This module's CAN IDs as (drive, steer, encoder)."""
        return (self.drive_can_id, self.steer_can_id, self.encoder_can_id)


@dataclass(frozen=True)
class DriveFeedforward:
    """How many volts the drive motor needs, worked out ahead of time.

    The motor controller's speed loop only corrects errors after they
    happen. A feedforward guesses the right voltage up front from the
    target speed, so the loop has little left to correct. The numbers come
    from the drive SysId test (on carpet, "SysId drive" on the test-mode
    Characterization chooser); until then kS is 0 and kV is worked out
    from the motor's free speed.

    voltage = kS x sign(speed) + kV x speed

    kA isn't used for driving yet. The simulator uses it (with kV) so the
    simulated wheels speed up like the real ones.

    Attributes:
        ks_volts: Volts needed just to overcome friction and start moving.
        kv_volts_per_mps: Volts per meter per second of wheel speed. ``None``
            means "use 12 V divided by the theoretical top speed".
        ka_volts_per_mps2: Volts per m/s^2 of wheel acceleration, with the
            robot's weight on the wheels. ``None`` until measured.

    Example:
        >>> from config.robot_config import DriveFeedforward
        >>> ff = DriveFeedforward(ks_volts=0.2, kv_volts_per_mps=2.5)
        >>> ff.volts(2.0, kv_volts_per_mps=ff.kv_volts_per_mps)  # 0.2 + 2.5 * 2.0
        5.2
    """

    ks_volts: float = 0.0
    kv_volts_per_mps: float | None = None
    ka_volts_per_mps2: float | None = None

    def volts(self, speed_mps: float, kv_volts_per_mps: float) -> float:
        """Feedforward voltage for a wheel speed, using the given kV."""
        if speed_mps == 0.0:
            return 0.0
        return math.copysign(self.ks_volts, speed_mps) + kv_volts_per_mps * speed_mps


@dataclass(frozen=True)
class SteerFeedforward:
    """Measured steer motor numbers from the steer SysId test (on blocks).

    The steer angle loop doesn't use these yet; they help pick the steer
    PID gains, and the simulator uses them so the simulated wheels turn
    like the real ones. All ``None`` until measured.

    The SysId app reports turning mechanisms per *rotation*. Divide its kV
    and kA by 2 pi (6.283) to get the per-radian numbers stored here. The
    dashboard estimate under ``/Characterization/steer/estimate/`` is
    already per radian.

    Attributes:
        ks_volts: Volts to overcome friction and start the wheel turning.
        kv_volts_per_rad_per_s: Volts per rad/s of steering speed.
        ka_volts_per_rad_per_s2: Volts per rad/s^2 of steering acceleration.

    Example:
        >>> from config.robot_config import SteerFeedforward
        >>> SteerFeedforward(ks_volts=0.15, kv_volts_per_rad_per_s=0.4).ka_volts_per_rad_per_s2 is None
        True
    """

    ks_volts: float = 0.0
    kv_volts_per_rad_per_s: float | None = None
    ka_volts_per_rad_per_s2: float | None = None


DRIVE_SYSID = SysIdSettings(ramp_volts_per_s=1.0, step_volts=6.0, timeout_s=5.0, dynamic_timeout_s=2.0)
"""Default drive SysId settings. At these values each test drives the robot up
to about 5 m, so start with the robot at one end of a long clear stretch of
carpet. The reverse tests drive it back. Disable at any time to stop."""

STEER_SYSID = SysIdSettings(ramp_volts_per_s=1.0, step_volts=4.0, timeout_s=6.0, dynamic_timeout_s=2.0)
"""Default steer SysId settings (robot on blocks, wheels off the ground)."""


@dataclass(frozen=True)
class SparkSettings:
    """Settings sent to every drive and steer SPARK MAX on each boot.

    The code configures each controller from its defaults every time the
    robot starts, so a swapped controller behaves exactly like the old one.
    These are starting values from doc/swerve/tuning-and-calibration.md
    section 2; tune from here and write down what you changed.

    The PID gains use our units, because the controllers are set up to
    count in meters and radians: a drive kP of 0.1 means "for each 1 m/s too
    slow, add 10% of full output (1.2 V)".

    Attributes:
        drive_current_limit_amps: Drive motor current limit. Lower it if the
            wheels slip or the battery browns out; raise it only if the robot
            accelerates too slowly.
        steer_current_limit_amps: Steer motor current limit.
        drive_kp: Drive speed loop gain, fraction of full output per m/s of error.
        steer_kp: Steer angle loop gain, fraction of full output per radian of error.
        steer_kd: Steer angle loop damping, fraction of full output per rad/s.
        encoder_period_ms: How often (ms) each controller sends its encoder
            position and speed. Odometry needs a fresh value every 20 ms loop.

    Example:
        A stiffer steer loop for one robot:

        >>> from config.robot_config import SparkSettings
        >>> SparkSettings(steer_kp=0.8).steer_kp
        0.8
    """

    drive_current_limit_amps: int = 40
    steer_current_limit_amps: int = 20
    drive_kp: float = 0.1
    steer_kp: float = 0.5
    steer_kd: float = 0.0
    encoder_period_ms: int = 20


@dataclass(frozen=True)
class RobotConfig:
    """Everything the drive code needs to know about one robot's swerve.

    Attributes:
        name: The robot's name. It must match the name set in NetworkTables
            at ``/robot/name`` (see :mod:`config.loader`).
        preset: Which modules and gears the robot has, like ``MK4I_L2``.
        corners: The four modules in :data:`CORNER_NAMES` order. Usually made
            with :func:`mirrored_corners`.
        wheel_radius_m: Wheel radius in meters. Start with
            ``NOMINAL_WHEEL_RADIUS_M``, then replace it with the measured
            value from the wheel radius test.
        drive_motor: The drive motor type. Defaults to a NEO.
        steer_motor: The steer motor type. Defaults to a NEO.
        gyro_can_id: The gyro's CAN ID, or ``None`` if the gyro isn't on CAN
            (for example the IMU built into SystemCore).
        gyro_inverted: ``True`` if the gyro reads clockwise as positive. WPILib
            expects counterclockwise to be positive.
        can_bus: Which CAN bus the drive is on. Empty means the default bus.
            Choosing a bus on SystemCore isn't supported yet.
        drive_feedforward: The drive motors' :class:`DriveFeedforward`.
            Replace with the drive SysId values.
        steer_feedforward: The steer motors' :class:`SteerFeedforward`,
            from the steer SysId test.
        spark: The :class:`SparkSettings` for every drive and steer controller.
        drive_sysid: How hard the drive SysId test pushes (:data:`DRIVE_SYSID`).
        steer_sysid: How hard the steer SysId test pushes (:data:`STEER_SYSID`).
        systemcore_imu_mount: How SystemCore is mounted on the robot, for its
            built-in IMU: ``"flat"``, ``"landscape"`` or ``"portrait"``
            (WPILib's ``OnboardIMU.MountOrientation``). Not used on a roboRIO.

    The properties below are worked out from these values, so they always
    agree with each other. Don't copy them into the config by hand.

    Example:
        >>> from config.robots.swerve_test_bot import CONFIG
        >>> CONFIG.preset.name
        'MK4i_L2'
        >>> round(CONFIG.free_speed_mps, 2)  # theoretical top speed, m/s
        4.47
        >>> round(CONFIG.drive_base_radius_m, 2)  # center to a wheel, m
        0.39
    """

    name: str
    preset: ModulePreset
    corners: tuple[CornerConfig, CornerConfig, CornerConfig, CornerConfig]
    wheel_radius_m: float
    drive_motor: MotorSpec = NEO
    steer_motor: MotorSpec = NEO
    gyro_can_id: int | None = None
    gyro_inverted: bool = False
    can_bus: str = ""
    drive_feedforward: DriveFeedforward = DriveFeedforward()
    steer_feedforward: SteerFeedforward = SteerFeedforward()
    spark: SparkSettings = SparkSettings()
    drive_sysid: SysIdSettings = DRIVE_SYSID
    steer_sysid: SysIdSettings = STEER_SYSID
    systemcore_imu_mount: str = "flat"

    @property
    def drive_m_per_motor_rot(self) -> float:
        """Meters the robot rolls for one drive motor rotation (preset gears plus this robot's wheel)."""
        return self.preset.drive_m_per_motor_rot(self.wheel_radius_m)

    @property
    def free_speed_mps(self) -> float:
        """Theoretical top speed in meters per second. Expect 80 to 90 percent of it in practice."""
        return self.preset.free_speed_mps(self.drive_motor, self.wheel_radius_m)

    @property
    def drive_kv_volts_per_mps(self) -> float:
        """Drive kV: the measured value if set, otherwise 12 V / theoretical top speed."""
        kv = self.drive_feedforward.kv_volts_per_mps
        return kv if kv is not None else 12.0 / self.free_speed_mps

    def drive_feedforward_volts(self, speed_mps: float) -> float:
        """Feedforward voltage for one wheel's target speed (kS and kV from the config)."""
        return self.drive_feedforward.volts(speed_mps, self.drive_kv_volts_per_mps)

    @property
    def drive_base_radius_m(self) -> float:
        """Distance in meters from the robot's center to the farthest wheel.

        When the robot spins in place, each wheel drives around a circle this
        size. It is used for the top spin rate and for PathPlanner.
        """
        return max(math.hypot(c.x_m, c.y_m) for c in self.corners)

    @property
    def max_angular_speed_rad_per_s(self) -> float:
        """Theoretical top spin rate in radians per second.

        A wheel at top speed going around a circle of radius r spins the
        robot at speed / r.
        """
        return self.free_speed_mps / self.drive_base_radius_m

    def all_can_ids(self) -> list[int]:
        """Every CAN ID this config uses (motors, encoders and gyro).

        The config tests use this to check that no two devices share an ID.
        """
        ids = [i for c in self.corners for i in c.can_ids]
        if self.gyro_can_id is not None:
            ids.append(self.gyro_can_id)
        return ids


def wrap_rotations(rot: float) -> float:
    """Wrap an angle in rotations into the range -0.5 to just under 0.5.

    Angles that differ by a whole rotation point the same way, so 0.75
    rotations (270 degrees) and -0.25 rotations (-90 degrees) are the same
    direction. Wrapping keeps every offset in one range so they are easy to
    compare and check.

    Args:
        rot: An angle in rotations, any size.

    Returns:
        The same direction as a number from -0.5 up to (but not including) 0.5.

    Example:
        >>> from config.robot_config import wrap_rotations
        >>> wrap_rotations(0.75)
        -0.25
        >>> wrap_rotations(1.25)  # a full turn plus a quarter turn
        0.25
    """
    # Shift by half a turn, keep the part after the whole turns, shift back.
    return (rot + 0.5) % 1.0 - 0.5


def mirrored_corners(
    x_m: float,
    y_m: float,
    first_can_id: int,
    offsets_rot: tuple[float, float, float, float],
    drive_inverted: tuple[bool, bool, bool, bool],
) -> tuple[CornerConfig, CornerConfig, CornerConfig, CornerConfig]:
    """Build all four corners for modules placed in a rectangle.

    Most swerve robots put a module at each corner of a rectangle (or a
    square, which is a rectangle too). Then you only need the front-left
    module's position: the others are the same distances, mirrored. x_m and
    y_m can be different, so the robot doesn't have to be square.

    If your modules aren't in a rectangle, skip this function and write the
    four :class:`CornerConfig` values out by hand.

    CAN IDs follow the team convention: each module uses 3 IDs in a row
    (drive, steer, encoder), starting at ``first_can_id`` for front-left and
    going front-left, front-right, back-left, back-right.

    Args:
        x_m: Distance in meters from the robot's center forward to the
            front-left wheel. Use a positive number.
        y_m: Distance in meters from the robot's center left to the
            front-left wheel. Use a positive number.
        first_can_id: The front-left drive motor's CAN ID (50 on our robots).
        offsets_rot: The four encoder offsets in rotations, in front-left,
            front-right, back-left, back-right order. Any value works; each is
            wrapped into -0.5 to 0.5 with :func:`wrap_rotations`.
        drive_inverted: Whether each drive motor is reversed, in the same
            order.

    Returns:
        The four corners as (front-left, front-right, back-left, back-right).

    Example:
        A robot with wheels 0.25 m forward/back and 0.3 m left/right of center:

        >>> from config.robot_config import mirrored_corners
        >>> fl, fr, bl, br = mirrored_corners(
        ...     x_m=0.25,
        ...     y_m=0.3,
        ...     first_can_id=50,
        ...     offsets_rot=(0.125, 0.25, 0.375, 0.75),
        ...     drive_inverted=(False, True, False, True),
        ... )
        >>> (br.name, br.x_m, br.y_m)  # back-right is behind and to the right
        ('backRight', -0.25, -0.3)
        >>> br.can_ids  # the 4th module uses IDs 59, 60, 61
        (59, 60, 61)
        >>> br.encoder_offset_rot  # 0.75 was wrapped
        -0.25
    """
    # (x sign, y sign) for FL, FR, BL, BR: front is +x, left is +y.
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
