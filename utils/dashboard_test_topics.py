"""Test topics for dashboard development (simulation only).

Publishes moving structs and enum structs under ``/DashTest/`` so a
dashboard (raptaboard, Elastic, AdvantageScope) has something to decode:

- ``OrbitPose`` (Pose2d): circles the field center once every ~6 s, facing
  along its path.
- ``OrbitPose3d`` (Pose3d): the same circle, bobbing up and down.
- ``BatteryVoltage`` (double): a battery resting near 12.6 V that sags
  to about 11.2 V for 2 s of every 8 (as if under load) and drains slowly
  (0.1 V a minute, back to full every 10 minutes), for a battery widget.
- ``CommandedSpeeds`` and ``MeasuredSpeeds`` (ChassisSpeeds, robot-relative):
  a scripted drive that goes forward and back, strafes and spins, and what a
  sluggish robot would do with it: the measured speeds lag 0.3 s behind and
  reach 90% of the speed and 80% of the spin, so a dashboard shows the gap.
  (The keyboard joystick can't spin the real sim drivetrain: rotation is on
  Xbox axis 4 and the keyboard joystick has three axes.)
- ``TumblePose`` (Pose3d): ``OrbitPose3d``'s place, yawing with it while
  it pitches (20 degrees each way) and rolls (15 degrees each way) back and
  forth, for a 3D pose view's axes.
- ``OrbitHeading`` (Rotation2d): ``OrbitPose``'s heading, for a gyro dial
  (counterclockwise, as WPILib counts).
- ``Gyro`` (a table with ``.type`` "Gyro" and ``Value``, as a Sendable
  gyro publishes): the same heading in degrees counted clockwise, as most
  gyros' ``getAngle`` counts, so a dashboard set to read it clockwise
  shows the same as ``OrbitHeading``.
- ``Fleet`` (Pose2d[]): three robots spaced evenly round the circle.
- ``OrbitPath`` (Pose2d[]): the orbit's circle, as a trajectory to draw.
- ``Fuel`` (Translation2d[]): balls lying on the field, a ring of them
  round the orbit (field positions).
- ``SeenFuel`` (Translation2d[]): the balls a front camera would see from
  ``OrbitPose`` (within 4 m and 35 degrees of straight ahead), relative to
  the robot (+X its front, +Y its left), as a camera reports them. Drawn
  relative to the robot, each should land on its ball in ``Fuel``.
- ``IntakeDeployed`` (boolean): out for 2 s of every 4, for a part drawn on
  the robot only while it's out.
- ``IntakeExtensionIn`` (double): how far the intake reaches, 0 to 12
  inches and back while it's out (inches, so a dashboard scales it).
- Reads ``/raptaboard/target`` (Pose2d) and ``/raptaboard/target_set``,
  the target a raptaboard Field sets with a click, and logs each new one.
- ``SeenTags`` (int[]): the 2026 field's AprilTags that Robot-2026's two
  cameras would see from ``OrbitPose``: in a camera's 70 degree FOV, within
  6 m, and facing it within 60 degrees (the camera visualizer's rules, in
  2D and without its line-of-sight check).
- ``ModuleStates`` (SwerveModuleState[]): four modules spinning.
- ``CyclingMode`` (DashTestMode): an enum that steps every second through
  kIdle, kIntake, kShoot, kClimb and then 7, a value with no name, so a
  dashboard's fallback for unknown enum values gets exercised too.
- ``SelectedMode`` (DashTestMode): published once as kIntake and never
  overwritten, so a dashboard can change it later; the robot logs changes.
- ``Blink`` (boolean): flips every second, for a Boolean Box.
- ``Toggle`` (boolean): published once as false and never overwritten, for
  a dashboard's Toggle Switch or Toggle Button to change; the robot logs
  changes.
- ``ToggleEcho`` (boolean): set every loop to the ``Toggle`` value the robot
  read, so a dashboard can show that the robot really got its write.
- ``/SmartDashboard/DashTest/Lift``: a WPILib Mechanism2d (3 by 2): an
  elevator rising and falling (0.4 to 1.2) with an arm on top swinging
  between -60 and 60 degrees, for a dashboard's Mechanism2d view.
- ``/SmartDashboard/DashTest/Auto mode`` and
  ``/SmartDashboard/DashTest/Test mode``: WPILib SendableChoosers, as a
  team would make for picking the auto routine (four options) and which
  test to run in test mode (two: Module check, Drive square), for a
  dashboard's Dropdown or Split Button Chooser or Enum Toggle; the robot
  logs the choice it reads when it changes. (A "/" in a putData key makes
  a sub-table, so they sit under SmartDashboard/DashTest.)
- ``ManualChooser``: a chooser made by hand from plain topics (no
  SendableChooser, not under SmartDashboard), to show a dashboard finds a
  chooser by its ``.type`` anywhere: ``.type`` ("String Chooser"),
  ``options``, ``default`` and ``active`` are published here, and the
  dashboard publishes ``selected``, which becomes ``active`` if it's an
  option; the robot logs changes.
- ``IntakeMode`` (string): a state robot code and dashboards both change.
  The robot steps it to the next of ``IntakeModeOptions`` (string[],
  published once) every 5 s, from whatever it is now, so a dashboard's
  change is picked up and carried on from; the robot logs each change.
- ``ArmLevel`` (int): published once as 0 and never overwritten, for an
  enum widget whose names (0 = Low, 1 = Mid, 2 = High) are listed in the
  dashboard; the robot logs changes.
- Alerts (WPILib ``Alert``, in the default group ``/SmartDashboard/Alerts``
  with the robot's own): a warning and an info that stay on, and an error
  that comes on for 3 s every 10 s, so a dashboard's Alerts widget shows a
  new error arriving and clearing.
- ``Setpoint`` (double): published once as 0.0 and never overwritten, for a
  dashboard's Number Slider to change; the robot logs changes.
- ``SetpointEcho`` (double): set every loop to the ``Setpoint`` value the
  robot read.
- ``Seconds`` (int), ``Sine`` (double) and ``ModeName`` (string, the name of
  ``CyclingMode``'s mode): plain values for a Text Display.

DashTestMode is a hand-written struct (no wpimath type has an enum field)::

    enum {kIdle=0, kIntake=1, kShoot=2, kClimb=3} uint8 mode;
    enum {kOk=0, kWarn=1, kFault=-1} int8 health;
    double elapsed_s

Not for competition code: robot.py only creates this in simulation.
"""

from __future__ import annotations

import logging
import math
import struct

import ntcore
import wpilib

from utils.alerts import error_alert, info_alert, warning_alert

from utils.wpimath_compat import ChassisVelocities, Pose2d, Rotation2d, SwerveModuleVelocity, Translation2d, make_module_velocity

try:  # RobotPy 2026 and earlier (wpimath_compat has no 3D types yet)
    from wpimath.geometry import Pose3d, Rotation3d, Translation3d
except ImportError:  # RobotPy 2027 moved geometry up into wpimath
    from wpimath import Pose3d, Rotation3d, Translation3d

PREFIX = "/DashTest/"
SPEEDS_LAG_S = 0.3
"""How far the scripted measured speeds lag behind the commanded ones (s)."""
SPEEDS_REACHED = 0.9
"""How much of the commanded speed the scripted measured speeds reach."""
SPIN_REACHED = 0.8
"""How much of the commanded spin the scripted measured speeds reach."""

# Field size used by WPILib's 2026 field (meters); only the center matters here.
FIELD_LENGTH_M = 16.54
FIELD_WIDTH_M = 8.07
CENTER = Translation2d(FIELD_LENGTH_M / 2, FIELD_WIDTH_M / 2)
ORBIT_RADIUS_M = 2.0
ORBIT_RATE_RAD_PER_S = 1.0

# Robot-2026's cameras (constants/robot_geometry.py): name, inches forward
# and left of the robot's center, which way it faces (degrees, 0 is
# forward, counterclockwise), and its horizontal FOV (degrees).
CAMERAS = [
    ("Side_Camera", -6.0, 12.0, 90.0, 70.0),
    ("Back_Camera", -9.0, 7.5, 180.0, 70.0),
]
TAG_RANGE_M = 6.0
TAG_FACING_DEG = 60.0
PATH_POINTS = 36
INTAKE_CYCLE_S = 4.0
INTAKE_OUT_S = 2.0
INTAKE_REACH_IN = 12.0
FUEL_COUNT = 10
FUEL_RING_M = 3.2
FUEL_VIEW_M = 4.0
FUEL_VIEW_DEG = 35.0

MODE_STRUCT = "DashTestMode"
MODE_SCHEMA = "enum {kIdle=0, kIntake=1, kShoot=2, kClimb=3} uint8 mode;enum {kOk=0, kWarn=1, kFault=-1} int8 health;double elapsed_s"
MODE_NAMES = {0: "kIdle", 1: "kIntake", 2: "kShoot", 3: "kClimb"}
# The cycle ends on 7, which has no name in the schema.
CYCLE = [0, 1, 2, 3, 7]
# Health follows the mode so both enum fields change: ok, ok, warn, fault, ok.
CYCLE_HEALTH = [0, 0, 1, -1, 0]
SELECTED_DEFAULT = 1  # kIntake
MANUAL_OPTIONS = ["Slow", "Medium", "Fast"]
INTAKE_MODES = ["Stowed", "Intaking", "Shooting"]
INTAKE_STEP_S = 5.0
ALERT_CYCLE_S = 10.0
ALERT_ON_S = 3.0
MANUAL_DEFAULT = "Medium"


def pack_mode(mode: int, health: int, elapsed_s: float) -> bytes:
    """Pack one DashTestMode value: uint8, int8, double, little-endian, no padding."""
    return struct.pack("<Bbd", mode, health, elapsed_s)


def unpack_mode(data: bytes) -> tuple[int, int, float]:
    """Read back a DashTestMode value packed by :func:`pack_mode`."""
    return struct.unpack("<Bbd", bytes(data[:10]))


class DashboardTestTopics:
    """Publishes the ``/DashTest/`` topics; call :meth:`periodic` every loop."""

    def __init__(self, inst: ntcore.NetworkTableInstance | None = None) -> None:
        self.inst = inst or ntcore.NetworkTableInstance.getDefault()
        inst = self.inst

        self.orbit = inst.getStructTopic(PREFIX + "OrbitPose", Pose2d).publish()
        self.orbit3d = inst.getStructTopic(PREFIX + "OrbitPose3d", Pose3d).publish()
        self.tumble = inst.getStructTopic(PREFIX + "TumblePose", Pose3d).publish()
        self.battery = inst.getDoubleTopic(PREFIX + "BatteryVoltage").publish()
        self.commanded_speeds = inst.getStructTopic(PREFIX + "CommandedSpeeds", ChassisVelocities).publish()
        self.measured_speeds = inst.getStructTopic(PREFIX + "MeasuredSpeeds", ChassisVelocities).publish()
        self.orbit_heading = inst.getStructTopic(PREFIX + "OrbitHeading", Rotation2d).publish()
        gyro = inst.getTable(PREFIX + "Gyro")
        self.gyro_type = gyro.getStringTopic(".type").publish()
        self.gyro_type.set("Gyro")
        self.gyro_value = gyro.getDoubleTopic("Value").publish()
        self.fleet = inst.getStructArrayTopic(PREFIX + "Fleet", Pose2d).publish()
        self.orbit_path = inst.getStructArrayTopic(PREFIX + "OrbitPath", Pose2d).publish()
        self.orbit_path.set(
            [self._orbit_pose(i * 2 * math.pi / PATH_POINTS) for i in range(PATH_POINTS + 1)]
        )
        self.seen_tags = inst.getIntegerArrayTopic(PREFIX + "SeenTags").publish()
        self.fuel_spots = [
            Translation2d(
                CENTER.X() + FUEL_RING_M * math.cos(i * 2 * math.pi / FUEL_COUNT),
                CENTER.Y() + FUEL_RING_M * math.sin(i * 2 * math.pi / FUEL_COUNT),
            )
            for i in range(FUEL_COUNT)
        ]
        self.fuel = inst.getStructArrayTopic(PREFIX + "Fuel", Translation2d).publish()
        self.fuel.set(self.fuel_spots)
        self.seen_fuel = inst.getStructArrayTopic(PREFIX + "SeenFuel", Translation2d).publish()
        self.intake_deployed = inst.getBooleanTopic(PREFIX + "IntakeDeployed").publish()
        self.target_sub = inst.getStructTopic("/raptaboard/target", Pose2d).subscribe(Pose2d())
        self.target_set_sub = inst.getBooleanTopic("/raptaboard/target_set").subscribe(False)
        self._last_target = None
        self.intake_extension = inst.getDoubleTopic(PREFIX + "IntakeExtensionIn").publish()
        self.tags = self._load_tags()
        self.modules = inst.getStructArrayTopic(PREFIX + "ModuleStates", SwerveModuleVelocity).publish()

        inst.addSchema("struct:" + MODE_STRUCT, "structschema", MODE_SCHEMA)
        self.cycling = inst.getRawTopic(PREFIX + "CyclingMode").publish("struct:" + MODE_STRUCT)

        self.blink = inst.getBooleanTopic(PREFIX + "Blink").publish()
        self.seconds = inst.getIntegerTopic(PREFIX + "Seconds").publish()
        self.sine = inst.getDoubleTopic(PREFIX + "Sine").publish()
        self.mode_name = inst.getStringTopic(PREFIX + "ModeName").publish()

        selected = inst.getRawTopic(PREFIX + "SelectedMode")
        self.selected_pub = selected.publish("struct:" + MODE_STRUCT)
        self.selected_pub.setDefault(pack_mode(SELECTED_DEFAULT, 0, 0.0))
        self.selected_sub = selected.subscribe("struct:" + MODE_STRUCT, b"")
        self._last_selected: bytes | None = None

        toggle = inst.getBooleanTopic(PREFIX + "Toggle")
        self.toggle_pub = toggle.publish()
        self.toggle_pub.setDefault(False)
        self.toggle_sub = toggle.subscribe(False)
        self.toggle_echo = inst.getBooleanTopic(PREFIX + "ToggleEcho").publish()
        self._last_toggle: bool | None = None

        setpoint = inst.getDoubleTopic(PREFIX + "Setpoint")
        self.setpoint_pub = setpoint.publish()
        self.setpoint_pub.setDefault(0.0)
        self.setpoint_sub = setpoint.subscribe(0.0)
        self.setpoint_echo = inst.getDoubleTopic(PREFIX + "SetpointEcho").publish()
        self._last_setpoint: float | None = None

        self.auto_chooser = wpilib.SendableChooser()
        self.auto_chooser.setDefaultOption("Do nothing", "Do nothing")
        for name in ("Taxi", "Two piece", "Three piece"):
            self.auto_chooser.addOption(name, name)
        wpilib.SmartDashboard.putData("DashTest/Auto mode", self.auto_chooser)
        self.test_chooser = wpilib.SendableChooser()
        self.test_chooser.setDefaultOption("Module check", "Module check")
        self.test_chooser.addOption("Drive square", "Drive square")
        wpilib.SmartDashboard.putData("DashTest/Test mode", self.test_chooser)
        self.lift = wpilib.Mechanism2d(3.0, 2.0, wpilib.Color8Bit(0, 0, 32))
        lift_base = self.lift.getRoot("base", 1.5, 0.2)
        self.elevator = lift_base.appendLigament("elevator", 0.8, 90.0, 6.0, wpilib.Color8Bit(255, 0, 0))
        self.arm = self.elevator.appendLigament("arm", 0.5, 0.0, 4.0, wpilib.Color8Bit(0, 255, 0))
        wpilib.SmartDashboard.putData("DashTest/Lift", self.lift)
        self._last_choices: tuple[str, str] | None = None

        # A chooser by hand: the same topics a SendableChooser publishes.
        # ".type" is an ordinary string topic; the leading dot only marks it
        # as metadata that dashboards hide.
        manual = inst.getTable(PREFIX + "ManualChooser")
        self.manual_type = manual.getStringTopic(".type").publish()
        self.manual_type.set("String Chooser")
        self.manual_options = manual.getStringArrayTopic("options").publish()
        self.manual_options.set(MANUAL_OPTIONS)
        self.manual_default = manual.getStringTopic("default").publish()
        self.manual_default.set(MANUAL_DEFAULT)
        self.manual_active = manual.getStringTopic("active").publish()
        self.manual_active.set(MANUAL_DEFAULT)
        # Read only: the dashboard publishes "selected".
        self.manual_selected = manual.getStringTopic("selected").subscribe("")
        self.manual_choice = MANUAL_DEFAULT

        # Two-way state: published and read back, so the newest value wins
        # whether the robot or a dashboard set it.
        intake = inst.getStringTopic(PREFIX + "IntakeMode")
        self.intake_pub = intake.publish()
        self.intake_pub.set(INTAKE_MODES[0])
        self.intake_sub = intake.subscribe(INTAKE_MODES[0])
        self.intake_options = inst.getStringArrayTopic(PREFIX + "IntakeModeOptions").publish()
        self.intake_options.set(INTAKE_MODES)
        self._intake_last = INTAKE_MODES[0]
        self._intake_next_step_s: float | None = None
        arm = inst.getIntegerTopic(PREFIX + "ArmLevel")
        self.arm_pub = arm.publish()
        self.arm_pub.setDefault(0)
        self.arm_sub = arm.subscribe(0)
        self._arm_last = 0

        self.test_warning = warning_alert("DashTest: battery is simulated")
        self.test_warning.set(True)
        self.test_info = info_alert("DashTest: test topics are on")
        self.test_info.set(True)
        self.test_error = error_alert("DashTest: pretend encoder fault (3 s every 10 s)")

    def periodic(self, now_s: float) -> None:
        """Update every moving topic for time ``now_s`` (seconds)."""
        angle = now_s * ORBIT_RATE_RAD_PER_S
        self.orbit.set(self._orbit_pose(angle))
        bob_m = 0.5 + 0.25 * math.sin(now_s * 2.0)
        p = self._orbit_pose(angle)
        self.orbit3d.set(
            Pose3d(
                Translation3d(p.X(), p.Y(), bob_m),
                Rotation3d(0.0, 0.0, p.rotation().radians()),
            )
        )
        self.tumble.set(
            Pose3d(
                Translation3d(p.X(), p.Y(), bob_m),
                Rotation3d(
                    math.radians(15.0) * math.sin(now_s * 1.3),
                    math.radians(20.0) * math.sin(now_s),
                    p.rotation().radians(),
                ),
            )
        )
        self.battery.set(self._battery_volts(now_s))
        commanded = self._scripted_speeds(now_s)
        self.commanded_speeds.set(ChassisVelocities(*commanded))
        late = self._scripted_speeds(now_s - SPEEDS_LAG_S)
        self.measured_speeds.set(
            ChassisVelocities(late[0] * SPEEDS_REACHED, late[1] * SPEEDS_REACHED, late[2] * SPIN_REACHED)
        )
        self.orbit_heading.set(p.rotation())
        self.gyro_value.set(-p.rotation().degrees())
        self.fleet.set([self._orbit_pose(angle + i * 2 * math.pi / 3) for i in range(3)])
        self.seen_tags.set(self._seen_tags(p))
        self.seen_fuel.set(self._seen_fuel(p))
        self._log_target()
        into = now_s % INTAKE_CYCLE_S
        out = into < INTAKE_OUT_S
        self.intake_deployed.set(out)
        self.intake_extension.set(
            INTAKE_REACH_IN * math.sin(math.pi * into / INTAKE_OUT_S) if out else 0.0
        )
        self.modules.set([make_module_velocity(2.0, angle + i * math.pi / 2) for i in range(4)])

        step = int(now_s) % len(CYCLE)
        self.cycling.set(pack_mode(CYCLE[step], CYCLE_HEALTH[step], now_s))
        self.mode_name.set(MODE_NAMES.get(CYCLE[step], f"unnamed ({CYCLE[step]})"))

        self.blink.set(int(now_s) % 2 == 0)
        self.seconds.set(int(now_s))
        self.sine.set(math.sin(now_s))

        self._log_selected_changes()
        self._echo_toggle()
        self._echo_setpoint()
        self._log_choices()
        self._take_manual_choice()
        self._step_intake(now_s)
        self._log_arm_level()
        self.test_error.set(now_s % ALERT_CYCLE_S < ALERT_ON_S)
        self.elevator.setLength(0.8 + 0.4 * math.sin(now_s * 0.7))
        self.arm.setAngle(60.0 * math.sin(now_s * 1.1))

    @staticmethod
    def _battery_volts(t: float) -> float:
        """The scripted battery at time ``t`` (s): 12.6 V resting, draining
        0.1 V a minute (full again every 10 minutes), sagging 1.4 V under a
        2 s load every 8 s."""
        resting = 12.6 - 0.1 * ((t % 600.0) / 60.0)
        load = 1.4 if t % 8.0 < 2.0 else 0.0
        return resting - load

    @staticmethod
    def _scripted_speeds(t: float) -> tuple[float, float, float]:
        """The scripted drive at time ``t`` (s): forward/back up to 2 m/s,
        strafing up to 1 m/s and spinning up to 1.5 rad/s, each on its own
        slow wave so the arrow wanders round."""
        return (2.0 * math.sin(0.4 * t), 1.0 * math.sin(0.25 * t), 1.5 * math.sin(0.3 * t))

    @staticmethod
    def _load_tags() -> list[tuple[int, float, float, float]]:
        """The 2026 field's AprilTags as (id, x, y, yaw radians), blue-corner frame."""
        try:
            import robotpy_apriltag

            layout = robotpy_apriltag.AprilTagFieldLayout.loadField(
                robotpy_apriltag.AprilTagField.k2026RebuiltWelded
            )
        except Exception:  # no AprilTag library: no tags seen
            return []
        return [
            (t.ID, t.pose.X(), t.pose.Y(), t.pose.rotation().Z())
            for t in layout.getTags()
        ]

    def _log_target(self) -> None:
        """Log the dashboard's target when it's set, moved or cleared."""
        target = self.target_sub.get() if self.target_set_sub.get() else None
        key = None if target is None else (round(target.X(), 2), round(target.Y(), 2), round(target.rotation().degrees()))
        if key != self._last_target:
            if key is None:
                logging.info("DashTest target cleared")
            else:
                logging.info("DashTest target: x %.2f m, y %.2f m, facing %d deg", *key)
            self._last_target = key

    def _seen_fuel(self, robot: Pose2d) -> list[Translation2d]:
        """The balls in view ahead of ``robot``, relative to it."""
        heading = robot.rotation().radians()
        seen = []
        for ball in self.fuel_spots:
            dx, dy = ball.X() - robot.X(), ball.Y() - robot.Y()
            # Into the robot's frame: +X its front, +Y its left.
            fwd = dx * math.cos(heading) + dy * math.sin(heading)
            left = -dx * math.sin(heading) + dy * math.cos(heading)
            if math.hypot(fwd, left) <= FUEL_VIEW_M and abs(math.atan2(left, fwd)) <= math.radians(FUEL_VIEW_DEG):
                seen.append(Translation2d(fwd, left))
        return seen

    def _seen_tags(self, robot: Pose2d) -> list[int]:
        """The tag IDs any camera would see from ``robot``, in ID order."""
        seen = set()
        heading = robot.rotation().radians()
        for _name, x_in, y_in, yaw_deg, fov_deg in CAMERAS:
            fx, fy = x_in * 0.0254, y_in * 0.0254
            cx = robot.X() + fx * math.cos(heading) - fy * math.sin(heading)
            cy = robot.Y() + fx * math.sin(heading) + fy * math.cos(heading)
            facing = heading + math.radians(yaw_deg)
            for tag_id, tx, ty, tyaw in self.tags:
                dx, dy = tx - cx, ty - cy
                if math.hypot(dx, dy) > TAG_RANGE_M:
                    continue
                off = math.atan2(dy, dx) - facing
                off = math.atan2(math.sin(off), math.cos(off))
                if abs(off) > math.radians(fov_deg / 2):
                    continue
                # The tag faces the camera: its normal points back along the view.
                back = math.atan2(-dy, -dx) - tyaw
                back = math.atan2(math.sin(back), math.cos(back))
                if abs(back) > math.radians(TAG_FACING_DEG):
                    continue
                seen.add(tag_id)
        return sorted(seen)

    @staticmethod
    def _orbit_pose(angle: float) -> Pose2d:
        """A pose on the circle round the field center, facing along the path."""
        return Pose2d(
            CENTER.X() + ORBIT_RADIUS_M * math.cos(angle),
            CENTER.Y() + ORBIT_RADIUS_M * math.sin(angle),
            Rotation2d(angle + math.pi / 2),
        )

    def _echo_toggle(self) -> None:
        on = self.toggle_sub.get()
        self.toggle_echo.set(on)
        if self._last_toggle is not None and on != self._last_toggle:
            logging.info("DashTest Toggle is now %s", on)
        self._last_toggle = on

    def _echo_setpoint(self) -> None:
        value = self.setpoint_sub.get()
        self.setpoint_echo.set(value)
        if self._last_setpoint is not None and value != self._last_setpoint:
            logging.info("DashTest Setpoint is now %s", value)
        self._last_setpoint = value

    def _take_manual_choice(self) -> None:
        """Use the dashboard's pick if it's one of the options, as
        SendableChooser does, and publish it as the active choice."""
        picked = self.manual_selected.get()
        if picked in MANUAL_OPTIONS and picked != self.manual_choice:
            logging.info("DashTest ManualChooser is now %s", picked)
            self.manual_choice = picked
        self.manual_active.set(self.manual_choice)

    def _step_intake(self, now_s: float) -> None:
        """Log changes to IntakeMode, and step it to the next mode every
        INTAKE_STEP_S seconds, from whatever it is now."""
        mode = self.intake_sub.get()
        if mode != self._intake_last:
            logging.info("DashTest IntakeMode is now %s", mode)
            self._intake_last = mode
        if self._intake_next_step_s is None:
            self._intake_next_step_s = now_s + INTAKE_STEP_S
        if now_s >= self._intake_next_step_s:
            i = INTAKE_MODES.index(mode) if mode in INTAKE_MODES else -1
            self.intake_pub.set(INTAKE_MODES[(i + 1) % len(INTAKE_MODES)])
            self._intake_next_step_s = now_s + INTAKE_STEP_S

    def _log_arm_level(self) -> None:
        level = self.arm_sub.get()
        if level != self._arm_last:
            logging.info("DashTest ArmLevel is now %s", level)
            self._arm_last = level

    def _log_choices(self) -> None:
        choices = (self.auto_chooser.getSelected(), self.test_chooser.getSelected())
        if self._last_choices is not None and choices != self._last_choices:
            logging.info("DashTest choosers now auto=%s test=%s", *choices)
        self._last_choices = choices

    def _log_selected_changes(self) -> None:
        data = bytes(self.selected_sub.get())
        if len(data) < 10 or data == self._last_selected:
            return
        if self._last_selected is not None:
            mode, health, _ = unpack_mode(data)
            logging.info("DashTest SelectedMode is now %s", MODE_NAMES.get(mode, mode))
        self._last_selected = data
