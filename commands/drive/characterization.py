"""
Swerve characterization: drive and steer SysId, wheel radius, and the steer step test.

Everything here runs in **test mode**, picked on the dashboard's
``Characterization`` chooser (:mod:`utils.sysid.chooser`). Pick a test,
enable Test, then **hold A** on the driver controller to run it. Let go of A
to stop at once (B and the left bumper step through the chooser).

The tests, and where the robot must be:

==========================  ==================  =========================================
Chooser option              Robot               What it measures
==========================  ==================  =========================================
SysId drive: ...            on carpet, clear    drive kS, kV, kA (wheels locked straight)
Drive feedforward (slow)    on carpet, clear    drive kS, kV only, from a very slow ramp
SysId steer: ...            on blocks           steer kS, kV, kA
Wheel radius                on carpet           real wheel radius (spins slowly in place)
Steer step test             on blocks           how fast and cleanly the wheels turn 90 deg
==========================  ==================  =========================================

Each SysId mechanism's ramp, step and timeouts can be changed on the
dashboard under ``/Characterization/<name>/settings/``, or loaded from the
``SysId <name> preset`` chooser (see :mod:`utils.sysid.tunable`).

Where the results go:
    - SysId: open the wpilog in the WPILib SysId app ("General Mechanism",
      the ``drive`` or ``steer`` motor) and copy kS, kV and kA into
      ``drive_feedforward`` or ``steer_feedforward`` in the robot's config file.
      A quick estimate also shows on the dashboard under
      ``/Characterization/<drive or steer>/estimate/``.
    - Drive feedforward (slow): ``/Characterization/driveSlowRamp/estimate/``
      has kS and kV straight away, no SysId app needed. Many top teams
      (6328 among them) use this for drive kS and kV and only use the SysId
      app for kA. If it and the SysId app disagree a lot, look at the data
      before trusting either.
    - Wheel radius: ``/Characterization/WheelRadius/radius_m``; copy it into
      ``wheel_radius_m``.
    - Steer step test: ``/Characterization/SteerStep/<corner>/``; use it to
      tune ``steer_kp`` and ``steer_kd`` in ``SparkSettings``. SysId only
      measures the steer feedforward. The SysId app's feedback panel can
      suggest a starting kP; check it with this test.

    Log every change in doc/swerve/calibration-log.md.

Example:
    These tests are for the real robot; ``robot.py`` registers them on the
    real drivetrain. This example uses the simulated drivetrain only so it
    can run in CI without hardware.

    >>> import commands2
    >>> from config.robots.swerve_test_bot import CONFIG
    >>> from commands.drive.characterization import register_swerve
    >>> from subsystem.drivetrain.drivetrain_sim import DrivetrainSim
    >>> from utils.sysid.chooser import CharacterizationChooser
    >>> sim = DrivetrainSim(CONFIG)
    >>> chooser = CharacterizationChooser("Do nothing", commands2.InstantCommand())
    >>> tests = register_swerve(chooser, sim.drivetrain)
    >>> sorted(tests)
    ['drive', 'driveSlowRamp', 'steer']
    >>> "Wheel radius" in chooser.names
    True
"""

import dataclasses
import logging
import math
import statistics

import commands2
import ntcore

from subsystem.drivetrain.drivetrain import Drivetrain
from subsystem.drivetrain.io.gyro_io_onboard import YawUnwrapper
from subsystem.drivetrain.swerve_math import ChassisSpeeds
from utils.sysid.characterizable import Characterizable, Reading
from utils.sysid.mech.drivetrain import drivetrain as mech_drivetrain
from utils.sysid.mech.rotary import rotary
from utils.sysid.settings import SysIdSettings
from utils.sysid.chooser import CharacterizationChooser
from utils.sysid.routines import SysIdTests

log = logging.getLogger(__name__)


# -- SysId mechanisms ---------------------------------------------------------

MAX_TRAVEL_M = 5.0
"""Every drive SysId test stops after the robot moves this far, meters.
Change it to fit the clear carpet you have, leaving a meter spare."""


def drive_mechanism(drivetrain: Drivetrain) -> Characterizable:
    """The drive motors as one SysId mechanism, all wheels pointed straight forward.

    The reading is the average of the four drive motors, in meters and m/s.

    Args:
        drivetrain: The drivetrain.

    Returns:
        A :class:`~utils.sysid.characterizable.Characterizable` named ``"drive"``.
    """

    def read() -> Reading:
        inputs = [m.inputs for m in drivetrain.modules]
        return Reading(
            volts=statistics.fmean(i.drive_applied_volts for i in inputs),
            position=statistics.fmean(i.drive_position_m for i in inputs),
            velocity=statistics.fmean(i.drive_velocity_mps for i in inputs),
        )

    return mech_drivetrain(
        "drive",
        subsystem=drivetrain,
        set_voltage=lambda volts: drivetrain.run_drive_volts(volts, 0.0),
        read=read,
        max_travel_m=MAX_TRAVEL_M,
        settings=drivetrain.config.drive_sysid,
    )


def steer_mechanism(drivetrain: Drivetrain) -> Characterizable:
    """The steer motors as one SysId mechanism (robot on blocks).

    The reading is the average of the four steer motors, in radians and
    rad/s. Each wheel's angle is unwrapped so it keeps counting past half a
    turn instead of jumping from +180 to -180 degrees.

    Args:
        drivetrain: The drivetrain.

    Returns:
        A :class:`~utils.sysid.characterizable.Characterizable` named ``"steer"``.
    """
    unwrappers = [YawUnwrapper() for _ in drivetrain.modules]

    def read() -> Reading:
        inputs = [m.inputs for m in drivetrain.modules]
        angles = [u.update(i.steer_angle_rad) for u, i in zip(unwrappers, inputs)]
        return Reading(
            volts=statistics.fmean(i.steer_applied_volts for i in inputs),
            position=statistics.fmean(angles),
            velocity=statistics.fmean(i.steer_velocity_rad_per_s for i in inputs),
        )

    # Swerve steering turns forever, so it has no limits.
    return rotary(
        "steer",
        subsystem=drivetrain,
        set_voltage=drivetrain.run_steer_volts,
        read=read,
        settings=drivetrain.config.steer_sysid,
    )


SLOW_RAMP_SETTINGS = SysIdSettings(ramp_volts_per_s=0.1, timeout_s=15.0)
"""The slow drive ramp: 0.1 V/s for up to 15 s (team 6328's ramp rate).

The robot creeps forward a few meters. At this ramp it barely accelerates,
so kS and kV come out clean, but kA can't be measured."""


def slow_ramp_drive_mechanism(drivetrain: Drivetrain) -> Characterizable:
    """The drive motors, set up for the slow-ramp kS and kV test.

    Same as :func:`drive_mechanism` but named ``"driveSlowRamp"`` (so its
    estimate and log are kept apart) and using :data:`SLOW_RAMP_SETTINGS`.

    Args:
        drivetrain: The drivetrain.

    Returns:
        A :class:`~utils.sysid.characterizable.Characterizable`.
    """
    return dataclasses.replace(drive_mechanism(drivetrain), name="driveSlowRamp", settings=SLOW_RAMP_SETTINGS)


# -- Wheel radius -------------------------------------------------------------


class WheelRadiusCharacterization(commands2.Command):
    """Spin in place and work out the real wheel radius from the gyro.

    When the robot spins in place, each wheel rolls around a circle whose
    size we know from the config (the module positions). The gyro says how
    far the robot turned, so we know how far each wheel really rolled. The
    drive encoder says how many times the wheel turned. Distance divided by
    wheel turns (in radians) is the wheel radius.

    Run it on carpet, since tread sinks in a little and that is the radius
    that matters. Worn tread reads smaller: re-run it when tread is replaced.

    Args:
        drivetrain: The drivetrain.
        spin_rad_per_s: How fast to spin, rad/s. Slow keeps the wheels from
            slipping; 0.25 rad/s is what team 6328 uses.
        turns: How many full turns to measure over.

    Attributes:
        radius_m: The measured wheel radius in meters, or ``None`` until a run finishes.
    """

    RAMP_RAD_PER_S2 = 0.05
    """How fast the spin speeds up, rad/s per second, so the wheels don't slip at the start."""

    SETTLE_S = 1.0
    """Seconds at full spin speed before measuring starts."""

    def __init__(self, drivetrain: Drivetrain, spin_rad_per_s: float = 0.25, turns: float = 1.0) -> None:
        super().__init__()
        self.drivetrain = drivetrain
        self.spin_rad_per_s = spin_rad_per_s
        self.turns = turns
        self.radius_m: float | None = None
        self._omega = 0.0
        self._at_speed_s = 0.0
        self._start_heading: float | None = None
        self._start_distances: list[float] = []
        table = ntcore.NetworkTableInstance.getDefault().getTable("/Characterization/WheelRadius")
        self._radius_pub = table.getDoubleTopic("radius_m").publish()
        self._radius_in_pub = table.getDoubleTopic("radius_in").publish()
        self.addRequirements(drivetrain)

    def initialize(self) -> None:
        self._omega = 0.0
        self._at_speed_s = 0.0
        self._start_heading = None

    def execute(self) -> None:
        dt = self.drivetrain.loop_period_s
        self._omega = min(self.spin_rad_per_s, self._omega + self.RAMP_RAD_PER_S2 * dt)
        self.drivetrain.drive(ChassisSpeeds(0.0, 0.0, self._omega))
        if self._omega < self.spin_rad_per_s:
            return
        self._at_speed_s += dt
        if self._start_heading is None and self._at_speed_s >= self.SETTLE_S:
            self._start_heading = self.drivetrain.heading_rad
            self._start_distances = [m.distance_m for m in self.drivetrain.modules]

    def isFinished(self) -> bool:
        if self._start_heading is None:
            return False
        return abs(self.drivetrain.heading_rad - self._start_heading) >= self.turns * math.tau

    def end(self, interrupted: bool) -> None:
        self.drivetrain.stop()
        if self._start_heading is None:
            log.warning("Wheel radius: stopped before measuring started")
            return
        turned_rad = abs(self.drivetrain.heading_rad - self._start_heading)
        if turned_rad < math.pi / 2:
            log.warning("Wheel radius: only turned %.0f degrees, not enough to measure", math.degrees(turned_rad))
            return
        config = self.drivetrain.config
        radii = []
        for module, start, corner in zip(self.drivetrain.modules, self._start_distances, config.corners):
            # The encoder counts meters using the config's wheel radius, so
            # this undoes that to get how far the wheel turned, in radians.
            wheel_turned_rad = abs(module.distance_m - start) / config.wheel_radius_m
            rolled_m = turned_rad * math.hypot(corner.x_m, corner.y_m)
            radii.append(rolled_m / wheel_turned_rad)
        self.radius_m = statistics.fmean(radii)
        self._radius_pub.set(self.radius_m)
        self._radius_in_pub.set(self.radius_m / 0.0254)
        log.info(
            "Wheel radius: %.5f m (%.4f in) over %.1f turns; config has %.5f m",
            self.radius_m,
            self.radius_m / 0.0254,
            turned_rad / math.tau,
            config.wheel_radius_m,
        )


# -- Steer step test ------------------------------------------------------------


class SteerStepTest(commands2.Command):
    """Turn every wheel 90 degrees and back a few times, and measure how it responds.

    Run it on blocks. For each corner it publishes, under
    ``/Characterization/SteerStep/<corner>/``:

    - ``settleTime_s``: the slowest time to get within :data:`TOLERANCE_DEG`
      of the target (the wheel stays there from then on). Lower is better.
    - ``overshoot_deg``: the most the wheel went past the target. A few
      degrees is fine; more means steer kP is too high or kD too low.

    Raise ``steer_kp`` until the settle time stops improving or overshoot
    appears, then add a little ``steer_kd`` to calm the overshoot.

    Args:
        drivetrain: The drivetrain.

    Attributes:
        settle_time_s: Slowest settle time per corner name, seconds
            (``inf`` if a wheel never got there).
        overshoot_deg: Largest overshoot per corner name, degrees.
    """

    STEP_DEG = 90.0
    """How far each step turns, degrees."""

    STEP_S = 1.0
    """Seconds for each step."""

    STEPS = 4
    """Number of steps: 90, 0, 90, 0 degrees."""

    TOLERANCE_DEG = 2.0
    """A wheel counts as there when it is within this many degrees of the target."""

    def __init__(self, drivetrain: Drivetrain) -> None:
        super().__init__()
        self.drivetrain = drivetrain
        self.settle_time_s: dict[str, float] = {}
        self.overshoot_deg: dict[str, float] = {}
        self._elapsed_s = 0.0
        self._step = -1
        self._step_start_s = 0.0
        self._direction: list[float] = []
        self._settled_at: list[float | None] = []
        self._step_settle: list[list[float]] = []
        self._step_overshoot: list[float] = []
        nt = ntcore.NetworkTableInstance.getDefault()
        self._pubs = {
            c.name: (
                nt.getDoubleTopic(f"/Characterization/SteerStep/{c.name}/settleTime_s").publish(),
                nt.getDoubleTopic(f"/Characterization/SteerStep/{c.name}/overshoot_deg").publish(),
            )
            for c in drivetrain.config.corners
        }
        self.addRequirements(drivetrain)

    def target_rad(self, step: int) -> float:
        """The target angle for step number ``step`` (0, 1, 2, ...), radians."""
        return math.radians(self.STEP_DEG if step % 2 == 0 else 0.0)

    def initialize(self) -> None:
        count = len(self.drivetrain.modules)
        self._elapsed_s = 0.0
        self._step = -1
        self._step_settle = [[] for _ in range(count)]
        self._step_overshoot = [0.0] * count

    def execute(self) -> None:
        step = int(self._elapsed_s // self.STEP_S)
        if step >= self.STEPS:
            return
        if step != self._step:
            self._finish_step()
            self._start_step(step)
        target = self.target_rad(step)
        tolerance = math.radians(self.TOLERANCE_DEG)
        t = self._elapsed_s - self._step_start_s
        for i, module in enumerate(self.drivetrain.modules):
            module.set_angle(target, short_way=False)
            error = math.remainder(target - module.angle_rad, math.tau)
            past_target = -error * self._direction[i]
            self._step_overshoot[i] = max(self._step_overshoot[i], math.degrees(past_target))
            if abs(error) <= tolerance:
                if self._settled_at[i] is None:
                    self._settled_at[i] = t
            else:
                self._settled_at[i] = None
        self._elapsed_s += self.drivetrain.loop_period_s

    def _start_step(self, step: int) -> None:
        self._step = step
        self._step_start_s = self._elapsed_s
        target = self.target_rad(step)
        self._direction = [
            math.copysign(1.0, math.remainder(target - m.angle_rad, math.tau)) for m in self.drivetrain.modules
        ]
        self._settled_at = [None] * len(self.drivetrain.modules)

    def _finish_step(self) -> None:
        if self._step < 0:
            return
        for i, settled in enumerate(self._settled_at):
            self._step_settle[i].append(math.inf if settled is None else settled)

    def isFinished(self) -> bool:
        return self._elapsed_s >= self.STEPS * self.STEP_S

    def end(self, interrupted: bool) -> None:
        self.drivetrain.stop()
        if not interrupted:
            self._finish_step()
        for i, corner in enumerate(self.drivetrain.config.corners):
            # The first step starts wherever the wheel was, so leave it out.
            measured = self._step_settle[i][1:] or self._step_settle[i]
            if not measured:
                continue
            settle = max(measured)
            overshoot = max(0.0, self._step_overshoot[i])
            self.settle_time_s[corner.name] = settle
            self.overshoot_deg[corner.name] = overshoot
            settle_pub, overshoot_pub = self._pubs[corner.name]
            settle_pub.set(settle)
            overshoot_pub.set(overshoot)
            log.info("Steer step %s: settles in %.3f s, overshoot %.1f deg", corner.name, settle, overshoot)

    def runsWhenDisabled(self) -> bool:
        return False


# -- Registering it all ---------------------------------------------------------


def register_swerve(chooser: CharacterizationChooser, drivetrain: Drivetrain) -> dict[str, SysIdTests]:
    """Put every swerve characterization test on the Characterization chooser.

    Args:
        chooser: The robot's test-mode chooser.
        drivetrain: The drivetrain.

    Returns:
        The ``drive``, ``driveSlowRamp`` and ``steer``
        :class:`~utils.sysid.routines.SysIdTests`, by name,
        so the robot (or a test) can read their estimates.
    """
    period = drivetrain.loop_period_s
    tests = {
        "drive": SysIdTests(drive_mechanism(drivetrain), period_s=period),
        "steer": SysIdTests(steer_mechanism(drivetrain), period_s=period),
    }
    for sysid in tests.values():
        chooser.add_sysid(sysid)
    slow = SysIdTests(slow_ramp_drive_mechanism(drivetrain), period_s=period, fit_ka=False)
    tests["driveSlowRamp"] = slow
    chooser.add("Drive feedforward (slow)", slow.quasistatic(True))
    chooser.add_settings(slow.tunable)
    chooser.add("Wheel radius", WheelRadiusCharacterization(drivetrain))
    chooser.add("Steer step test", SteerStepTest(drivetrain))
    return tests
