"""
The four SysId tests as commands, for any :class:`~utils.sysid.characterizable.Characterizable`.

The four tests:
    - **Quasistatic forward / reverse**: the voltage rises slowly, so the
      mechanism is never accelerating much. This measures kS and kV.
    - **Dynamic forward / reverse**: the voltage jumps straight to
      ``step_volts``. This measures kA.

Run all four (:meth:`SysIdTests.all_tests` does them in order), then open
the robot's wpilog in the WPILib SysId app to get the numbers. Forward then
reverse brings a drivetrain back near where it started.

On the real robot the readings go into the wpilog in the format the SysId
app reads (URCL adds the SPARK MAX data). In simulation nothing is written to
disk. Either way, after each test the dashboard shows a quick estimate under
``/Characterization/<name>/estimate/`` (see :mod:`utils.sysid.fit`).

Example:
    >>> import commands2
    >>> from utils.sysid.characterizable import Characterizable, Reading
    >>> from utils.sysid.routines import SysIdTests
    >>> motor = Characterizable(
    ...     name="example",
    ...     subsystem=commands2.Subsystem(),
    ...     set_voltage=lambda volts: None,
    ...     read=lambda: Reading(0.0, 0.0, 0.0),
    ... )
    >>> tests = SysIdTests(motor)
    >>> sorted(tests.commands())
    ['all four tests', 'dynamic forward', 'dynamic reverse', 'quasistatic forward', 'quasistatic reverse']
"""

import logging
import math

import commands2
import ntcore
import wpilib
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog

from utils.sysid.characterizable import Characterizable, Reading
from utils.sysid.fit import FeedforwardFit, fit_feedforward

log = logging.getLogger(__name__)

PAUSE_BETWEEN_TESTS_S = 1.5
"""Seconds of 0 V between the tests in :meth:`SysIdTests.all_tests`, so each starts still."""


class SysIdTests:
    """Builds the SysId test commands for one mechanism and keeps its readings.

    Args:
        mechanism: The mechanism to test.
        period_s: Loop period, seconds (used to work out acceleration).
        log_to_wpilog: Write readings to the wpilog for the SysId app.
            Defaults to ``True`` on the real robot and ``False`` in simulation,
            so tests don't leave log files behind.

    Attributes:
        mechanism: The mechanism being tested.
        runs: The readings from each test run so far, one list per test.
        estimate: The latest quick fit, or ``None`` before the first test.
    """

    def __init__(
        self,
        mechanism: Characterizable,
        period_s: float = 0.02,
        log_to_wpilog: bool | None = None,
    ) -> None:
        self.mechanism = mechanism
        self.period_s = period_s
        self.log_to_wpilog = wpilib.RobotBase.isReal() if log_to_wpilog is None else log_to_wpilog
        self.runs: list[list[Reading]] = []
        self.estimate: FeedforwardFit | None = None
        settings = mechanism.settings
        config = SysIdRoutine.Config(
            rampRate=settings.ramp_volts_per_s,
            stepVoltage=settings.step_volts,
            timeout=settings.timeout_s,
            # In simulation, keep the test state out of the wpilog too.
            recordState=None if self.log_to_wpilog else (lambda state: None),
        )
        self._routine = SysIdRoutine(
            config, SysIdRoutine.Mechanism(self._drive, self._log, mechanism.subsystem, mechanism.name)
        )
        table = ntcore.NetworkTableInstance.getDefault().getTable(f"/Characterization/{mechanism.name}/estimate")
        self._ks_pub = table.getDoubleTopic("ks").publish()
        self._kv_pub = table.getDoubleTopic("kv").publish()
        self._ka_pub = table.getDoubleTopic("ka").publish()
        self._r2_pub = table.getDoubleTopic("rSquared").publish()

    # -- Commands -----------------------------------------------------------

    def quasistatic(self, forward: bool = True) -> commands2.Command:
        """The quasistatic (slow voltage ramp) test.

        Args:
            forward: ``True`` for positive volts, ``False`` for negative.

        Returns:
            A command that settles, runs the test, then updates the estimate.
        """
        return self._wrap(self._routine.quasistatic(_direction(forward)))

    def dynamic(self, forward: bool = True) -> commands2.Command:
        """The dynamic (voltage step) test.

        Args:
            forward: ``True`` for positive volts, ``False`` for negative.

        Returns:
            A command that settles, runs the test, then updates the estimate.
        """
        test = self._routine.dynamic(_direction(forward))
        return self._wrap(test.withTimeout(self.mechanism.settings.dynamic_timeout_s).withName(test.getName()))

    def all_tests(self) -> commands2.Command:
        """All four tests in a row: quasistatic forward, reverse, then dynamic forward, reverse."""
        return commands2.SequentialCommandGroup(
            self.quasistatic(True),
            self.quasistatic(False),
            self.dynamic(True),
            self.dynamic(False),
        ).withName(f"SysId {self.mechanism.name}: all four tests")

    def commands(self) -> dict[str, commands2.Command]:
        """Every test as a new command, by name, for a dashboard chooser."""
        return {
            "all four tests": self.all_tests(),
            "quasistatic forward": self.quasistatic(True),
            "quasistatic reverse": self.quasistatic(False),
            "dynamic forward": self.dynamic(True),
            "dynamic reverse": self.dynamic(False),
        }

    # -- Internals ------------------------------------------------------------

    def _wrap(self, test: commands2.Command) -> commands2.Command:
        """Start a new run, hold 0 V to settle, run ``test``, then hold 0 V and update the estimate."""
        subsystem = self.mechanism.subsystem
        settle = self.mechanism.settings.settle_s
        return (
            subsystem.runOnce(lambda: self.runs.append([]))
            .andThen(subsystem.run(lambda: self._drive(0.0)).withTimeout(settle))
            .andThen(test)
            .andThen(subsystem.run(lambda: self._drive(0.0)).withTimeout(PAUSE_BETWEEN_TESTS_S))
            .finallyDo(lambda interrupted: self._update_estimate())
            .withName(test.getName())
        )

    def _drive(self, volts: float) -> None:
        limit = self.mechanism.settings.max_volts
        self.mechanism.set_voltage(max(-limit, min(limit, volts)))

    def _log(self, routine_log: SysIdRoutineLog) -> None:
        reading = self.mechanism.read()
        if self.runs:
            self.runs[-1].append(reading)
        if not self.log_to_wpilog:
            return
        motor = routine_log.motor(self.mechanism.name).voltage(reading.volts)
        if self.mechanism.angular:
            # WPILib logs turning mechanisms in rotations, so the SysId app's
            # kV and kA are per rotation: divide them by 2 pi for radians.
            motor.angularPosition(reading.position / math.tau).angularVelocity(reading.velocity / math.tau)
        else:
            motor.position(reading.position).velocity(reading.velocity)

    def _update_estimate(self) -> None:
        try:
            fit = fit_feedforward(self.runs, self.period_s)
        except ValueError as error:
            log.info("SysId %s: no estimate yet (%s)", self.mechanism.name, error)
            return
        self.estimate = fit
        self._ks_pub.set(fit.ks)
        self._kv_pub.set(fit.kv)
        self._ka_pub.set(fit.ka)
        self._r2_pub.set(fit.r_squared)
        log.info(
            "SysId %s estimate: kS=%.3f kV=%.3f kA=%.3f (r^2=%.3f, %d readings)",
            self.mechanism.name,
            fit.ks,
            fit.kv,
            fit.ka,
            fit.r_squared,
            fit.samples,
        )


def _direction(forward: bool) -> SysIdRoutine.Direction:
    return SysIdRoutine.Direction.kForward if forward else SysIdRoutine.Direction.kReverse
