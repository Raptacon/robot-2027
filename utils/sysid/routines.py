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

The tests are meant for the real robot, where the readings go into the
wpilog in the format the SysId app reads (URCL adds the SPARK MAX data).
They also run in simulation, which is how CI checks them; there nothing is
written to disk, so test runs don't leave log files behind. Either way, after each test the dashboard shows a quick estimate under
``/Characterization/<name>/estimate/`` (see :mod:`utils.sysid.fit`).

The ramp, step and timeouts can be changed on the dashboard before each test
(see :mod:`utils.sysid.tunable`); each test reads them when it starts.

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
from collections.abc import Callable

import commands2
import ntcore
import wpilib
from commands2.sysid import SysIdRoutine
from wpilib.sysid import SysIdRoutineLog

from utils.sysid.characterizable import Characterizable, Reading
from utils.sysid.fit import FeedforwardFit, fit_feedforward
from utils.sysid.settings import SysIdSettings
from utils.sysid.tunable import TunableSettings

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
        fit_ka: ``False`` to leave kA out of the dashboard estimate (for a
            slow-ramp-only test, where kA can't be measured).

    Attributes:
        mechanism: The mechanism being tested.
        runs: The readings from each test run so far, one list per test.
        estimate: The latest quick fit, or ``None`` before the first test.
        tunable: The dashboard copy of the mechanism's settings.
    """

    def __init__(
        self,
        mechanism: Characterizable,
        period_s: float = 0.02,
        log_to_wpilog: bool | None = None,
        fit_ka: bool = True,
    ) -> None:
        self.mechanism = mechanism
        self.period_s = period_s
        self.fit_ka = fit_ka
        self.log_to_wpilog = wpilib.RobotBase.isReal() if log_to_wpilog is None else log_to_wpilog
        self.runs: list[list[Reading]] = []
        self.estimate: FeedforwardFit | None = None
        self.tunable = TunableSettings(mechanism.name, mechanism.settings)
        # The settings of the test running now (or the last one).
        self._settings = mechanism.settings
        table = ntcore.NetworkTableInstance.getDefault().getTable(f"/Characterization/{mechanism.name}/estimate")
        self._ks_pub = table.getDoubleTopic("ks").publish()
        self._kv_pub = table.getDoubleTopic("kv").publish()
        self._ka_pub = table.getDoubleTopic("ka").publish()
        self._kg_pub = table.getDoubleTopic("kg").publish()
        self._r2_pub = table.getDoubleTopic("rSquared").publish()

    # -- Commands -----------------------------------------------------------

    def quasistatic(self, forward: bool = True) -> commands2.Command:
        """The quasistatic (slow voltage ramp) test.

        Args:
            forward: ``True`` for positive volts, ``False`` for negative.

        Returns:
            A command that settles, runs the test, then updates the estimate.
        """
        return self._deferred(lambda routine: routine.quasistatic(_direction(forward)), forward)

    def dynamic(self, forward: bool = True) -> commands2.Command:
        """The dynamic (voltage step) test.

        Args:
            forward: ``True`` for positive volts, ``False`` for negative.

        Returns:
            A command that settles, runs the test, then updates the estimate.
        """
        return self._deferred(
            lambda routine: routine.dynamic(_direction(forward)).withTimeout(self._settings.dynamic_timeout_s),
            forward,
            dynamic=True,
        )

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

    def _deferred(
        self, make_test: Callable[[SysIdRoutine], commands2.Command], forward: bool, dynamic: bool = False
    ) -> commands2.Command:
        """A command that reads the dashboard settings when it starts, then builds and runs the test."""

        def build() -> commands2.Command:
            self._settings = self.tunable.current()
            return self._wrap(make_test(self._routine(self._settings)), forward)

        kind = "dynamic" if dynamic else "quasistatic"
        direction = "forward" if forward else "reverse"
        return commands2.DeferredCommand(build, self.mechanism.subsystem).withName(
            f"SysId {self.mechanism.name}: {kind} {direction}"
        )

    def _routine(self, settings: SysIdSettings) -> SysIdRoutine:
        config = SysIdRoutine.Config(
            rampRate=settings.ramp_volts_per_s,
            stepVoltage=settings.step_volts,
            timeout=settings.timeout_s,
            # In simulation, keep the test state out of the wpilog too.
            recordState=None if self.log_to_wpilog else (lambda state: None),
        )
        return SysIdRoutine(
            config, SysIdRoutine.Mechanism(self._drive, self._log, self.mechanism.subsystem, self.mechanism.name)
        )

    def _wrap(self, test: commands2.Command, forward: bool) -> commands2.Command:
        """Start a new run, hold 0 V to settle, run ``test`` (stopping at a position limit),
        then hold 0 V and update the estimate."""
        subsystem = self.mechanism.subsystem
        settle = self._settings.settle_s
        return (
            subsystem.runOnce(lambda: self.runs.append([]))
            .andThen(subsystem.run(lambda: self._drive(0.0)).withTimeout(settle))
            .andThen(test.until(lambda: self._at_limit(forward)))
            .andThen(subsystem.run(lambda: self._drive(0.0)).withTimeout(PAUSE_BETWEEN_TESTS_S))
            .finallyDo(lambda interrupted: self._update_estimate())
            .withName(test.getName())
        )

    def _at_limit(self, forward: bool) -> bool:
        if not self.mechanism.past_limit(self.mechanism.read().position, forward):
            return False
        log.warning("SysId %s: stopped at its position limit", self.mechanism.name)
        return True

    def _drive(self, volts: float) -> None:
        limit = self._settings.max_volts
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
            fit = fit_feedforward(self.runs, self.period_s, self.mechanism.gravity, self.fit_ka)
        except ValueError as error:
            log.info("SysId %s: no estimate yet (%s)", self.mechanism.name, error)
            return
        self.estimate = fit
        self._ks_pub.set(fit.ks)
        self._kv_pub.set(fit.kv)
        self._ka_pub.set(fit.ka)
        self._kg_pub.set(fit.kg)
        self._r2_pub.set(fit.r_squared)
        log.info(
            "SysId %s estimate: kS=%.3f kV=%.3f kA=%.3f kG=%.3f (r^2=%.3f, %d readings)",
            self.mechanism.name,
            fit.ks,
            fit.kv,
            fit.ka,
            fit.kg,
            fit.r_squared,
            fit.samples,
        )


def _direction(forward: bool) -> SysIdRoutine.Direction:
    return SysIdRoutine.Direction.kForward if forward else SysIdRoutine.Direction.kReverse
