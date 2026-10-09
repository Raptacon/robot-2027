"""
The "Characterization" chooser: pick a test-mode test, then hold a button to run it.

Every mechanism's SysId tests, plus other test-mode tools (wheel radius,
module check, steer step test), go on one dashboard chooser. To run one:

1. Put the robot on blocks or on carpet, as the test needs.
2. Pick the test on the dashboard's ``Characterization`` chooser, or step
   through the list with the controller (see :meth:`CharacterizationChooser.bind`).
3. Enable **Test** mode. Nothing moves yet.
4. **Hold** the run button. The test runs only while it is held; let go and
   it stops at once. This is WPILib's advice for SysId: the person holding
   the button can stop a test the moment it looks unsafe.

Example:
    >>> import commands2
    >>> from utils.sysid.chooser import CharacterizationChooser
    >>> chooser = CharacterizationChooser("Do nothing", commands2.InstantCommand())
    >>> chooser.add("Another test", commands2.InstantCommand())
    >>> chooser.names
    ['Do nothing', 'Another test']
"""

import logging
from collections.abc import Callable

import commands2
import ntcore
import wpilib
from commands2.button import Trigger

from utils.sysid.routines import SysIdTests
from utils.sysid.tunable import TunableSettings

log = logging.getLogger(__name__)

DASHBOARD_KEY = "Characterization"
"""The chooser's name on the dashboard (under ``/SmartDashboard/``)."""


def is_test_enabled() -> bool:
    """``True`` while the robot is enabled in test mode.

    RobotPy 2027 replaces test mode with "op modes", so this is always
    ``False`` there until the op mode move.
    """
    is_test: Callable[[], bool] | None = getattr(wpilib.RobotState, "isTest", None)
    return is_test is not None and is_test() and wpilib.RobotState.isEnabled()


class CharacterizationChooser:
    """A dashboard chooser of test-mode commands, run while a button is held.

    Args:
        default_name: The option picked until someone changes it.
        default_command: The command for that option.

    Attributes:
        names: Every option, in the order added.
        running: The command started by the run button, while it is held.
    """

    def __init__(self, default_name: str, default_command: commands2.Command) -> None:
        self._chooser = wpilib.SendableChooser()
        self._commands: dict[str, commands2.Command] = {default_name: default_command}
        self._default_name = default_name
        self._chooser.setDefaultOption(default_name, default_name)
        self.names = [default_name]
        self.running: commands2.Command | None = None
        self._tunables: list[TunableSettings] = []
        self._selected_pub = (
            ntcore.NetworkTableInstance.getDefault()
            .getStringTopic(f"/SmartDashboard/{DASHBOARD_KEY}/selected")
            .publish()
        )

    def add(self, name: str, command: commands2.Command) -> None:
        """Add an option.

        Args:
            name: What the dashboard shows. Must be unique.
            command: The command to run while the run button is held.
        """
        if name in self._commands:
            raise ValueError(f"Characterization chooser already has {name!r}")
        self._commands[name] = command
        self._chooser.addOption(name, name)
        self.names.append(name)

    def add_sysid(self, tests: SysIdTests) -> None:
        """Add a mechanism's SysId tests, named like ``"SysId drive: dynamic forward"``."""
        for test_name, command in tests.commands().items():
            self.add(f"SysId {tests.mechanism.name}: {test_name}", command)
        self.add_settings(tests.tunable)

    def add_settings(self, tunable: TunableSettings) -> None:
        """Show a mechanism's SysId settings preset chooser when :meth:`publish` is called.

        :meth:`add_sysid` does this already; use it for a SysId test added
        with :meth:`add` under its own name.
        """
        self._tunables.append(tunable)

    @property
    def selected_name(self) -> str:
        """The name of the option picked on the dashboard (or with the controller)."""
        return self._chooser.getSelected() or self._default_name

    def selected(self) -> commands2.Command:
        """The command for the picked option."""
        return self._commands[self.selected_name]

    def pick(self, name: str) -> None:
        """Pick an option from code, the same way the dashboard does.

        The chooser sees the change on the next loop.

        Args:
            name: One of :attr:`names`.
        """
        if name not in self._commands:
            raise KeyError(f"No characterization option named {name!r}")
        self._selected_pub.set(name)
        log.info("Characterization: picked %r", name)

    def step(self, offset: int) -> None:
        """Pick the next (``offset=1``) or previous (``offset=-1``) option, wrapping around."""
        index = self.names.index(self.selected_name)
        self.pick(self.names[(index + offset) % len(self.names)])

    def start(self) -> None:
        """Start the picked test (called when the run button is pressed)."""
        self.stop()
        self.running = self.selected()
        log.info("Characterization: running %r", self.selected_name)
        self.running.schedule()

    def stop(self) -> None:
        """Stop the running test, if any (called when the run button is let go)."""
        if self.running is not None:
            self.running.cancel()
            self.running = None

    def bind(self, run: Trigger, next_option: Trigger | None = None, previous_option: Trigger | None = None) -> None:
        """Connect controller buttons. They only do anything while enabled in test mode.

        Args:
            run: Hold to run the picked test; let go to stop it.
            next_option: Press to pick the next option.
            previous_option: Press to pick the previous option.
        """
        in_test = Trigger(is_test_enabled)
        # Stop when the button is let go, and also if test mode ends while
        # it is still held.
        held = run.and_(in_test)
        held.onTrue(commands2.InstantCommand(self.start))
        held.onFalse(commands2.InstantCommand(self.stop).ignoringDisable(True))
        if next_option is not None:
            next_option.and_(in_test).onTrue(commands2.InstantCommand(lambda: self.step(1)))
        if previous_option is not None:
            previous_option.and_(in_test).onTrue(commands2.InstantCommand(lambda: self.step(-1)))

    def publish(self) -> None:
        """Show the chooser on the dashboard as ``Characterization``, and each SysId preset chooser."""
        wpilib.SmartDashboard.putData(DASHBOARD_KEY, self._chooser)
        for tunable in self._tunables:
            tunable.publish()
