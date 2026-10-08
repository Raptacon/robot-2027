"""
The "Characterization" chooser: pick which test runs when the robot enters test mode.

Every mechanism's SysId tests, plus other test-mode tools (wheel radius,
module check, steer step test), go on one dashboard chooser. To run one:

1. Put the robot on blocks or on carpet, as the test needs.
2. Pick the test on the dashboard's ``Characterization`` chooser.
3. Enable **Test** mode. The test starts; disable to stop it early.

Example:
    >>> import commands2
    >>> from utils.sysid.chooser import CharacterizationChooser
    >>> chooser = CharacterizationChooser("Do nothing", commands2.InstantCommand())
    >>> chooser.add("Another test", commands2.InstantCommand())
    >>> chooser.names
    ['Do nothing', 'Another test']
"""

import commands2
import wpilib

from utils.sysid.routines import SysIdTests

DASHBOARD_KEY = "Characterization"
"""The chooser's name on the dashboard (under ``/SmartDashboard/``)."""


class CharacterizationChooser:
    """A dashboard chooser of test-mode commands.

    Args:
        default_name: The option picked until someone changes it.
        default_command: The command for that option.

    Attributes:
        names: Every option, in the order added.
    """

    def __init__(self, default_name: str, default_command: commands2.Command) -> None:
        self._chooser = wpilib.SendableChooser()
        self._commands: dict[str, commands2.Command] = {default_name: default_command}
        self._default_name = default_name
        self._chooser.setDefaultOption(default_name, default_name)
        self.names = [default_name]

    def add(self, name: str, command: commands2.Command) -> None:
        """Add an option.

        Args:
            name: What the dashboard shows. Must be unique.
            command: The command to run in test mode when it is picked.
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

    def selected(self) -> commands2.Command:
        """The command for the option picked on the dashboard."""
        name = self._chooser.getSelected() or self._default_name
        return self._commands[name]

    def publish(self) -> None:
        """Show the chooser on the dashboard as ``Characterization``."""
        wpilib.SmartDashboard.putData(DASHBOARD_KEY, self._chooser)
