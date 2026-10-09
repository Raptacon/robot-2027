"""
SysId settings you can change from the dashboard, with presets.

Each mechanism's settings show up under
``/Characterization/<name>/settings/`` (ramp, step, timeouts, settle time,
volt limit). Change a number there and the next test you start uses it; a
test that is already running keeps the values it started with.

There is also a ``SysId <name> preset`` chooser. Picking a preset loads all
of its values into the dashboard at once:

- ``Config default``: the values in the robot's config file.
- ``Gentle``: half the ramp and half the step, for a first try on a new
  mechanism or a robot short on room.
- ``Slow ramp``: 0.1 V/s for kS and kV only (kA can't be measured from it).

The dashboard values reset to the config file every time the robot code
starts, so once you find settings that work, copy them into the config file
(the git copy is the one that counts).

Safety: the volt limit can be lowered from the dashboard but never raised
above the config file's ``max_volts``.

Example:
    >>> from utils.sysid.settings import SysIdSettings
    >>> from utils.sysid.tunable import TunableSettings
    >>> tunable = TunableSettings("example", SysIdSettings(ramp_volts_per_s=1.0, step_volts=4.0))
    >>> tunable.load_preset("Gentle")
    >>> tunable.current().ramp_volts_per_s, tunable.current().step_volts
    (0.5, 2.0)
    >>> tunable.load_preset("Config default")
    >>> tunable.current().step_volts
    4.0
"""

import dataclasses
import logging

import ntcore
import wpilib

from utils.sysid.settings import SysIdSettings

log = logging.getLogger(__name__)

CONFIG_DEFAULT = "Config default"
"""Name of the preset that holds the config file's values."""


def standard_presets(default: SysIdSettings) -> dict[str, SysIdSettings]:
    """The presets every mechanism gets, worked out from its config values.

    Args:
        default: The mechanism's settings from the config file.

    Returns:
        Preset name to settings, with ``Config default`` first.

    Example:
        >>> from utils.sysid.settings import SysIdSettings
        >>> from utils.sysid.tunable import standard_presets
        >>> list(standard_presets(SysIdSettings()))
        ['Config default', 'Gentle', 'Slow ramp']
    """
    return {
        CONFIG_DEFAULT: default,
        "Gentle": dataclasses.replace(
            default,
            ramp_volts_per_s=default.ramp_volts_per_s / 2,
            step_volts=default.step_volts / 2,
        ),
        # 0.1 V/s is the ramp team 6328 uses for drive kS and kV.
        "Slow ramp": dataclasses.replace(default, ramp_volts_per_s=0.1, timeout_s=15.0),
    }


class TunableSettings:
    """A mechanism's :class:`SysIdSettings`, editable on the dashboard.

    Args:
        name: The mechanism's name (used in the dashboard paths).
        default: The settings from the config file. Loaded at startup and
            by the ``Config default`` preset.
        presets: Extra presets to offer, by name, on top of
            :func:`standard_presets`.
    """

    def __init__(
        self,
        name: str,
        default: SysIdSettings,
        presets: dict[str, SysIdSettings] | None = None,
    ) -> None:
        self.name = name
        self.default = default
        self.presets = standard_presets(default) | (presets or {})
        table = ntcore.NetworkTableInstance.getDefault().getTable(f"/Characterization/{name}/settings")
        self._entries = {
            field.name: table.getDoubleTopic(field.name).getEntry(getattr(default, field.name))
            for field in dataclasses.fields(SysIdSettings)
        }
        self._preset_chooser = wpilib.SendableChooser()
        for preset in self.presets:
            if preset == CONFIG_DEFAULT:
                self._preset_chooser.setDefaultOption(preset, preset)
            else:
                self._preset_chooser.addOption(preset, preset)
        self._preset_chooser.onChange(self.load_preset)
        self.load_preset(CONFIG_DEFAULT)

    def load_preset(self, preset: str) -> None:
        """Copy a preset's values onto the dashboard.

        Args:
            preset: One of :attr:`presets`.
        """
        settings = self.presets[preset]
        for field, entry in self._entries.items():
            entry.set(getattr(settings, field))
        log.info("SysId %s: loaded preset %r", self.name, preset)

    def current(self) -> SysIdSettings:
        """The settings on the dashboard right now, made safe.

        Negative numbers are made positive and the volt limit is capped at
        the config file's ``max_volts``.

        Returns:
            The :class:`SysIdSettings` to run the next test with.
        """
        values = {field: abs(entry.get()) for field, entry in self._entries.items()}
        values["max_volts"] = min(values["max_volts"], self.default.max_volts)
        return SysIdSettings(**values)

    def publish(self) -> None:
        """Show the preset chooser on the dashboard as ``SysId <name> preset``."""
        wpilib.SmartDashboard.putData(f"SysId {self.name} preset", self._preset_chooser)
