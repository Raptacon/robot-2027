"""
Pick which robot's config to use.

What is this file for?
    The same code runs on more than one robot (a test bot, the competition
    robot, maybe a practice robot). Each robot has different encoder offsets
    and maybe different gears, so the code needs to know which robot it is
    running on.

    Each robot stores its own name in NetworkTables at ``/robot/name``. The
    value is *persistent*: it is saved on the robot controller and survives
    reboots and code deploys. If it has never been set, the name is
    ``"swerve_test_bot"``.

How do I set a robot's name?
    Once per robot: connect to it, open a NetworkTables tool (AdvantageScope,
    Elastic or OutlineViewer), set ``/robot/name`` to the robot's name, and
    restart the robot code.

How do I use it in code?
    >>> from config.loader import load_robot_config
    >>> config = load_robot_config("swerve_test_bot")
    >>> config.preset.name
    'MK4i_L2'

    In robot code, call ``load_robot_config()`` with no name and it reads
    ``/robot/name``.

How do I add a new robot?
    1. Copy ``config/robots/swerve_test_bot.py`` to a new file, for example
       ``config/robots/comp_bot.py``, and change ``name="comp_bot"`` and the
       values that differ.
    2. Import it below and add it to :data:`ROBOTS`.
    3. Run ``python -m robotpy test``. The config tests check every robot in
       :data:`ROBOTS`.

If ``/robot/name`` holds a name that isn't in :data:`ROBOTS`, the code stops
with an error instead of driving with the wrong offsets.
"""

from ntcore.util import ntproperty

from config.robot_config import RobotConfig
from config.robots import swerve_test_bot

DEFAULT_ROBOT = "swerve_test_bot"
"""The name used when ``/robot/name`` has never been set."""

ROBOTS: dict[str, RobotConfig] = {
    swerve_test_bot.CONFIG.name: swerve_test_bot.CONFIG,
}
"""Every robot the code knows about, by name. Add new robots here."""

ROBOT_NAME_TOPIC = "/robot/name"
"""The NetworkTables key that holds this robot's name."""


class _RobotName:
    # ntproperty links this attribute to a NetworkTables value.
    # persistent=True saves it on the robot; writeDefault=False means the
    # default is only used when nothing is saved yet.
    value = ntproperty(ROBOT_NAME_TOPIC, DEFAULT_ROBOT, writeDefault=False, persistent=True)


_ROBOT_NAME = _RobotName()


def robot_name_from_nt() -> str:
    """Return the robot name saved in NetworkTables at ``/robot/name``.

    Returns:
        The saved name, or :data:`DEFAULT_ROBOT` if none has been saved yet.
    """
    return _ROBOT_NAME.value


def load_robot_config(name: str | None = None) -> RobotConfig:
    """Return the config for a robot.

    Args:
        name: The robot's name, like ``"swerve_test_bot"``. Leave it out to
            use the name saved on the robot in ``/robot/name``. Tests pass a
            name so they don't depend on NetworkTables.

    Returns:
        That robot's :class:`~config.robot_config.RobotConfig`.

    Raises:
        KeyError: If the name isn't in :data:`ROBOTS`. The message lists the
            names that are.

    Example:
        >>> from config.loader import load_robot_config
        >>> load_robot_config("not_a_robot")
        Traceback (most recent call last):
            ...
        KeyError: "Unknown robot 'not_a_robot'; known robots: ['swerve_test_bot']"
    """
    if name is None:
        name = robot_name_from_nt()
    try:
        return ROBOTS[name]
    except KeyError:
        raise KeyError(f"Unknown robot {name!r}; known robots: {sorted(ROBOTS)}") from None
