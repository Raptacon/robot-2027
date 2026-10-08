"""
Pick the robot config for this robot.

The name comes from the persistent NetworkTables value /robot/name, so one
code base runs on several robots. Set it once per robot from a dashboard; it
survives reboots. Unknown names fail loudly instead of driving with the wrong
offsets.
"""

from ntcore.util import ntproperty

from config.robot_config import RobotConfig
from config.robots import swerve_test_bot

DEFAULT_ROBOT = "swerve_test_bot"

ROBOTS: dict[str, RobotConfig] = {
    swerve_test_bot.CONFIG.name: swerve_test_bot.CONFIG,
}

ROBOT_NAME_TOPIC = "/robot/name"


class _RobotName:
    # writeDefault=False keeps the name already stored on the robot.
    value = ntproperty(ROBOT_NAME_TOPIC, DEFAULT_ROBOT, writeDefault=False, persistent=True)


_ROBOT_NAME = _RobotName()


def robot_name_from_nt() -> str:
    """Read /robot/name; it starts as DEFAULT_ROBOT and persists across reboots."""
    return _ROBOT_NAME.value


def load_robot_config(name: str | None = None) -> RobotConfig:
    """Return the config for name, or for the name in NetworkTables."""
    if name is None:
        name = robot_name_from_nt()
    try:
        return ROBOTS[name]
    except KeyError:
        raise KeyError(f"Unknown robot {name!r}; known robots: {sorted(ROBOTS)}") from None
