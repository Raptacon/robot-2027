"""
Robot configuration: the numbers that describe each robot's hardware.

- :mod:`config.module_presets`: gear ratios built into each swerve module type.
- :mod:`config.robot_config`: the types that describe one robot's swerve.
- :mod:`config.robots`: one file per robot with its real values.
- :mod:`config.loader`: picks the right robot's config at startup.

Config code is plain Python with no WPILib imports, so it can be read and
tested on any computer.
"""
