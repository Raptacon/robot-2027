"""
This test module imports the standard robot tests, and can be used
to test basic functionality of just about any robot.

RobotPy 2026 ships them in pyfrc; RobotPy 2027 dropped pyfrc and ships
them inside wpilib, so try both.
"""

try:  # RobotPy 2026
    from pyfrc.tests import *  # noqa: F401,F403
except ImportError:  # RobotPy 2027
    from wpilib.testing.robot_tests import *  # noqa: F401,F403
