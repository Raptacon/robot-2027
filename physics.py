"""
The physics for ``robotpy sim`` (and the whole-robot tests).

RobotPy loads this file automatically when it simulates ``robot.py``. The
robot code runs exactly as on the real robot; this file moves the simulated
motors, wheels and sensors behind it (see :mod:`sim.swerve_physics`) and
draws the robot's true position on the ``Field`` widget in the sim GUI.

Nothing in ``robot.py`` or the subsystems knows this file exists.

Example:
    Run the simulator, then open the ``Field`` widget (NetworkTables >
    SmartDashboard > Field) to watch the robot drive:

    .. code-block:: bash

        python -m robotpy sim
"""

from pyfrc.physics.core import PhysicsInterface

from sim.swerve_physics import SwervePhysics
from utils.wpimath_compat import Pose2d, Rotation2d


class PhysicsEngine:
    """Steps the swerve physics each simulator tick.

    Args:
        physics_controller: RobotPy's physics interface (has the ``field``).
        robot: The robot object ``robot.py`` built.
    """

    def __init__(self, physics_controller: PhysicsInterface, robot) -> None:
        self.physics_controller = physics_controller
        self.swerve = SwervePhysics(robot.drivetrain)
        # For the whole-robot tests to check where the robot really went.
        # The robot code never reads it.
        robot.sim_physics = self.swerve

    def update_sim(self, now: float, tm_diff: float) -> None:
        """Move the simulated robot forward by ``tm_diff`` seconds.

        Args:
            now: Simulator time, seconds.
            tm_diff: Time since the last call, seconds (about 0.02).
        """
        self.swerve.step(tm_diff)
        pose = self.swerve.true_pose
        field = self.physics_controller.field
        if field is not None:
            field.setRobotPose(Pose2d(pose.x_m, pose.y_m, Rotation2d(pose.heading_rad)))
