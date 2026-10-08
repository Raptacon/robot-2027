"""
A whole simulated drivetrain, plus the "ground truth" of where the robot really is.

Use it in tests and in ``robotpy sim``. It builds a
:class:`~subsystem.drivetrain.drivetrain.Drivetrain` from simulated IO, and
each loop, when the drivetrain reads the gyro, the sim works out how the
simulated robot really moved:

- It feeds the true spin rate to the simulated gyro.
- It keeps :attr:`DrivetrainSim.true_pose`, the robot's real position, so
  tests can check how far odometry has drifted from the truth.

The ground truth uses each wheel's *real* motion (real gears, real wheel
size), while the drive code only knows the config. So a test that gives the
sim the wrong gears sees odometry drift away from the truth, like on a real
robot.

Example:
    A robot whose front-left absolute encoder never answers:

    >>> from config.robots.swerve_test_bot import CONFIG
    >>> from subsystem.drivetrain.drivetrain_sim import DrivetrainSim
    >>> sim = DrivetrainSim(CONFIG, encoder_options={"frontLeft": {"never_connects": True}})
    >>> for _ in range(60):
    ...     sim.drivetrain.periodic()
    >>> [m.encoder_failed for m in sim.drivetrain.modules]
    [True, False, False, False]
"""

from collections.abc import Mapping
from typing import Any

from config.robot_config import RobotConfig
from subsystem.drivetrain.drivetrain import Drivetrain
from subsystem.drivetrain.io.abs_encoder_sim import AbsoluteEncoderIOSim
from subsystem.drivetrain.io.gyro_io_sim import GyroIOSim
from subsystem.drivetrain.io.module_io_sim import ModuleIOSim, ModuleSimOptions
from subsystem.drivetrain.module import SwerveModule
from subsystem.drivetrain.swerve_math import ModuleTarget, Pose, forward_kinematics, pose_exp


class DrivetrainSim:
    """Builds a simulated drivetrain and tracks where the robot really is.

    Args:
        config: The robot config the drive code uses.
        module_options: Optional :class:`~subsystem.drivetrain.io.module_io_sim.ModuleSimOptions`
            per corner name, like ``{"frontLeft": ModuleSimOptions(latency_loops=2)}``.
        encoder_options: Optional keyword options for each corner's
            :class:`~subsystem.drivetrain.io.abs_encoder_sim.AbsoluteEncoderIOSim`,
            like ``{"backRight": {"boot_delay_loops": 10}}``.
        gyro: The simulated gyro. Leave out for a healthy one.
        loop_period_s: Loop period in seconds.

    Attributes:
        drivetrain: The :class:`~subsystem.drivetrain.drivetrain.Drivetrain` to test.
        module_sims: The four simulated modules (FL, FR, BL, BR).
        gyro_sim: The simulated gyro.
        true_pose: Where the robot really is.
    """

    def __init__(
        self,
        config: RobotConfig,
        module_options: Mapping[str, ModuleSimOptions] | None = None,
        encoder_options: Mapping[str, Mapping[str, Any]] | None = None,
        gyro: GyroIOSim | None = None,
        loop_period_s: float = 0.02,
    ) -> None:
        module_options = module_options or {}
        encoder_options = encoder_options or {}
        self.loop_period_s = loop_period_s
        self.module_sims: list[ModuleIOSim] = []
        modules = []
        for corner in config.corners:
            io = ModuleIOSim(config, module_options.get(corner.name))
            encoder = AbsoluteEncoderIOSim.for_module(io, corner, **encoder_options.get(corner.name, {}))
            self.module_sims.append(io)
            modules.append(SwerveModule(corner, config, io, encoder))
        self.gyro_sim = gyro or GyroIOSim()
        self.gyro_sim._before_update = self._update_truth
        self.drivetrain = Drivetrain(config, modules, self.gyro_sim, loop_period_s)
        self.true_pose = Pose()
        self._module_xy = [(c.x_m, c.y_m) for c in config.corners]
        self._last_true_distances = [m.true_drive_position_m for m in self.module_sims]

    def _update_truth(self) -> None:
        """Update the ground truth and the gyro from how the wheels really moved this loop.

        Called automatically each time the drivetrain reads the gyro, right
        after the modules have simulated this loop.
        """
        distances = [m.true_drive_position_m for m in self.module_sims]
        # Treat each wheel's real distance this loop as a "speed over one second"
        # and run forward kinematics: that gives the robot's real motion this loop.
        moves = [
            ModuleTarget(d - last, m.true_steer_angle_rad)
            for d, last, m in zip(distances, self._last_true_distances, self.module_sims)
        ]
        self._last_true_distances = distances
        motion = forward_kinematics(moves, self._module_xy)
        self.true_pose = pose_exp(self.true_pose, motion, 1.0)
        self.gyro_sim.step(motion.omega_rad_per_s / self.loop_period_s, self.loop_period_s)
