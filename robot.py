#!/usr/bin/env python3

import inspect
import logging
import typing

import commands2
import wpilib

from commands.drive.bindings import bind_driver_controls, bind_test_controls
from config.loader import load_robot_config
from subsystem.drivetrain.drivetrain import Drivetrain
from subsystem.health_and_status import HealthAndStatus
from utils.datalog_bridge import setup_logging
from utils.input import InputFactory
from utils.loop_timing import LoopTimer


class MyRobot(commands2.TimedCommandRobot):
    """
    Our default robot class, pass it to wpilib.run
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    runs the command scheduler for you every loop.

    It sets up logging, loop timing and health telemetry, the driver
    controller, and the swerve drivetrain with teleop driving. In simulation
    the drivetrain runs on simulated IO; on the robot it uses the SPARK MAX,
    CANcoder and gyro IO. In test mode, holding A runs the test picked on
    the dashboard's Characterization chooser: the module check (wheels to 0,
    90 and 180 degrees) by default, or a SysId or calibration test.
    """

    # 20 ms default period (50 Hz)
    kDefaultPeriod: typing.ClassVar[float] = 20.0
    autonomousCommand: typing.Optional[commands2.Command] = None

    def __init__(self) -> None:

        self.__errorLogged = False
        self.__lastError = None
        self.__errorCaughtCount = 0

        self.__initFrameTiming()

        # Bridge Python logging -> wpilog + NT-controlled log level
        setup_logging()

        # Loop period in seconds (20 ms, 50 Hz)
        super().__init__(period=MyRobot.kDefaultPeriod / 1000)

        self.telemInit()

        # Controllers first: InputFactory must exist before any subsystem so
        # stick values are fresh when subsystems read them each loop.
        self.inputs = InputFactory(config_path="data/inputs/swerve_test_bot.yaml")

        self.health = HealthAndStatus()

        self.robot_config = load_robot_config()
        self.drivetrain = self.drivetrainInit()
        self.teleop = bind_driver_controls(self.inputs, self.drivetrain)
        # Test mode: hold A to run the test picked on the Characterization chooser.
        self.characterization = bind_test_controls(self.inputs, self.drivetrain)

    def drivetrainInit(self) -> Drivetrain:
        """Build the drivetrain: simulated in the simulator, real hardware on the robot.

        Returns:
            The :class:`Drivetrain`. In simulation, ``self.drive_sim`` also
            holds the :class:`DrivetrainSim` around it.
        """
        loop_period_s = MyRobot.kDefaultPeriod / 1000
        if self.isSimulation():
            from subsystem.drivetrain.drivetrain_sim import DrivetrainSim

            self.drive_sim = DrivetrainSim(self.robot_config, loop_period_s=loop_period_s)
            return self.drive_sim.drivetrain

        from subsystem.drivetrain.drivetrain_hardware import build_drivetrain

        drivetrain = build_drivetrain(self.robot_config, loop_period_s=loop_period_s)
        self.startRevLogging()
        return drivetrain

    def telemInit(self) -> None:
        """Initialize data logging: NT logging, console, DS, and vendor loggers.

        WPILib DataLogManager auto-detects USB and falls back to
        /home/lvuser/logs.  REV also auto-detects USB.
        Logging is disabled in simulation to avoid junk files.
        """
        if self.isSimulation():
            return

        # WPILib data log (.wpilog), auto-uses USB if present
        wpilib.DataLogManager.start()
        wpilib.DataLogManager.logNetworkTables(True)
        wpilib.DataLogManager.logConsoleOutput(True)
        wpilib.DriverStation.startDataLog(wpilib.DataLogManager.getLog())

        # REV status logging (.revlog files)
        from rev import StatusLogger

        StatusLogger.start()

    def startRevLogging(self) -> None:
        """Log every SPARK MAX's CAN data to the wpilog with URCL, named by corner.

        AdvantageScope shows these as ``frontLeft drive`` and so on, which
        is what you want when checking currents and SysId runs. URCL has no
        RobotPy 2027 build yet, so this is skipped there.
        """
        try:
            from urcl import URCL
        except ImportError:
            logging.warning("URCL not installed; SPARK MAX CAN data won't be logged")
            return
        aliases = {}
        for corner in self.robot_config.corners:
            aliases[corner.drive_can_id] = f"{corner.name} drive"
            aliases[corner.steer_can_id] = f"{corner.name} steer"
        URCL.start(aliases, wpilib.DataLogManager.getLog())

    def robotPeriodic(self) -> None:
        self.__frameTimingPeriodic()

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        self.__timing.reset_all()

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""
        self.__timing.start("userCode")

    def autonomousInit(self) -> None:
        self.__timing.reset_all()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""
        self.__timing.start("userCode")

    def teleopInit(self) -> None:
        self.__timing.reset_all()

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        self.__timing.start("userCode")

    def testInit(self) -> None:
        """Test mode: hold A to run the test picked on the Characterization chooser.

        Nothing moves until A is held, and letting go stops the test. B and
        the left bumper step through the chooser. See
        commands/drive/characterization.py for the tests.
        """
        self.__timing.reset_all()

    def testExit(self) -> None:
        self.characterization.stop()

    def testPeriodic(self) -> None:
        self.__timing.start("userCode")

    def __initFrameTiming(self):
        """Set up loop timing instrumentation.

        Wraps CommandScheduler.run() BEFORE super().__init__() captures it
        so the scheduler channel measures the real execution cost.
        """
        self.__timing = LoopTimer(budget_sec=MyRobot.kDefaultPeriod / 1000)
        self.__timing.add_channel("userCode")
        self.__timing.add_channel("scheduler")

        scheduler = commands2.CommandScheduler.getInstance()
        original_run = scheduler.run
        timing = self.__timing

        def _timed_run():
            timing.start("scheduler")
            original_run()
            timing.stop("scheduler")

        scheduler.run = _timed_run

    def __frameTimingPeriodic(self):
        """Stop the userCode channel and publish all timing stats."""
        self.__timing.stop("userCode")
        self.__timing.publish()

    def callAndCatch(self, func: typing.Callable[[], None]) -> None:
        """Run func, logging instead of crashing on hardware.

        Exceptions are re-raised in simulation so tests and CI catch them.
        """
        try:
            func()

            # if we returned, it didnt crash so clear the last error if it was set
            if self.__errorLogged and self.__lastError is not None:
                logging.info(f"Logged error cleared for: {str(self.__lastError)}")
                self.__errorLogged = False
                self.__lastError = None
        except Exception as e:
            self.__lastError = e
            name = inspect.currentframe().f_back.f_code.co_name
            if self.isSimulation():
                raise e

            self.__errorCaughtCount = self.__errorCaughtCount + 1
            wpilib.SmartDashboard.putNumber("Code Crash Count", self.__errorCaughtCount)

            if not self.__errorLogged:
                logging.exception(f"(CRASH CATCH) {name} error: ")
                self.__errorLogged = True


if __name__ == "__main__":
    print("Please run python -m robotpy <args>")
    exit(1)
