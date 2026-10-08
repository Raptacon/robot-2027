#!/usr/bin/env python3

import inspect
import logging
import typing

import commands2
import wpilib

from subsystem.health_and_status import HealthAndStatus
from utils.datalog_bridge import setup_logging
from utils.loop_timing import LoopTimer


class MyRobot(commands2.TimedCommandRobot):
    """
    Our default robot class, pass it to wpilib.run
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    runs the command scheduler for you every loop.

    The seed robot has no drive subsystems yet. It sets up logging, loop
    timing and health telemetry; the swerve drivetrain is added on the
    swerve-dev branch.
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

        self.health = HealthAndStatus()

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
        self.__timing.reset_all()

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
