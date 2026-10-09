"""
Connect the driver's controller to the drive commands.

The button layout lives in ``data/inputs/swerve_test_bot.yaml`` (which
button does what); this file says what each action *does*. To move a
button, edit the YAML, not this file.

==========================  ===========================================
Action (YAML name)          What it does
==========================  ===========================================
drivetrain.translate_x/y    Left stick: drive (field-relative)
drivetrain.rotate           Right stick X: spin
drivetrain.robot_relative   Hold: drive robot-relative
drivetrain.slow_toggle      Press: slow mode on/off
drivetrain.reset_heading    Press: "the robot faces away from me now"
drivetrain.x_lock           Hold: wheels in an X
drivetrain.snap_0/90/...    Press: face away / left / toward / right
drivetrain.cancel_all       Press: cancel every running command
==========================  ===========================================

``drivetrain.auto_align`` stays unbound until vision is added.

Test mode has its own buttons (:func:`bind_test_controls`), which only work
while the robot is enabled in Test:

==============================  ==========================================
Action (YAML name)              What it does
==============================  ==========================================
characterization.run_test       Hold A: run the test picked on the
                                Characterization chooser; let go to stop
characterization.next_test      Press B: pick the next test
characterization.previous_test  Press left bumper: pick the previous test
==============================  ==========================================

Example:
    In ``robot.py`` (the InputFactory must be created before the drivetrain):

    .. code-block:: python

        self.inputs = InputFactory(config_path="data/inputs/swerve_test_bot.yaml")
        ...
        self.teleop = bind_driver_controls(self.inputs, self.drivetrain)
        self.characterization = bind_test_controls(self.inputs, self.drivetrain)
"""

import commands2
import wpilib
from commands2.button import Trigger

from commands.drive.calibrate_offsets import CalibrateOffsets
from commands.drive.characterization import register_swerve
from commands.drive.module_check import ModuleCheck
from commands.drive.teleop_drive import TeleopDrive, TeleopSettings
from commands.drive.x_lock import XLock
from subsystem.drivetrain.drivetrain import Drivetrain
from utils.input import InputFactory
from utils.sysid.chooser import CharacterizationChooser

SNAP_ACTIONS = {
    "drivetrain.snap_0": 0.0,
    "drivetrain.snap_90": 90.0,
    "drivetrain.snap_180": 180.0,
    "drivetrain.snap_270": 270.0,
}
"""Each heading-snap action and the direction it faces, in degrees from the driver's view."""


def bind_driver_controls(
    factory: InputFactory, drivetrain: Drivetrain, settings: TeleopSettings | None = None
) -> TeleopDrive:
    """Make TeleopDrive the drivetrain's default command and bind the driver's buttons.

    Args:
        factory: The robot's InputFactory (controller config).
        drivetrain: The drivetrain to drive.
        settings: Driver feel settings, or ``None`` for the defaults.

    Returns:
        The TeleopDrive command, so tests and the dashboard can look at it.
    """
    translate_x = factory.getAnalog("drivetrain.translate_x")
    translate_y = factory.getAnalog("drivetrain.translate_y")
    rotate = factory.getAnalog("drivetrain.rotate")
    robot_relative = factory.getButton("drivetrain.robot_relative")

    teleop = TeleopDrive(
        drivetrain,
        # Stick up reads -1 and stick right reads +1, so flip both: we want
        # up = forward (+) and left = left (+).
        forward=lambda: -translate_x(),
        left=lambda: -translate_y(),
        rotate=rotate,
        robot_relative=robot_relative.get,
        settings=settings,
    )
    drivetrain.setDefaultCommand(teleop)

    factory.getButton("drivetrain.slow_toggle").onTrue(commands2.InstantCommand(teleop.toggle_slow))
    factory.getButton("drivetrain.reset_heading").onTrue(commands2.InstantCommand(teleop.reset_heading))
    factory.getButton("drivetrain.x_lock").whileTrue(XLock(drivetrain))
    for action, degrees in SNAP_ACTIONS.items():
        factory.getButton(action).onTrue(commands2.InstantCommand(lambda d=degrees: teleop.snap_heading(d)))
    factory.getButton("drivetrain.cancel_all").onTrue(
        commands2.InstantCommand(commands2.CommandScheduler.getInstance().cancelAll)
    )
    return teleop


def bind_test_controls(factory: InputFactory, drivetrain: Drivetrain) -> CharacterizationChooser:
    """Set up test mode: the Characterization chooser, its buttons, and the offset calibration button.

    The chooser starts on the module check and also holds every swerve
    SysId and calibration test (see ``commands/drive/characterization.py``).
    Holding A runs the picked test and letting go stops it, so a person can
    always stop a test at once. The ``Characterization/Calibrate offsets``
    dashboard button works while disabled.

    Args:
        factory: The robot's InputFactory (controller config).
        drivetrain: The drivetrain the tests run on.

    Returns:
        The :class:`~utils.sysid.chooser.CharacterizationChooser`, so the
        robot can stop a running test when test mode ends.
    """
    chooser = CharacterizationChooser("Module check", ModuleCheck(drivetrain))
    register_swerve(chooser, drivetrain)
    chooser.publish()
    # Trigger(button.get) keeps working if the buttons are remapped at runtime.
    chooser.bind(
        run=Trigger(factory.getButton("characterization.run_test").get),
        next_option=Trigger(factory.getButton("characterization.next_test").get),
        previous_option=Trigger(factory.getButton("characterization.previous_test").get),
    )
    wpilib.SmartDashboard.putData("Characterization/Calibrate offsets", CalibrateOffsets(drivetrain))
    return chooser
