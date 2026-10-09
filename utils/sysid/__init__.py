"""
A SysId tool any mechanism can use: drive, steer, and later a flywheel or arm.

SysId ("system identification") measures how many volts a mechanism needs
to move. It runs four short tests, logs voltage, position and velocity, and
the WPILib SysId app fits the feedforward numbers (kS, kV, kA) from the log.

How to add a mechanism:
    1. Describe it with a :class:`~utils.sysid.characterizable.Characterizable`
       (how to apply volts and how to read it back).
    2. Wrap it in :class:`~utils.sysid.routines.SysIdTests`.
    3. Add it to the robot's
       :class:`~utils.sysid.chooser.CharacterizationChooser`; it then shows
       up on the dashboard's "Characterization" chooser in test mode.

See ``commands/drive/characterization.py`` for the swerve drive and steer,
which were the first two users.
"""
