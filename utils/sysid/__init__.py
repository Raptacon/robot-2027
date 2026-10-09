"""
A SysId tool any mechanism can use: drivetrain, flywheel, turret, arm or elevator.

SysId ("system identification") measures how many volts a mechanism needs
to move. It runs four short tests, logs voltage, position and velocity, and
the WPILib SysId app fits the feedforward numbers (kS, kV, kA) from the log.

How to add a mechanism:
    1. Describe it with the builder for its kind in :mod:`utils.sysid.mech`
       (flywheel, drivetrain, rotary, arm or elevator). Each builder gives
       safe defaults and asks for what that kind needs, like an arm's limits.
       Anything else can use a
       :class:`~utils.sysid.characterizable.Characterizable` directly.
    2. Wrap it in :class:`~utils.sysid.routines.SysIdTests`.
    3. Add it to the robot's
       :class:`~utils.sysid.chooser.CharacterizationChooser`; it then shows
       up on the dashboard's "Characterization" chooser in test mode.

See ``commands/drive/characterization.py`` for the swerve drive and steer,
which were the first two users.
"""
