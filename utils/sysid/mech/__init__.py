"""
One builder per kind of mechanism, so each gets safe SysId defaults.

Pick the one that matches your mechanism. Each returns a
:class:`~utils.sysid.characterizable.Characterizable` ready for
:class:`~utils.sysid.routines.SysIdTests`, and refuses to build if
something that kind of mechanism needs is missing (an arm without limits,
say).

- :mod:`~utils.sysid.mech.flywheel`: shooter wheels, rollers, intakes.
  Spins freely, nothing to run into.
- :mod:`~utils.sysid.mech.drivetrain`: the drive wheels. Carries the
  robot's weight, needs room, slips on big steps.
- :mod:`~utils.sysid.mech.rotary`: turrets, swerve steering. Turns, and
  gravity doesn't pull it around.
- :mod:`~utils.sysid.mech.arm`: arms and wrists. Turns, and gravity pulls
  hardest when it is level.
- :mod:`~utils.sysid.mech.elevator`: elevators and lifts. Moves in a line,
  and gravity pulls the same at every height.

Flywheel and drivetrain are both "spin a wheel", but a drivetrain carries
the robot, so it gets a smaller voltage step (to stop wheel slip) and a
travel limit (so it stops before the wall).
"""
