"""
The swerve drivetrain.

- :mod:`subsystem.drivetrain.io`: talks to the hardware (or the simulator).
  Everything else in the drivetrain goes through these interfaces, so the
  drive code never imports REV, CTRE or NavX code directly.
"""
