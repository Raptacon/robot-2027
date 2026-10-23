"""
The simulator's physics: how the robot's motors, wheels and sensors behave in ``robotpy sim`` and in tests.

Nothing in here is used by the robot code. In simulation the robot code runs
exactly as it does on the robot: it creates real ``rev.SparkMax``,
``phoenix6.hardware.CANcoder`` and gyro objects. Each vendor library has a
simulated backend behind those objects, and the physics model here reads
what the motor controllers are told to do and writes back what the encoders
and gyro would see.

- :mod:`sim.motor_plant`: one motor's physics (volts in, speed out).
- :mod:`sim.swerve_physics`: the swerve drivetrain's physics, the
  connection to the vendor simulators, and the robot's true position.

``physics.py`` at the top of the repo hands :class:`sim.swerve_physics.SwervePhysics`
to ``robotpy sim``; tests build it themselves (see ``tests/drivetrain/sim_harness.py``).
"""
