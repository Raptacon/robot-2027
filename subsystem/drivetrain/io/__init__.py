"""
Hardware IO for the drivetrain: one interface per kind of device.

What is an "IO layer"?
    The drive code needs to read sensors and command motors, but it shouldn't
    care which brand of motor controller or encoder is on the robot. So each
    kind of device gets an *interface*: a class that lists what you can ask the
    device to do, without saying how.

    - :class:`~subsystem.drivetrain.io.module_io.ModuleIO`: a module's drive and
      steer motors.
    - :class:`~subsystem.drivetrain.io.abs_encoder_io.AbsoluteEncoderIO`: the
      absolute encoder that knows which way the wheel points at power-on.
    - :class:`~subsystem.drivetrain.io.gyro_io.GyroIO`: the gyro that measures
      which way the robot is facing.

    Each interface has implementations. The *sim* ones (files ending
    ``_sim.py``) run on any computer and in CI. Real hardware ones (files ending
    ``_spark.py``, ``_cancoder.py`` and so on) come in milestone M5. Vendor
    libraries are only imported in those hardware files.

Reading sensors: the "inputs" pattern
    Each interface has a matching *inputs* dataclass, like
    :class:`~subsystem.drivetrain.io.module_io.ModuleInputs`. Once per loop the
    drive code calls ``io.update_inputs(inputs)``, which fills in every sensor
    reading at once. The rest of the loop only reads from that ``inputs``
    object, and the same object is logged. That way the log shows exactly what
    the code saw, and each signal is read from the CAN bus only once per loop.
"""
