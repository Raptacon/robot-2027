"""
The interface for one swerve module's two motors.

Each module has a *drive* motor (spins the wheel) and a *steer* motor (turns
the wheel to point somewhere). :class:`ModuleIO` lists everything the drive
code can ask of those motors; :class:`ModuleInputs` holds everything it can
read back.

Units are always SI: meters, meters per second, radians, volts, amps.

Example:
    Every loop, read the sensors once, then send commands:

    >>> from config.robots.swerve_test_bot import CONFIG
    >>> from subsystem.drivetrain.io.module_io import ModuleInputs
    >>> from subsystem.drivetrain.io.module_io_sim import ModuleIOSim
    >>> io = ModuleIOSim(CONFIG)
    >>> inputs = ModuleInputs()
    >>> io.update_inputs(inputs)  # read every sensor into `inputs`
    >>> inputs.drive_velocity_mps
    0.0
    >>> io.set_steer_angle(0.5)  # point the wheel 0.5 rad left of forward
    >>> io.set_drive_velocity(1.0)  # roll forward at 1 m/s
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass
class ModuleInputs:
    """Everything read from one module's motors in one loop.

    The IO fills this in with :meth:`ModuleIO.update_inputs`. Positive drive
    values mean the wheel rolls in the direction it points. Steer angles are
    counterclockwise-positive when seen from above, with 0 meaning the wheel
    points toward the front of the robot.

    Attributes:
        drive_connected: ``False`` if the drive motor controller isn't
            answering on CAN.
        drive_position_m: Total distance the wheel has rolled since boot, in
            meters. Odometry uses the change in this between loops.
        drive_velocity_mps: Wheel speed in meters per second.
        drive_applied_volts: Voltage the controller is sending to the drive motor.
        drive_current_amps: Current the drive motor is drawing.
        drive_temp_c: Drive motor temperature in degrees Celsius.
        steer_connected: ``False`` if the steer motor controller isn't answering.
        steer_angle_rad: Which way the wheel points, in radians from -pi to pi.
            This comes from the steer motor's own encoder after it has been
            set from the absolute encoder, and it is the only angle the drive
            code uses.
        steer_velocity_rad_per_s: How fast the wheel is turning to a new
            direction, in radians per second.
        steer_applied_volts: Voltage sent to the steer motor.
        steer_current_amps: Current the steer motor is drawing.
        steer_temp_c: Steer motor temperature in degrees Celsius.
    """

    drive_connected: bool = True
    drive_position_m: float = 0.0
    drive_velocity_mps: float = 0.0
    drive_applied_volts: float = 0.0
    drive_current_amps: float = 0.0
    drive_temp_c: float = 0.0
    steer_connected: bool = True
    steer_angle_rad: float = 0.0
    steer_velocity_rad_per_s: float = 0.0
    steer_applied_volts: float = 0.0
    steer_current_amps: float = 0.0
    steer_temp_c: float = 0.0


class ModuleIO(ABC):
    """What the drive code can ask of one module's drive and steer motors.

    This is an *abstract* class: it describes the methods but doesn't do
    anything itself. Real hardware and the simulator each provide a class
    that fills them in (for example :class:`~subsystem.drivetrain.io.module_io_sim.ModuleIOSim`).
    Code that uses a module only ever talks to a ``ModuleIO``, so it works the
    same on the robot and in simulation.
    """

    @abstractmethod
    def update_inputs(self, inputs: ModuleInputs) -> None:
        """Read every sensor and write the values into ``inputs``.

        Call this once at the start of each loop, before using ``inputs``.
        """

    @abstractmethod
    def set_drive_velocity(self, velocity_mps: float, feedforward_volts: float = 0.0) -> None:
        """Run the drive wheel at a speed using the motor controller's speed loop.

        Args:
            velocity_mps: Target wheel speed in meters per second.
            feedforward_volts: Extra voltage added to the controller's own
                correction. The module code works this out from the measured
                kS, kV and kA (milestone M6); 0 is fine before then.
        """

    @abstractmethod
    def set_drive_voltage(self, volts: float) -> None:
        """Send a fixed voltage to the drive motor (used by SysId tests)."""

    @abstractmethod
    def set_steer_angle(self, angle_rad: float) -> None:
        """Turn the wheel to point at an angle using the controller's position loop.

        Args:
            angle_rad: Target direction in radians, counterclockwise from
                forward. Any value works; the controller takes the short way
                around, so going from 170 to -170 degrees turns 20 degrees.
        """

    @abstractmethod
    def set_steer_voltage(self, volts: float) -> None:
        """Send a fixed voltage to the steer motor (used by SysId tests)."""

    @abstractmethod
    def reset_steer_angle(self, angle_rad: float) -> None:
        """Tell the steer motor's encoder which way the wheel points right now.

        At power-on the steer motor's encoder reads 0 no matter where the
        wheel points. The module code reads the absolute encoder and calls
        this to line the two up ("seeding").

        Args:
            angle_rad: The wheel's real direction, from the absolute encoder.
        """

    def stop(self) -> None:
        """Stop both motors by sending 0 volts."""
        self.set_drive_voltage(0.0)
        self.set_steer_voltage(0.0)
