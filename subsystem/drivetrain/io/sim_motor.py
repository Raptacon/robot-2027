"""
A simple simulated motor, written in plain Python.

How it works
    A DC motor's speed follows the same equation SysId measures:

        volts = kS * sign(speed) + kV * speed + kA * acceleration

    kV is the voltage needed to hold a speed, kA is the extra voltage needed
    to speed up, and kS is the voltage needed just to get moving. Solving for
    acceleration and stepping forward in small time steps gives the motor's
    speed and position over time.

    We use our own model instead of WPILib's ``DCMotorSim`` because the same
    numbers come straight out of a SysId run (milestone M6), and because this
    file doesn't change when WPILib's API changes between seasons.

Example:
    A motor that needs 0.1 V per rotation/s, and 0.01 V per rotation/s^2:

    >>> from subsystem.drivetrain.io.sim_motor import SimMotor
    >>> motor = SimMotor(kv=0.1, ka=0.01)
    >>> for _ in range(1000):  # 1 second in 1 ms steps
    ...     motor.step(volts=6.0, dt_s=0.001)
    >>> round(motor.velocity, 1)  # top speed is 6 V / 0.1 = 60 rotations/s
    60.0
"""

import math

# REV NEO electrical constants, from REV's motor curve: 105 A stall at 12 V.
NEO_STALL_CURRENT_AMPS = 105.0
NOMINAL_VOLTS = 12.0


class SimMotor:
    """One simulated motor shaft, in rotations and rotations per second.

    Args:
        kv: Volts per (rotation per second) to hold a steady speed.
        ka: Volts per (rotation per second squared) to accelerate.
        ks: Volts needed to overcome friction and start moving. Default 0.
        free_speed_rps: The motor's free speed, used to estimate current.
            Defaults to the speed kv gives at 12 V.

    Attributes:
        position: Shaft position in rotations.
        velocity: Shaft speed in rotations per second.
        applied_volts: The voltage used in the last step (after limiting to
            +/- 12 V).
    """

    def __init__(self, kv: float, ka: float, ks: float = 0.0, free_speed_rps: float | None = None) -> None:
        self.kv = kv
        self.ka = ka
        self.ks = ks
        self._emf_per_rps = NOMINAL_VOLTS / (free_speed_rps or NOMINAL_VOLTS / kv)
        self._resistance_ohms = NOMINAL_VOLTS / NEO_STALL_CURRENT_AMPS
        self.position = 0.0
        self.velocity = 0.0
        self.applied_volts = 0.0

    def step(self, volts: float, dt_s: float) -> None:
        """Advance the motor by ``dt_s`` seconds with ``volts`` applied.

        Keep ``dt_s`` small compared with ka / kv (the motor's time constant);
        1 ms works for every module motor we simulate.
        """
        volts = max(-NOMINAL_VOLTS, min(NOMINAL_VOLTS, volts))
        self.applied_volts = volts
        friction = self.ks * math.copysign(1.0, self.velocity) if self.velocity else 0.0
        if not self.velocity and abs(volts) <= self.ks:
            # Not enough voltage to break free of friction.
            return
        accel = (volts - friction - self.kv * self.velocity) / self.ka
        self.velocity += accel * dt_s
        self.position += self.velocity * dt_s

    @property
    def current_amps(self) -> float:
        """Estimated current draw: (applied volts - back EMF) / winding resistance."""
        return abs(self.applied_volts - self._emf_per_rps * self.velocity) / self._resistance_ohms
