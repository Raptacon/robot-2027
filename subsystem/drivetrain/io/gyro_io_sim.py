"""
A simulated gyro.

The simulated drivetrain (milestone M3) works out how fast the robot is
turning and calls :meth:`GyroIOSim.step` each loop; the gyro adds that up
into a yaw angle. Options copy real gyro problems for tests.

Example:
    A gyro whose readings arrive one loop late:

    >>> from subsystem.drivetrain.io.gyro_io import GyroInputs
    >>> from subsystem.drivetrain.io.gyro_io_sim import GyroIOSim
    >>> gyro = GyroIOSim(latency_loops=1)
    >>> inputs = GyroInputs()
    >>> gyro.step(yaw_rate_rad_per_s=2.0, dt_s=0.5)
    >>> gyro.update_inputs(inputs)
    >>> inputs.yaw_rad  # still the old reading
    0.0
    >>> gyro.update_inputs(inputs)
    >>> inputs.yaw_rad
    1.0
"""

from collections import deque
from collections.abc import Callable

from subsystem.drivetrain.io.gyro_io import GyroInputs, GyroIO


class GyroIOSim(GyroIO):
    """A :class:`~subsystem.drivetrain.io.gyro_io.GyroIO` for simulation.

    Args:
        latency_loops: How many loops late each reading arrives.
        boot_delay_loops: Loops after boot before the gyro reports
            ``connected``. Until then it reads 0.
        never_connects: ``True`` to simulate an unplugged gyro.
        drift_rad_per_s: Yaw drift added while the robot sits still, in
            radians per second. 1 degree per minute is about 0.0003 rad/s.
        before_update: A function called at the start of each
            :meth:`update_inputs`. :class:`~subsystem.drivetrain.drivetrain_sim.DrivetrainSim`
            uses it to call :meth:`step` with the robot's true motion just
            before the gyro is read, so the gyro and wheels agree in time.
    """

    def __init__(
        self,
        latency_loops: int = 0,
        boot_delay_loops: int = 0,
        never_connects: bool = False,
        drift_rad_per_s: float = 0.0,
        before_update: Callable[[], None] | None = None,
    ) -> None:
        self._boot_delay_loops = boot_delay_loops
        self._never_connects = never_connects
        self._drift_rad_per_s = drift_rad_per_s
        self._before_update = before_update
        self._loops = 0
        self._yaw_rad = 0.0
        self._true_yaw_rad = 0.0
        self._yaw_rate = 0.0
        # Start with blank readings so the first real one arrives `latency_loops` loops late.
        self._history: deque[GyroInputs] = deque([GyroInputs() for _ in range(latency_loops)], maxlen=latency_loops + 1)

    def step(self, yaw_rate_rad_per_s: float, dt_s: float) -> None:
        """Turn the simulated robot at ``yaw_rate_rad_per_s`` for ``dt_s`` seconds."""
        self._yaw_rate = yaw_rate_rad_per_s + self._drift_rad_per_s
        self._yaw_rad += self._yaw_rate * dt_s
        self._true_yaw_rad += yaw_rate_rad_per_s * dt_s

    @property
    def true_yaw_rad(self) -> float:
        """The robot's real heading change since boot, without drift (for tests)."""
        return self._true_yaw_rad

    def update_inputs(self, inputs: GyroInputs) -> None:
        if self._before_update is not None:
            self._before_update()
        self._loops += 1
        connected = not self._never_connects and self._loops > self._boot_delay_loops
        if connected:
            reading = GyroInputs(connected=True, yaw_rad=self._yaw_rad, yaw_rate_rad_per_s=self._yaw_rate)
        else:
            reading = GyroInputs(connected=False)
        self._history.append(reading)
        delayed = self._history[0]
        inputs.connected = delayed.connected
        inputs.yaw_rad = delayed.yaw_rad
        inputs.yaw_rate_rad_per_s = delayed.yaw_rate_rad_per_s
