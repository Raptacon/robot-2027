"""
Publish an inputs dataclass to NetworkTables so it is logged every loop.

Why?
    The IO layer fills an *inputs* dataclass with every sensor reading once
    per loop (see :mod:`subsystem.drivetrain.io`). Publishing that same object
    means the wpilog (which records all of NetworkTables) shows exactly what
    the code saw, and AdvantageScope can graph any field.

    Each field becomes one NetworkTables value under a prefix, for example
    ``/Drive/Module/frontLeft/drive_velocity_mps``. Fields that are ``float``,
    ``int`` or ``bool`` are supported.

Example:
    >>> from dataclasses import dataclass
    >>> import ntcore
    >>> from utils.inputs_publisher import InputsPublisher
    >>> @dataclass
    ... class ArmInputs:
    ...     angle_rad: float = 0.0
    ...     at_limit: bool = False
    >>> inst = ntcore.NetworkTableInstance.create()
    >>> publisher = InputsPublisher("/Arm", ArmInputs, inst=inst)
    >>> publisher.publish(ArmInputs(angle_rad=1.5, at_limit=True))
    >>> inst.getDoubleTopic("/Arm/angle_rad").subscribe(0.0).get()
    1.5
    >>> ntcore.NetworkTableInstance.destroy(inst)
"""

import dataclasses
import typing

import ntcore


class InputsPublisher:
    """Publishes every field of one inputs dataclass under a NetworkTables prefix.

    Create one per device (for example one per swerve module) when the robot
    starts, then call :meth:`publish` once per loop right after
    ``update_inputs``.

    Args:
        prefix: The NetworkTables folder, like ``"/Drive/Module/frontLeft"``.
        inputs_type: The dataclass type (not an instance), like ``ModuleInputs``.
        inst: The NetworkTables instance. Leave out to use the robot's default.

    Raises:
        TypeError: If a field isn't ``float``, ``int`` or ``bool``.
    """

    def __init__(self, prefix: str, inputs_type: type, inst: ntcore.NetworkTableInstance | None = None) -> None:
        if inst is None:
            inst = ntcore.NetworkTableInstance.getDefault()
        hints = typing.get_type_hints(inputs_type)
        self._publishers: list[tuple[str, typing.Any]] = []
        for field in dataclasses.fields(inputs_type):
            topic_name = f"{prefix}/{field.name}"
            kind = hints[field.name]
            if kind is bool:
                pub = inst.getBooleanTopic(topic_name).publish()
            elif kind is int:
                pub = inst.getIntegerTopic(topic_name).publish()
            elif kind is float:
                pub = inst.getDoubleTopic(topic_name).publish()
            else:
                raise TypeError(f"{inputs_type.__name__}.{field.name}: can't publish type {kind!r}")
            self._publishers.append((field.name, pub))

    def publish(self, inputs: typing.Any) -> None:
        """Send every field of ``inputs`` to NetworkTables."""
        for name, pub in self._publishers:
            pub.set(getattr(inputs, name))
