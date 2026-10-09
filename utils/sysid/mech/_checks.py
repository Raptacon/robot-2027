"""Checks shared by the mechanism builders."""

import math


def check_limits(low: float | None, high: float | None, angular: bool) -> None:
    """Raise ``ValueError`` if position limits are backwards or look like degrees.

    Args:
        low: The lowest position allowed, or ``None``.
        high: The highest position allowed, or ``None``.
        angular: ``True`` if the limits are in radians.
    """
    if low is not None and high is not None and low >= high:
        raise ValueError(f"The low limit ({low}) must be below the high limit ({high})")
    if angular:
        for limit in (low, high):
            if limit is not None and abs(limit) > math.tau:
                raise ValueError(f"Limit {limit} is more than a full turn: use radians, not degrees")
