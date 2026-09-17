from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


HIP_SPACING_M = 0.040
PROXIMAL_LINK_M = 0.060
DISTAL_LINK_M = 0.100

HOME_LEG_LENGTH_M = 0.120
HOME_WHEEL_X_M = 0.0

# Static body rotations encoded in two_wheel_legged.xml.
FRONT_HOME_UPPER_ANGLE = -0.7925588591
FRONT_HOME_RELATIVE_KNEE_ANGLE = 1.4706289323
REAR_HOME_UPPER_ANGLE = 0.7925588591
REAR_HOME_RELATIVE_KNEE_ANGLE = -1.4706289323


@dataclass(frozen=True)
class FiveBarPose:
    front_hip: float
    rear_hip: float
    front_knee: float
    rear_knee: float


def _link_angle(dx: float, dz: float) -> float:
    """Return MuJoCo Y-axis angle for a link whose local axis points along -Z."""
    return math.atan2(-dx, -dz)


def _outward_elbow(
    pivot_x: float,
    target_x: float,
    target_z: float,
    outward_sign: float,
) -> tuple[float, float]:
    dx = target_x - pivot_x
    dz = target_z
    distance = math.hypot(dx, dz)

    minimum_reach = abs(PROXIMAL_LINK_M - DISTAL_LINK_M)
    maximum_reach = PROXIMAL_LINK_M + DISTAL_LINK_M
    if not minimum_reach < distance < maximum_reach:
        raise ValueError(
            "Five-bar target is unreachable or singular: "
            f"distance={distance:.6f} m, valid open interval="
            f"({minimum_reach:.6f}, {maximum_reach:.6f}) m"
        )

    along = (
        PROXIMAL_LINK_M**2
        - DISTAL_LINK_M**2
        + distance**2
    ) / (2.0 * distance)
    height = math.sqrt(max(0.0, PROXIMAL_LINK_M**2 - along**2))

    ux = dx / distance
    uz = dz / distance
    perpendicular_x = -uz
    perpendicular_z = ux

    candidate_a = (
        pivot_x + along * ux + height * perpendicular_x,
        along * uz + height * perpendicular_z,
    )
    candidate_b = (
        pivot_x + along * ux - height * perpendicular_x,
        along * uz - height * perpendicular_z,
    )

    if outward_sign * (candidate_a[0] - pivot_x) >= 0.0:
        return candidate_a
    return candidate_b


def solve_five_bar(
    leg_length_m: float,
    wheel_x_m: float = HOME_WHEEL_X_M,
) -> FiveBarPose:
    """
    Solve one five-bar leg.

    Args:
        leg_length_m:
            Vertical distance from the hip midpoint to the wheel centre.
        wheel_x_m:
            Wheel-centre X offset relative to the hip midpoint. Positive is
            forward.

    Returns:
        Joint offsets relative to the XML home pose. The two hip values are
        active targets; knee values are useful for validation because the
        MuJoCo equality closure determines them passively.
    """
    target_z = -float(leg_length_m)
    target_x = float(wheel_x_m)
    front_pivot_x = HIP_SPACING_M / 2.0
    rear_pivot_x = -HIP_SPACING_M / 2.0

    front_elbow_x, front_elbow_z = _outward_elbow(
        front_pivot_x,
        target_x,
        target_z,
        outward_sign=1.0,
    )
    rear_elbow_x, rear_elbow_z = _outward_elbow(
        rear_pivot_x,
        target_x,
        target_z,
        outward_sign=-1.0,
    )

    front_upper = _link_angle(
        front_elbow_x - front_pivot_x,
        front_elbow_z,
    )
    front_lower = _link_angle(
        target_x - front_elbow_x,
        target_z - front_elbow_z,
    )
    rear_upper = _link_angle(
        rear_elbow_x - rear_pivot_x,
        rear_elbow_z,
    )
    rear_lower = _link_angle(
        target_x - rear_elbow_x,
        target_z - rear_elbow_z,
    )

    return FiveBarPose(
        front_hip=front_upper - FRONT_HOME_UPPER_ANGLE,
        rear_hip=rear_upper - REAR_HOME_UPPER_ANGLE,
        front_knee=(
            front_lower
            - front_upper
            - FRONT_HOME_RELATIVE_KNEE_ANGLE
        ),
        rear_knee=(
            rear_lower
            - rear_upper
            - REAR_HOME_RELATIVE_KNEE_ANGLE
        ),
    )


def bilateral_hip_targets(
    left_leg_length_m: float,
    right_leg_length_m: float,
    wheel_x_m: float = HOME_WHEEL_X_M,
) -> np.ndarray:
    """Return actuator-order hip targets for the MuJoCo model."""
    left = solve_five_bar(left_leg_length_m, wheel_x_m)
    right = solve_five_bar(right_leg_length_m, wheel_x_m)
    return np.array(
        [
            left.front_hip,
            left.rear_hip,
            right.front_hip,
            right.rear_hip,
        ],
        dtype=np.float64,
    )

