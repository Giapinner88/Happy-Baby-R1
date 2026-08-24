"""Conservative pre-solve workspace projection for R1 wrist targets."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .kinematics import ArmChain
from .upper_body_ik import UpperBodyIKTarget
from .upper_body_kinematics import R1A5UpperBodyModel


@dataclass(frozen=True)
class WorkspaceProjection:
    target: UpperBodyIKTarget
    left_projected: bool
    right_projected: bool
    left_projection_distance_m: float
    right_projection_distance_m: float
    left_reach_limit_m: float
    right_reach_limit_m: float

    @property
    def any_projected(self) -> bool:
        return self.left_projected or self.right_projected


def _project_to_reach_sphere(
    position: np.ndarray,
    shoulder: np.ndarray,
    radius: float,
) -> tuple[np.ndarray, bool, float]:
    delta = np.asarray(position, dtype=float) - np.asarray(shoulder, dtype=float)
    distance = float(np.linalg.norm(delta))
    if distance <= radius:
        return np.asarray(position, dtype=float).copy(), False, 0.0
    if distance <= 1e-12:
        return np.asarray(shoulder, dtype=float).copy(), False, 0.0
    projected = shoulder + delta * (radius / distance)
    return projected, True, distance - radius


def project_arm_position_to_reach_sphere(
    chain: ArmChain,
    position_m: np.ndarray,
    margin_m: float,
) -> tuple[np.ndarray, bool, float, float]:
    """Project one waist-frame arm target onto its URDF reach outer bound.

    TeleVuer's processed wrist poses and :class:`ArmChain` use the same
    ``waist_yaw_link`` root.  Keeping this one-arm form public lets offline
    continuation apply exactly the same pre-solve reach rule as the live
    upper-body controller without inventing a pelvis-frame transform.

    Returns ``(projected_position, was_projected, projection_distance,
    reach_limit)``.
    """

    if not np.isfinite(margin_m) or not 0.0 <= margin_m < chain.max_reach_from_shoulder_m:
        raise ValueError("margin_m must leave a positive finite arm reach.")
    radius = float(chain.max_reach_from_shoulder_m - margin_m)
    projected, changed, distance = _project_to_reach_sphere(
        np.asarray(position_m, dtype=float),
        chain.shoulder_origin(),
        radius,
    )
    return projected, changed, distance, radius


def project_upper_body_target(
    model: R1A5UpperBodyModel,
    q: np.ndarray,
    target: UpperBodyIKTarget,
    margin_m: float,
) -> WorkspaceProjection:
    """Project each wrist onto its URDF-derived conservative reach sphere.

    The sphere is an outer bound derived by triangle inequality over the arm
    geometry. Projection therefore removes targets that are definitely beyond
    reach before differential IK. It does not claim that every point inside the
    sphere is realizable under the complete joint limits.
    """

    if not np.isfinite(margin_m) or margin_m < 0.0:
        raise ValueError("margin_m must be finite and non-negative.")
    values = model.clamp(np.asarray(q, dtype=float))
    target.validate()
    waist = model.waist_transform_from_q(values)
    results: dict[str, object] = {}
    for side in ("left", "right"):
        chain = getattr(model, f"{side}_arm")
        radius = float(chain.max_reach_from_shoulder_m - margin_m)
        if radius <= 0.0:
            raise ValueError("margin_m leaves no positive arm reach.")
        shoulder = (waist @ np.append(chain.shoulder_origin(), 1.0))[:3]
        position, projected, distance = _project_to_reach_sphere(
            getattr(target, f"{side}_position_m"), shoulder, radius
        )
        results[f"{side}_position"] = position
        results[f"{side}_projected"] = projected
        results[f"{side}_distance"] = distance
        results[f"{side}_radius"] = radius
    projected_target = UpperBodyIKTarget(
        np.asarray(results["left_position"]),
        np.asarray(target.left_orientation),
        np.asarray(results["right_position"]),
        np.asarray(target.right_orientation),
        np.asarray(target.head_orientation),
    )
    return WorkspaceProjection(
        target=projected_target,
        left_projected=bool(results["left_projected"]),
        right_projected=bool(results["right_projected"]),
        left_projection_distance_m=float(results["left_distance"]),
        right_projection_distance_m=float(results["right_distance"]),
        left_reach_limit_m=float(results["left_radius"]),
        right_reach_limit_m=float(results["right_radius"]),
    )


__all__ = [
    "WorkspaceProjection",
    "project_arm_position_to_reach_sphere",
    "project_upper_body_target",
]
