"""Offline position-manifold continuation for recorded R1 wrist trajectories.

The live controller only knows the past.  Recorded-data validation can use the
complete trace to select one connected posture branch, then refine it with both
temporal neighbours.  This module deliberately solves each five-DoF arm
independently; head angles are already directly observable and the torso is
fixed in the T007 ``arms_head`` protocol.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .kinematics import ArmChain
from .upper_body_ik import so3_log


@dataclass(frozen=True)
class OfflineContinuationConfig:
    position_weight: float = 1000.0
    orientation_weight: float = 0.01
    velocity_weight: float = 0.15
    acceleration_weight: float = 0.002
    posture_weight: float = 0.02
    elbow_outward_weight: float = 0.0
    elbow_forward_up_weight: float = 0.0
    elbow_direction_minimum_pole_m: float = 0.01
    elbow_direction_maximum_straightness_deg: float = 165.0
    elbow_direction_violation_tolerance_m: float = 0.005
    elbow_direction_margin_m: float = 0.0
    """Safety margin pushing the elbow inside its natural sector, metres.

    The sector cost is one-sided: it penalises ``max(0, pole_x)``. With no
    margin the optimum sits exactly on the constraint boundary, and run
    ``t007_soft_straightness_continuation_20260823_v23b`` did exactly that:
    279 left and 360 right samples landed at ``pole_x`` within 0.5 mm of zero.
    A 2 deg elbow tracking error then moved that whole block to +7.5 mm in
    PhysX, past the 5 mm classification tolerance, so the offline posture
    policy did not survive physics.

    A positive margin penalises ``max(0, pole_x + margin)`` instead, so the
    solver settles that far inside the sector and a small tracking error no
    longer changes the classification. ``0.0`` reproduces the boundary-hugging
    behaviour.
    """
    elbow_direction_straightness_softening_deg: float = 0.0
    """Half-width of the linear ramp that replaces the hard straightness cutoff.

    ``0.0`` keeps the original hard cutoff. A positive value ramps a sample's
    contribution from full at ``max_straightness - width`` down to zero at
    ``max_straightness + width``, so no sample flips on an arbitrarily small
    perturbation. See the rationale in `solve_r1_t007_offline_continuation`.
    """
    extended_arm_straightness_weight: float = 0.0
    extended_arm_reach_start_m: float = 0.45
    extended_arm_reach_full_m: float = 0.55
    bilateral_symmetry_weight: float = 0.0
    bilateral_target_mirror_tolerance_m: float = 0.12
    maximum_candidate_forward_or_up_fraction: float = 0.15
    initial_posture_score_weight: float = 0.2
    straightness_score_weight: float = 0.15
    branch_jump_score_weight: float = 0.2
    anchor_straightness_weights: tuple[float, ...] = (0.0, 20.0, 80.0, 160.0)
    anchor_position_candidate_window_m: float = 0.08
    minimum_anchor_straightness_deg: float = 150.0
    maximum_step_rad: float = 0.15
    anchor_multistart_count: int = 48
    retained_anchor_candidates: int = 6
    continuation_max_nfev: int = 50
    refinement_sweeps: int = 3
    manifold_reprojection_sweeps: int = 0
    manifold_reprojection_continuity_weight: float = 0.3162277660
    random_seed: int = 7

    def validate(self) -> None:
        positive = (
            "position_weight",
            "velocity_weight",
            "initial_posture_score_weight",
            "straightness_score_weight",
            "branch_jump_score_weight",
            "manifold_reprojection_continuity_weight",
        )
        for name in positive:
            value = float(getattr(self, name))
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive.")
        for name in (
            "orientation_weight",
            "acceleration_weight",
            "posture_weight",
            "elbow_outward_weight",
            "elbow_forward_up_weight",
            "extended_arm_straightness_weight",
            "bilateral_symmetry_weight",
        ):
            value = float(getattr(self, name))
            if not np.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative.")
        if not (
            np.isfinite(self.extended_arm_reach_start_m)
            and np.isfinite(self.extended_arm_reach_full_m)
            and 0.0 <= self.extended_arm_reach_start_m
            < self.extended_arm_reach_full_m
        ):
            raise ValueError(
                "extended arm reach thresholds must be finite with 0 <= start < full."
            )
        if not np.isfinite(self.bilateral_target_mirror_tolerance_m) or (
            self.bilateral_target_mirror_tolerance_m <= 0.0
        ):
            raise ValueError(
                "bilateral_target_mirror_tolerance_m must be finite and positive."
            )
        if not np.isfinite(self.maximum_candidate_forward_or_up_fraction) or not (
            0.0 <= self.maximum_candidate_forward_or_up_fraction <= 1.0
        ):
            raise ValueError(
                "maximum_candidate_forward_or_up_fraction must be in [0, 1]."
            )
        if not np.isfinite(self.elbow_direction_minimum_pole_m) or (
            self.elbow_direction_minimum_pole_m < 0.0
        ):
            raise ValueError(
                "elbow_direction_minimum_pole_m must be finite and non-negative."
            )
        if not np.isfinite(self.elbow_direction_maximum_straightness_deg) or not (
            0.0 <= self.elbow_direction_maximum_straightness_deg <= 180.0
        ):
            raise ValueError(
                "elbow_direction_maximum_straightness_deg must be in [0, 180]."
            )
        if not np.isfinite(self.elbow_direction_margin_m) or (
            self.elbow_direction_margin_m < 0.0
        ):
            raise ValueError("elbow_direction_margin_m must be finite and non-negative.")
        if not np.isfinite(self.elbow_direction_straightness_softening_deg) or (
            self.elbow_direction_straightness_softening_deg < 0.0
        ):
            raise ValueError(
                "elbow_direction_straightness_softening_deg must be finite and non-negative."
            )
        if not np.isfinite(self.elbow_direction_violation_tolerance_m) or (
            self.elbow_direction_violation_tolerance_m < 0.0
        ):
            raise ValueError(
                "elbow_direction_violation_tolerance_m must be finite and non-negative."
            )
        for name in (
            "anchor_multistart_count",
            "retained_anchor_candidates",
            "continuation_max_nfev",
        ):
            if int(getattr(self, name)) <= 0:
                raise ValueError(f"{name} must be positive.")
        if self.retained_anchor_candidates > self.anchor_multistart_count:
            raise ValueError(
                "retained_anchor_candidates cannot exceed anchor_multistart_count."
            )
        if int(self.refinement_sweeps) < 0:
            raise ValueError("refinement_sweeps must be non-negative.")
        if int(self.manifold_reprojection_sweeps) < 0:
            raise ValueError("manifold_reprojection_sweeps must be non-negative.")
        straightness_weights = np.asarray(self.anchor_straightness_weights, dtype=float)
        if (
            straightness_weights.ndim != 1
            or len(straightness_weights) == 0
            or np.any(~np.isfinite(straightness_weights))
            or np.any(straightness_weights < 0.0)
        ):
            raise ValueError("anchor_straightness_weights must be finite and non-negative.")
        if not np.isfinite(self.anchor_position_candidate_window_m) or not (
            0.0 < self.anchor_position_candidate_window_m <= 0.2
        ):
            raise ValueError("anchor_position_candidate_window_m must be in (0, 0.2].")
        if not np.isfinite(self.minimum_anchor_straightness_deg) or not (
            0.0 <= self.minimum_anchor_straightness_deg <= 180.0
        ):
            raise ValueError("minimum_anchor_straightness_deg must be in [0, 180].")
        if not np.isfinite(self.maximum_step_rad) or self.maximum_step_rad <= 0.0:
            raise ValueError("maximum_step_rad must be finite and positive.")


@dataclass(frozen=True)
class OfflineArmTrajectoryResult:
    q_rad: np.ndarray
    position_error_m: np.ndarray
    orientation_error_rad: np.ndarray
    straightness_deg: np.ndarray
    selected_anchor_rank: int
    anchor_index: int
    candidate_scores: tuple[dict[str, float], ...]


def arm_straightness_deg(chain: ArmChain, q: np.ndarray) -> float:
    """Shoulder–elbow–virtual-EE angle; 180 degrees is straight."""

    values = np.asarray(q, dtype=float)
    shoulder = chain.shoulder_origin()
    elbow = chain.link_transforms(values)[3][:3, 3]
    endpoint = chain.endpoint_position(values)
    upper = shoulder - elbow
    lower = endpoint - elbow
    denominator = float(np.linalg.norm(upper) * np.linalg.norm(lower))
    if denominator <= 1.0e-12:
        return float("nan")
    cosine = float(np.clip(np.dot(upper, lower) / denominator, -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def elbow_outward_pole_m(chain: ArmChain, q: np.ndarray) -> float:
    """Signed elbow-pole displacement; positive points away from the torso.

    The elbow is projected off the shoulder-to-endpoint line before its lateral
    component is measured.  Left uses +y and right uses -y.  Each arm is still
    solved independently; the opposite arm never appears in this quantity.
    """

    pole = elbow_pole_vector_m(chain, q)
    outward_sign = 1.0 if chain.side == "left" else -1.0
    return float(outward_sign * pole[1])


def elbow_pole_vector_m(chain: ArmChain, q: np.ndarray) -> np.ndarray:
    """Elbow offset from the shoulder-to-endpoint ray in the robot frame.

    R1 uses +x forward, +y left and +z up.  The neutral URDF posture places
    this vector backward (-x), outward (side-aware y), and down (-z).
    """

    values = np.asarray(q, dtype=float)
    shoulder = chain.shoulder_origin()
    elbow = chain.link_transforms(values)[3][:3, 3]
    endpoint = chain.endpoint_position(values)
    shoulder_to_endpoint = endpoint - shoulder
    denominator = float(shoulder_to_endpoint @ shoulder_to_endpoint)
    if denominator <= 1.0e-12:
        return np.zeros(3, dtype=float)
    return elbow - (
        shoulder
        + shoulder_to_endpoint
        * float(np.dot(elbow - shoulder, shoulder_to_endpoint) / denominator)
    )


def _elbow_branch_residual(
    chain: ArmChain, q: np.ndarray, config: OfflineContinuationConfig
) -> np.ndarray | None:
    if (
        config.elbow_outward_weight <= 0.0
        and config.elbow_forward_up_weight <= 0.0
    ):
        return None
    pole = elbow_pole_vector_m(chain, q)
    # The margin shifts each one-sided penalty so its zero-cost region starts
    # inside the natural sector rather than on its boundary.
    margin = float(config.elbow_direction_margin_m)
    inward_pole_m = max(0.0, -elbow_outward_pole_m(chain, q) + margin)
    return np.asarray(
        [
            config.elbow_outward_weight * inward_pole_m,
            config.elbow_forward_up_weight * max(0.0, float(pole[0]) + margin),
            config.elbow_forward_up_weight * max(0.0, float(pole[2]) + margin),
        ]
    )


def _validate_trace(
    chain: ArmChain,
    time_s: np.ndarray,
    positions_m: np.ndarray,
    orientations: np.ndarray,
    nominal_q: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    time = np.asarray(time_s, dtype=float)
    positions = np.asarray(positions_m, dtype=float)
    rotations = np.asarray(orientations, dtype=float)
    nominal = np.asarray(nominal_q, dtype=float)
    count = len(time)
    if count < 3 or time.shape != (count,) or np.any(np.diff(time) <= 0.0):
        raise ValueError("time_s must contain at least three strictly increasing samples.")
    if positions.shape != (count, 3) or not np.all(np.isfinite(positions)):
        raise ValueError("positions_m must be a finite (N, 3) array.")
    if rotations.shape != (count, 3, 3) or not np.all(np.isfinite(rotations)):
        raise ValueError("orientations must be a finite (N, 3, 3) array.")
    if nominal.shape != (chain.dof,) or not np.all(np.isfinite(nominal)):
        raise ValueError(f"nominal_q must be a finite ({chain.dof},)-vector.")
    return time, positions, rotations, chain.clamp(nominal)


def _pose_residual(
    chain: ArmChain,
    q: np.ndarray,
    position: np.ndarray,
    orientation: np.ndarray,
    config: OfflineContinuationConfig,
) -> list[np.ndarray]:
    pose = chain.forward_kinematics(q)
    residuals = [config.position_weight * (pose[:3, 3] - position)]
    if config.orientation_weight > 0.0:
        residuals.append(
            config.orientation_weight * so3_log(orientation @ pose[:3, :3].T)
        )
    if config.extended_arm_straightness_weight > 0.0:
        reach_m = float(np.linalg.norm(position - chain.shoulder_origin()))
        activation = float(
            np.clip(
                (reach_m - config.extended_arm_reach_start_m)
                / (
                    config.extended_arm_reach_full_m
                    - config.extended_arm_reach_start_m
                ),
                0.0,
                1.0,
            )
        )
        if activation > 0.0:
            bend_rad = np.deg2rad(180.0 - arm_straightness_deg(chain, q))
            residuals.append(
                np.asarray(
                    [
                        config.extended_arm_straightness_weight
                        * activation
                        * bend_rad
                    ]
                )
            )
    return residuals


def _bounded_least_squares(
    residual_function: object,
    seed: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    maximum_iterations: int,
) -> np.ndarray:
    """Small deterministic box-constrained damped Gauss–Newton solve.

    Keeping this dependency-free is important because the repository's system
    Python intentionally does not share the Isaac environment's SciPy/NumPy
    ABI.  The problem has only five variables, so central differences and a
    backtracking line search are adequate for offline continuation.
    """

    q = np.clip(np.asarray(seed, dtype=float), lower, upper)
    residual = np.asarray(residual_function(q), dtype=float)
    score = float(residual @ residual)
    best_q = q.copy()
    best_score = score
    damping = 1.0e-4
    finite_difference = 1.0e-5
    for _ in range(int(maximum_iterations)):
        jacobian = np.zeros((len(residual), len(q)), dtype=float)
        for index in range(len(q)):
            plus = q.copy()
            minus = q.copy()
            plus[index] = min(upper[index], plus[index] + finite_difference)
            minus[index] = max(lower[index], minus[index] - finite_difference)
            span = plus[index] - minus[index]
            if span > 0.0:
                jacobian[:, index] = (
                    np.asarray(residual_function(plus), dtype=float)
                    - np.asarray(residual_function(minus), dtype=float)
                ) / span
        hessian = jacobian.T @ jacobian + damping * np.eye(len(q))
        try:
            step = -np.linalg.solve(hessian, jacobian.T @ residual)
        except np.linalg.LinAlgError:
            damping *= 10.0
            continue
        step_norm = float(np.linalg.norm(step))
        if step_norm > 0.35:
            step *= 0.35 / step_norm
        accepted = False
        for scale in (1.0, 0.5, 0.25, 0.125, 0.0625):
            candidate = np.clip(q + scale * step, lower, upper)
            candidate_residual = np.asarray(residual_function(candidate), dtype=float)
            candidate_score = float(candidate_residual @ candidate_residual)
            if candidate_score < score - 1.0e-12:
                q = candidate
                residual = candidate_residual
                score = candidate_score
                if score < best_score:
                    best_q = q.copy()
                    best_score = score
                damping = max(1.0e-9, damping * 0.5)
                accepted = True
                break
        if not accepted:
            damping *= 10.0
        if float(np.max(np.abs(step))) < 1.0e-8 or best_score < 1.0e-14:
            break
    return best_q


def _anchor_candidates(
    chain: ArmChain,
    position: np.ndarray,
    orientation: np.ndarray,
    nominal: np.ndarray,
    config: OfflineContinuationConfig,
) -> list[np.ndarray]:
    rng = np.random.default_rng(config.random_seed + (0 if chain.side == "left" else 1009))
    seeds = [nominal.copy()]
    for _ in range(config.anchor_multistart_count - 1):
        seeds.append(rng.uniform(chain.lower_limits, chain.upper_limits))
    candidates: list[np.ndarray] = []
    straightness_weights = tuple(float(value) for value in config.anchor_straightness_weights)
    for seed_index, seed in enumerate(seeds):
        straightness_weight = straightness_weights[seed_index % len(straightness_weights)]

        def anchor_residual(values: np.ndarray) -> np.ndarray:
            residuals = _pose_residual(
                chain, values, position, orientation, config
            )
            if straightness_weight > 0.0:
                deficit_rad = np.deg2rad(
                    180.0 - arm_straightness_deg(chain, values)
                )
                residuals.append(np.asarray([straightness_weight * deficit_rad]))
            elbow_residual = _elbow_branch_residual(chain, values, config)
            if elbow_residual is not None:
                residuals.append(elbow_residual)
            return np.concatenate(residuals)

        q = _bounded_least_squares(
            anchor_residual,
            seed,
            chain.lower_limits,
            chain.upper_limits,
            config.continuation_max_nfev,
        )
        if not any(float(np.max(np.abs(q - other))) < 1.0e-3 for other in candidates):
            candidates.append(q)
    candidates.sort(
        key=lambda q: (
            float(np.linalg.norm(chain.endpoint_position(q) - position)),
            -arm_straightness_deg(chain, q),
        )
    )
    # Retain the declared Cartesian-error window, then let whole-path score
    # select the connected branch.  The window makes the position-versus-pose
    # trade-off explicit instead of hiding it inside a single anchor weight.
    best_error = float(np.linalg.norm(chain.endpoint_position(candidates[0]) - position))
    accurate = [
        q
        for q in candidates
        if float(np.linalg.norm(chain.endpoint_position(q) - position))
        <= best_error + config.anchor_position_candidate_window_m
    ]
    retained: list[np.ndarray] = []

    def append_unique(candidate: np.ndarray) -> None:
        if len(retained) >= config.retained_anchor_candidates:
            return
        if not any(float(np.max(np.abs(candidate - other))) < 1.0e-3 for other in retained):
            retained.append(candidate)

    # Preserve both ends of the scientific trade-off.  Retaining only the
    # straightest anchor silently discards the branch that connects back to the
    # measured session posture; retaining only the nearest posture reproduces
    # the bent-elbow failure.
    append_unique(min(accurate, key=lambda q: float(np.linalg.norm(q - nominal))))
    append_unique(max(accurate, key=lambda q: arm_straightness_deg(chain, q)))
    for candidate in sorted(
        accurate,
        key=lambda q: (
            float(np.linalg.norm(chain.endpoint_position(q) - position))
            + 0.01 * float(np.linalg.norm(q - nominal))
            - 0.001 * arm_straightness_deg(chain, q)
        ),
    ):
        append_unique(candidate)
    return retained


def _solve_local(
    chain: ArmChain,
    seed: np.ndarray,
    position: np.ndarray,
    orientation: np.ndarray,
    posture: np.ndarray,
    neighbours: tuple[tuple[np.ndarray, float], ...],
    acceleration_neighbours: tuple[np.ndarray, np.ndarray, float, float] | None,
    config: OfflineContinuationConfig,
) -> np.ndarray:
    def residual(q: np.ndarray) -> np.ndarray:
        values = _pose_residual(chain, q, position, orientation, config)
        for neighbour, dt in neighbours:
            values.append(config.velocity_weight * (q - neighbour) / dt)
        if acceleration_neighbours is not None and config.acceleration_weight > 0.0:
            previous, following, dt_previous, dt_following = acceleration_neighbours
            v_previous = (q - previous) / dt_previous
            v_following = (following - q) / dt_following
            acceleration = 2.0 * (v_following - v_previous) / (
                dt_previous + dt_following
            )
            values.append(config.acceleration_weight * acceleration)
        if config.posture_weight > 0.0:
            values.append(config.posture_weight * (q - posture))
        elbow_residual = _elbow_branch_residual(chain, q, config)
        if elbow_residual is not None:
            values.append(elbow_residual)
        return np.concatenate(values)

    lower = chain.lower_limits.copy()
    upper = chain.upper_limits.copy()
    for neighbour, _ in neighbours:
        lower = np.maximum(lower, neighbour - config.maximum_step_rad)
        upper = np.minimum(upper, neighbour + config.maximum_step_rad)
    if np.any(lower > upper + 1.0e-12):
        raise RuntimeError("continuation neighbours imply incompatible branch-step bounds")
    return _bounded_least_squares(
        residual,
        np.clip(seed, lower, upper),
        lower,
        upper,
        config.continuation_max_nfev,
    )


def _continue_from_anchor(
    chain: ArmChain,
    time: np.ndarray,
    positions: np.ndarray,
    orientations: np.ndarray,
    anchor_index: int,
    anchor_q: np.ndarray,
    nominal_q: np.ndarray,
    config: OfflineContinuationConfig,
) -> np.ndarray:
    count = len(time)
    path = np.empty((count, chain.dof), dtype=float)
    path[anchor_index] = anchor_q
    posture = nominal_q
    for index in range(anchor_index + 1, count):
        dt = float(time[index] - time[index - 1])
        path[index] = _solve_local(
            chain,
            path[index - 1],
            positions[index],
            orientations[index],
            posture,
            ((path[index - 1], dt),),
            None,
            config,
        )
    for index in range(anchor_index - 1, -1, -1):
        dt = float(time[index + 1] - time[index])
        path[index] = _solve_local(
            chain,
            path[index + 1],
            positions[index],
            orientations[index],
            posture,
            ((path[index + 1], dt),),
            None,
            config,
        )
    return path


def _refine_path(
    chain: ArmChain,
    time: np.ndarray,
    positions: np.ndarray,
    orientations: np.ndarray,
    path: np.ndarray,
    posture: np.ndarray,
    anchor_index: int,
    config: OfflineContinuationConfig,
) -> np.ndarray:
    refined = path.copy()
    count = len(time)
    for sweep in range(config.refinement_sweeps):
        indices = range(1, count - 1) if sweep % 2 == 0 else range(count - 2, 0, -1)
        for index in indices:
            if index == anchor_index:
                continue
            dt_previous = float(time[index] - time[index - 1])
            dt_following = float(time[index + 1] - time[index])
            refined[index] = _solve_local(
                chain,
                refined[index],
                positions[index],
                orientations[index],
                posture,
                (
                    (refined[index - 1], dt_previous),
                    (refined[index + 1], dt_following),
                ),
                (
                    refined[index - 1],
                    refined[index + 1],
                    dt_previous,
                    dt_following,
                ),
                config,
            )
    return refined


def _reproject_position_manifold(
    chain: ArmChain,
    time: np.ndarray,
    positions: np.ndarray,
    orientations: np.ndarray,
    path: np.ndarray,
    posture: np.ndarray,
    config: OfflineContinuationConfig,
) -> np.ndarray:
    """Re-open local branches at every node, then select by trace continuity.

    Anchor continuation alone can remain trapped on a connected but spatially
    poor branch after a fast gesture transition.  For recorded-data analysis
    we can use both temporal neighbours: solve the position-manifold node from
    the current, neutral, previous and following postures, then choose the
    lowest target-plus-continuity cost.  The direct joint-difference term is
    the offline bilateral-independent analogue of upstream R1-A5's
    ``0.1 * ||q - q_last||^2``.
    """

    if config.manifold_reprojection_sweeps <= 0:
        return path
    projected = path.copy()

    def node_residual(index: int, q: np.ndarray) -> np.ndarray:
        residuals = _pose_residual(
            chain, q, positions[index], orientations[index], config
        )
        if config.posture_weight > 0.0:
            residuals.append(config.posture_weight * (q - posture))
        elbow = _elbow_branch_residual(chain, q, config)
        if elbow is not None:
            residuals.append(elbow)
        return np.concatenate(residuals)

    for sweep in range(config.manifold_reprojection_sweeps):
        order = range(len(time)) if sweep % 2 == 0 else range(len(time) - 1, -1, -1)
        for index in order:
            neighbours: list[np.ndarray] = []
            if index > 0:
                neighbours.append(projected[index - 1])
            if index + 1 < len(time):
                neighbours.append(projected[index + 1])
            seeds = [projected[index], posture, *neighbours]
            candidates: list[np.ndarray] = []
            for seed in seeds:
                candidate = _bounded_least_squares(
                    lambda q, i=index: node_residual(i, q),
                    seed,
                    chain.lower_limits,
                    chain.upper_limits,
                    config.continuation_max_nfev,
                )
                if not any(
                    float(np.max(np.abs(candidate - other))) < 1.0e-4
                    for other in candidates
                ):
                    candidates.append(candidate)

            def candidate_score(candidate: np.ndarray) -> float:
                residual = node_residual(index, candidate)
                score = float(residual @ residual)
                if neighbours:
                    continuity = np.concatenate(
                        [candidate - neighbour for neighbour in neighbours]
                    )
                    score += float(
                        config.manifold_reprojection_continuity_weight**2
                        * (continuity @ continuity)
                        / len(neighbours)
                    )
                return score

            projected[index] = min(candidates, key=candidate_score)
    return projected


def reproject_position_manifold(
    chain: ArmChain,
    time_s: np.ndarray,
    positions_m: np.ndarray,
    orientations: np.ndarray,
    initial_path_rad: np.ndarray,
    nominal_q: np.ndarray,
    config: OfflineContinuationConfig,
) -> np.ndarray:
    """Public, validated entry point for a trace-level reprojection rerun."""

    config.validate()
    time, positions, rotations, nominal = _validate_trace(
        chain, time_s, positions_m, orientations, nominal_q
    )
    path = np.asarray(initial_path_rad, dtype=float)
    if path.shape != (len(time), chain.dof) or not np.all(np.isfinite(path)):
        raise ValueError("initial_path_rad must be a finite (N, arm_dof) array")
    if np.any(path < chain.lower_limits) or np.any(path > chain.upper_limits):
        raise ValueError("initial_path_rad exceeds arm joint limits")
    return _reproject_position_manifold(
        chain, time, positions, rotations, path, nominal, config
    )


def _path_metrics(
    chain: ArmChain,
    time: np.ndarray,
    positions: np.ndarray,
    orientations: np.ndarray,
    nominal: np.ndarray,
    path: np.ndarray,
    anchor_index: int,
    config: OfflineContinuationConfig,
) -> tuple[float, dict[str, float]]:
    poses = [chain.forward_kinematics(q) for q in path]
    position_error = np.asarray(
        [np.linalg.norm(pose[:3, 3] - target) for pose, target in zip(poses, positions)]
    )
    orientation_error = np.asarray(
        [
            np.linalg.norm(so3_log(target @ pose[:3, :3].T))
            for pose, target in zip(poses, orientations)
        ]
    )
    straightness = np.asarray([arm_straightness_deg(chain, q) for q in path])
    outward_pole = np.asarray([elbow_outward_pole_m(chain, q) for q in path])
    pole_vectors = np.asarray([elbow_pole_vector_m(chain, q) for q in path])
    direction_defined = (
        np.linalg.norm(pole_vectors, axis=1)
        >= config.elbow_direction_minimum_pole_m
    ) & (straightness <= config.elbow_direction_maximum_straightness_deg)

    def defined_fraction(mask: np.ndarray) -> float:
        if not np.any(direction_defined):
            return 0.0
        return float(np.mean(np.asarray(mask, dtype=bool)[direction_defined]))

    jumps = np.max(np.abs(np.diff(path, axis=0)), axis=1)
    score = (
        config.position_weight * float(np.sqrt(np.mean(position_error**2)))
        + config.initial_posture_score_weight * float(np.linalg.norm(path[0] - nominal))
        + config.straightness_score_weight
        * float(180.0 - straightness[anchor_index])
        + config.branch_jump_score_weight * float(np.max(jumps))
        + config.elbow_outward_weight
        * float(np.sqrt(np.mean(np.minimum(outward_pole, 0.0) ** 2)))
        + config.elbow_forward_up_weight
        * float(
            np.sqrt(
                np.mean(
                    np.maximum(pole_vectors[:, (0, 2)], 0.0) ** 2
                )
            )
        )
    )
    metrics = {
        "score": score,
        "position_rmse_m": float(np.sqrt(np.mean(position_error**2))),
        "position_p95_m": float(np.quantile(position_error, 0.95)),
        "position_max_m": float(np.max(position_error)),
        "anchor_straightness_deg": float(straightness[anchor_index]),
        "initial_posture_distance_rad": float(np.linalg.norm(path[0] - nominal)),
        "maximum_joint_step_rad": float(np.max(jumps)),
        "maximum_joint_velocity_rad_s": float(
            np.max(np.abs(np.diff(path, axis=0) / np.diff(time)[:, None]))
        ),
        "elbow_direction_defined_fraction": float(np.mean(direction_defined)),
        "inward_elbow_fraction": defined_fraction(
            outward_pole < -config.elbow_direction_violation_tolerance_m
        ),
        "minimum_outward_elbow_pole_m": float(np.min(outward_pole)),
        "forward_elbow_fraction": defined_fraction(
            pole_vectors[:, 0] > config.elbow_direction_violation_tolerance_m
        ),
        "upward_elbow_fraction": defined_fraction(
            pole_vectors[:, 2] > config.elbow_direction_violation_tolerance_m
        ),
    }
    return score, metrics


def solve_offline_arm_trajectory(
    chain: ArmChain,
    time_s: np.ndarray,
    positions_m: np.ndarray,
    orientations: np.ndarray,
    nominal_q: np.ndarray,
    anchor_index: int,
    config: OfflineContinuationConfig,
) -> OfflineArmTrajectoryResult:
    """Select an anchor branch, continue both ways, and refine the full path."""

    config.validate()
    time, positions, rotations, nominal = _validate_trace(
        chain, time_s, positions_m, orientations, nominal_q
    )
    if not 0 <= int(anchor_index) < len(time):
        raise ValueError("anchor_index lies outside the trajectory.")
    anchors = _anchor_candidates(
        chain,
        positions[anchor_index],
        rotations[anchor_index],
        nominal,
        config,
    )
    if not anchors:
        raise RuntimeError("anchor multi-start produced no candidate.")
    evaluated: list[tuple[float, np.ndarray, dict[str, float]]] = []
    for anchor in anchors:
        path = _continue_from_anchor(
            chain,
            time,
            positions,
            rotations,
            int(anchor_index),
            anchor,
            nominal,
            config,
        )
        score, metrics = _path_metrics(
            chain,
            time,
            positions,
            rotations,
            nominal,
            path,
            int(anchor_index),
            config,
        )
        evaluated.append((score, path, metrics))
    evaluated.sort(key=lambda item: item[0])
    straight_candidates = [
        item
        for item in evaluated
        if item[2]["anchor_straightness_deg"]
        >= config.minimum_anchor_straightness_deg
    ]
    natural_candidates = [
        item
        for item in straight_candidates
        if max(
            item[2]["forward_elbow_fraction"],
            item[2]["upward_elbow_fraction"],
        )
        <= config.maximum_candidate_forward_or_up_fraction
    ]
    _, selected, _ = (natural_candidates or straight_candidates or evaluated)[0]
    selected = _refine_path(
        chain,
        time,
        positions,
        rotations,
        selected,
        nominal,
        int(anchor_index),
        config,
    )
    selected = _reproject_position_manifold(
        chain,
        time,
        positions,
        rotations,
        selected,
        nominal,
        config,
    )
    poses = [chain.forward_kinematics(q) for q in selected]
    position_error = np.asarray(
        [np.linalg.norm(pose[:3, 3] - target) for pose, target in zip(poses, positions)]
    )
    orientation_error = np.asarray(
        [
            np.linalg.norm(so3_log(target @ pose[:3, :3].T))
            for pose, target in zip(poses, rotations)
        ]
    )
    straightness = np.asarray([arm_straightness_deg(chain, q) for q in selected])
    return OfflineArmTrajectoryResult(
        q_rad=selected,
        position_error_m=position_error,
        orientation_error_rad=orientation_error,
        straightness_deg=straightness,
        selected_anchor_rank=0,
        anchor_index=int(anchor_index),
        candidate_scores=tuple(item[2] for item in evaluated),
    )


def refine_offline_bilateral_symmetry(
    left_chain: ArmChain,
    right_chain: ArmChain,
    time_s: np.ndarray,
    left_positions_m: np.ndarray,
    left_orientations: np.ndarray,
    right_positions_m: np.ndarray,
    right_orientations: np.ndarray,
    left_q_rad: np.ndarray,
    right_q_rad: np.ndarray,
    nominal_left_q: np.ndarray,
    nominal_right_q: np.ndarray,
    config: OfflineContinuationConfig,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Softly select mirrored branches without copying either arm trajectory.

    The R1 URDF mirror map is ``[+,-,-,+,-]`` for pitch, roll, yaw, elbow and
    wrist roll.  The residual is active only while the two measured wrist
    positions themselves are approximately mirrored.  Each wrist pose,
    natural elbow sector, limits and temporal neighbours remain independent.
    """

    time = np.asarray(time_s, dtype=float)
    left_positions = np.asarray(left_positions_m, dtype=float)
    right_positions = np.asarray(right_positions_m, dtype=float)
    left_orientations = np.asarray(left_orientations, dtype=float)
    right_orientations = np.asarray(right_orientations, dtype=float)
    left_path = np.asarray(left_q_rad, dtype=float).copy()
    right_path = np.asarray(right_q_rad, dtype=float).copy()
    count = len(time)
    expected_path = (count, left_chain.dof)
    if left_path.shape != expected_path or right_path.shape != expected_path:
        raise ValueError("bilateral paths must be matching (N, arm_dof) arrays")
    mirror_sign = np.asarray([1.0, -1.0, -1.0, 1.0, -1.0])
    if left_chain.dof != len(mirror_sign) or right_chain.dof != len(mirror_sign):
        raise ValueError("R1-A5 bilateral symmetry requires two five-DoF arms")
    mirrored_right_target = right_positions * np.asarray([1.0, -1.0, 1.0])
    mirror_error = np.linalg.norm(left_positions - mirrored_right_target, axis=1)
    activation = np.clip(
        1.0 - mirror_error / config.bilateral_target_mirror_tolerance_m,
        0.0,
        1.0,
    )
    if config.bilateral_symmetry_weight <= 0.0:
        return left_path, right_path, activation

    nominal_left = np.asarray(nominal_left_q, dtype=float)
    nominal_right = np.asarray(nominal_right_q, dtype=float)
    for sweep in range(max(1, int(config.refinement_sweeps))):
        indices = range(count) if sweep % 2 == 0 else range(count - 1, -1, -1)
        for index in indices:
            neighbours: list[tuple[np.ndarray, float]] = []
            if index > 0:
                neighbours.append(
                    (
                        np.r_[left_path[index - 1], right_path[index - 1]],
                        float(time[index] - time[index - 1]),
                    )
                )
            if index + 1 < count:
                neighbours.append(
                    (
                        np.r_[left_path[index + 1], right_path[index + 1]],
                        float(time[index + 1] - time[index]),
                    )
                )

            def residual(values: np.ndarray) -> np.ndarray:
                left_q = values[: left_chain.dof]
                right_q = values[left_chain.dof :]
                residuals = _pose_residual(
                    left_chain,
                    left_q,
                    left_positions[index],
                    left_orientations[index],
                    config,
                )
                residuals.extend(
                    _pose_residual(
                        right_chain,
                        right_q,
                        right_positions[index],
                        right_orientations[index],
                        config,
                    )
                )
                for chain, q in ((left_chain, left_q), (right_chain, right_q)):
                    pole_residual = _elbow_branch_residual(chain, q, config)
                    if pole_residual is not None:
                        residuals.append(pole_residual)
                residuals.append(config.posture_weight * (left_q - nominal_left))
                residuals.append(config.posture_weight * (right_q - nominal_right))
                for neighbour, dt in neighbours:
                    residuals.append(config.velocity_weight * (values - neighbour) / dt)
                residuals.append(
                    config.bilateral_symmetry_weight
                    * activation[index]
                    * (right_q - mirror_sign * left_q)
                )
                return np.concatenate(residuals)

            lower = np.r_[left_chain.lower_limits, right_chain.lower_limits]
            upper = np.r_[left_chain.upper_limits, right_chain.upper_limits]
            for neighbour, _ in neighbours:
                lower = np.maximum(lower, neighbour - config.maximum_step_rad)
                upper = np.minimum(upper, neighbour + config.maximum_step_rad)
            seed = np.r_[left_path[index], right_path[index]]
            refined = _bounded_least_squares(
                residual,
                seed,
                lower,
                upper,
                config.continuation_max_nfev,
            )
            left_path[index] = refined[: left_chain.dof]
            right_path[index] = refined[left_chain.dof :]
    return left_path, right_path, activation


__all__ = [
    "OfflineArmTrajectoryResult",
    "OfflineContinuationConfig",
    "arm_straightness_deg",
    "elbow_outward_pole_m",
    "elbow_pole_vector_m",
    "refine_offline_bilateral_symmetry",
    "reproject_position_manifold",
    "solve_offline_arm_trajectory",
]
