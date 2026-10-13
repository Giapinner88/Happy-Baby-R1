"""Declared 3-D target grids for the T002 workspace sweep.

The grid is generated from bounds an experiment declares, and is traversed in a
serpentine (boustrophedon) order so that consecutive targets are spatial
neighbours. That ordering is not cosmetic: the IK solver is seeded with the
previous solution, and continuity is what keeps a sweep inside one elbow branch.
A raster order would jump the full width of the grid on every row change and
invite branch switches that have nothing to do with reachability.

Grid bounds are experiment parameters, not method constants, so nothing here has
a default extent. `docs/teleop/r1_arm_wrist_ik.md` fixes the frame: targets are
endpoint positions in the waist frame, in metres.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


class WorkspaceError(ValueError):
    """Raised when a grid specification cannot produce targets."""


@dataclass(frozen=True)
class GridSpec:
    """Axis-aligned target grid, declared by the experiment configuration."""

    x_range_m: tuple[float, float]
    y_range_m: tuple[float, float]
    z_range_m: tuple[float, float]
    counts: tuple[int, int, int]
    wrist_roll_rad: float

    def validate(self) -> None:
        for name, bounds in (("x", self.x_range_m), ("y", self.y_range_m), ("z", self.z_range_m)):
            if len(bounds) != 2:
                raise WorkspaceError(f"{name}_range_m must hold exactly two values.")
            if bounds[1] < bounds[0]:
                raise WorkspaceError(f"{name}_range_m is inverted: {bounds}")
        if len(self.counts) != 3 or any(count < 1 for count in self.counts):
            raise WorkspaceError(f"counts must be three positive integers, got {self.counts}")

    @property
    def target_count(self) -> int:
        return int(np.prod(self.counts))

    def axis_values(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        self.validate()
        return tuple(  # type: ignore[return-value]
            np.linspace(bounds[0], bounds[1], count)
            for bounds, count in zip((self.x_range_m, self.y_range_m, self.z_range_m), self.counts)
        )


@dataclass(frozen=True)
class GridTarget:
    """One declared target and where it sits in the grid."""

    index: int
    grid_index: tuple[int, int, int]
    position_m: np.ndarray
    wrist_roll_rad: float


def serpentine_targets(spec: GridSpec) -> list[GridTarget]:
    """Grid targets ordered so consecutive entries are spatial neighbours.

    The sweep runs along z, reverses direction each time y steps, and reverses
    the y direction each time x steps, so no two consecutive targets differ by
    more than one grid spacing on one axis.
    """

    xs, ys, zs = spec.axis_values()
    targets: list[GridTarget] = []
    index = 0
    for xi, x in enumerate(xs):
        y_order = range(len(ys)) if xi % 2 == 0 else range(len(ys) - 1, -1, -1)
        for yi in y_order:
            flip_z = (xi + yi) % 2 == 1
            z_order = range(len(zs) - 1, -1, -1) if flip_z else range(len(zs))
            for zi in z_order:
                targets.append(
                    GridTarget(
                        index=index,
                        grid_index=(xi, yi, zi),
                        position_m=np.array([x, ys[yi], zs[zi]]),
                        wrist_roll_rad=spec.wrist_roll_rad,
                    )
                )
                index += 1
    return targets


def max_consecutive_step_m(targets: list[GridTarget]) -> float:
    """Largest jump between consecutive targets; the continuity guarantee."""

    if len(targets) < 2:
        return 0.0
    return max(
        float(np.linalg.norm(targets[i + 1].position_m - targets[i].position_m))
        for i in range(len(targets) - 1)
    )


def grid_spacing_m(spec: GridSpec) -> tuple[float, float, float]:
    """Spacing on each axis; a single-sample axis has zero spacing."""

    xs, ys, zs = spec.axis_values()
    return tuple(  # type: ignore[return-value]
        float(values[1] - values[0]) if len(values) > 1 else 0.0 for values in (xs, ys, zs)
    )


__all__ = [
    "GridSpec",
    "GridTarget",
    "WorkspaceError",
    "grid_spacing_m",
    "max_consecutive_step_m",
    "serpentine_targets",
]
