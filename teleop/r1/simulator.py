"""Simulation-only adapter boundary for an IsaacLab R1 teleop controller."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

from .mapping import R1JointOwnership, R1TeleopTargets


class IsaacLabR1TeleopSink(Protocol):
    """Implemented by the IsaacLab R1 controller integration, never by DDS."""

    def hold(self, reason: str) -> None: ...

    def apply_upper_body(self, targets: R1TeleopTargets, joints: tuple[str, ...]) -> None: ...

    def apply_base_velocity(self, targets: R1TeleopTargets, joints: tuple[str, ...]) -> None: ...


class SimulationOnlyAdapter:
    """Dispatches disjoint commands to a simulator sink with no hardware path."""

    def __init__(self, sink: IsaacLabR1TeleopSink, ownership: R1JointOwnership | None = None):
        self.sink = sink
        self.ownership = ownership or R1JointOwnership()
        self.ownership.validate()

    def apply(self, targets: R1TeleopTargets) -> None:
        if not targets.enabled:
            self.sink.hold(targets.reason or "disabled")
            return
        self.sink.apply_upper_body(targets, self.ownership.upper_body)
        if targets.base_velocity_enabled:
            self.sink.apply_base_velocity(targets, self.ownership.lower_body)


@dataclass
class FakeIsaacLabSink:
    """Deterministic test sink; it is not a simulator or hardware bridge."""

    events: list[tuple[str, object]] = field(default_factory=list)

    def hold(self, reason: str) -> None:
        self.events.append(("hold", reason))

    def apply_upper_body(self, targets: R1TeleopTargets, joints: tuple[str, ...]) -> None:
        self.events.append(("upper_body", (targets, joints)))

    def apply_base_velocity(self, targets: R1TeleopTargets, joints: tuple[str, ...]) -> None:
        self.events.append(("base_velocity", (targets, joints)))
