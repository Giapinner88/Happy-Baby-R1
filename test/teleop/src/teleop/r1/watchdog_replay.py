"""Deterministic event replay for T005 mapper/watchdog verification."""
from __future__ import annotations

from dataclasses import dataclass
from math import sin, cos

from .mapping import R1TeleopMapper, R1TeleopTargets
from .schema import BaseVelocity, Pose, Quaternion, R1TeleopCommand, Vector3
from .simulator import FakeIsaacLabSink, SimulationOnlyAdapter


@dataclass(frozen=True)
class ReplayResult:
    raw_commands: list[dict[str, object]]
    records: list[dict[str, object]]
    sink_events: list[tuple[str, object]]


def _command(event: dict[str, object]) -> R1TeleopCommand:
    sequence = int(event["sequence_id"])
    yaw = 0.1 * sequence
    pose = Pose(Vector3(0.1, 0.0, 0.2), Quaternion(0.0, 0.0, sin(yaw / 2.0), cos(yaw / 2.0)))
    return R1TeleopCommand(
        sequence_id=sequence,
        timestamp_monotonic_s=float(event["timestamp_s"]),
        deadman_enabled=bool(event.get("deadman_enabled", True)),
        head_pose=pose, left_wrist_pose=pose, right_wrist_pose=pose,
        base_velocity=BaseVelocity.zero(), source_frame="quest_headset",
    )


def replay(events: list[dict[str, object]], mapper: R1TeleopMapper) -> ReplayResult:
    sink = FakeIsaacLabSink()
    adapter = SimulationOnlyAdapter(sink, mapper.ownership)
    previous_sequence = -1
    last_command: R1TeleopCommand | None = None
    raw_commands: list[dict[str, object]] = []
    records: list[dict[str, object]] = []
    for index, event in enumerate(events):
        kind = str(event["kind"])
        received_s = float(event["received_s"])
        if kind == "command":
            command = _command(event)
            raw_commands.append(command.as_dict())
            if command.sequence_id <= previous_sequence:
                target = R1TeleopTargets(command.sequence_id, False, "sequence_id_not_increasing", None, None, 0.0, 0.0, BaseVelocity.zero(), False, mapper.calibration.robot_frame)
            else:
                previous_sequence = command.sequence_id
                last_command = command
                target = mapper.map(command, received_s)
        elif kind == "watchdog_tick":
            if last_command is None:
                raise ValueError("watchdog_tick requires an earlier valid command")
            target = mapper.map(last_command, received_s)
        else:
            raise ValueError(f"Unknown T005 event kind {kind!r}")
        before = len(sink.events)
        adapter.apply(target)
        records.append({"event_index": index, "kind": kind, "received_s": received_s, "enabled": target.enabled, "reason": target.reason, "sequence_id": target.sequence_id, "command_age_s": received_s - (last_command.timestamp_monotonic_s if last_command else received_s), "sink_events": [item[0] for item in sink.events[before:]]})
    return ReplayResult(raw_commands, records, sink.events)
