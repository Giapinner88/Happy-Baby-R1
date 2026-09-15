# Reusable deterministic inputs

This directory owns small, versioned JSONL traces used to exercise the project
mapper without a Quest headset. They are fixtures and preflight inputs, **not
T001 live-transport evidence** and never a substitute for a recorded Quest
session.

- `example_trace.jsonl`: one valid schema example for the simulation-only
  command path.
- `t001_mapper_trace.jsonl`: valid/deadman/foreign-frame mapper preflight.
- `t001_sequence_violation_trace.jsonl`: non-increasing sequence rejection
  fixture.

Evidence-producing inputs are copied or referenced into the individual run;
do not place generated traces or raw Quest captures here.
