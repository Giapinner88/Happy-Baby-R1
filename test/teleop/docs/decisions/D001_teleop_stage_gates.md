# D001 — Gate stages for R1 Quest teleoperation

## Context

The prior sequence used one T001 label for two different questions: whether
Quest state data reaches the host, and whether a project bridge can connect that
live source to the Isaac Sim R1 sink. Those questions have different evidence,
failure modes and execution environments. The project also needs a visible
boundary between simulation evidence and any future hardware work.

## Decision

Use T001-A for capture-only Quest transport/state data and T001-B for live
Quest → project bridge → Isaac Sim connectivity. A valid T001-B result is the
gate to T002. Keep T002–T008 simulation-only. Reserve two provisional hardware
experiments, T009 and T010, after T008; neither has an executable protocol yet.

## Alternatives

- Keep a single T001: rejected because a successful host capture does not prove
  that the Isaac Sim sink received or safely handled an R1 command.
- Start hardware work immediately after T001-B: rejected because arm/wrist,
  robustness, ownership and combined-policy simulation gates remain unresolved.
- Treat T009/T010 as ordinary extensions of T008: rejected because hardware
  evidence requires its own safety, deployment and validity contract.

## Rationale

The split makes the smallest discriminating evidence explicit: T001-A isolates
data acquisition; T001-B isolates the simulation connection and fail-closed
behaviour. Separating T009/T010 prevents simulation claims, including a passing
T008, from being read as hardware authorization.

## Consequences

- Existing completed transport data remains T001-A evidence and the fake-sink
  record remains an unnumbered regression preflight.
- T002 cannot use either capture-only or fake-sink results as a substitute for
  T001-B evidence.
- T009 and T010 require separate hardware experiment records, approved safety
  procedure, operator/E-stop arrangements and explicit deployment compatibility;
  this decision supplies none of those details.

## Validation

Check that future T001-A runs contain only capture data and no R1 command or
simulator action, while T001-B runs contain the live bridge log, simulator
acknowledgements, fail-closed events and temporal evidence. Before creating
T009, review the full T001-A/B and T002–T008 inventories against Gate H in the
simulation plan.

## Reversal Condition

Revisit this decision if the Quest input bridge and Isaac Sim sink become one
verifiable process without losing their separately auditable evidence, or if a
hardware safety review determines that more or fewer than two hardware stages
are needed.
