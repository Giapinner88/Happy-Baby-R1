# AGENTS.md

**Version:** 6.0

Root instruction and routing file for AI agents working in this repository.

Keep this file compact. Project-specific facts belong in Section 0. Detailed procedures and reusable records live under `.agents/` and must be read only when the current task requires them.

## 0. Project Context

> This section is project-owned. Rewrite it for each repository. Sections 1 onward are framework-owned and should remain canonical unless the framework itself is intentionally revised.

### 0.1 Identity and scope

**Project:** Happy Baby R1 Teleop — internal AiRA-Laboratory workspace for Meta Quest 3 control of the Unitree R1-A5 arms and head.

**Primary objective:** Evaluate and operate one traceable Quest 3 → frame mapping → upstream R1-A5 IK → Isaac/hardware target pipeline. This branch deliberately excludes locomotion training, MuJoCo policy work, vision/voice and child-interaction implementation.

**Research question:** Does the initial-head-anchored upstream IK pipeline produce independent, bounded and reproducible arm/head targets, and what simulation and suspended-hardware evidence is required before any broader hardware use?

**Systems:** An Ubuntu 22.04 workstation runs Vuer/Quest transport, upstream IK and Isaac simulation; the Ubuntu 20.04 R1 computer runs the read-only sidecar plus the sole-owner high-level command process. Vendor source required by the active pipeline is limited to the two pinned `xr_teleoperate` revisions under `third_party/`.

**Framework role:** This repository consumes the shared research-agent framework, it does not own it. Supporting guides live under `.agents/`; Section 1 onward is upstream text and must not be edited to accommodate a local task. Root `AGENTS.md` is the active instruction and routing file for agents working here, and `CLAUDE.md` only points to it.

**Current stage:** The upstream vendor simulation baseline exists; bounded suspended-hardware tracking after the latest source-alignment changes remains evidence-required. No evidence authorizes operation on the floor.

**Canonical project sources:** root `README.md` and `AGENTS.md`; implementation in `teleop/r1/`, `scripts/teleop/` and `hardware/`; shared configuration in `config/`; accepted method/runbooks under `docs/`; baseline configuration and evidence under `experiments/r1_teleop/quest3_sim_v1/baseline/`.

**Primary commands:**

```bash
# Inspect available teleop commands and run the simulation path safely
make help
make teleop-dry-run HOST_IP=<IP-workstation>
make teleop HOST_IP=<IP-workstation>

# Run code-level teleop checks without opening a hardware command channel
make test
```

### 0.2 Project-specific authority

For project facts, use the narrowest authoritative source available:

- human-facing purpose and commands: root `README.md`, `docs/README.md`, and `scripts/teleop/README.md`;
- operating and safety procedure: `docs/safety/`, `docs/teleop/r1_quest3_teleop_hardware.md`, and `hardware/teleop/docs/hardware_gate.md`;
- executed behavior: `teleop/r1/`, `scripts/teleop/`, `hardware/teleop/`, and `hardware/high_level_lock/`;
- physical/model parameters: `assets/R1.urdf`, the R1 USD tree, and their meshes under `assets/`;
- accepted method and historical interpretation: `docs/teleop/` and the retained baseline/history records;
- observed evidence: immutable baseline run directories and hardware outputs, together with the code and resolved configuration that generated them;
- framework guidance: `.agents/`.

Do not transfer conventions, results, or assumptions from another robot, another R1 workspace, or vendor code merely because the code or research topic is similar. `third_party/` is reference material, not project authority.

### 0.3 Project-specific invariants

1. Hardware-facing work follows the repository safety procedure: dry-run and simulation evidence first, an E-stop operator present, recorded results. MuJoCo or bridge parity establishes simulation parity only, never hardware readiness.
2. Exactly one component owns the low-level command stream to the robot at a time. A second concurrent `rt/lowcmd` writer is a safety defect, not a configuration choice; DDS does not arbitrate writers and the gamepad/E-stop path must stay authoritative.
3. The two OS tiers are not interchangeable. Validate workstation-built artifacts and dependencies against the Ubuntu 20.04 / ROS 2 Foxy embedded target before deployment.
4. `third_party/` is upstream/vendor code and stays unmodified; adapt it through project-owned wrappers under `scripts/teleop/` or `teleop/r1/`.
5. Quest/vendor IK runs in the `tv` environment, Isaac runs in its documented environment, and the Ubuntu 20.04 robot tier uses its pinned system environment.
6. Regenerated output (`build/`, `install/`, `log/`, run artifacts) is never a documentation source, and no README is added to vendor or regenerated directories.
7. Completed teleop evidence belongs under the owning baseline run or documented hardware output, not under framework templates.
8. Do not create empty framework demonstration folders or records, and keep the repository useful as an engineering project rather than optimizing it for AI navigation alone.
9. Project-specific paths, commands, assumptions, and evidence rules belong in Section 0 or their authoritative project files, not in the shared sections below.

### 0.4 Current project limitations

- ROS 2 Foxy is end-of-life but retained to match the robot baseline; dependency, tooling, and security decisions inherit that constraint.
- Locomotion, training, MuJoCo, vision/voice and child interaction are outside this branch, not verified capabilities of it.
- Section 0 is intentionally conservative and must be updated when accepted project facts change.
- Generic framework guidance cannot determine scientific importance, novelty, or research direction.
- Tests and validators remain necessary; instruction files do not enforce correctness by themselves.

---

## 1. Human Research Authority

The human researcher is the scientific decision-maker.

The human owns:

- the research question and why it matters;
- project scope and priorities;
- novelty judgement;
- consequential method choices;
- interpretation of ambiguous evidence;
- decisions to expand or redirect the study;
- promotion from development work to canonical main;
- final claim wording;
- publication narrative and figure storytelling;
- Git commits unless explicitly delegated.

The agent supports these decisions through implementation, search, comparison, verification, documentation, evidence organization, and adversarial checking.

The agent must not expand the research program merely because more experiments, models, baselines, or plots are possible.

The researcher may deliberately bypass a workflow artifact for a quick exploratory task. Mandatory integrity rules still apply.

---

## 2. Mandatory Invariants

Unless explicitly overridden by the researcher:

1. Do not invent missing technical information, equations, commands, citations, evidence, measurements, or implementation facts.
2. Do not present expected, intended, documented, or simulated behavior as behavior that was actually verified in the relevant run.
3. Preserve negative, contradictory, failed, abnormal, borderline, inconclusive, and superseded evidence when it matters to interpretation.
4. Claim strength must not exceed evidence strength.
5. Prefer the smallest discriminating test over broad scope expansion.
6. Do not silently change success criteria, exclusions, metrics, baselines, protocol, or scientific semantics after inspecting results.
7. Every consequential result must be traceable to code, resolved configuration, data, processing, and analysis appropriate to the claim.
8. Do not reorganize a repository merely to match a generic template.
9. Reusable implementation must not depend on experiment-local outputs, reports, or historical result directories.
10. Each consequential responsibility or source of truth should have one identifiable owner.
11. A clean Git working tree is not a completion criterion.
12. A consequential implementation choice must not remain discoverable only by reading source code.
13. If a consequential uncertainty cannot be resolved, label it as unresolved, temporarily assumed, researcher decision required, or evidence required. Do not silently choose an interpretation.
14. Do not optimize the repository for the agent at the expense of the researcher.
15. Generated framework records must preserve information that would otherwise be ambiguous or lost; otherwise do not create them.

---

## 3. Required Starting Procedure

Before substantial work:

1. Read this file.
2. Read the root `README.md` and the smallest set of project files needed to understand the task.
3. Identify the authoritative implementation, configuration, evidence, and documentation for the affected responsibility.
4. Inspect the current Git working-tree state before modifying files.
5. Distinguish pre-existing dirty files from changes made during the current task.
6. Route to the relevant `.agents/` guide or template only when the task requires it.
7. Identify consequential unknowns before implementation or execution.

Do not load every `.agents/` file by default.

For trivial edits, use the smallest safe path.

---

## 4. Task Routing

Read `.agents/repo_structure.md` when creating, restructuring, integrating, or auditing repository architecture.

Read `.agents/readme-guide.md` when creating, substantially rewriting, or auditing the root README. Do not fabricate commands or status.

Read `.agents/workflow.md` for substantial research work spanning multiple stages, development branches, evidence generation, promotion, or publication preparation.

Read the relevant template when the task matches its role:

- `method.md` — accepted model, controller, observer, estimator, learner, solver, derivation, or scientific algorithm;
- `implementation-disclosure.md` — consequential implementation semantics, hidden/default choices, runtime behavior, data flow, or AI-selected decisions;
- `experiment.md` — one bounded development, validation, calibration, diagnostic, or canonical evidence protocol;
- `simulation-study.md` — declared multi-case comparison, sweep, robustness, sensitivity, repeated-trial, or numerical study;
- `analysis.md` — interpretation of already valid evidence;
- `integration-and-promotion.md` — synthesis of development branches into a canonical main state;
- `figure-design-and-evidence.md` — durable evidence or publication figures;
- `technical-decision.md` — non-obvious consequential technical or scientific choice;
- `literature-note.md` — external source important enough to influence the project.

Ordinary debug plots, small implementation notes, and routine refactors do not require dedicated records unless they preserve consequential information.

---

## 5. Research Scope and Decision Discipline

### 5.1 No autonomous scope expansion

When evidence is weak or inconclusive:

```text
identify dominant uncertainty
→ identify competing explanations
→ propose the smallest discriminating test
```

Do not automatically respond with a larger sweep, more models, more controllers, more seeds, or a new research objective.

### 5.2 No method proliferation

A new method, baseline, model, controller, or ablation must have a clear decision target, such as:

- testing a scientific hypothesis;
- resolving a known confound;
- establishing a meaningful baseline;
- evaluating a boundary or robustness question;
- enabling or rejecting promotion to main.

“More complete benchmarking” is not sufficient by itself.

### 5.3 Experiment justification

Before scaling an experiment or study, be able to state:

```text
question
→ unresolved uncertainty
→ evidence needed
→ decision affected
```

If the final arrow is unclear, do not scale by default.

### 5.4 Suggestion versus decision

The agent may recommend a consequential scientific choice and explain the evidence and trade-offs. It must not silently redefine the research question, success criterion, main-paper scope, or canonical claim.

Minor implementation choices may be resolved autonomously when they do not alter scientific or behavioral semantics.

---

## 6. Implementation Transparency

For consequential implementation work, use `.agents/templates/implementation-disclosure.md`.

At minimum, expose:

- requested behavior;
- behavior actually implemented;
- architecture and interfaces;
- runtime/control semantics;
- data flow;
- defaults and inherited choices;
- AI-selected choices;
- approximations and shortcuts;
- fallbacks, clipping, buffering, interpolation, warm-start, initialization, and randomness when relevant;
- code symbols that own each behavior;
- compatibility impact on existing evidence.

Classify consequential choices by origin:

- **specified** — explicitly requested or defined by an authoritative project source;
- **inherited** — preserved from existing code/configuration;
- **AI-selected** — chosen by the agent because no authoritative value was specified;
- **empirically selected** — chosen from declared evidence;
- **temporary assumption** — used only to unblock work and requiring review.

A useful disclosure is an inspection surface, not a code dump and not a chain-of-thought transcript.

### Gate D — Design disclosure accepted

Before an evidence-producing run after a consequential implementation change, verify that the researcher can inspect the effective architecture, interfaces, runtime semantics, consequential defaults, approximations, and code ownership without reading the entire codebase.

---

## 7. Semantic Compatibility and Validation

After a consequential change, explicitly check whether each relevant contract changed:

```text
observation contract
actuation/action contract
state or latent definition
physical boundary conditions
control/replan timing
training/inference semantics
success/failure metric
baseline definition
data schema
numerical method or fidelity
```

If any changed, identify which prior evidence remains comparable, becomes caveated, requires reproduction, or is superseded.

Use precise validation language:

- **code verified** — relevant tests/static checks passed;
- **workflow verified** — a representative documented workflow executed successfully;
- **scientifically reproduced** — the canonical evidence-producing protocol was rerun and its evidence checked.

Do not collapse these into one generic “verified”.

---

## 8. Evidence and Claim Discipline

Evidence states may include:

- exploratory;
- indicative;
- valid bounded evidence;
- measured/reproduced;
- caveated;
- invalid;
- refuted;
- superseded;
- not established.

A project targeting a paper or thesis should maintain a compact claim registry when useful. It may live under `papers/`, `docs/`, or another project-owned location and should map important claims to status, evidence, and limitations.

Do not create one claim file per claim by default.

A publication narrative may consume only evidence whose provenance and compatibility are understood.

---

## 9. Development and Canonical Main

Development branches may contain prototypes, bounded experiments, diagnostics, temporary configs, and rejected ideas.

Canonical main should contain the smallest coherent implementation, workflow, evidence structure, claims, and figures needed to represent the accepted research state.

Main is synthesized from development; it is not a dump of development history.

Use `.agents/templates/integration-and-promotion.md` for consequential branch integration.

Promotion decisions may include:

- `PROMOTE` — accepted largely as-is;
- `PORT` — preserve the idea/semantics but adapt the implementation to main architecture;
- `REPRODUCE` — regenerate evidence on canonical main before using it;
- `REFERENCE` — retain as historical or conceptual reference only;
- `ARCHIVE` — preserve inactive material without active dependency;
- `REJECT` — do not integrate;
- `SUPERSEDE` — replace an older canonical item while preserving compatibility history.

Development figures should normally be treated as visual references and regenerated from canonical evidence when they become publication figures.

---

## 10. Figure and Image Rules

For durable evidence or publication figures, follow `.agents/templates/figure-design-and-evidence.md`.

Core rules:

1. Figure planning may occur before data collection to ensure required signals are logged; final publication rendering follows valid analysis.
2. Data generation, analysis, and figure rendering are separate responsibilities.
3. Restyling a figure must not require rerunning a scientific experiment.
4. A project with multiple publication figures should use one project-wide visual style source for dimensions, typography, semantic colors, line styles, and export behavior.
5. Do not use arbitrary semantic colors or font sizes inside individual publication plotting functions when a project-wide style system exists.
6. Photographic or hardware panels may be represented by declared placeholders until real assets exist.
7. Never fabricate a hardware photograph or experimental image to fill a placeholder.
8. Image edits must preserve the original scientific content and comply with the target venue's image-integrity rules.

When targeting Nature or a Nature-family workflow, use the Nature profile in the figure template and verify the current journal requirements before submission.

---

## 11. Git and Change Authority

### 11.1 Commit authority

Do not create commits unless the researcher explicitly requests a commit.

Do not commit merely to obtain a clean working tree.

Do not amend, squash, rebase, reset, revert, discard changes, switch branches, or rewrite history unless explicitly authorized.

A dirty working tree is an acceptable development state.

### 11.2 Change ownership

Before modifying files, inspect current status.

Do not claim ownership of a diff merely because it is uncommitted.

At completion, distinguish:

- pre-existing changes;
- changes made by the current task;
- generated artifacts;
- files intentionally left untouched.

For substantial uncommitted work, maintain `.agents/change-ledger.md` when useful. It is a local development ledger, not Git history and not a replacement for commits.

A useful ledger records intent, files changed, behavioral/scientific impact, validation, unresolved items, and current uncommitted status. Do not store giant diffs in it.

---

## 12. Repository and Documentation Discipline

1. Stable scientific capability should have one owner.
2. Thin human-facing runners should compose reusable modules rather than duplicate them.
3. Avoid parallel active source/config/result hierarchies unless their responsibilities are explicitly different.
4. Do not create a directory merely because a new run, seed, figure, or temporary script exists.
5. Documentation should link to authoritative values rather than duplicate them without need.
6. The root README is a human entry point, not a thesis, changelog, or duplicate of `AGENTS.md`.
7. Archive inactive material only when active code does not depend on it.
8. Prefer inspectable consequential behavior over clever abstractions that hide scientific semantics.

---

## 13. Completion and Researcher Handoff

A substantial task is complete when the requested work is implemented or analyzed, relevant validation has been performed, scientific/behavioral compatibility is understood, and remaining uncertainty is explicit.

Do not require `git status` to be clean.

Final handoff should concisely report, when relevant:

```text
Implemented
AI-selected or temporary choices
Scientific / behavioral changes
Validation actually performed
Evidence compatibility
Unresolved items
Files or symbols worth human inspection
Git state
```

The human should be able to inspect consequential decisions without rereading the full codebase.

If knowing an implementation detail would materially change how the researcher describes the method in a paper, that detail must be disclosed.
